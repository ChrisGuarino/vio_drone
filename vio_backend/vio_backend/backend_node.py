"""
VIO Backend Node

Orchestrates:
1. IMU buffering and preintegration
2. Static initialization
3. Factor graph state estimation
4. Odometry publishing

Subscribes to:
  - /imu/data (sensor_msgs/Imu) at 200 Hz
  - /vio/tracked_features (vio_interfaces/TrackedFeatures) at 30 Hz

Publishes:
  - /vio/odometry (nav_msgs/Odometry) at keyframe rate (~30 Hz)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Vector3, PoseWithCovariance, TwistWithCovariance
from vio_interfaces.msg import TrackedFeatures
import numpy as np
import cv2

from .initializer import VioInitializer
from .imu_preintegrator import ImuPreintegrator
from .state_estimator import StateEstimator


class VioBackendNode(Node):
    def __init__(self):
        super().__init__('vio_backend_node')

        # Parameters
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('init_samples', 400)
        self.declare_parameter('window_size', 10)
        self.declare_parameter('camera_fx', 462.14)
        self.declare_parameter('camera_fy', 462.14)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        self.declare_parameter('min_features_for_keyframe', 15)

        gravity = self.get_parameter('gravity').value
        init_samples = self.get_parameter('init_samples').value
        window_size = self.get_parameter('window_size').value

        # Camera intrinsics
        self.fx = self.get_parameter('camera_fx').value
        self.fy = self.get_parameter('camera_fy').value
        self.cx = self.get_parameter('camera_cx').value
        self.cy = self.get_parameter('camera_cy').value
        self.K = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float64)

        # Components
        self.initializer = VioInitializer(
            required_samples=init_samples, gravity=gravity)
        self.preintegrator = ImuPreintegrator(gravity_magnitude=gravity)
        self.estimator = StateEstimator(
            window_size=window_size,
            camera_cal={'fx': self.fx, 'fy': self.fy,
                        'cx': self.cx, 'cy': self.cy}
        )

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback,
            qos_profile_sensor_data)
        self.features_sub = self.create_subscription(
            TrackedFeatures, '/vio/tracked_features',
            self.features_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/vio/odometry', 10)

        # State
        self.initialized = False
        self.last_keyframe_time = None
        self.imu_count = 0

        self.get_logger().info('VIO Backend initialized, waiting for IMU data...')

    def imu_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        accel = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        gyro = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]

        if not self.initialized:
            self.initializer.add_imu(accel, gyro)
            self.imu_count += 1
            if self.imu_count % 200 == 0:
                self.get_logger().info(
                    f'Collecting IMU for init: {self.imu_count} samples')
        else:
            self.preintegrator.add_imu_measurement(t, accel, gyro)

    def features_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # --- Initialization phase ---
        if not self.initialized:
            success, R, vel, gyro_bias, accel_bias = \
                self.initializer.try_initialize()
            if success:
                self.estimator.initialize(R, vel, accel_bias, gyro_bias)
                self.preintegrator.update_bias(accel_bias, gyro_bias)
                self.initialized = True
                self.last_keyframe_time = t
                self.get_logger().info(
                    'VIO initialized! Gyro bias: '
                    f'[{gyro_bias[0]:.5f}, {gyro_bias[1]:.5f}, {gyro_bias[2]:.5f}], '
                    f'Accel bias: [{accel_bias[0]:.5f}, {accel_bias[1]:.5f}, {accel_bias[2]:.5f}]')
            else:
                if self.initializer.ready():
                    self.get_logger().warn(
                        'Init failed: drone may not be stationary')
            return

        # --- Check minimum features ---
        min_feats = self.get_parameter('min_features_for_keyframe').value
        if msg.num_features < min_feats:
            self.get_logger().warn(
                f'Only {msg.num_features} features, need {min_feats} for keyframe')
            return

        # --- Preintegrate IMU between last keyframe and now ---
        pim = self.preintegrator.preintegrate_between(
            self.last_keyframe_time, t)

        # --- Prepare feature observations ---
        # Convert pixel coordinates to normalized camera coordinates
        feature_observations = []
        for i in range(msg.num_features):
            fid = msg.feature_ids[i]
            u_px = msg.u_cur[i]
            v_px = msg.v_cur[i]

            # Pixel to normalized coordinates: x_norm = (u - cx) / fx
            u_norm = (u_px - self.cx) / self.fx
            v_norm = (v_px - self.cy) / self.fy

            feature_observations.append((fid, u_norm, v_norm))

        # --- Add keyframe to factor graph ---
        result = self.estimator.add_keyframe(pim, feature_observations)

        if result is not None:
            pose, vel, bias = result

            # Update preintegrator with new bias estimate
            self.preintegrator.update_bias(
                bias.accelerometer(), bias.gyroscope())

            # Publish odometry
            self._publish_odometry(msg.header, pose, vel)

        self.last_keyframe_time = t

        # Clean up old IMU measurements
        oldest = self.last_keyframe_time - 5.0  # keep 5s of history
        self.preintegrator.clear_old_measurements(oldest)

    def _publish_odometry(self, header, pose, velocity):
        """Publish VIO odometry in ENU frame."""
        odom = Odometry()
        odom.header.stamp = header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        t = pose.translation()
        odom.pose.pose.position = Point(
            x=float(t[0]), y=float(t[1]), z=float(t[2]))

        # Orientation (GTSAM quaternion: w, x, y, z)
        q = pose.rotation().toQuaternion()
        odom.pose.pose.orientation = Quaternion(
            x=float(q.x()), y=float(q.y()),
            z=float(q.z()), w=float(q.w()))

        # Linear velocity
        odom.twist.twist.linear = Vector3(
            x=float(velocity[0]),
            y=float(velocity[1]),
            z=float(velocity[2]))

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = VioBackendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
