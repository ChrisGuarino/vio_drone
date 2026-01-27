"""
VIO-PX4 Bridge Node

Converts VIO odometry from ROS 2 conventions (ENU frame, nav_msgs/Odometry)
to PX4 conventions (NED/FRD frame, px4_msgs/VehicleOdometry) and publishes
to the uXRCE-DDS topic for PX4 EKF2 external vision fusion.

Frame conventions:
  ROS 2 / ENU: x=East, y=North, z=Up
  PX4 / NED:   x=North, y=East, z=Down

Quaternion conventions:
  ROS 2: [x, y, z, w]
  PX4:   [w, x, y, z]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
import numpy as np

try:
    from px4_msgs.msg import VehicleOdometry
    HAS_PX4_MSGS = True
except ImportError:
    HAS_PX4_MSGS = False


class VioBridgeNode(Node):
    def __init__(self):
        super().__init__('vio_bridge_node')

        if not HAS_PX4_MSGS:
            self.get_logger().error(
                'px4_msgs not found. Install from: '
                'https://github.com/PX4/px4_msgs')
            return

        # Parameters
        self.declare_parameter('position_variance', [0.1, 0.1, 0.1])
        self.declare_parameter('orientation_variance', [0.01, 0.01, 0.01])
        self.declare_parameter('velocity_variance', [0.1, 0.1, 0.1])

        # QoS matching PX4 uXRCE-DDS expectations
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber: VIO odometry in ENU
        self.odom_sub = self.create_subscription(
            Odometry, '/vio/odometry', self.odom_callback, 10)

        # Publisher: PX4 VehicleOdometry in NED
        self.px4_pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            px4_qos
        )

        self.msg_count = 0
        self.get_logger().info('VIO-PX4 Bridge initialized')

    def odom_callback(self, msg):
        if not HAS_PX4_MSGS:
            return

        px4_msg = VehicleOdometry()

        # Timestamp in microseconds
        now_ns = self.get_clock().now().nanoseconds
        px4_msg.timestamp = now_ns // 1000
        px4_msg.timestamp_sample = now_ns // 1000

        # --- Position: ENU -> NED ---
        p = msg.pose.pose.position
        px4_msg.position = [
            float(p.y),     # NED x = ENU y (North)
            float(p.x),     # NED y = ENU x (East)
            float(-p.z)     # NED z = -ENU z (Down)
        ]

        # --- Orientation: ENU -> NED ---
        # Convert quaternion from ENU body frame to NED body frame
        q = msg.pose.pose.orientation
        q_enu = np.array([q.w, q.x, q.y, q.z])  # [w, x, y, z]

        # Rotation from ENU to NED:
        # R_NED_ENU = Rz(pi/2) * Rx(pi)
        # As quaternion: q_NED_ENU = [0, sqrt(2)/2, sqrt(2)/2, 0]
        # Also need FLU body -> FRD body: Rx(pi) = [0, 1, 0, 0]
        #
        # Full conversion: q_ned = q_NED_ENU * q_enu * q_FLU_FRD
        # Simplified for standard ENU<->NED:
        # q_ned = [q_enu[0], q_enu[2], q_enu[1], -q_enu[3]]
        q_ned = np.array([q_enu[0], q_enu[2], q_enu[1], -q_enu[3]])

        # Normalize
        q_ned = q_ned / np.linalg.norm(q_ned)

        # PX4 expects [w, x, y, z]
        px4_msg.q = [float(q_ned[0]), float(q_ned[1]),
                     float(q_ned[2]), float(q_ned[3])]

        # --- Velocity: ENU -> NED ---
        v = msg.twist.twist.linear
        px4_msg.velocity = [
            float(v.y),     # NED vx = ENU vy
            float(v.x),     # NED vy = ENU vx
            float(-v.z)     # NED vz = -ENU vz
        ]

        # --- Frame conventions ---
        px4_msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        px4_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_FRD

        # --- Covariance ---
        pos_var = self.get_parameter('position_variance').value
        ori_var = self.get_parameter('orientation_variance').value
        vel_var = self.get_parameter('velocity_variance').value

        px4_msg.position_variance = [float(v) for v in pos_var]
        px4_msg.orientation_variance = [float(v) for v in ori_var]
        px4_msg.velocity_variance = [float(v) for v in vel_var]

        self.px4_pub.publish(px4_msg)

        self.msg_count += 1
        if self.msg_count % 100 == 0:
            self.get_logger().info(
                f'Published {self.msg_count} VehicleOdometry messages to PX4')


def main(args=None):
    rclpy.init(args=args)
    node = VioBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
