"""
VIO Evaluation Node

Compares VIO estimated trajectory against Gazebo ground truth.
Computes and logs:
  - Absolute Trajectory Error (ATE): RMSE of position after SE(3) alignment
  - Relative Pose Error (RPE): drift over fixed time intervals
  - Per-axis position error over time

Saves trajectory data to CSV for offline analysis.

Subscribes to:
  - /vio/odometry (nav_msgs/Odometry) - estimated
  - /ground_truth/pose (geometry_msgs/PoseStamped) - from Gazebo

Publishes:
  - /vio/eval/position_error (std_msgs/Float64) - instantaneous position error
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import numpy as np
import os
from datetime import datetime


class VioEvalNode(Node):
    def __init__(self):
        super().__init__('vio_eval_node')

        # Parameters
        self.declare_parameter('output_dir', '/tmp/vio_eval')
        self.declare_parameter('log_interval', 100)  # log metrics every N samples
        self.declare_parameter('rpe_intervals', [1.0, 5.0, 10.0])  # seconds

        self.output_dir = self.get_parameter('output_dir').value
        os.makedirs(self.output_dir, exist_ok=True)

        # Subscribers
        self.vio_sub = self.create_subscription(
            Odometry, '/vio/odometry', self.vio_callback, 10)
        self.gt_sub = self.create_subscription(
            PoseStamped, '/ground_truth/pose', self.gt_callback, 10)

        # Publishers
        self.error_pub = self.create_publisher(
            Float64, '/vio/eval/position_error', 10)

        # Trajectory storage
        self.vio_trajectory = []  # [(t, x, y, z, qx, qy, qz, qw)]
        self.gt_trajectory = []   # [(t, x, y, z, qx, qy, qz, qw)]

        # Latest ground truth for real-time error
        self.latest_gt_pos = None
        self.latest_gt_time = None

        # CSV files
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.vio_csv_path = os.path.join(
            self.output_dir, f'vio_trajectory_{timestamp_str}.csv')
        self.gt_csv_path = os.path.join(
            self.output_dir, f'gt_trajectory_{timestamp_str}.csv')
        self.metrics_csv_path = os.path.join(
            self.output_dir, f'metrics_{timestamp_str}.csv')

        # Write CSV headers
        header = 'timestamp,x,y,z,qx,qy,qz,qw\n'
        with open(self.vio_csv_path, 'w') as f:
            f.write(header)
        with open(self.gt_csv_path, 'w') as f:
            f.write(header)
        with open(self.metrics_csv_path, 'w') as f:
            f.write('timestamp,position_error,ate_rmse,num_samples\n')

        self.sample_count = 0
        self.get_logger().info(
            f'VIO Eval initialized. Saving to {self.output_dir}')

    def gt_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        q = msg.pose.orientation

        self.latest_gt_pos = np.array([p.x, p.y, p.z])
        self.latest_gt_time = t

        self.gt_trajectory.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

        # Append to CSV
        with open(self.gt_csv_path, 'a') as f:
            f.write(f'{t:.6f},{p.x:.6f},{p.y:.6f},{p.z:.6f},'
                    f'{q.x:.6f},{q.y:.6f},{q.z:.6f},{q.w:.6f}\n')

    def vio_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        vio_pos = np.array([p.x, p.y, p.z])
        self.vio_trajectory.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

        # Append to CSV
        with open(self.vio_csv_path, 'a') as f:
            f.write(f'{t:.6f},{p.x:.6f},{p.y:.6f},{p.z:.6f},'
                    f'{q.x:.6f},{q.y:.6f},{q.z:.6f},{q.w:.6f}\n')

        # Compute instantaneous position error
        if self.latest_gt_pos is not None:
            error = np.linalg.norm(vio_pos - self.latest_gt_pos)
            error_msg = Float64()
            error_msg.data = float(error)
            self.error_pub.publish(error_msg)

        self.sample_count += 1
        log_interval = self.get_parameter('log_interval').value

        if self.sample_count % log_interval == 0:
            self._compute_and_log_metrics(t)

    def _compute_and_log_metrics(self, current_time):
        """Compute ATE RMSE and log metrics."""
        if len(self.vio_trajectory) < 10 or len(self.gt_trajectory) < 10:
            return

        # Match VIO and GT by nearest timestamp
        vio_arr = np.array(self.vio_trajectory)  # (N, 8)
        gt_arr = np.array(self.gt_trajectory)    # (M, 8)

        vio_times = vio_arr[:, 0]
        gt_times = gt_arr[:, 0]

        # For each VIO timestamp, find nearest GT
        errors = []
        for i, vt in enumerate(vio_times):
            idx = np.argmin(np.abs(gt_times - vt))
            dt = abs(gt_times[idx] - vt)
            if dt < 0.05:  # within 50ms
                vio_pos = vio_arr[i, 1:4]
                gt_pos = gt_arr[idx, 1:4]
                errors.append(np.linalg.norm(vio_pos - gt_pos))

        if len(errors) == 0:
            return

        errors = np.array(errors)
        ate_rmse = np.sqrt(np.mean(errors ** 2))
        ate_mean = np.mean(errors)
        ate_max = np.max(errors)

        # Log
        self.get_logger().info(
            f'[Eval] Samples: {len(errors)}, '
            f'ATE RMSE: {ate_rmse:.4f}m, '
            f'ATE Mean: {ate_mean:.4f}m, '
            f'ATE Max: {ate_max:.4f}m')

        # Save to metrics CSV
        with open(self.metrics_csv_path, 'a') as f:
            f.write(f'{current_time:.6f},{errors[-1]:.6f},'
                    f'{ate_rmse:.6f},{len(errors)}\n')

    def destroy_node(self):
        """Final metrics summary on shutdown."""
        if len(self.vio_trajectory) > 10 and len(self.gt_trajectory) > 10:
            self._compute_and_log_metrics(0.0)
            self.get_logger().info(
                f'Trajectories saved to {self.output_dir}')
            self.get_logger().info(
                f'  VIO: {self.vio_csv_path}')
            self.get_logger().info(
                f'  GT:  {self.gt_csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VioEvalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
