"""
Full VIO Drone System Launch

Starts all components:
1. PX4 SITL + Gazebo (external, must be started separately)
2. Micro XRCE-DDS Agent
3. ROS-Gazebo bridge
4. VIO Frontend
5. VIO Backend
6. VIO-PX4 Bridge
7. VIO Evaluation

Usage:
  # Terminal 1: Start PX4 SITL
  cd ~/PX4-Autopilot && make px4_sitl gz_x500

  # Terminal 2: Launch VIO system
  ros2 launch vio_sim full_system_launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument('max_features', default_value='150'),
        DeclareLaunchArgument('min_features', default_value='80'),
        DeclareLaunchArgument('window_size', default_value='10'),
        DeclareLaunchArgument('eval_output_dir', default_value='/tmp/vio_eval'),

        # --- Micro XRCE-DDS Agent ---
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen',
            name='micro_xrce_agent',
        ),

        # --- ROS-Gazebo Bridge ---
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/model/x500/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            ],
            remappings=[
                ('/camera', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info'),
                ('/imu', '/imu/data'),
                ('/model/x500/pose', '/ground_truth/pose'),
            ],
            output='screen',
            name='ros_gz_bridge',
        ),

        # --- VIO Frontend ---
        Node(
            package='vio_frontend',
            executable='frontend_node',
            name='vio_frontend_node',
            parameters=[{
                'max_features': LaunchConfiguration('max_features'),
                'min_features': LaunchConfiguration('min_features'),
                'quality_level': 0.01,
                'min_distance': 20.0,
                'lk_win_size': 21,
                'lk_max_level': 3,
                'ransac_threshold': 1.0,
                'fb_threshold': 1.0,
                'publish_debug_image': True,
            }],
            output='screen',
        ),

        # --- VIO Backend ---
        Node(
            package='vio_backend',
            executable='backend_node',
            name='vio_backend_node',
            parameters=[{
                'gravity': 9.81,
                'init_samples': 400,
                'window_size': LaunchConfiguration('window_size'),
                'camera_fx': 462.14,
                'camera_fy': 462.14,
                'camera_cx': 320.0,
                'camera_cy': 240.0,
                'min_features_for_keyframe': 15,
            }],
            output='screen',
        ),

        # --- VIO-PX4 Bridge ---
        Node(
            package='vio_bridge',
            executable='bridge_node',
            name='vio_bridge_node',
            parameters=[{
                'position_variance': [0.1, 0.1, 0.1],
                'orientation_variance': [0.01, 0.01, 0.01],
                'velocity_variance': [0.1, 0.1, 0.1],
            }],
            output='screen',
        ),

        # --- VIO Evaluation ---
        Node(
            package='vio_eval',
            executable='eval_node',
            name='vio_eval_node',
            parameters=[{
                'output_dir': LaunchConfiguration('eval_output_dir'),
                'log_interval': 100,
            }],
            output='screen',
        ),
    ])
