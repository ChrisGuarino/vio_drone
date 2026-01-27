"""Launch PX4 SITL + Gazebo + ROS-GZ bridge for VIO development."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    px4_dir = LaunchConfiguration('px4_dir', default='~/PX4-Autopilot')

    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir',
            default_value='~/PX4-Autopilot',
            description='Path to PX4-Autopilot directory'
        ),

        # Start Micro XRCE-DDS Agent (bridges PX4 uORB <-> ROS 2 DDS)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen',
            name='micro_xrce_agent'
        ),

        # Start PX4 SITL with Gazebo
        ExecuteProcess(
            cmd=['bash', '-c',
                 'cd ~/PX4-Autopilot && make px4_sitl gz_x500'],
            output='screen',
            name='px4_sitl'
        ),

        # Bridge Gazebo topics to ROS 2 (delayed to let Gazebo start)
        TimerAction(
            period=5.0,
            actions=[
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
                    name='ros_gz_bridge'
                ),
            ]
        ),
    ])
