# VIO Drone System

A modular Visual-Inertial Odometry (VIO) pipeline for autonomous drone navigation, built on ROS 2 with PX4 integration and Gazebo simulation.

The system estimates a drone's position, orientation, and velocity by fusing monocular camera images with IMU data using factor graph optimization, and feeds the result into PX4's EKF2 for flight control.

## Architecture

```
Camera + IMU (Gazebo)
    → vio_frontend  (feature tracking ~30 Hz)
    → vio_backend   (IMU preintegration + factor graph optimization)
    → /vio/odometry
        ├─→ vio_bridge → PX4 EKF2 (flight control)
        └─→ vio_eval   → metrics + CSV logs
```

## Packages

| Package | Description |
|---|---|
| `vio_interfaces` | Custom ROS 2 messages (`TrackedFeatures`, `VioState`) |
| `vio_frontend` | Shi-Tomasi corner detection, Lucas-Kanade optical flow tracking, forward-backward consistency check, RANSAC outlier rejection |
| `vio_backend` | GTSAM iSAM2 sliding-window factor graph fusing IMU preintegration with smart visual projection factors |
| `vio_bridge` | ENU → NED coordinate frame conversion for PX4 via uXRCE-DDS |
| `vio_eval` | Absolute Trajectory Error (ATE) computation against Gazebo ground truth with CSV logging |
| `vio_sim` | Gazebo models, worlds, and launch files |

## Prerequisites

- ROS 2 (Humble or later)
- PX4-Autopilot (with Gazebo SITL support)
- Gazebo (Garden or later)
- Micro XRCE-DDS Agent
- Python 3.10+

### Python Dependencies

```
numpy >= 1.24
opencv-python >= 4.8
scipy >= 1.10
gtsam >= 4.2
transforms3d
matplotlib
```

### ROS 2 Dependencies

- `px4_msgs`
- `ros_gz_bridge`, `ros_gz_image`, `ros_gz_sim`
- `cv_bridge`
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`

## Build

```bash
cd <workspace_root>
pip install -r src/vio_drone/requirements.txt
colcon build
source install/setup.bash
```

## Run

### Terminal 1 — PX4 SITL + Gazebo

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

### Terminal 2 — VIO System

```bash
ros2 launch vio_sim full_system_launch.py
```

This launches the full pipeline: uXRCE-DDS agent, Gazebo-ROS bridge, frontend, backend, PX4 bridge, and evaluation node.

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `max_features` | 150 | Maximum features to track |
| `min_features` | 80 | Minimum features before re-detection |
| `window_size` | 10 | Sliding window keyframe count |
| `eval_output_dir` | `/tmp/vio_eval` | Directory for evaluation CSV output |

Example with custom parameters:

```bash
ros2 launch vio_sim full_system_launch.py max_features:=200 window_size:=15
```

## Topics

| Topic | Type | Description |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Grayscale camera feed |
| `/imu/data` | `sensor_msgs/Imu` | IMU at 200 Hz |
| `/vio/tracked_features` | `vio_interfaces/TrackedFeatures` | Tracked feature observations |
| `/vio/odometry` | `nav_msgs/Odometry` | VIO pose and velocity estimate |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | Odometry sent to PX4 (NED frame) |
| `/vio/eval/position_error` | `std_msgs/Float64` | Real-time position error |
| `/vio/debug/image` | `sensor_msgs/Image` | Debug visualization with feature tracks |
| `/ground_truth/pose` | `geometry_msgs/PoseStamped` | Gazebo ground truth |

## Project Structure

```
vio_drone/
├── vio_interfaces/
│   └── msg/
│       ├── TrackedFeatures.msg
│       └── VioState.msg
├── vio_frontend/
│   └── vio_frontend/
│       └── frontend_node.py
├── vio_backend/
│   └── vio_backend/
│       ├── backend_node.py
│       ├── initializer.py
│       ├── imu_preintegrator.py
│       └── state_estimator.py
├── vio_bridge/
│   └── vio_bridge/
│       └── bridge_node.py
├── vio_eval/
│   └── vio_eval/
│       └── eval_node.py
├── vio_sim/
│   ├── launch/
│   │   ├── full_system_launch.py
│   │   └── sim_launch.py
│   ├── models/x500_vio/
│   ├── config/
│   └── worlds/
└── requirements.txt
```
