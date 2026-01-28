# VIO Drone System

A modular Visual-Inertial Odometry (VIO) pipeline for autonomous drone navigation, built on ROS 2 with PX4 integration and Gazebo simulation.

The system estimates a drone's position, orientation, and velocity by fusing monocular camera images with IMU data using factor graph optimization, and feeds the result into PX4's EKF2 for flight control.

## Architecture Overview

```
Gazebo Simulation
  │
  ├── /camera/image_raw (30 Hz) ──────→ FRONTEND
  ├── /camera/camera_info (once) ─────→ FRONTEND
  ├── /imu/data (200 Hz) ─────────────→ BACKEND (imu_preintegrator)
  └── /ground_truth/pose (30 Hz) ─────→ EVAL
                                          ↑
FRONTEND                                  │
  │ /vio/tracked_features (30 Hz)         │
  ↓                                       │
BACKEND                                   │
  │ /vio/odometry (30 Hz) ────────────→ EVAL
  │                         └─────────→ BRIDGE
  ↓                                       │
                                          │ /fmu/in/vehicle_visual_odometry
                                          ↓
                                     PX4 EKF2
```

## Package Descriptions

| Package | Description |
|---------|-------------|
| `vio_interfaces` | Custom ROS 2 messages (`TrackedFeatures`, `VioState`) |
| `vio_frontend` | Shi-Tomasi corner detection, Lucas-Kanade optical flow tracking, forward-backward consistency check, RANSAC outlier rejection |
| `vio_backend` | GTSAM iSAM2 sliding-window factor graph fusing IMU preintegration with smart visual projection factors |
| `vio_bridge` | ENU → NED coordinate frame conversion for PX4 via uXRCE-DDS |
| `vio_eval` | Absolute Trajectory Error (ATE) computation against Gazebo ground truth with CSV logging |
| `vio_sim` | Gazebo models, worlds, and launch files |

## How the Components Relate

### 1. Custom Messages (`vio_interfaces`)

Defines the data contracts between nodes:

- **`TrackedFeatures.msg`** — Carries per-frame feature data from frontend to backend: pixel coordinates (`u_cur`, `v_cur`, `u_prev`, `v_prev`), unique `feature_ids`, and `track_lengths`. This is the only interface between frontend and backend.
- **`VioState.msg`** — A richer state representation (pose, twist, biases, covariance). Defined for future use.

### 2. Frontend (`vio_frontend`)

**File:** `vio_frontend/vio_frontend/frontend_node.py`

**Inputs:**
- `/camera/image_raw` — 30 Hz grayscale camera frames from Gazebo
- `/camera/camera_info` — Camera intrinsics (K matrix), consumed once

**Outputs:**
- `/vio/tracked_features` — Feature observations sent to backend
- `/vio/debug/image` — Visualization with drawn feature tracks

**Processing pipeline each frame:**
1. Detect Shi-Tomasi corners on first frame
2. Track existing features using Lucas-Kanade optical flow (forward pass)
3. Run backward pass and reject features with high round-trip error (forward-backward check)
4. Run RANSAC via essential matrix to reject geometric outliers
5. If features drop below threshold, detect new corners avoiding existing feature regions
6. Publish all surviving features with their IDs and track lengths

The frontend knows nothing about 3D geometry or IMU — it only produces 2D pixel correspondences.

### 3. Backend (`vio_backend`)

**Main file:** `vio_backend/vio_backend/backend_node.py`

**Sub-modules:**
- `initializer.py` — Static initialization from stationary IMU data
- `imu_preintegrator.py` — Buffers and preintegrates IMU measurements between keyframes
- `state_estimator.py` — GTSAM iSAM2 factor graph optimization

**Inputs:**
- `/imu/data` — 200 Hz IMU (accel + gyro) from Gazebo
- `/vio/tracked_features` — 30 Hz features from frontend

**Output:**
- `/vio/odometry` — Pose + velocity in ENU frame

**Two-phase operation:**

**Phase 1 — Initialization:**
- Buffers 400 IMU samples (~2 seconds at 200 Hz)
- Checks stationarity (low variance in accel/gyro)
- Computes initial orientation by aligning gravity vectors
- Estimates gyro and accelerometer biases
- Seeds the factor graph and preintegrator with initial state

**Phase 2 — Tracking (after init):**
1. Preintegrate IMU measurements since last keyframe
2. Convert pixel observations to normalized camera coordinates
3. Add IMU factor and visual factors to the factor graph
4. Run iSAM2 incremental optimization
5. Extract optimized pose, velocity, and bias
6. Update preintegrator with new bias estimate (feedback loop)
7. Publish odometry
8. Prune old feature tracks outside sliding window

### 4. Bridge (`vio_bridge`)

**File:** `vio_bridge/vio_bridge/bridge_node.py`

**Input:** `/vio/odometry` (ENU frame)
**Output:** `/fmu/in/vehicle_visual_odometry` (NED frame for PX4)

Pure coordinate transform node:
- Position: `(x,y,z)_ENU → (y,x,-z)_NED`
- Quaternion: reorders and applies frame rotation
- Attaches variance values for PX4's EKF2 weighting

### 5. Evaluation (`vio_eval`)

**File:** `vio_eval/vio_eval/eval_node.py`

**Inputs:**
- `/vio/odometry` — Estimated trajectory
- `/ground_truth/pose` — True pose from Gazebo

**Outputs:**
- `/vio/eval/position_error` — Real-time scalar error
- CSV files in `/tmp/vio_eval/` for offline analysis

Passive observer that compares VIO estimates to ground truth. No effect on the pipeline.

### 6. Simulation (`vio_sim`)

**Key files:**
- `models/x500_vio/model.sdf` — Gazebo drone model with camera and IMU
- `config/camera_params.yaml` — Camera intrinsics and extrinsics
- `worlds/default.sdf` — Gazebo world with ground plane and visual landmarks
- `launch/full_system_launch.py` — Orchestrates all nodes

---

## Installation

### Option A: Docker (Recommended)

Docker provides a consistent environment with all dependencies pre-installed.

**Prerequisites:**
- [Docker Desktop](https://www.docker.com/products/docker-desktop/)
- For GPU simulation: Linux host with NVIDIA GPU + [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

**Build the images:**

```bash
git clone https://github.com/ChrisGuarino/vio_drone.git
cd vio_drone

# Build VIO image only (works on any OS)
docker compose build vio

# Build both VIO + PX4/Gazebo images (Linux + NVIDIA GPU)
docker compose build
```

### Option B: Native Installation

**Prerequisites:**
- ROS 2 (Humble or later)
- PX4-Autopilot (with Gazebo SITL support)
- Gazebo (Garden or later)
- Micro XRCE-DDS Agent
- Python 3.10+

**Python dependencies:**
```
numpy>=1.24,<2
opencv-python>=4.8
scipy>=1.10
gtsam>=4.2
transforms3d
matplotlib
```

**ROS 2 dependencies:**
- `px4_msgs`
- `ros_gz_bridge`, `ros_gz_image`, `ros_gz_sim`
- `cv_bridge`
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`

**Build:**
```bash
cd <workspace_root>
pip install -r src/vio_drone/requirements.txt
colcon build
source install/setup.bash
```

---

## Running the System

### Docker with GPU (Full Pipeline)

Requires a Linux host with an NVIDIA GPU and `nvidia-container-toolkit` installed.

```bash
# Allow Docker to access the display (for Gazebo GUI)
xhost +local:docker

# Terminal 1 — Start PX4 SITL + Gazebo (GPU-accelerated)
docker compose up px4_gz

# Terminal 2 — Start VIO pipeline
docker compose up -d vio
docker compose exec vio bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch vio_sim full_system_launch.py
```

**What happens:**
1. `px4_gz` container starts Gazebo with the X500 drone, publishing camera + IMU topics
2. `vio` container picks up those topics via shared host network (DDS)
3. The VIO pipeline processes the data and sends odometry back to PX4's EKF2

### Docker without GPU (VIO Only)

Works on any OS. Useful for development, testing node startup, and verifying builds. Nodes will start but idle without sensor data.

```bash
# Start VIO container
docker compose up -d vio

# Open a shell
docker compose exec vio bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Launch VIO nodes (will wait for sensor data)
ros2 launch vio_sim full_system_launch.py

# Or run individual nodes for testing
ros2 run vio_frontend frontend_node
ros2 run vio_backend backend_node
```

### Native (No Docker)

```bash
# Terminal 1 — PX4 SITL + Gazebo
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2 — VIO System
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch vio_sim full_system_launch.py
```

### Docker Cheat Sheet

| Command | What it does |
|---------|-------------|
| `docker compose build` | Build all images |
| `docker compose build vio` | Build VIO image only |
| `docker compose up -d` | Start all containers in background |
| `docker compose up px4_gz` | Start PX4 + Gazebo (foreground) |
| `docker compose exec vio bash` | Open shell in VIO container |
| `docker compose down` | Stop and remove all containers |
| `docker compose logs -f` | Follow container logs |
| `docker ps` | List running containers |

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `max_features` | 150 | Maximum features to track |
| `min_features` | 80 | Minimum features before re-detection |
| `window_size` | 10 | Sliding window keyframe count |
| `eval_output_dir` | `/tmp/vio_eval` | Directory for evaluation CSV output |

Example with custom parameters:

```bash
ros2 launch vio_sim full_system_launch.py max_features:=200 window_size:=15
```

---

## ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Grayscale camera feed (30 Hz) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/imu/data` | `sensor_msgs/Imu` | IMU measurements (200 Hz) |
| `/vio/tracked_features` | `vio_interfaces/TrackedFeatures` | Tracked feature observations |
| `/vio/odometry` | `nav_msgs/Odometry` | VIO pose and velocity estimate |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | Odometry sent to PX4 (NED frame) |
| `/vio/eval/position_error` | `std_msgs/Float64` | Real-time position error |
| `/vio/debug/image` | `sensor_msgs/Image` | Debug visualization with feature tracks |
| `/ground_truth/pose` | `geometry_msgs/PoseStamped` | Gazebo ground truth |

---

## Project Structure

```
vio_drone/
├── vio_interfaces/               # Custom ROS 2 message definitions
│   └── msg/
│       ├── TrackedFeatures.msg
│       └── VioState.msg
├── vio_frontend/                 # Visual feature tracking
│   └── vio_frontend/
│       └── frontend_node.py
├── vio_backend/                  # State estimation
│   └── vio_backend/
│       ├── backend_node.py           # Main orchestrator
│       ├── initializer.py            # Static initialization
│       ├── imu_preintegrator.py      # IMU buffering & preintegration
│       └── state_estimator.py        # GTSAM factor graph
├── vio_bridge/                   # ROS 2 ↔ PX4 coordinate conversion
│   └── vio_bridge/
│       └── bridge_node.py
├── vio_eval/                     # Trajectory evaluation
│   └── vio_eval/
│       └── eval_node.py
├── vio_sim/                      # Simulation environment
│   ├── launch/
│   │   ├── full_system_launch.py
│   │   └── sim_launch.py
│   ├── models/x500_vio/
│   ├── config/
│   └── worlds/
├── Dockerfile                    # VIO image (ROS 2 + GTSAM + OpenCV)
├── Dockerfile.px4                # PX4 + Gazebo image (NVIDIA GPU)
├── docker-compose.yml            # Multi-container orchestration
├── docker-entrypoint.sh          # VIO container startup script
├── docker-entrypoint-px4.sh      # PX4 container startup script
├── .dockerignore                 # Files excluded from Docker build
├── requirements.txt              # Python dependencies
└── README.md
```

---

## License

MIT
