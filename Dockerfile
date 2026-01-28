# VIO Drone System - Docker Image
# Base: ROS 2 Humble + Gazebo Harmonic on Ubuntu 22.04

FROM ros:humble-ros-base-jammy

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    # Python
    python3-pip \
    python3-venv \
    # ROS 2 packages
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-opencv \
    # Gazebo Harmonic
    ros-humble-ros-gz \
    # OpenCV
    libopencv-dev \
    python3-opencv \
    # GTSAM dependencies
    libboost-all-dev \
    libtbb-dev \
    # Visualization (optional, for rviz)
    ros-humble-rviz2 \
    # Micro XRCE-DDS Agent
    && rm -rf /var/lib/apt/lists/*

# Install Micro XRCE-DDS Agent
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent \
    && cd /tmp/agent \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && rm -rf /tmp/agent

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    "numpy>=1.24,<2" \
    opencv-python>=4.8 \
    scipy>=1.10 \
    gtsam>=4.2 \
    transforms3d \
    matplotlib

# Create workspace
WORKDIR /ros2_ws

# Clone px4_msgs
RUN mkdir -p src \
    && cd src \
    && git clone https://github.com/PX4/px4_msgs.git

# Copy the vio_drone packages (will be overridden by volume mount in dev)
COPY . /ros2_ws/src/vio_drone/

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

# Source ROS 2 in every bash session (needed for docker exec)
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Setup entrypoint
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
