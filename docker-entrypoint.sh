#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Execute the command passed to docker run
exec "$@"
