#!/bin/bash
set -e

# Set Gazebo model path to include custom models
export GZ_SIM_RESOURCE_PATH="/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}"

cd /PX4-Autopilot

exec "$@"
