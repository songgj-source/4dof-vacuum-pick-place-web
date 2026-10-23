#!/usr/bin/env bash
set -euo pipefail

# ROBOT side (Raspberry Pi): car2 4-wheel driver + LD-series 2D lidar (/scan).
# From the laptop use ./scripts/remote_car2_lidar.sh instead.
#
# Usage: ./scripts/run_car2_lidar.sh [launch args...]
#   e.g. ./scripts/run_car2_lidar.sh lidar_z:=0.25 lidar_yaw:=3.1416
#        ./scripts/run_car2_lidar.sh use_car2_driver:=false     # lidar only
#        ./scripts/run_car2_lidar.sh use_camera:=true           # + depth/RGB camera

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# Avoid colliding with other ROS2 traffic on the default domain (0).
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_DIR}/install/setup.bash"
set -u

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
exec ros2 launch car2_bringup robot_lidar.launch.py "$@"
