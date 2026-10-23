#!/usr/bin/env bash
set -euo pipefail

# ROBOT side (Raspberry Pi): car2 driver + Astra Pro depth (OpenNI2) + RGB
# (usb_cam) + camera TFs, for remote RTAB-Map mapping on the laptop
# (./scripts/run_rtabmap_mapping.sh). Must use the same ROS_DOMAIN_ID as the laptop.
#
# Usage: ./scripts/run_robot_rgbd.sh [launch args...]
#   e.g. ./scripts/run_robot_rgbd.sh video_device:=/dev/video1
#        ./scripts/run_robot_rgbd.sh use_car2_driver:=false

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# Avoid colliding with other ROS2 traffic on the default domain (0) or any
# domain left with stale/leftover participants from earlier sessions.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_DIR}/install/setup.bash"
set -u

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} (laptop must match)"
echo "Astra Pro RGB UVC device candidates:"
v4l2-ctl --list-devices 2>/dev/null | grep -A2 -i -E "astra|orbbec|usb 2.0 camera" || echo "  (none found -- is the camera plugged in?)"
echo ""

exec ros2 launch car2_bringup robot_rgbd.launch.py "$@"
