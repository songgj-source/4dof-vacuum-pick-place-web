#!/usr/bin/env bash
set -euo pipefail

# Build the ros2_slam workspace (car2_driver + car2_bringup).
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

cd "$WS_DIR"
colcon build --symlink-install
echo ""
echo "Build done. Source it with:"
echo "  source ${WS_DIR}/install/setup.bash"
