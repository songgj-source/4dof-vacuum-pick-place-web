#!/usr/bin/env bash
set -euo pipefail

# Keyboard teleop: publishes geometry_msgs/Twist on /cmd_vel for the car2
# driver. Must use the same ROS_DOMAIN_ID as run_mapping.sh and
# run_car2_driver.sh.
#
# Usage: ./scripts/run_teleop.sh [extra ROS args...]
#   e.g. ./scripts/run_teleop.sh -p speed:=0.2 -p turn:=0.8

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

# Avoid colliding with other ROS2 traffic on the default domain (0) or any
# domain left with stale/leftover participants from earlier sessions.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, teleop -> /cmd_vel"

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args "$@"
