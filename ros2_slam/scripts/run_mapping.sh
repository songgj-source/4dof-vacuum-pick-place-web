#!/usr/bin/env bash
set -euo pipefail

# Start the sensor side of mapping (depth camera + depth->laserscan) and
# slam_toolbox. The car2 driver (/cmd_vel -> car, /odom + odom->base_link TF)
# is NOT started here -- run ./scripts/run_car2_driver.sh in another terminal.
# Drive the robot around with teleop, then run ./scripts/save_map.sh <name>
# once the map looks complete.
#
# Usage: ./scripts/run_mapping.sh [launch args...]   e.g. use_rviz:=false

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

echo "In other terminals (same ROS_DOMAIN_ID=${ROS_DOMAIN_ID}!):"
echo "  1) car2 driver : ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ${SCRIPT_DIR}/run_car2_driver.sh [serial_port]"
echo "  2) teleop      : ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ${SCRIPT_DIR}/run_teleop.sh"
echo ""

exec ros2 launch car2_bringup mapping.launch.py use_car2_driver:=false "$@"
