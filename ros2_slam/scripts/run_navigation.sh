#!/usr/bin/env bash
set -euo pipefail

# Navigate a previously saved map: bringup (car2 driver + camera + depth->scan)
# + Nav2 (map_server + AMCL + planner/controller).
#
# Usage: ./scripts/run_navigation.sh maps/my_map.yaml [serial_port]

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <map.yaml> [serial_port]"
  exit 1
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
MAP_YAML="$1"
SERIAL_PORT="${2:-/dev/ttyUSB0}"

if [[ ! -f "$MAP_YAML" ]]; then
  echo "Map file not found: $MAP_YAML"
  exit 1
fi

# Avoid colliding with other ROS2 traffic on the default domain (0) or any
# domain left with stale/leftover participants from earlier sessions.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_DIR}/install/setup.bash"
set -u

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "In RViz:"
echo "  1) '2D Pose Estimate' -> click+drag where the robot actually is/faces."
echo "  2) 'Nav2 Goal' -> click+drag at the destination."
echo ""

exec ros2 launch car2_bringup navigation.launch.py \
  map:="${MAP_YAML}" serial_port:="${SERIAL_PORT}"
