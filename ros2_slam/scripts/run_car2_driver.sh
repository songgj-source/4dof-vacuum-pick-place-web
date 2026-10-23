#!/usr/bin/env bash
set -euo pipefail

# Start only the car2 serial driver: /cmd_vel -> car, and /odom +
# odom->base_link TF. Pair with ./scripts/run_mapping.sh (which no longer
# starts the driver). Must use the same ROS_DOMAIN_ID as run_mapping.sh.
#
# Usage: ./scripts/run_car2_driver.sh [serial_port] [extra ROS args...]
#   e.g. ./scripts/run_car2_driver.sh /dev/ttyS0 -p wheel_radius_m:=0.033

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
SERIAL_PORT="${1:-/dev/ttyS0}"
shift || true

# Avoid colliding with other ROS2 traffic on the default domain (0) or any
# domain left with stale/leftover participants from earlier sessions.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_DIR}/install/setup.bash"
set -u

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, car2 driver: serial ${SERIAL_PORT}"

exec ros2 run car2_driver car2_serial_node --ros-args \
  -p serial_port:="${SERIAL_PORT}" "$@"
