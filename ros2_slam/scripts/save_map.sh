#!/usr/bin/env bash
set -euo pipefail

# Save the map currently being built by run_mapping.sh.
# Run this WHILE run_mapping.sh is still running, after driving the robot
# around enough to cover the whole space.
#
# Usage: ./scripts/save_map.sh [name]   (default name: my_map)

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
MAP_NAME="${1:-my_map}"

# Must match the ROS_DOMAIN_ID that run_mapping.sh is using (defaults the
# same way here so this works out of the box if you didn't override it).
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

mkdir -p "${WS_DIR}/maps"
OUT="${WS_DIR}/maps/${MAP_NAME}"

echo "Saving map to ${OUT}.yaml / ${OUT}.pgm ..."
ros2 run nav2_map_server map_saver_cli \
  -f "${OUT}" \
  --ros-args \
  -p save_map_timeout:=10.0 \
  -p map_subscribe_transient_local:=true

echo ""
echo "Done:"
ls -la "${OUT}.yaml" "${OUT}.pgm"
echo ""
echo "Navigate with this map:"
echo "  ./scripts/run_navigation.sh ${OUT}.yaml"
