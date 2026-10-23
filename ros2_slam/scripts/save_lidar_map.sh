#!/usr/bin/env bash
set -euo pipefail

# Save the map being built by run_lidar_mapping.sh (run while mapping is running):
#   maps/<name>.yaml + .pgm      -> navigation (./scripts/run_lidar_navigation.sh maps/<name>.yaml)
#   maps/<name>.posegraph + .data -> continue mapping later (./scripts/run_lidar_mapping.sh <name>)
#
# Usage: ./scripts/save_lidar_map.sh <name>

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
NAME="${1:-}"
[[ -n "$NAME" ]] || { echo "Usage: $0 <name>"; exit 1; }
OUT="${WS_DIR}/maps/${NAME}"
mkdir -p "${WS_DIR}/maps"
if [[ -e "${OUT}.yaml" || -e "${OUT}.posegraph" ]]; then
  echo "maps/${NAME}.* already exists -- pick another name (or delete it first)"; exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if ! ros2 service list 2>/dev/null | grep -qx /slam_toolbox/serialize_map; then
  echo "slam_toolbox is not running -- start ./scripts/run_lidar_mapping.sh first"; exit 1
fi

echo "[1/2] occupancy grid -> ${OUT}.yaml/.pgm"
ros2 run nav2_map_server map_saver_cli -f "${OUT}" --ros-args \
  -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
echo "[2/2] slam_toolbox pose graph -> ${OUT}.posegraph/.data"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${OUT}'}" \
  | grep -E "result" || true

echo ""
ls -la "${OUT}".* 2>/dev/null
echo ""
echo "Navigate:        ./scripts/run_lidar_navigation.sh maps/${NAME}.yaml"
echo "Continue mapping: ./scripts/run_lidar_mapping.sh ${NAME}"
