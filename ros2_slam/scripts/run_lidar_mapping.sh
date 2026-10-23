#!/usr/bin/env bash
set -euo pipefail

# LAPTOP side: 2D lidar mapping (slam_toolbox) from the robot's /scan + /odom.
# Robot side first: ./scripts/remote_car2_lidar.sh start
# Drive slowly (camera web UI / ./scripts/run_teleop.sh), watch the map grow in
# RViz, then save it with ./scripts/save_lidar_map.sh <name> (while this runs).
#
# Usage: ./scripts/run_lidar_mapping.sh [saved_map_name] [launch args...]
#   ./scripts/run_lidar_mapping.sh                 # new map
#   ./scripts/run_lidar_mapping.sh office          # continue maps/office.posegraph (start at its dock pose)
#   ./scripts/run_lidar_mapping.sh use_rviz:=false

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

MAP_ARG=()
if [[ $# -gt 0 && "$1" != *:=* ]]; then
  pg="${WS_DIR}/maps/${1%.posegraph}"
  [[ -f "${pg}.posegraph" ]] || { echo "No saved pose graph: ${pg}.posegraph"; exit 1; }
  MAP_ARG=(map_file:="${pg}")
  shift
fi

# One SLAM at a time (two would both publish map->odom).
if pgrep -f "async_slam_toolbox_node|lib/rtabmap_slam/rtabmap|lib/nav2_amcl/amcl|lib/nav2_map_server/map_server" >/dev/null; then
  echo "ERROR: a SLAM / localization node (slam_toolbox, rtabmap, AMCL navigation) is already running on this machine:"
  pgrep -af "async_slam_toolbox_node|lib/rtabmap_slam/rtabmap|lib/nav2_amcl/amcl|lib/nav2_map_server/map_server" | grep -v pgrep | cut -c1-120
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_DIR}/install/setup.bash"
set -u

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} (robot must match)"
if ! timeout 4 ros2 topic list 2>/dev/null | grep -qx /scan; then
  echo "WARNING: no /scan yet -- is the robot side running?  ./scripts/remote_car2_lidar.sh start"
fi
echo "Save the map (in another terminal, while this runs): ./scripts/save_lidar_map.sh <name>"
echo ""
exec ros2 launch car2_bringup lidar_mapping.launch.py "${MAP_ARG[@]}" "$@"
