#!/usr/bin/env bash
set -euo pipefail

# LAPTOP side: navigate in a 2D lidar map saved by save_lidar_map.sh
# (map_server + AMCL on /scan + Nav2 -> /cmd_vel). Opens Nav2's RViz.
# Robot side first: ./scripts/remote_car2_lidar.sh start
#
# Usage: ./scripts/run_lidar_navigation.sh <maps/name.yaml> [launch args...]
# The initial pose defaults to the map origin (= where mapping started); else pass
# initial_x:=.. initial_y:=.. initial_yaw:=.. or fix it in RViz with "2D Pose Estimate".
# Then "Nav2 Goal" in RViz.

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

[[ $# -ge 1 ]] || { echo "Usage: $0 <maps/name.yaml> [launch args...]"; ls "${WS_DIR}"/maps/*.yaml 2>/dev/null; exit 1; }
MAP="$(realpath "$1")"; shift
[[ -f "$MAP" ]] || { echo "Map not found: $MAP"; exit 1; }

if pgrep -f "async_slam_toolbox_node|lib/rtabmap_slam/rtabmap|nav2_amcl/amcl" >/dev/null; then
  echo "ERROR: a SLAM / localization node is already running on this machine:"
  pgrep -af "async_slam_toolbox_node|lib/rtabmap_slam/rtabmap|nav2_amcl/amcl" | grep -v pgrep | cut -c1-120
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_DIR}/install/setup.bash"
set -u

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} (robot must match), map: ${MAP}"
if ! timeout 4 ros2 topic list 2>/dev/null | grep -qx /scan; then
  echo "WARNING: no /scan yet -- is the robot side running?  ./scripts/remote_car2_lidar.sh start"
fi
echo "In RViz: (1) 2D Pose Estimate  (2) Nav2 Goal.   Stop: Ctrl-C here or the camera web UI's ■ button."
echo ""
exec ros2 launch car2_bringup lidar_navigation.launch.py map:="${MAP}" "$@"
