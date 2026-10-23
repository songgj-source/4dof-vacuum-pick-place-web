#!/usr/bin/env bash
set -euo pipefail

# LAPTOP side: RTAB-Map RGB-D mapping from the robot's compressed camera
# streams (robot runs ./scripts/run_robot_rgbd.sh). Opens RViz by default.
# Drive with ./scripts/run_teleop.sh; the RTAB-Map database is kept at
# maps/rtabmap.db, and ./scripts/save_map.sh <name> saves the 2D grid (/map).
#
# Usage: ./scripts/run_rtabmap_mapping.sh [launch args...]
#   e.g. ./scripts/run_rtabmap_mapping.sh new_map:=true           # start a fresh database
#        ./scripts/run_rtabmap_mapping.sh use_rtabmap_viz:=true   # RTAB-Map's own GUI too

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# Avoid colliding with other ROS2 traffic on the default domain (0) or any
# domain left with stale/leftover participants from earlier sessions.
# Only one rtabmap at a time: two instances on the same database lock it, the
# running one dies ("database is locked") and the database is left unclosed
# (no visual-word dictionary -> no relocalization; fix with rtabmap-recovery).
if pgrep -x rtabmap >/dev/null; then
  echo "ERROR: rtabmap is already running on this machine:"
  pgrep -ax rtabmap | cut -c1-150
  echo "Stop that mapping/navigation first (Ctrl-C in its terminal, wait until it exits)."
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_DIR}/install/setup.bash"
set -u

mkdir -p "${WS_DIR}/maps"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} (robot must match)"
echo "In other terminals:"
echo "  robot  : ssh to the Pi, then ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ./scripts/run_robot_rgbd.sh"
echo "  teleop : ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ${SCRIPT_DIR}/run_teleop.sh"
echo ""

exec ros2 launch car2_bringup rtabmap_mapping.launch.py \
  database_path:="${WS_DIR}/maps/rtabmap.db" "$@"
