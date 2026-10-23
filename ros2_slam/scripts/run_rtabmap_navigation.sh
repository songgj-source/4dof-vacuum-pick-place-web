#!/usr/bin/env bash
set -euo pipefail

# LAPTOP side: navigate autonomously in a map saved by run_rtabmap_mapping.sh.
# rtabmap localizes in the saved database (read-only: nothing is added), Nav2
# plans and drives (/cmd_vel -> car2 driver on the robot). Opens Nav2's RViz.
#
# Usage: ./scripts/run_rtabmap_navigation.sh [database.db] [launch args...]
#   default database: maps/rtabmap.db
#   e.g. ./scripts/run_rtabmap_navigation.sh maps/office.db
#        ./scripts/run_rtabmap_navigation.sh maps/rtabmap.db use_rtabmap_viz:=true
#        ./scripts/run_rtabmap_navigation.sh maps/rtabmap.db body_roi_ratios:="0 0 0 0.25"
#
# Robot side first: ./scripts/remote_robot_rgbd.sh start
# In RViz:
#   1) robot pose: rtabmap relocalizes by itself once the camera sees a mapped
#      place (start near where mapping started). Otherwise use "2D Pose Estimate".
#   2) "Nav2 Goal": click + drag at the destination.

if [[ -z "${ROS_DISTRO:-}" ]]; then
  export ROS_DISTRO=jazzy
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

DB="${WS_DIR}/maps/rtabmap.db"
if [[ $# -gt 0 && "$1" != *:=* ]]; then
  DB="$(realpath "$1")"
  shift
fi
if [[ ! -f "$DB" ]]; then
  echo "Map database not found: $DB"
  echo "Make one with ./scripts/run_rtabmap_mapping.sh first."
  exit 1
fi

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

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} (robot must match)"
echo "map database: ${DB} (localization only -- not modified)"
if ! timeout 4 ros2 topic list 2>/dev/null | grep -qx /odom; then
  echo ""
  echo "WARNING: no /odom yet -- is the robot side running?  ./scripts/remote_robot_rgbd.sh start"
fi
echo ""
echo "In RViz: (1) check/fix the robot pose (2D Pose Estimate)  (2) Nav2 Goal"
echo "Stop the robot any time: Ctrl-C here, or the camera web UI's ■ button."
echo ""

exec ros2 launch car2_bringup rtabmap_navigation.launch.py database_path:="${DB}" "$@"
