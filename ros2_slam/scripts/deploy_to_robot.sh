#!/usr/bin/env bash
set -euo pipefail

# Copy this workspace's packages + scripts to the robot (Raspberry Pi) and
# build them there. Deploys into a separate directory (not the Pi's git clone)
# so the Pi's ~/dapier_project checkout stays clean for git pull.
#
# Usage: ./scripts/deploy_to_robot.sh [user@host] [remote_ws_dir]
#   defaults: user@192.168.0.31  ~/ros2_slam_robot
# Needs ssh access (you'll be asked for the password unless you have a key).
# The first deploy also installs Orbbec's OpenNI2 runtime on the robot (sudo).

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
ROBOT="${1:-${ROBOT:-user@192.168.0.31}}"
REMOTE_WS="${2:-ros2_slam_robot}"

# One ssh connection shared by rsync + the build (one password prompt).
CTL="$(mktemp -u /tmp/deploy_robot_ssh.XXXXXX)"
SSH_OPTS=(-o ControlMaster=auto -o ControlPath="$CTL" -o ControlPersist=60)
trap 'ssh "${SSH_OPTS[@]}" -O exit "$ROBOT" 2>/dev/null || true' EXIT

echo "[1/3] Syncing src/, scripts/ and test/ to ${ROBOT}:~/${REMOTE_WS} ..."
ssh "${SSH_OPTS[@]}" "$ROBOT" "mkdir -p ~/${REMOTE_WS}"
rsync -az --delete -e "ssh ${SSH_OPTS[*]}" \
  --exclude '__pycache__' \
  "${WS_DIR}/src" "${WS_DIR}/scripts" "${WS_DIR}/test" "${ROBOT}:~/${REMOTE_WS}/"

echo "[2/3] Building on the robot ..."
ssh "${SSH_OPTS[@]}" "$ROBOT" "bash -lc 'cd ~/${REMOTE_WS} && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install 2>&1 | tail -3'"

echo "[3/3] Orbbec OpenNI2 runtime on the robot ..."
if ssh "${SSH_OPTS[@]}" "$ROBOT" "test -f ~/${REMOTE_WS}/third_party/orbbec_openni2/libOpenNI2.so.0"; then
  echo "  already installed"
else
  ssh -t "${SSH_OPTS[@]}" "$ROBOT" "cd ~/${REMOTE_WS} && ./scripts/install_orbbec_openni2.sh"
fi

echo ""
echo "Done. On the robot:"
echo "  ssh ${ROBOT}"
echo "  cd ~/${REMOTE_WS} && ./scripts/run_robot_rgbd.sh"
