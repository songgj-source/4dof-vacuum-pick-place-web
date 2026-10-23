#!/usr/bin/env bash
set -euo pipefail

# Run the car2 serial driver on the robot (Raspberry Pi) from the laptop.
# The driver runs in a tmux session 'car2_driver' on the Pi, so it keeps
# running after this script / the ssh connection ends.
#
# Usage: ./scripts/remote_car2_driver.sh <command> [serial_port] [extra ROS args...]
#   start   [port] [args]  start the driver (default port /dev/ttyS0)
#   stop                   stop it (SIGINT -> the driver sends STOP to the car)
#   restart [port] [args]  stop + start
#   status                 is it running, last log lines
#   log                    follow the driver log (Ctrl-C to quit)
#   attach                 attach to the tmux session (Ctrl-b d to detach)
#
# Examples:
#   ./scripts/remote_car2_driver.sh start
#   ./scripts/remote_car2_driver.sh start /dev/ttyS0 -p odom_poll_every_n:=2   # 10Hz odom for RTAB-Map
#
# Environment: ROBOT, REMOTE_WS, ROS_DOMAIN_ID (see remote_common.sh).
#
# Refuses to start while another car2_serial_node holds the robot -- e.g. the
# one inside run_robot_rgbd.sh -- because two drivers on /dev/ttyS0 corrupt
# each other's replies (/odom drops out).

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=remote_common.sh
source "${SCRIPT_DIR}/remote_common.sh"

SESSION=car2_driver
REMOTE_LOG='~/car2_driver.log'

CMD="${1:-}"
shift || true

do_status() {
  remote "bash -s" <<EOF
if tmux has-session -t $SESSION 2>/dev/null; then echo "tmux '$SESSION': running"; else echo "tmux '$SESSION': not running"; fi
pids=\$(pgrep -f 'car2_driver/car2_serial_node' || true)
if [ -n "\$pids" ]; then
  echo "car2_serial_node pid(s): \$pids"
  pgrep -af 'ros2 launch car2_bringup robot_rgbd.launch.py' >/dev/null && echo "  (note: robot_rgbd.launch.py is running -- it includes a driver)"
else
  echo "car2_serial_node: none"
fi
if [ -f $REMOTE_LOG ]; then echo "--- $REMOTE_LOG (last lines)"; tail -n 5 $REMOTE_LOG | cut -c1-160; fi
EOF
}

do_stop() {
  remote "bash -s" <<EOF
if ! tmux has-session -t $SESSION 2>/dev/null; then echo "not running"; exit 0; fi
tmux send-keys -t $SESSION C-c
for i in \$(seq 1 20); do
  pgrep -f 'car2_driver/car2_serial_node' >/dev/null || break
  sleep 0.5
done
tmux kill-session -t $SESSION 2>/dev/null || true
echo "stopped"
EOF
}

do_start() {
  local port="${1:-/dev/ttyS0}"
  shift || true
  local extra=""
  if [[ $# -gt 0 ]]; then extra="$(printf ' %q' "$@")"; fi
  remote "bash -s" <<EOF
set -e
if tmux has-session -t $SESSION 2>/dev/null; then
  echo "already running (tmux '$SESSION'). Use: restart / stop"; exit 1
fi
if pgrep -af '$PAT_CAR2_WEB' | grep -qv -- '--dry-run'; then
  echo "ERROR: car2_web.py is using the serial port -- stop it first: ./scripts/remote_car2_web.sh stop"; exit 1
fi
if pgrep -f 'car2_driver/car2_serial_node' >/dev/null; then
  echo "ERROR: another car2_serial_node is already running on the robot:"
  pgrep -af 'car2_driver/car2_serial_node' | cut -c1-120
  pgrep -af 'ros2 launch car2_bringup robot_rgbd.launch.py' >/dev/null && \
    echo "  -> it belongs to robot_rgbd.launch.py (driver + camera). Stop that first, or run it with use_car2_driver:=false."
  exit 1
fi
if [ ! -x ~/$REMOTE_WS/scripts/run_car2_driver.sh ]; then
  echo "ERROR: ~/$REMOTE_WS/scripts/run_car2_driver.sh not found -- run ./scripts/deploy_to_robot.sh first"; exit 1
fi
rm -f $REMOTE_LOG   # an old log's 'ready on' line must not count as this start
tmux new-session -d -s $SESSION "$REMOTE_ROS_ENV; cd ~/$REMOTE_WS && ./scripts/run_car2_driver.sh $(printf '%q' "$port")$extra 2>&1 | tee $REMOTE_LOG"
for i in \$(seq 1 30); do
  grep -q 'ready on' $REMOTE_LOG 2>/dev/null && { grep 'ready on' $REMOTE_LOG | tail -1 | cut -c1-170; echo "started (tmux '$SESSION', ROS_DOMAIN_ID=$DOMAIN)"; exit 0; }
  tmux has-session -t $SESSION 2>/dev/null || break
  sleep 0.5
done
echo "ERROR: driver did not report ready. Log:"; tail -n 15 $REMOTE_LOG; exit 1
EOF
}

case "$CMD" in
  start)   do_start "$@" ;;
  stop)    do_stop ;;
  restart) do_stop; do_start "$@" ;;
  status)  do_status ;;
  log)     remote_tty "tail -n 30 -f $REMOTE_LOG" ;;
  attach)  remote_tty "tmux attach -t $SESSION" ;;
  *)       usage; exit 1 ;;
esac
