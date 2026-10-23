#!/usr/bin/env bash
set -euo pipefail

# Run car2/test/car2_web.py (web UI + REST API car control, no ROS) on the
# robot (Raspberry Pi) from the laptop. The laptop's car2_web.py + car2_test.py
# are copied to ~/ros2_slam_robot/car2_web/ on every start, so the robot always
# runs the current code. Runs in tmux session 'car2_web' (log ~/car2_web.log).
#
# Usage: ./scripts/remote_car2_web.sh <command> [car2_web.py args...]
#   start   [args]   start (default args: --port /dev/ttyS0) and print the URL
#   stop             stop (Ctrl-C: car2_web.py sends STOP to the car)
#   restart [args]   stop + start
#   status           running?, URL, /api/status
#   log              follow the log (Ctrl-C to quit)
#   attach           attach to the tmux session (Ctrl-b d to detach)
#
# Examples:
#   ./scripts/remote_car2_web.sh start
#   ./scripts/remote_car2_web.sh start --port /dev/ttyS0 --speed 1200
#   ./scripts/remote_car2_web.sh start --dry-run          # no serial, commands are only printed
#
# Environment: ROBOT, REMOTE_WS (see remote_common.sh).
#
# car2_web.py talks to /dev/ttyS0 itself, so it refuses to start while a ROS
# car2 driver (robot_rgbd / remote_car2_driver.sh) holds the port -- unless
# --dry-run is given.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=remote_common.sh
source "${SCRIPT_DIR}/remote_common.sh"

SESSION=car2_web
REMOTE_LOG='~/car2_web.log'
REMOTE_DIR="${REMOTE_WS}/car2_web"
SRC_DIR="$(cd "${SCRIPT_DIR}/../../car2/test" && pwd)"
ROBOT_HOST="${ROBOT#*@}"

CMD="${1:-}"
shift || true

http_port() {   # --http-port N from the args, else car2_web.py's default
  local prev=""
  for a in "$@"; do
    [[ "$prev" == --http-port ]] && { echo "$a"; return; }
    [[ "$a" == --http-port=* ]] && { echo "${a#*=}"; return; }
    prev="$a"
  done
  echo 8766
}

do_status() {
  remote "bash -s" <<EOF
if tmux has-session -t $SESSION 2>/dev/null; then echo "tmux '$SESSION': running"; else echo "tmux '$SESSION': not running"; fi
pgrep -af '$PAT_CAR2_WEB' | cut -c1-120 || true
if [ -f $REMOTE_LOG ]; then echo "--- $REMOTE_LOG (last lines)"; tail -n 5 $REMOTE_LOG | cut -c1-160; fi
EOF
  local port
  port="$(remote "pgrep -af '$PAT_CAR2_WEB' | grep -o -- '--http-port[ =][0-9]*' | grep -o '[0-9]*$' | head -1" || true)"
  port="${port:-8766}"
  if out="$(curl -s -m 3 "http://${ROBOT_HOST}:${port}/api/status")"; then
    echo "web UI: http://${ROBOT_HOST}:${port}/"
    echo "/api/status: ${out}"
  fi
}

do_stop() {
  remote "bash -s" <<EOF
pids=\$(pgrep -f '$PAT_CAR2_WEB' || true)
if [ -z "\$pids" ] && ! tmux has-session -t $SESSION 2>/dev/null; then echo "not running"; exit 0; fi
tmux has-session -t $SESSION 2>/dev/null && tmux send-keys -t $SESSION C-c
[ -n "\$pids" ] && kill -INT \$pids 2>/dev/null || true
for i in \$(seq 1 20); do pgrep -f '$PAT_CAR2_WEB' >/dev/null || break; sleep 0.5; done
pids=\$(pgrep -f '$PAT_CAR2_WEB' || true)
[ -n "\$pids" ] && kill -TERM \$pids 2>/dev/null || true
tmux kill-session -t $SESSION 2>/dev/null || true
echo "stopped"
EOF
}

do_start() {
  [[ $# -eq 0 ]] && set -- --port /dev/ttyS0
  local args="" dry=0 a port
  for a in "$@"; do
    args+=" $(printf '%q' "$a")"
    [[ "$a" == --dry-run ]] && dry=1
  done
  port="$(http_port "$@")"

  remote "bash -s" <<EOF
if tmux has-session -t $SESSION 2>/dev/null || pgrep -f '$PAT_CAR2_WEB' >/dev/null; then
  echo "already running. Use: restart / stop"; exit 1
fi
if [ $dry = 0 ] && pgrep -f 'car2_driver/car2_serial_node' >/dev/null; then
  echo "ERROR: a ROS car2 driver is using the serial port:"
  pgrep -af 'car2_driver/car2_serial_node' | cut -c1-110
  pgrep -f 'ros2 launch car2_bringup robot_rgbd.launch.py' >/dev/null && echo "  -> from robot_rgbd: ./scripts/remote_robot_rgbd.sh stop"
  tmux has-session -t car2_driver 2>/dev/null && echo "  -> from remote_car2_driver.sh: ./scripts/remote_car2_driver.sh stop"
  echo "  (or start with --dry-run to test the UI without the car)"
  exit 1
fi
mkdir -p ~/$REMOTE_DIR
EOF
  # Always run the laptop's current code.
  rsync -az -e "ssh ${SSH_OPTS[*]}" "${SRC_DIR}/car2_web.py" "${SRC_DIR}/car2_test.py" "${ROBOT}:~/${REMOTE_DIR}/"

  remote "bash -s" <<EOF
rm -f $REMOTE_LOG
tmux new-session -d -s $SESSION "cd ~/$REMOTE_DIR && python3 -u car2_web.py$args 2>&1 | tee $REMOTE_LOG"
for i in \$(seq 1 30); do
  curl -s -m 1 http://localhost:$port/api/status >/dev/null 2>&1 && { echo "started (tmux '$SESSION')"; exit 0; }
  tmux has-session -t $SESSION 2>/dev/null || break
  sleep 0.5
done
echo "ERROR: car2_web.py did not come up. Log:"; tail -n 15 $REMOTE_LOG; exit 1
EOF
  echo "web UI : http://${ROBOT_HOST}:${port}/"
  echo "status : $(curl -s -m 3 "http://${ROBOT_HOST}:${port}/api/status")"
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
