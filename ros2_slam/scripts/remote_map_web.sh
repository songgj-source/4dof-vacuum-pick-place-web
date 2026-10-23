#!/usr/bin/env bash
set -euo pipefail

# Run test/map_web.py (map web UI: map + robot position/heading, lidar scan,
# trail, Nav2 path, drive pad) on the robot (Raspberry Pi) from the laptop.
# The laptop's map_web.py + drive_control.py are copied to
# ~/ros2_slam_robot/test/ on every start, so the robot runs the current code.
# Runs in tmux session 'map_web' (log ~/map_web.log).
#
# Usage: ./scripts/remote_map_web.sh <command> [ROS args...]
#   start   [args]   start and print the URL
#   stop             stop
#   restart [args]   stop + start
#   status           running?, URL, pose / map / lidar
#   log              follow the log (Ctrl-C to quit)
#   attach           attach to the tmux session (Ctrl-b d to detach)
#
# Examples:
#   ./scripts/remote_map_web.sh start
#   ./scripts/remote_map_web.sh start -p port:=8082 -p max_linear:=0.1
#
# Environment: ROBOT, REMOTE_WS, ROS_DOMAIN_ID (see remote_common.sh).
#
# Needs the robot driver (remote_car2_lidar.sh / remote_robot_rgbd.sh) for the
# pose and driving; the map and map frame come from the laptop's SLAM or
# navigation (without them the page shows the odom frame).

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=remote_common.sh
source "${SCRIPT_DIR}/remote_common.sh"

SESSION=map_web
REMOTE_LOG='~/map_web.log'
REMOTE_DIR="${REMOTE_WS}/test"
SRC_DIR="$(cd "${SCRIPT_DIR}/../test" && pwd)"
ROBOT_HOST="${ROBOT#*@}"
PROC_PAT="$PAT_MAP_WEB"

CMD="${1:-}"
shift || true

web_port() {   # -p port:=N from the args, else map_web.py's default
  local a
  for a in "$@"; do
    [[ "$a" == port:=* ]] && { echo "${a#port:=}"; return; }
  done
  echo 8081
}

print_stats() {   # $1 = port
  local st
  if st="$(curl -s -m 3 "http://${ROBOT_HOST}:$1/api/state")" && [[ -n "$st" ]]; then
    echo "web UI : http://${ROBOT_HOST}:$1/"
    python3 - "$st" <<'PY'
import json, math, sys
s = json.loads(sys.argv[1])
p, m = s.get('pose'), s.get('map')
print("pose   : " + (f"{s['frame']} x {p['x']:.2f} y {p['y']:.2f} yaw {math.degrees(p['yaw']):.1f} deg" if p else
                     "no TF odom->base_link  <- robot driver not running? (./scripts/remote_car2_lidar.sh start)"))
print("map    : " + (f"{m['width']}x{m['height']} @ {m['resolution']:.3f} m" if m else
                     "none (start lidar mapping / navigation on the laptop)"))
print(f"lidar  : {s.get('scan_hz', 0):.1f} Hz")
PY
  else
    echo "web UI : not answering on http://${ROBOT_HOST}:$1/"
  fi
}

do_status() {
  remote "bash -s" <<EOF
if tmux has-session -t $SESSION 2>/dev/null; then echo "tmux '$SESSION': running"; else echo "tmux '$SESSION': not running"; fi
if [ -f $REMOTE_LOG ]; then echo "--- $REMOTE_LOG (last lines)"; tail -n 4 $REMOTE_LOG | cut -c1-160; fi
EOF
  local port
  port="$(remote "pgrep -af '$PROC_PAT' | grep -o 'port:=[0-9]*' | head -1 | cut -d= -f2" || true)"
  print_stats "${port:-8081}"
}

do_stop() {
  remote "bash -s" <<EOF
pids=\$(pgrep -f '$PROC_PAT' || true)
if [ -z "\$pids" ] && ! tmux has-session -t $SESSION 2>/dev/null; then echo "not running"; exit 0; fi
tmux has-session -t $SESSION 2>/dev/null && tmux send-keys -t $SESSION C-c
for i in \$(seq 1 20); do pgrep -f '$PROC_PAT' >/dev/null || break; sleep 0.5; done
pids=\$(pgrep -f '$PROC_PAT' || true)
[ -n "\$pids" ] && kill -TERM \$pids 2>/dev/null || true
tmux kill-session -t $SESSION 2>/dev/null || true
echo "stopped"
EOF
}

do_start() {
  local args="" a port
  port="$(web_port "$@")"
  [[ $# -gt 0 ]] && args=" --ros-args"
  for a in "$@"; do args+=" $(printf '%q' "$a")"; done

  remote "bash -s" <<EOF
if tmux has-session -t $SESSION 2>/dev/null || pgrep -f '$PROC_PAT' >/dev/null; then
  echo "already running. Use: restart / stop"; exit 1
fi
mkdir -p ~/$REMOTE_DIR
EOF
  # Always run the laptop's current code.
  rsync -az -e "ssh ${SSH_OPTS[*]}" "${SRC_DIR}/map_web.py" "${SRC_DIR}/drive_control.py" "${ROBOT}:~/${REMOTE_DIR}/"

  remote "bash -s" <<EOF
rm -f $REMOTE_LOG
tmux new-session -d -s $SESSION "$REMOTE_ROS_ENV; cd ~/$REMOTE_DIR && python3 -u map_web.py$args 2>&1 | tee $REMOTE_LOG"
for i in \$(seq 1 30); do
  curl -s -m 1 http://localhost:$port/api/state >/dev/null 2>&1 && { echo "started (tmux '$SESSION', ROS_DOMAIN_ID=$DOMAIN)"; exit 0; }
  tmux has-session -t $SESSION 2>/dev/null || break
  sleep 0.5
done
echo "ERROR: map_web.py did not come up. Log:"; tail -n 15 $REMOTE_LOG; exit 1
EOF
  sleep 2   # let TF / map arrive before reporting
  print_stats "$port"
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
