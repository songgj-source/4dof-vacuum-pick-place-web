#!/usr/bin/env bash
set -euo pipefail

# Run test/camera_test.py (camera web UI: RGB/depth view, camera tilt,
# RoiRatios editor, robot_rgbd restart button) on the robot (Raspberry Pi)
# from the laptop. The laptop's camera_test.py + camera_tilt.py are copied to
# ~/ros2_slam_robot/test/ on every start, so the robot runs the current code.
# Runs in tmux session 'camera_web' (log ~/camera_web.log).
#
# Usage: ./scripts/remote_camera_web.sh <command> [ROS args...]
#   start   [args]   start and print the URL
#   stop             stop
#   restart [args]   stop + start
#   status           running?, URL, camera fps
#   log              follow the log (Ctrl-C to quit)
#   attach           attach to the tmux session (Ctrl-b d to detach)
#
# Examples:
#   ./scripts/remote_camera_web.sh start
#   ./scripts/remote_camera_web.sh start -p port:=8081 -p max_fps:=10.0
#
# Environment: ROBOT, REMOTE_WS, ROS_DOMAIN_ID (see remote_common.sh).
#
# It only subscribes to the camera topics, so it never conflicts with
# robot_rgbd; without robot_rgbd running the page just shows no images (its
# "robot_rgbd 시작" button can start it).

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=remote_common.sh
source "${SCRIPT_DIR}/remote_common.sh"

SESSION=camera_web
REMOTE_LOG='~/camera_web.log'
REMOTE_DIR="${REMOTE_WS}/test"
SRC_DIR="$(cd "${SCRIPT_DIR}/../test" && pwd)"
ROBOT_HOST="${ROBOT#*@}"
PROC_PAT="$PAT_CAMERA_WEB"

CMD="${1:-}"
shift || true

web_port() {   # -p port:=N from the args, else camera_test.py's default
  local a
  for a in "$@"; do
    [[ "$a" == port:=* ]] && { echo "${a#port:=}"; return; }
  done
  echo 8080
}

print_stats() {   # $1 = port
  local stats
  if stats="$(curl -s -m 3 "http://${ROBOT_HOST}:$1/api/stats")" && [[ -n "$stats" ]]; then
    echo "web UI : http://${ROBOT_HOST}:$1/"
    python3 - "$stats" <<'PY'
import json, sys
s = json.loads(sys.argv[1])
print(f"camera : RGB {s.get('rgb_fps', 0):.1f} fps, depth {s.get('depth_fps', 0):.1f} fps"
      + ("" if s.get('rgb_fps') or s.get('depth_fps') else
         "  <- no images: robot_rgbd not running? (./scripts/remote_robot_rgbd.sh start, or the page's start button)"))
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
  print_stats "${port:-8080}"
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
  rsync -az -e "ssh ${SSH_OPTS[*]}" "${SRC_DIR}/camera_test.py" "${SRC_DIR}/camera_tilt.py" "${ROBOT}:~/${REMOTE_DIR}/"

  remote "bash -s" <<EOF
rm -f $REMOTE_LOG
tmux new-session -d -s $SESSION "$REMOTE_ROS_ENV; cd ~/$REMOTE_DIR && python3 -u camera_test.py$args 2>&1 | tee $REMOTE_LOG"
for i in \$(seq 1 30); do
  curl -s -m 1 http://localhost:$port/api/stats >/dev/null 2>&1 && { echo "started (tmux '$SESSION', ROS_DOMAIN_ID=$DOMAIN)"; exit 0; }
  tmux has-session -t $SESSION 2>/dev/null || break
  sleep 0.5
done
echo "ERROR: camera_test.py did not come up. Log:"; tail -n 15 $REMOTE_LOG; exit 1
EOF
  sleep 2   # let a couple of frames arrive before reporting fps
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
