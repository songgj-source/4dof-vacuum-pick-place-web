#!/usr/bin/env bash
set -euo pipefail

# Run robot_rgbd.launch.py (car2 driver + Astra Pro depth/RGB + camera TFs) on
# the robot (Raspberry Pi) from the laptop, for RTAB-Map mapping on the laptop
# (./scripts/run_rtabmap_mapping.sh). Runs in tmux session 'robot_rgbd' on the
# Pi (log ~/robot_rgbd.log) -- the same session the camera web UI's restart
# button uses, so either can stop / restart what the other started.
#
# Usage: ./scripts/remote_robot_rgbd.sh <command> [launch args...]
#   start   [args]   start (e.g. video_device:=/dev/video1, use_car2_driver:=false)
#   stop             stop (SIGINT: driver sends STOP to the car, camera is released)
#   restart [args]   stop + start
#   status           what is running, last important log lines
#   log              follow the log (Ctrl-C to quit)
#   attach           attach to the tmux session (Ctrl-b d to detach)
#
# Examples:
#   ./scripts/remote_robot_rgbd.sh start
#   ./scripts/remote_robot_rgbd.sh start use_car2_driver:=false   # driver via remote_car2_driver.sh
#
# Environment: ROBOT, REMOTE_WS, ROS_DOMAIN_ID (see remote_common.sh).
#
# Refuses to start while the camera or (unless use_car2_driver:=false) the
# serial port is already in use on the robot -- two drivers on /dev/ttyS0
# corrupt each other's replies, and the depth camera can only be opened once.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=remote_common.sh
source "${SCRIPT_DIR}/remote_common.sh"

SESSION=robot_rgbd
REMOTE_LOG='~/robot_rgbd.log'
LAUNCH_PAT='ros2 launch car2_bringup robot_rgbd.launch.py'
# Processes that belong to a robot_rgbd launch (the launch-started driver has
# --params-file; a run_car2_driver.sh driver uses -p and is left alone).
CHILD_PATS="'car2_serial_node.*--params-file' 'openni2_camera_driver' 'usb_cam_node_exe.*__ns:=/camera' 'component_container.*__ns:=/camera'"

CMD="${1:-}"
shift || true

do_status() {
  remote "bash -s" <<EOF
if tmux has-session -t $SESSION 2>/dev/null; then echo "tmux '$SESSION': running"; else echo "tmux '$SESSION': not running"; fi
if pgrep -f '$LAUNCH_PAT' >/dev/null; then echo "robot_rgbd.launch.py: running (pid \$(pgrep -f '$LAUNCH_PAT' | tr '\n' ' '))"; else echo "robot_rgbd.launch.py: not running"; fi
for p in 'car2_driver/car2_serial_node' 'openni2_camera_driver' 'usb_cam_node_exe'; do
  n=\$(pgrep -fc "\$p" || true); echo "  \$p: \${n:-0} process(es)"
done
tmux has-session -t car2_driver 2>/dev/null && echo "  (tmux 'car2_driver' is running a separate driver -- remote_car2_driver.sh)"
if [ -f $REMOTE_LOG ]; then
  echo "--- $REMOTE_LOG (important lines)"
  grep -E 'camera mount|placeholder camera mount|ready on|Device .* found|No matching device|Starting depth stream|Starting .astra|has died|ERROR' $REMOTE_LOG \
    | grep -vE 'Unsupported color video mode|camera_calibration_parsers' | tail -n 8 | cut -c1-170
fi
EOF
}

do_stop() {
  remote "bash -s" <<EOF
launch=\$(pgrep -f '$LAUNCH_PAT' || true)
if [ -z "\$launch" ] && ! tmux has-session -t $SESSION 2>/dev/null; then echo "not running"; exit 0; fi
[ -n "\$launch" ] && kill -INT \$launch 2>/dev/null || true
for i in \$(seq 1 30); do pgrep -f '$LAUNCH_PAT' >/dev/null || break; sleep 0.5; done
launch=\$(pgrep -f '$LAUNCH_PAT' || true)
[ -n "\$launch" ] && { echo "launch did not exit on SIGINT, sending SIGTERM"; kill -TERM \$launch 2>/dev/null || true; sleep 3; }
# Leftovers = matching processes whose parent is no longer a 'ros2 launch'
# (so a driver belonging to a running robot_lidar launch is left alone).
for p in $CHILD_PATS; do
  for pid in \$(pgrep -f "\$p" || true); do
    pp=\$(ps -o ppid= -p \$pid | tr -d ' ')
    ps -o args= -p "\$pp" 2>/dev/null | grep -q 'ros2 launch' && continue
    echo "cleaning up leftover: \$p (\$pid)"; kill -TERM \$pid 2>/dev/null || true
  done
done
sleep 1
tmux kill-session -t $SESSION 2>/dev/null || true
echo "stopped"
EOF
}

do_start() {
  local args="" with_driver=1 a
  for a in "$@"; do
    args+=" $(printf '%q' "$a")"
    [[ "$a" == use_car2_driver:=false ]] && with_driver=0
  done
  remote "bash -s" <<EOF
if tmux has-session -t $SESSION 2>/dev/null || pgrep -f '$LAUNCH_PAT' >/dev/null; then
  echo "already running. Use: restart / stop"; exit 1
fi
busy=0
for p in 'openni2_camera_driver' 'usb_cam_node_exe' 'component_container.*__ns:=/camera'; do
  pgrep -f "\$p" >/dev/null && { echo "ERROR: camera already in use by: \$(pgrep -af "\$p" | head -1 | cut -c1-110)"; busy=1; }
done
if [ $with_driver = 1 ] && pgrep -f 'car2_driver/car2_serial_node' >/dev/null; then
  echo "ERROR: a car2 driver is already running on the robot:"
  pgrep -af 'car2_driver/car2_serial_node' | cut -c1-110
  tmux has-session -t car2_driver 2>/dev/null && echo "  -> it's remote_car2_driver.sh's. Stop it (remote_car2_driver.sh stop) or start with use_car2_driver:=false."
  busy=1
fi
if [ $with_driver = 1 ] && pgrep -af '$PAT_CAR2_WEB' | grep -qv -- '--dry-run'; then
  echo "ERROR: car2_web.py is using the serial port -- stop it (./scripts/remote_car2_web.sh stop) or start with use_car2_driver:=false"
  busy=1
fi
[ \$busy = 1 ] && exit 1
if [ ! -x ~/$REMOTE_WS/scripts/run_robot_rgbd.sh ]; then
  echo "ERROR: ~/$REMOTE_WS/scripts/run_robot_rgbd.sh not found -- run ./scripts/deploy_to_robot.sh first"; exit 1
fi
rm -f $REMOTE_LOG   # an old log's lines must not count as this start
tmux new-session -d -s $SESSION "$REMOTE_ROS_ENV; cd ~/$REMOTE_WS && ./scripts/run_robot_rgbd.sh$args 2>&1 | tee $REMOTE_LOG"

want_driver=$with_driver
for i in \$(seq 1 80); do
  depth=0; rgb=0; drv=0
  # OpenNI2 opens the device at startup but only starts streaming once someone
  # subscribes, so "device found" (not "Starting depth stream") means ready.
  grep -qE 'Device ".*" found' $REMOTE_LOG 2>/dev/null && depth=1
  grep -q "Starting 'astra_pro_rgb'" $REMOTE_LOG 2>/dev/null && rgb=1
  grep -q 'car2_serial_node ready on' $REMOTE_LOG 2>/dev/null && drv=1
  [ \$depth = 1 ] && [ \$rgb = 1 ] && { [ \$want_driver = 0 ] || [ \$drv = 1 ]; } && break
  tmux has-session -t $SESSION 2>/dev/null || break
  sleep 0.5
done
grep -E 'camera mount|placeholder camera mount' $REMOTE_LOG | tail -1 | cut -c1-170
echo "depth camera : \$([ \$depth = 1 ] && echo OK || echo 'NOT started')"
echo "RGB camera   : \$([ \$rgb = 1 ] && echo OK || echo 'NOT started')"
if [ \$want_driver = 1 ]; then echo "car2 driver  : \$([ \$drv = 1 ] && echo OK || echo 'NOT ready')"; else echo "car2 driver  : not included (use_car2_driver:=false)"; fi
if [ \$depth = 1 ] && [ \$rgb = 1 ] && { [ \$want_driver = 0 ] || [ \$drv = 1 ]; }; then
  echo "started (tmux '$SESSION', ROS_DOMAIN_ID=$DOMAIN)"
else
  grep -q 'No matching device' $REMOTE_LOG && echo "depth camera not found by OpenNI2 -- is the Astra Pro plugged in? (lsusb | grep -i orbbec)"
  echo "ERROR: not everything came up. Recent log:"
  grep -vE 'static_transform_publisher-[0-9]+\] (translation|rotation|from)' $REMOTE_LOG | tail -n 12 | cut -c1-170
  exit 1
fi
EOF
}

case "$CMD" in
  start)   do_start "$@" ;;
  stop)    do_stop ;;
  restart) do_stop; do_start "$@" ;;
  status)  do_status ;;
  log)     remote_tty "tail -n 40 -f $REMOTE_LOG" ;;
  attach)  remote_tty "tmux attach -t $SESSION" ;;
  *)       usage; exit 1 ;;
esac
