#!/usr/bin/env bash
set -euo pipefail

# Run robot_lidar.launch.py (car2 4-wheel driver + LD-series 2D lidar -> /scan,
# optionally the Astra Pro depth + RGB camera) on the robot (Raspberry Pi) from
# the laptop. Runs in tmux session 'car2_lidar' on the Pi (log ~/car2_lidar.log).
#
# Usage: ./scripts/remote_car2_lidar.sh <command> [--camera] [launch args...]
#   start   [args]   start (e.g. lidar_z:=0.25 lidar_yaw:=3.1416, use_car2_driver:=false)
#                    --camera (= use_camera:=true): also start the depth + RGB camera
#                    (same nodes as remote_robot_rgbd.sh, without a second driver);
#                    video_device:=/dev/video1 if the RGB camera isn't /dev/video0
#   stop             stop (SIGINT: driver sends STOP to the car, lidar port is released)
#   restart [args]   stop + start
#   status           what is running, scan rate, last important log lines
#   log              follow the log (Ctrl-C to quit)
#   attach           attach to the tmux session (Ctrl-b d to detach)
#
# Examples:
#   ./scripts/remote_car2_lidar.sh start
#   ./scripts/remote_car2_lidar.sh start --camera                 # driver + lidar + camera
#   ./scripts/remote_car2_lidar.sh start use_car2_driver:=false   # lidar only, next to
#                                                                 # remote_robot_rgbd.sh (driver + camera)
#
# Environment: ROBOT, REMOTE_WS, ROS_DOMAIN_ID (see remote_common.sh).
#
# Refuses to start while the lidar is already being read, or (unless
# use_car2_driver:=false) while another driver / car2_web.py holds /dev/ttyS0.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=remote_common.sh
source "${SCRIPT_DIR}/remote_common.sh"

SESSION=car2_lidar
REMOTE_LOG='~/car2_lidar.log'
LAUNCH_PAT='ros2 launch car2_bringup robot_lidar.launch.py'
LIDAR_PAT='car2_driver/ld_lidar_node'
CHILD_PATS="'car2_serial_node.*--params-file' '$LIDAR_PAT' 'static_transform_publisher.*base_to_laser_tf' 'openni2_camera_driver' 'usb_cam_node_exe' 'static_transform_publisher.*base_to_camera_tf'"

CMD="${1:-}"
shift || true

do_status() {
  remote "bash -s" <<EOF
if tmux has-session -t $SESSION 2>/dev/null; then echo "tmux '$SESSION': running"; else echo "tmux '$SESSION': not running"; fi
if pgrep -f '$LAUNCH_PAT' >/dev/null; then echo "robot_lidar.launch.py: running (pid \$(pgrep -f '$LAUNCH_PAT' | tr '\n' ' '))"; else echo "robot_lidar.launch.py: not running"; fi
for p in 'car2_driver/car2_serial_node' '$LIDAR_PAT' 'openni2_camera_driver' 'usb_cam_node_exe'; do
  n=\$(pgrep -fc "\$p" || true); echo "  \$p: \${n:-0} process(es)"
done
lsusb | grep -q '10c4:ea60' && echo "  lidar USB (CP2102): connected" || echo "  lidar USB (CP2102): NOT connected"
lsusb | grep -q '2bc5:060f' && echo "  camera USB (Astra Pro): connected" || echo "  camera USB (Astra Pro): NOT connected"
if [ -f $REMOTE_LOG ]; then
  echo "--- $REMOTE_LOG (important lines)"
  grep -E 'ready on|first scan published|scans [0-9.]+ Hz|cannot open|no data from|has died|Device ".*" found|Starting .astra_pro_rgb.|No matching device|ERROR|WARN' $REMOTE_LOG \
    | grep -vE 'Unsupported color video mode|camera calibration file' | tail -n 8 | cut -c1-170
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
# (so a driver belonging to a running robot_rgbd launch is left alone).
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
  local args="" with_driver=1 with_camera=0 a
  for a in "$@"; do
    case "$a" in
      --camera)    a=use_camera:=true ;;
      --no-camera) a=use_camera:=false ;;
    esac
    args+=" $(printf '%q' "$a")"
    [[ "$a" == use_car2_driver:=false ]] && with_driver=0
    [[ "$a" == use_camera:=true ]] && with_camera=1
    [[ "$a" == use_camera:=false ]] && with_camera=0
  done
  remote "bash -s" <<EOF
if tmux has-session -t $SESSION 2>/dev/null || pgrep -f '$LAUNCH_PAT' >/dev/null; then
  echo "already running. Use: restart / stop"; exit 1
fi
busy=0
if ! lsusb | grep -q '10c4:ea60'; then
  echo "ERROR: lidar (CP2102 USB serial, 10c4:ea60) is not connected"; busy=1
fi
if pgrep -f '$LIDAR_PAT' >/dev/null; then
  echo "ERROR: the lidar is already being read by: \$(pgrep -af '$LIDAR_PAT' | head -1 | cut -c1-110)"; busy=1
fi
if [ $with_driver = 1 ] && pgrep -f 'car2_driver/car2_serial_node' >/dev/null; then
  echo "ERROR: a car2 driver is already running on the robot:"
  pgrep -af 'car2_driver/car2_serial_node' | cut -c1-110
  pgrep -f 'ros2 launch car2_bringup robot_rgbd.launch.py' >/dev/null && \
    echo "  -> robot_rgbd (driver + camera) is running: start this with use_car2_driver:=false to add the lidar"
  tmux has-session -t car2_driver 2>/dev/null && echo "  -> or stop remote_car2_driver.sh's driver: ./scripts/remote_car2_driver.sh stop"
  busy=1
fi
if [ $with_driver = 1 ] && pgrep -af '$PAT_CAR2_WEB' | grep -qv -- '--dry-run'; then
  echo "ERROR: car2_web.py is using the serial port -- stop it (./scripts/remote_car2_web.sh stop) or start with use_car2_driver:=false"
  busy=1
fi
if [ $with_camera = 1 ]; then
  lsusb | grep -q '2bc5:060f' || { echo "ERROR: camera (Astra Pro, 2bc5:060f) is not connected"; busy=1; }
  for p in 'openni2_camera_driver' 'usb_cam_node_exe' 'component_container.*__ns:=/camera'; do
    pgrep -f "\$p" >/dev/null && { echo "ERROR: camera already in use by: \$(pgrep -af "\$p" | head -1 | cut -c1-110)"; busy=1; }
  done
  pgrep -f 'ros2 launch car2_bringup robot_rgbd.launch.py' >/dev/null && \
    echo "  -> robot_rgbd is running: stop it (./scripts/remote_robot_rgbd.sh stop) or start without --camera"
fi
[ \$busy = 1 ] && exit 1
if [ ! -x ~/$REMOTE_WS/scripts/run_car2_lidar.sh ]; then
  echo "ERROR: ~/$REMOTE_WS/scripts/run_car2_lidar.sh not found -- run ./scripts/deploy_to_robot.sh first"; exit 1
fi
rm -f $REMOTE_LOG   # an old log's lines must not count as this start
tmux new-session -d -s $SESSION "$REMOTE_ROS_ENV; cd ~/$REMOTE_WS && ./scripts/run_car2_lidar.sh$args 2>&1 | tee $REMOTE_LOG"

want_driver=$with_driver
want_camera=$with_camera
for i in \$(seq 1 80); do
  scan=0; drv=0; depth=0; rgb=0
  grep -q 'first scan published' $REMOTE_LOG 2>/dev/null && scan=1
  grep -q 'car2_serial_node ready on' $REMOTE_LOG 2>/dev/null && drv=1
  # OpenNI2 opens the device at startup but streams only once subscribed: "found" = ready
  grep -qE 'Device ".*" found' $REMOTE_LOG 2>/dev/null && depth=1
  grep -q "Starting 'astra_pro_rgb'" $REMOTE_LOG 2>/dev/null && rgb=1
  cam_ok=1; [ \$want_camera = 1 ] && { [ \$depth = 1 ] && [ \$rgb = 1 ] || cam_ok=0; }
  [ \$scan = 1 ] && { [ \$want_driver = 0 ] || [ \$drv = 1 ]; } && [ \$cam_ok = 1 ] && break
  tmux has-session -t $SESSION 2>/dev/null || break
  sleep 0.5
done
grep 'first scan published' $REMOTE_LOG | head -1 | sed 's/.*\]: /lidar        : /' | cut -c1-150
echo "lidar /scan  : \$([ \$scan = 1 ] && echo OK || echo 'NOT publishing')"
if [ \$want_driver = 1 ]; then echo "car2 driver  : \$([ \$drv = 1 ] && echo OK || echo 'NOT ready')"; else echo "car2 driver  : not included (use_car2_driver:=false)"; fi
if [ \$want_camera = 1 ]; then
  echo "depth camera : \$([ \$depth = 1 ] && echo OK || echo 'NOT started')"
  echo "RGB camera   : \$([ \$rgb = 1 ] && echo OK || echo 'NOT started')"
else
  echo "camera       : not included (add --camera)"
fi
if [ \$scan = 1 ] && { [ \$want_driver = 0 ] || [ \$drv = 1 ]; } && [ \$cam_ok = 1 ]; then
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
