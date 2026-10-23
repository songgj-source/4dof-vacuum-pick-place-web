#!/usr/bin/env bash
# Pi에서 내 주행 스택(우리 폴더의 ros2_slam 사본)을 켜고 끈다. 팀 폴더(~/ros2_slam_robot)·팀 프로그램은 건드리지 않는다.
#
#   bash pi/drive_ctl.sh build      # ros2_slam 사본을 이 Pi에서 colcon 빌드(코드를 바꾼 뒤 한 번)
#   bash pi/drive_ctl.sh start      # 드라이버+라이다 → 위치 추정(AMCL)+Nav2 → map_web(8082). 차는 명령을 받아야 움직인다
#   bash pi/drive_ctl.sh stop | status | log [robot|nav|web] | relocalize(지도 위 위치 다시 잡기)
#
# 팀과 겹치지 않게: tmux 세션 arm_drive_robot / arm_drive_nav / arm_drive_web, ROS_DOMAIN_ID 43(팀 42),
# map_web의 라벨·도착 오차·근접 설정은 ~/4dof_ik_pi_test/drive_data(처음 한 번 팀 라벨 파일을 읽어 복사).
# 팀 주행(드라이버·라이다·map_web·AMCL)이 이미 돌고 있으면 같은 장치를 두고 부딪히므로 켜지 않고 알린다.
# 환경 변수: ARM_ROS_DOMAIN(43), ARM_DRIVE_MAP(remap_20261023_1156: ros2_slam/maps/<이름>.yaml), ARM_DRIVE_INIT("x y yaw", 기본 지도 원점),
#            ARM_CAR_PORT(차 ESP32 시리얼, 기본 /dev/ttyAMA0 - 이 Pi에서 팀 드라이버도 이것으로 켰다. 사본 launch 기본값 ttyS0은 입출력 오류)
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_slam"
DATA="$ROOT/drive_data"
DOMAIN="${ARM_ROS_DOMAIN:-43}"
MAP_NAME="${ARM_DRIVE_MAP:-remap_20261023_1156}"
MAP="$WS/maps/$MAP_NAME.yaml"
CAR_PORT="${ARM_CAR_PORT:-/dev/ttyAMA0}"
read -r INIT_X INIT_Y INIT_YAW <<< "${ARM_DRIVE_INIT:-0.0 0.0 0.0}"
ENV="export ROS_DISTRO=jazzy ROS_DOMAIN_ID=$DOMAIN RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; source /opt/ros/jazzy/setup.bash; source '$WS/install/setup.bash'"
SESSIONS=(arm_drive_robot arm_drive_nav arm_drive_web)
LOGS="$HOME/arm_drive"

say() { printf '== %s\n' "$*"; }
running() { tmux has-session -t "$1" 2>/dev/null; }
ours_running() { for s in "${SESSIONS[@]}"; do running "$s" && return 0; done; return 1; }

# 다른 주행 프로그램(팀 것 등): 같은 이름의 프로세스 중 ROS_DOMAIN_ID가 우리 것(43)이 아닌 것.
others() {
  local pid cmd dom
  # 실행 파일이 명령 줄 맨 앞(argv[0])인 것만: 'bash -c ...' 같은 껍데기나 이 이름이 들어간 다른 명령은 빼고 진짜 프로그램만 센다
  pgrep -af '^(\S*/)?(car2_serial_node|ld_lidar_node|amcl|controller_server|async_slam_toolbox_node)( |$)|^\S*python[0-9.]* .*map_web\.py' 2>/dev/null \
    | grep -v pgrep | while read -r pid cmd; do
        case "$cmd" in *"ROS_DOMAIN_ID=$DOMAIN "*|*"$WS"*) continue ;; esac     # 우리 세션의 실행 줄·우리 사본
        dom="$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p')"
        [ "$dom" = "$DOMAIN" ] && continue
        echo "$pid (ROS_DOMAIN_ID=${dom:-?}) ${cmd:0:110}"
      done
}

build() {
  [ -f /opt/ros/jazzy/setup.bash ] || { echo "ROS 2 Jazzy가 없습니다."; exit 1; }
  say "ros2_slam 사본 빌드 ($WS)"
  ( set +u; cd "$WS" && export ROS_DISTRO=jazzy && source /opt/ros/jazzy/setup.bash && colcon build 2>&1 | tail -3 )   # ROS 설정 파일은 set -u에서 멈춘다
}

seed_data() {
  mkdir -p "$DATA" "$LOGS"
  if [ ! -f "$DATA/labels.json" ] && [ -f "$WS/maps/$MAP_NAME.labels.json" ]; then
    cp "$WS/maps/$MAP_NAME.labels.json" "$DATA/labels.json"
    echo "   현재 지도의 라벨을 복사했습니다: $DATA/labels.json"
  elif [ ! -f "$DATA/labels.json" ] && [ -f "$HOME/.ros/car2_map_labels.json" ]; then
    cp "$HOME/.ros/car2_map_labels.json" "$DATA/labels.json"          # 팀 파일은 읽기만 한다
    echo "   팀 라벨 파일을 읽어 우리 데이터로 복사했습니다: $DATA/labels.json"
  fi
}

start_session() {   # 이름, 로그, 명령
  local name="$1" log="$2"; shift 2
  running "$name" && { echo "   $name 이미 켜져 있음"; return 0; }
  tmux new-session -d -s "$name" "bash -c \"$ENV; $* 2>&1 | tee -a '$log'\""
  echo "   $name 켬 (로그 $log)"
}

start() {
  # start_all을 다시 실행해도 SLAM과 AMCL/map_server를 동시에 띄우지 않는다.
  if running arm_drive_mapping; then
    say "새 매핑이 실행 중입니다. 매핑을 유지하고 내비게이션은 켜지 않습니다."
    echo "   매핑 화면: http://$(hostname -I | awk '{print $1}'):8082"
    return 0
  fi
  [ -f "$WS/install/setup.bash" ] || build
  [ -f "$MAP" ] || { echo "지도가 없습니다: $MAP (ARM_DRIVE_MAP로 이름 지정)"; exit 1; }
  local busy; busy="$(others)"
  if [ -n "$busy" ]; then
    echo "다른 주행 프로그램(팀 것 등)이 이 Pi에서 돌고 있어 켜지 않습니다(바퀴 드라이버·라이다가 겹칩니다):"
    echo "$busy" | sed 's/^/   /'
    echo "팀원과 맞춰 그쪽을 끈 뒤 다시 실행하세요. 팀 프로그램은 제가 끄지 않습니다."
    exit 1
  fi
  seed_data
  say "내 주행 스택 켜기 (ROS_DOMAIN_ID=$DOMAIN, 지도 $(basename "$MAP"))"
  [ -e "$CAR_PORT" ] || echo "   주의: 차 시리얼 $CAR_PORT 이 없습니다(ARM_CAR_PORT로 지정)"
  start_session arm_drive_robot "$LOGS/robot.log" "ros2 launch car2_bringup robot_lidar.launch.py serial_port:=$CAR_PORT"
  sleep 4
  start_session arm_drive_nav "$LOGS/nav.log" "ros2 launch car2_bringup lidar_navigation.launch.py map:='$MAP' use_rviz:=false initial_x:=$INIT_X initial_y:=$INIT_Y initial_yaw:=$INIT_YAW"
  start_session arm_drive_web "$LOGS/web.log" "cd '$WS/test' && exec /usr/bin/python3 -u map_web.py --ros-args -p port:=8082 \
    -p labels_file:='$DATA/labels.json' -p tolerance_file:='$DATA/nav_tolerance.json' \
    -p approach_file:='$DATA/approach.json' -p wall_file:='$DATA/wall_approach.json'"
  for _ in $(seq 1 20); do curl -s -m 1 http://127.0.0.1:8082/api/drive >/dev/null && break; sleep 1; done
  curl -s -m 2 http://127.0.0.1:8082/api/drive >/dev/null && echo "   map_web 응답 OK (http://$(hostname -I | awk '{print $1}'):8082)" || { echo "   map_web가 아직 응답하지 않습니다: bash pi/drive_ctl.sh log web"; return 0; }
  relocalize
}

# 지도 위 위치를 한 번 잡는다(map_web의 '전체 지도에서 찾기' = 라이다 스캔 맞추기 → AMCL /initialpose). 차는 움직이지 않는다.
# 처음 위치(ARM_DRIVE_INIT, 기본 지도 원점)와 다른 곳에서 켜도 위치를 찾게 한다.
relocalize() {
  local js
  js() { curl -s -m 3 http://127.0.0.1:8082/api/state | /usr/bin/python3 -c "import sys, json; d = json.load(sys.stdin); print($1)" 2>/dev/null; }
  for _ in $(seq 1 60); do [ "$(js "(d.get('reloc') or {}).get('amcl')")" = True ] && [ "$(js "d.get('frame')")" = map ] && break; sleep 1; done
  if [ "$(js "(d.get('reloc') or {}).get('amcl')")" != True ]; then echo "   위치 추정(AMCL)이 아직 준비되지 않아 위치 잡기를 건너뜁니다: bash pi/drive_ctl.sh log nav"; return 0; fi
  curl -s -m 3 -X POST http://127.0.0.1:8082/api/relocalize -d '{"global": true}' >/dev/null
  for _ in $(seq 1 40); do [ "$(js "(d.get('reloc') or {}).get('state')")" != running ] && break; sleep 1; done
  echo "   위치 잡기: $(js "(d.get('reloc') or {}).get('message')") (스캔 일치율 $(js "((d.get('reloc') or {}).get('result') or {}).get('match')"))"
}

stop() {
  ours_running || { echo "내 주행 스택은 꺼져 있습니다."; return 0; }
  curl -s -m 2 -X POST http://127.0.0.1:8082/api/drive/stop >/dev/null 2>&1 || true      # 먼저 차를 세운다
  for s in arm_drive_web arm_drive_nav arm_drive_robot; do
    running "$s" || continue
    tmux send-keys -t "$s" C-c
    for _ in $(seq 1 10); do running "$s" || break; sleep 1; done
    running "$s" && tmux kill-session -t "$s"
    echo "   $s 끔"
  done
}

status() {
  for s in "${SESSIONS[@]}"; do printf '   %-16s %s\n' "$s" "$(running "$s" && echo 켜짐 || echo 꺼짐)"; done
  local d; d="$(curl -s -m 2 http://127.0.0.1:8082/api/drive 2>/dev/null)"
  [ -n "$d" ] && echo "   map_web: $d" || echo "   map_web: 응답 없음"
  local busy; busy="$(others)"; [ -n "$busy" ] && { echo "   다른 주행 프로그램:"; echo "$busy" | sed 's/^/     /'; }
  return 0
}

case "${1:-}" in
  build) build ;;
  start) start ;;
  stop) stop ;;
  restart) stop; start ;;
  status) status ;;
  relocalize) relocalize ;;
  log) tail -n 60 -f "$LOGS/${2:-web}.log" ;;
  *) sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
