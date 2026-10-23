#!/usr/bin/env bash
# Pi에서 로봇팔 서버(arm_api.py)를 켜고 끈다(서비스를 만들지 않는 --no-service 설치용). tmux가 있으면 세션 'arm_api'로, 없으면 백그라운드로 돈다.
#
#   bash pi/arm_api_ctl.sh start      # 켜기. 다른 OpenNI 카메라 프로그램이 돌고 있으면 알린다
#   bash pi/arm_api_ctl.sh stop       # 끄기: Ctrl+C(SIGINT)로 집기 프로세스 정리를 마치게 한 뒤 끝낸다
#   bash pi/arm_api_ctl.sh restart | status | log | attach | check
#
# 로그는 ~/arm_api.log. 켜는 것만으로 로봇은 움직이지 않는다(명령을 받아야 움직인다).
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION=arm_api
LOG="$HOME/arm_api.log"
ENV_FILE="${ARM_ENV_FILE:-}"
if [ -z "$ENV_FILE" ]; then
  if [ -r "$ROOT/pi/arm-api.env" ]; then ENV_FILE="$ROOT/pi/arm-api.env"; elif [ -r /etc/arm-api.env ]; then ENV_FILE=/etc/arm-api.env; fi
fi
PY="$ROOT/.venv/bin/python"
PIDFILE="$HOME/.arm_api.pid"            # tmux가 없을 때 쓰는 PID 기록

port() { local v; v="$(grep -E '^ARM_API_PORT=.+' "$ENV_FILE" 2>/dev/null | tail -1 | cut -d= -f2)"; echo "${v:-8000}"; }
have_tmux() { command -v tmux >/dev/null 2>&1; }
pid_alive() { [ -r "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; }
running() { if have_tmux; then tmux has-session -t "$SESSION" 2>/dev/null; else pid_alive; fi; }

# 팀의 카메라 노드(openni2_camera, robot_rgbd 등)도 OpenNI 장치를 연다. 우리 카메라 코드는 USB ID(ARM_ASTRA_ID, 기본 2bc5/0401)로
# 팔 카메라만 고르므로 같이 돌 수 있지만, 팀 노드가 device_id 없이 먼저 켜지면 '첫 장치'로 우리 Astra를 잡을 수 있다.
camera_users() {
  pgrep -af 'openni2_camera|astra_camera|robot_rgbd|robot_arm\.py|camera_test\.py|hand_eye' 2>/dev/null \
    | grep -v "pgrep\|arm_api" || true
}

start() {
  [ -x "$PY" ] || { echo "$PY 가 없습니다. 먼저 bash pi/install.sh --no-service 를 실행하세요."; exit 1; }
  [ -n "$ENV_FILE" ] || { echo "설정 파일이 없습니다(pi/arm-api.env). 먼저 bash pi/install.sh --no-service"; exit 1; }
  if running; then echo "이미 켜져 있습니다. status로 확인하세요."; exit 0; fi
  local p; p="$(port)"
  if ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]$p$"; then
    echo "포트 $p 를 다른 프로그램이 쓰고 있습니다. $ENV_FILE 의 ARM_API_PORT를 바꾸세요."; exit 1
  fi
  local users; users="$(camera_users)"
  if [ -n "$users" ]; then                    # 막지 않고 알린다: 사진이 안 찍히면 팀 노드가 우리 Astra를 잡은 것이다
    echo "주의: OpenNI 카메라를 여는 다른 프로그램이 돌고 있습니다:"; echo "$users" | sed 's/^/   /' | cut -c1-140
    echo "      팔 카메라는 USB ID로 골라 열지만, 사진 오류가 나면 팀 노드에 Astra Pro를 지정(device_id)하거나 잠시 끄세요."
  fi
  : >> "$LOG"
  if have_tmux; then
    tmux new-session -d -s "$SESSION" "cd '$ROOT' && set -a && . '$ENV_FILE' && set +a && exec '$PY' scripts/arm_api.py --host 0.0.0.0 2>&1 | tee -a '$LOG'"
  else
    # exec로 이 쉘이 서버 프로세스로 바뀌어야 $!가 서버의 PID가 된다(아니면 정지 신호가 쉘로 가고 서버가 남는다)
    ( cd "$ROOT" && set -a && . "$ENV_FILE" && set +a && exec nohup "$PY" scripts/arm_api.py --host 0.0.0.0 >> "$LOG" 2>&1 ) &
    echo $! > "$PIDFILE"
  fi
  sleep 2
  if running; then echo "켰습니다. 로그: bash pi/arm_api_ctl.sh log   접속: http://$(hostname -I | awk '{print $1}'):$p"; else echo "바로 꺼졌습니다. 로그를 확인하세요: tail $LOG"; tail -n 15 "$LOG"; exit 1; fi
}

stop() {
  if ! running; then echo "켜져 있지 않습니다."; return 0; fi
  # SIGINT(Ctrl+C): 집기 프로세스가 게인 복구·팔 고정·펌프 정지를 마친 뒤 끝난다
  if have_tmux; then tmux send-keys -t "$SESSION" C-c; else kill -INT "$(cat "$PIDFILE")"; fi
  for _ in $(seq 1 30); do running || { echo "껐습니다."; rm -f "$PIDFILE"; return 0; }; sleep 1; done
  echo "30초 안에 끝나지 않아 강제로 닫습니다(팔 상태를 확인하세요)."
  if have_tmux; then tmux kill-session -t "$SESSION"; else kill -KILL "$(cat "$PIDFILE")" 2>/dev/null; rm -f "$PIDFILE"; fi
}

status() {
  if running; then
    echo "켜짐($(have_tmux && echo "tmux 세션 $SESSION" || echo "PID $(cat "$PIDFILE")"))"
    local p; p="$(port)"; curl -s -m 3 "http://127.0.0.1:$p/health" && echo || echo "  (응답 없음: 아직 시작 중일 수 있습니다)"
  else
    echo "꺼짐"
  fi
  local users; users="$(camera_users)"; [ -n "$users" ] && { echo "카메라를 쓸 수 있는 다른 프로그램:"; echo "$users" | sed 's/^/   /' | cut -c1-140; }
  return 0
}

check() {
  [ -x "$PY" ] || { echo "$PY 가 없습니다."; exit 1; }
  mapfile -t SETTINGS < <(grep -E '^[A-Z0-9_]+=.+' "$ENV_FILE" 2>/dev/null || true)
  env "${SETTINGS[@]}" "$PY" "$ROOT/scripts/check_devices.py"
}

case "${1:-}" in
  start) start ;;
  stop) stop ;;
  restart) stop; start ;;
  status) status ;;
  log) tail -n "${2:-60}" -f "$LOG" ;;
  attach) have_tmux && tmux attach -t "$SESSION" || { echo "tmux가 없습니다. log로 보세요."; exit 1; } ;;
  check) check ;;
  *) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
