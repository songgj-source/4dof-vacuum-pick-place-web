#!/usr/bin/env bash
# 노트북에서 한 번에: 음성 인식 서버 + Ollama 중계 + Pi 설정 맞추기 + Pi 서버 + 화면 열기.
# 켜기만 해서는 로봇이 움직이지 않는다(명령을 받아야 움직인다). HOME에 놓인 팔은 토크를 켜서 그 자리에 붙잡는다.
#
#   bash pi/start_all.sh            # 켜기 (이미 켜진 것은 건너뛴다)
#   bash pi/start_all.sh stop       # 모두 끄기 (Pi 서버는 진행 중인 작업을 정리한 뒤 끝난다)
#   bash pi/start_all.sh status     # 무엇이 켜져 있는지
#   NO_BROWSER=1 bash pi/start_all.sh   # 화면은 열지 않는다
#   NO_TORQUE=1 bash pi/start_all.sh    # 토크는 켜지 않고 팔 상태만 확인
#
# 환경 변수: ROBOT (기본 user@192.168.0.31), REMOTE_DIR (4dof_ik_pi_test), ARM_SSH_KEY,
#            STT_PY (Whisper가 깔린 파이썬, 기본 ~/miniconda3/envs/lerobot_312/bin/python)
# 노트북 주소가 바뀌면 Pi의 pi/arm-api.env(ARM_STT_URL, OLLAMA_HOST)를 새 주소로 고치고 서버를 다시 켠다.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT="${ROBOT:-user@192.168.0.31}"
PI_IP="${ROBOT#*@}"
REMOTE_DIR="${REMOTE_DIR:-4dof_ik_pi_test}"
STT_PY="${STT_PY:-$HOME/miniconda3/envs/lerobot_312/bin/python}"
STT_PORT=8010
OLLAMA_PORT=11434
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=15); [ -n "${ARM_SSH_KEY:-}" ] && SSH+=(-i "$ARM_SSH_KEY")
REMOTE_ENV="~/$REMOTE_DIR/pi/arm-api.env"

say() { printf '== %s\n' "$*"; }
warn() { printf '   ! %s\n' "$*"; }
pi() { "${SSH[@]}" "$ROBOT" "$@"; }

laptop_ip() { ip -4 route get "$PI_IP" 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p' | head -1; }
stt_running() { curl -s -m 2 "http://127.0.0.1:$STT_PORT/health" | grep -q '"ok": true'; }
fwd_running() { ss -ltn 2>/dev/null | awk '{print $4}' | grep -qx "$1:$OLLAMA_PORT"; }
pi_env() { pi "grep -E '^$1=' $REMOTE_ENV | tail -1 | cut -d= -f2-"; }

reach_pi() {
  ping -c1 -W2 "$PI_IP" >/dev/null 2>&1 || { warn "Pi($PI_IP)가 응답하지 않습니다. 전원과 Wi-Fi(robot)를 확인하세요."; return 1; }
  pi true 2>/dev/null || { warn "ssh $ROBOT 접속 실패 (키 등록: ssh-copy-id $ROBOT)"; return 1; }
}

start() {
  local me; me="$(laptop_ip)"
  [ -n "$me" ] || { warn "Pi($PI_IP)로 가는 경로가 없습니다. 노트북 Wi-Fi를 robot에 연결하세요."; exit 1; }
  say "노트북 $me  →  Pi $PI_IP"
  reach_pi || exit 1

  say "음성 인식 서버 (Whisper, :$STT_PORT)"
  if stt_running; then echo "   이미 켜져 있음"
  else
    local token; token="$(pi_env ARM_STT_TOKEN)"
    [ -n "$token" ] || warn "Pi 설정에 ARM_STT_TOKEN이 없습니다 — 토큰 없이 켭니다(같은 네트워크 누구나 쓸 수 있음)"
    "$STT_PY" -c 'import faster_whisper' 2>/dev/null || { warn "$STT_PY 에 faster-whisper가 없습니다 (STT_PY=... 로 지정)"; exit 1; }
    ( cd "$ROOT" && env -u ARM_STT_URL nohup "$STT_PY" scripts/stt_server.py --host 0.0.0.0 --port "$STT_PORT" \
        ${token:+--token "$token"} >> "$HOME/stt_server.log" 2>&1 & )
    for _ in $(seq 1 60); do stt_running && break; sleep 1; done
    stt_running && echo "   켬 (로그 ~/stt_server.log)" || { warn "켜지지 않았습니다: tail ~/stt_server.log"; exit 1; }
  fi

  say "Ollama 중계 ($me:$OLLAMA_PORT → 127.0.0.1, Pi만 허용)"
  curl -s -m 3 "http://127.0.0.1:$OLLAMA_PORT/api/tags" >/dev/null || warn "노트북 Ollama가 꺼져 있습니다 (systemctl status ollama) — 대화는 규칙으로만 답합니다"
  if fwd_running "$me"; then echo "   이미 켜져 있음"
  else
    pkill -f "socat TCP4-LISTEN:$OLLAMA_PORT" 2>/dev/null     # 예전 주소에 묶인 중계
    command -v socat >/dev/null || { warn "socat이 없습니다 (sudo apt install socat)"; exit 1; }
    nohup socat "TCP4-LISTEN:$OLLAMA_PORT,bind=$me,reuseaddr,fork,range=$PI_IP/32" "TCP4:127.0.0.1:$OLLAMA_PORT" \
      >> "$HOME/ollama_fwd.log" 2>&1 &
    sleep 1; fwd_running "$me" && echo "   켬" || warn "켜지지 않았습니다: tail ~/ollama_fwd.log"
  fi

  say "Pi 설정 (노트북 주소)"
  local changed=0 want_stt="http://$me:$STT_PORT/api/stt" want_llm="http://$me:$OLLAMA_PORT" kv key
  for kv in "ARM_STT_URL=$want_stt" "OLLAMA_HOST=$want_llm"; do
    key="${kv%%=*}"
    if [ "$(pi_env "$key")" != "${kv#*=}" ]; then
      pi "f=$REMOTE_ENV; if grep -q '^$key=' \$f; then sed -i 's|^$key=.*|$kv|' \$f; else echo '$kv' >> \$f; fi" && changed=1
      echo "   $kv 로 고침"
    fi
  done
  [ "$changed" = 1 ] || echo "   맞음"

  say "펌프 포트 (설정한 USB 포트 이름이 사라졌으면 다시 찾는다)"
  local pump; pump="$(pi "cd ~/$REMOTE_DIR && set -a && . pi/arm-api.env && set +a && p=\$ARM_PUMP_PORT; \
    if [ -n \"\$p\" ] && [ ! -e \"\$p\" ]; then echo MISSING; .venv/bin/python scripts/find_pump_port.py --write 2>&1 | grep -E '^(우리 팔|펌프를)' ; fi" 2>&1)"
  if [ -z "$pump" ]; then echo "   맞음"
  else echo "$pump" | sed 's/^/   /'; case "$pump" in *ARM_PUMP_PORT=*) changed=1 ;; *) warn "펌프를 찾지 못했습니다 - 펌프 USB와 전원을 확인하세요" ;; esac; fi

  say "Pi Wi-Fi 절전 끄기"
  pi 'sudo -n /usr/sbin/iw dev wlan0 set power_save off 2>/dev/null && /usr/sbin/iw dev wlan0 get power_save' \
    | sed 's/^/   /' || warn "끄지 못했습니다(sudo 비밀번호 필요?) — 응답이 느릴 수 있습니다"

  say "Pi 장치 점검"
  pi "bash ~/$REMOTE_DIR/pi/arm_api_ctl.sh check" 2>&1 | grep -E '문제|\[ 문제 \]|OK  \] (servo|pump|wrist|openni2)' | sed 's/^/   /'

  say "팔 상태 · 토크 (HOME에 놓인 팔은 토크를 켜서 붙잡는다, NO_TORQUE=1이면 확인만)"
  local hold=--hold; [ -n "${NO_TORQUE:-}" ] && hold=
  if ! pi "cd ~/$REMOTE_DIR && set -a && . pi/arm-api.env && set +a && .venv/bin/python scripts/arm_ready.py $hold" 2>&1 | sed 's/^/   /'; then
    warn "팔을 움직일 수 없는 상태입니다 - 위 안내대로 고친 뒤 bash pi/start_all.sh 를 다시 실행하세요 (서버는 그대로 켭니다)"
  fi

  say "Pi 서버"
  if pi "bash ~/$REMOTE_DIR/pi/arm_api_ctl.sh status" | grep -q '^켜짐'; then
    if [ "$changed" = 1 ]; then
      pi "bash ~/$REMOTE_DIR/pi/arm_api_ctl.sh restart" | tail -1 | sed 's/^/   /'
    else echo "   이미 켜져 있음"; fi
  else
    pi "bash ~/$REMOTE_DIR/pi/arm_api_ctl.sh start" | tail -1 | sed 's/^/   /'
  fi
  pi "tail -n 3 ~/arm_api.log" | grep -E '음성 인식|로컬' | tail -1 | sed 's/^/   /'

  if [ -z "${NO_BROWSER:-}" ]; then
    say "화면"
    ARM_URL="http://$PI_IP:8000" bash "$ROOT/pi/open_jarvis.sh" | sed 's/^/   /'
  fi
  echo
  echo "준비됐습니다. 움직이는 명령(넣어줘, 정리해, 홈으로)은 묻지 않고 바로 실행됩니다. 끄기: bash pi/start_all.sh stop"
}

stop() {
  say "Pi 서버"
  if reach_pi; then pi "bash ~/$REMOTE_DIR/pi/arm_api_ctl.sh stop" | sed 's/^/   /'; fi
  say "음성 인식 서버"
  pkill -f "scripts/stt_server.py" && echo "   끔" || echo "   꺼져 있음"
  say "Ollama 중계"
  pkill -f "socat TCP4-LISTEN:$OLLAMA_PORT" && echo "   끔" || echo "   꺼져 있음"
}

status() {
  local me; me="$(laptop_ip)"
  echo "노트북 ${me:-(Pi로 가는 경로 없음)}  →  Pi $PI_IP"
  echo "음성 인식 서버: $(stt_running && echo 켜짐 || echo 꺼짐)"
  echo "Ollama 중계:    $([ -n "$me" ] && fwd_running "$me" && echo 켜짐 || echo 꺼짐)"
  if reach_pi 2>/dev/null; then
    echo "Pi 서버:        $(pi "bash ~/$REMOTE_DIR/pi/arm_api_ctl.sh status" | head -1)"
    echo "Pi Wi-Fi 절전:  $(pi '/usr/sbin/iw dev wlan0 get power_save 2>/dev/null')"
  fi
}

case "${1:-start}" in
  start) start ;;
  stop) stop ;;
  status) status ;;
  *) sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
