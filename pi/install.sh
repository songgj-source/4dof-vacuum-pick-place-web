#!/usr/bin/env bash
# Raspberry Pi(로봇 위)에 로봇팔 서버를 설치한다. 로봇을 움직이지 않는다.
#
#   bash pi/install.sh --openni2 /opt/openni2-orbbec        # Astra용 ARM64 OpenNI2 런타임 폴더(없으면 흔한 위치에서 찾음)
#   bash pi/install.sh --no-service                          # 다른 팀이 쓰는 Pi: 서비스·/etc 설정 없이 이 폴더 안에서만(arm_api_ctl.sh로 실행)
#   bash pi/install.sh --openni2 ... --start                 # 설치 뒤 서비스도 바로 켬(서비스 모드)
#   옵션: --no-apt(apt 건너뜀) --skip-udev(udev 규칙 건너뜀, Orbbec 규칙이 이미 있으면 자동으로 건너뜀)
#   DRY_RUN=1 bash pi/install.sh ...                         # 아무것도 바꾸지 않고 할 일만 출력
#
# 하는 일: apt 패키지 → 프로젝트 .venv와 pip 패키지 → dialout·video 그룹 → Astra udev 규칙
#          → 설정 파일(토큰 무작위, 서비스 모드는 /etc/arm-api.env, --no-service는 pi/arm-api.env)
#          → systemd 서비스(arm-api) 등록(--no-service면 건너뜀) → 장치 점검(check_devices.py)
# 권장 환경: Raspberry Pi 5, Ubuntu 24.04(arm64, Python 3.12). 노트북 개발 환경과 Python 버전이 같다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USER_NAME="${SUDO_USER:-$(id -un)}"
OPENNI2=""
LOCAL_STT=0
START=0
SERVICE=1
APT=1
UDEV=1
ENV_FILE=""
UNIT=/etc/systemd/system/arm-api.service

usage() { sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; }
while [ $# -gt 0 ]; do
  case "$1" in
    --openni2) OPENNI2="$(realpath -m "$2")"; shift 2 ;;
    --with-local-stt) LOCAL_STT=1; shift ;;      # Pi에서도 Whisper를 돌리려면(느림). 보통은 노트북 stt_server.py를 쓴다
    --start) START=1; shift ;;
    --no-service) SERVICE=0; shift ;;
    --no-apt) APT=0; shift ;;
    --skip-udev) UDEV=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "알 수 없는 옵션: $1"; usage; exit 2 ;;
  esac
done

if [ "$SERVICE" = 1 ]; then ENV_FILE=/etc/arm-api.env; else ENV_FILE="$ROOT/pi/arm-api.env"; fi

run() {                       # DRY_RUN=1이면 실행하지 않고 보여 주기만 한다
  if [ "${DRY_RUN:-0}" = 1 ]; then printf '  [실행 안 함] %s\n' "$*"; else "$@"; fi
}
step() { printf '\n== %s\n' "$*"; }

step "환경 확인"
ARCH="$(uname -m)"
echo "  컴퓨터: $ARCH, 사용자: $USER_NAME, 프로젝트: $ROOT"
[ "$ARCH" = aarch64 ] || echo "  주의: aarch64(Raspberry Pi 64비트)가 아닙니다. 계속하면 이 컴퓨터용으로 설치합니다."
PY_OK="$(python3 -c 'import sys; print(int(sys.version_info >= (3, 11)))')"
[ "$PY_OK" = 1 ] || { echo "  Python 3.11 이상이 필요합니다(권장: Ubuntu 24.04의 3.12)."; exit 1; }
python3 --version

step "apt 패키지"
if [ "$APT" = 1 ]; then
  run sudo apt-get update
  run sudo apt-get install -y python3-venv python3-dev libusb-1.0-0 v4l-utils tmux
  [ "$LOCAL_STT" = 1 ] && run sudo apt-get install -y ffmpeg
else
  echo "  --no-apt: 건너뜁니다(python3-venv, libusb, v4l-utils, tmux가 이미 있어야 합니다)"
fi

step "파이썬 가상환경(.venv)과 패키지"
run python3 -m venv "$ROOT/.venv"
run "$ROOT/.venv/bin/python" -m pip install --upgrade pip
run "$ROOT/.venv/bin/python" -m pip install -r "$ROOT/pi/requirements-pi.txt"
[ "$LOCAL_STT" = 1 ] && run "$ROOT/.venv/bin/python" -m pip install faster-whisper

step "장치 권한(서보·펌프 시리얼 dialout, 카메라 video)"
MISSING="$(for g in dialout video; do id -nG "$USER_NAME" | tr ' ' '\n' | grep -qx "$g" || printf '%s,' "$g"; done)"
if [ -n "$MISSING" ]; then run sudo usermod -aG "${MISSING%,}" "$USER_NAME"; else echo "  이미 dialout·video 그룹입니다."; fi

step "Astra udev 규칙(Orbbec 2bc5) — 일반 사용자로 카메라 열기"
if [ "$UDEV" = 0 ]; then
  echo "  --skip-udev: 건너뜁니다."
elif grep -rlsE "2bc5|2BC5" /etc/udev/rules.d /lib/udev/rules.d 2>/dev/null | head -1 | grep -q .; then
  echo "  이미 Orbbec(2bc5) udev 규칙이 있어 건드리지 않습니다: $(grep -rlsE '2bc5|2BC5' /etc/udev/rules.d /lib/udev/rules.d | head -1)"
else
  RULE='SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666", GROUP="video"'
  if [ "${DRY_RUN:-0}" = 1 ]; then
    echo "  [실행 안 함] /etc/udev/rules.d/99-orbbec-astra.rules ← $RULE"
  else
    echo "$RULE" | sudo tee /etc/udev/rules.d/99-orbbec-astra.rules >/dev/null
  fi
  run sudo udevadm control --reload-rules
  run sudo udevadm trigger
fi

step "OpenNI2 런타임(Astra)"
if [ -z "$OPENNI2" ]; then       # 이 Pi에 이미 설치된 Orbbec 런타임을 찾는다(libOpenNI2.so가 있는 폴더)
  for candidate in /opt/openni2-orbbec /usr/local/lib/openni2-orbbec "$HOME/ros2_ws/src/ros2_astra_camera/astra_camera/openni2_redist/arm64"; do
    if [ -f "$candidate/libOpenNI2.so" ]; then OPENNI2="$candidate"; echo "  찾았습니다: $OPENNI2"; break; fi
  done
fi
if [ -n "$OPENNI2" ]; then
  if [ -f "$OPENNI2/libOpenNI2.so" ]; then
    LIB_ARCH="$(file -b "$OPENNI2/libOpenNI2.so" | grep -oE 'aarch64|ARM aarch64|x86-64' | head -1 || true)"
    echo "  $OPENNI2/libOpenNI2.so ($LIB_ARCH)"
    case "$LIB_ARCH:$ARCH" in
      *x86-64*:aarch64) echo "  문제: x86-64용 런타임입니다. Pi에는 ARM64(aarch64) 빌드가 필요합니다."; exit 1 ;;
    esac
  else
    echo "  주의: $OPENNI2/libOpenNI2.so 가 없습니다. 경로를 확인하세요(설정 파일에는 그대로 적습니다)."
  fi
else
  echo "  찾지 못했습니다. --openni2 <폴더>로 알려 주거나 나중에 $ENV_FILE의 ARM_OPENNI2_RUNTIME에 적으세요."
fi

step "설정 파일 $ENV_FILE (권한 600)"
if [ -f "$ENV_FILE" ] && [ "${DRY_RUN:-0}" != 1 ]; then
  echo "  이미 있어 그대로 둡니다(토큰 유지). 바꾸려면 직접 고치세요: nano $ENV_FILE"
else
  TOKEN="$(python3 -c 'import secrets; print(secrets.token_hex(12))')"
  TMP="$(mktemp)"
  sed -e "s|^ARM_API_TOKEN=.*|ARM_API_TOKEN=$TOKEN|" -e "s|^ARM_OPENNI2_RUNTIME=.*|ARM_OPENNI2_RUNTIME=$OPENNI2|" \
      "$ROOT/pi/arm-api.env.example" > "$TMP"
  if [ "${DRY_RUN:-0}" = 1 ]; then
    echo "  [실행 안 함] $ENV_FILE 만들기(토큰 무작위). 내용:"; sed 's/^/    /' "$TMP"
  elif [ "$SERVICE" = 1 ]; then
    sudo install -m 600 -o root -g root "$TMP" "$ENV_FILE"
    echo "  만들었습니다. 접속 토큰: $TOKEN  (웹 화면 [설정]과 팀 NOMI의 ARM_API_TOKEN에 넣으세요)"
  else
    install -m 600 "$TMP" "$ENV_FILE"
    echo "  만들었습니다. 접속 토큰: $TOKEN  (웹 화면 [설정]과 팀 NOMI의 ARM_API_TOKEN에 넣으세요)"
  fi
  rm -f "$TMP"
fi

step "systemd 서비스 arm-api"
if [ "$SERVICE" = 0 ]; then
  echo "  --no-service: 서비스를 만들지 않습니다(부팅 때 자동 시작 없음). 실행은 bash pi/arm_api_ctl.sh start"
else
  TMP="$(mktemp)"
  sed -e "s|@USER@|$USER_NAME|g" -e "s|@ROOT@|$ROOT|g" "$ROOT/pi/arm-api.service.in" > "$TMP"
  if [ "${DRY_RUN:-0}" = 1 ]; then
    echo "  [실행 안 함] $UNIT 설치. 내용:"; sed 's/^/    /' "$TMP"
  else
    sudo install -m 644 "$TMP" "$UNIT"
  fi
  rm -f "$TMP"
  run sudo systemctl daemon-reload
  run sudo systemctl enable arm-api
  if [ "$START" = 1 ]; then run sudo systemctl restart arm-api; fi
fi

step "장치 점검(읽기 전용)"
if [ "${DRY_RUN:-0}" = 1 ] || [ ! -x "$ROOT/.venv/bin/python" ]; then
  echo "  [실행 안 함] $ROOT/.venv/bin/python scripts/check_devices.py"
else
  # 설정 파일(값이 있는 줄만)을 환경 변수로 넘겨 서비스와 같은 조건으로 점검한다. 방금 바꾼 그룹은 다시 로그인해야 반영된다
  mapfile -t SETTINGS < <( { [ -r "$ENV_FILE" ] && cat "$ENV_FILE" || sudo cat "$ENV_FILE"; } | grep -E '^[A-Z0-9_]+=.+' || true)
  env "${SETTINGS[@]}" "$ROOT/.venv/bin/python" "$ROOT/scripts/check_devices.py" || true
fi

cat <<EOF

설치를 마쳤습니다.
  - 그룹(dialout, video)이 바뀌었으면 한 번 로그아웃·로그인(또는 재부팅)해야 적용됩니다.
  - 실행:   $( [ "$SERVICE" = 1 ] && echo "sudo systemctl start arm-api / status arm-api / journalctl -u arm-api -f" || echo "bash $ROOT/pi/arm_api_ctl.sh start | stop | status | log | check" )
  - 설정:   nano $ENV_FILE  (OLLAMA_HOST, ARM_STT_URL에 노트북 주소)  → 서버 다시 시작
  - 접속:   http://$(hostname -I | awk '{print $1}'):8000  또는 http://$(hostname).local:8000  (마이크는 브라우저 보안상 https나 localhost에서만 열린다)
EOF
