#!/usr/bin/env bash
# 노트북에서 자비스 웹 화면(Pi)을 터널 없이 바로 연다. 마이크도 열린다.
# 브라우저는 https나 localhost에서만 마이크를 주므로, Pi 주소만 안전한 주소로 취급하는 전용 Chrome 창을 띄운다
# (평소 쓰는 Chrome 프로필·설정은 건드리지 않는다. 전용 프로필: ~/.config/jarvis-chrome).
#   bash pi/open_jarvis.sh                 # 기본 http://192.168.0.31:8000
#   ARM_URL=http://주소:8000 bash pi/open_jarvis.sh
#   bash pi/open_jarvis.sh http://192.168.0.31:8082   # 덧붙인 주소는 같은 창의 탭으로 연다(주행 화면 등)
set -euo pipefail
URL="${ARM_URL:-http://192.168.0.31:8000}"
CHROME="$(command -v google-chrome || command -v google-chrome-stable || command -v chromium || command -v chromium-browser || true)"
[ -n "$CHROME" ] || { echo "Chrome(또는 Chromium)이 없습니다."; exit 1; }
nohup "$CHROME" --user-data-dir="$HOME/.config/jarvis-chrome" --no-first-run --no-default-browser-check \
  --unsafely-treat-insecure-origin-as-secure="$URL" --new-window "$URL" "$@" >/dev/null 2>&1 &
echo "열었습니다: $URL $*  (전용 Chrome 창, 마이크 허용 가능)"
