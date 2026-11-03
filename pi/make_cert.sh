#!/usr/bin/env bash
# 폰 브라우저에서 마이크를 쓰려면 https가 필요하다. 자체 서명 인증서를 pi/tls/에 만든다(Pi에서 실행).
#   bash pi/make_cert.sh 192.168.0.115      # Pi 주소를 인증서에 넣는다(주소가 바뀌면 다시 실행)
# 아이폰은 serverAuth·SAN이 없는 인증서를 '서버의 신원을 확인할 수 없음'으로 거부한다(그래서 위 확장을 넣었다). 폰에 http://<Pi 주소>:8000/cert.pem 을 받아 설치·신뢰하면 경고가 사라진다.
# 만든 뒤 pi/arm-api.env에 아래 두 줄을 넣고 서버를 다시 켠다. 폰에서는 https://<Pi 주소>:8443 으로 열고 '안전하지 않음' 경고를 한 번 허용한다.
#   ARM_TLS_CERT=<이 폴더>/pi/tls/cert.pem
#   ARM_TLS_KEY=<이 폴더>/pi/tls/key.pem
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IP="${1:-$(hostname -I | awk '{print $1}')}"
DIR="$ROOT/pi/tls"
mkdir -p "$DIR"
openssl req -x509 -newkey rsa:2048 -nodes -days 825 -subj "/CN=$IP" -addext "subjectAltName=IP:$IP,DNS:$(hostname)" \
  -addext "basicConstraints=critical,CA:TRUE" -addext "keyUsage=digitalSignature,keyEncipherment,keyCertSign" -addext "extendedKeyUsage=serverAuth" \
  -keyout "$DIR/key.pem" -out "$DIR/cert.pem" 2>/dev/null
chmod 600 "$DIR/key.pem"
echo "만들었습니다: $DIR/cert.pem, key.pem (주소 $IP)"
echo "pi/arm-api.env에 추가:"
echo "  ARM_TLS_CERT=$DIR/cert.pem"
echo "  ARM_TLS_KEY=$DIR/key.pem"
