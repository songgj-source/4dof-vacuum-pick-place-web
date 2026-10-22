#!/usr/bin/env python3
"""Whisper만 도는 작은 음성 인식 서버(노트북용). 로봇·카메라는 쓰지 않는다.

Raspberry Pi에서는 Whisper가 느리므로(한 문장 수 초) 노트북에서 이 서버를 띄우고, Pi의 arm_api.py가 녹음을 넘긴다.

  노트북: python scripts/stt_server.py --host 0.0.0.0 --token 비밀값
  Pi:     ARM_STT_URL=http://노트북주소:8010/api/stt  ARM_STT_TOKEN=비밀값  python scripts/arm_api.py ...

  POST /api/stt  (본문: 녹음 파일, Content-Type: audio/webm 등) → {"ok", "text", "seconds"}  — arm_api의 /api/stt와 같은 형식
  GET  /health   → {"ok": true, "model": ...}
"""
import argparse
import hmac
import json
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

os.environ.pop('ARM_STT_URL', None)          # 이 서버가 다시 원격으로 넘기면 안 된다(자기 자신을 부르는 고리)
sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_stt                                # noqa: E402

SUFFIX = {'ogg': '.ogg', 'wav': '.wav', 'mp4': '.mp4', 'mpeg': '.mp3'}


def make_handler(token=None, transcriber=None):
    transcribe = transcriber or arm_stt.transcribe_local
    lock = threading.Lock()                   # CPU Whisper는 한 번에 하나씩(동시에 돌리면 둘 다 느려진다)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send(self, status, body):
            data = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/health':
                return self._send(200, dict(ok=True, model=arm_stt.MODEL, device=arm_stt.DEVICE))
            self._send(404, dict(ok=False, reason_code='not_found'))

        def do_POST(self):
            if self.path != '/api/stt':
                return self._send(404, dict(ok=False, reason_code='not_found'))
            if token:
                given = (self.headers.get('Authorization') or '').removeprefix('Bearer ').strip()
                if not hmac.compare_digest(given.encode(), token.encode()):
                    return self._send(401, dict(ok=False, reason_code='unauthorized'))
            try:
                length = int(self.headers.get('Content-Length') or 0)
            except ValueError:
                length = -1
            if not 0 < length <= arm_stt.MAX_AUDIO_BYTES:
                return self._send(413 if length > 0 else 400, dict(ok=False, reason_code='bad_request', reason='녹음 파일 크기가 맞지 않습니다.'))
            audio = self.rfile.read(length)
            kind = (self.headers.get('Content-Type') or '').lower()
            suffix = next((v for k, v in SUFFIX.items() if k in kind), '.webm')
            try:
                with lock:
                    return self._send(200, dict(ok=True, **transcribe(audio, suffix=suffix)))
            except arm_stt.SttUnavailable as exc:
                return self._send(503, dict(ok=False, reason_code='stt_unavailable', reason=str(exc)))
            except Exception as exc:
                return self._send(400, dict(ok=False, reason_code='stt_error', reason=f'{type(exc).__name__}: {str(exc)[:200]}'))
    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8010)
    parser.add_argument('--token', default=os.environ.get('ARM_STT_TOKEN') or None,
                        help='Pi의 ARM_STT_TOKEN과 같은 값. 다른 컴퓨터에서 접속을 받으면(--host 0.0.0.0) 필요')
    args = parser.parse_args(argv)
    if args.host not in ('127.0.0.1', 'localhost', '::1') and not args.token:
        parser.error('다른 컴퓨터에서 접속을 받으려면 --token이 필요합니다.')
    arm_stt.load_local()                      # 모델을 미리 불러 둔다(첫 요청이 느리지 않게)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(args.token))
    print(f'음성 인식 서버: http://{args.host}:{args.port}/api/stt  (Whisper {arm_stt.MODEL}, {arm_stt.DEVICE}, Ctrl+C로 종료)', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
