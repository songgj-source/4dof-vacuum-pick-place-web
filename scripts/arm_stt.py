"""로컬 음성 인식(faster-whisper). 웹 화면이 녹음한 소리를 글자로 바꾼다. 인터넷·외부 서버를 쓰지 않는다.

- 모델: ARM_STT_MODEL(기본 small), 장치 ARM_STT_DEVICE(기본 cpu, int8). CPU에서 짧은 명령 한 문장에 약 1초.
  GPU는 로컬 LLM(Ollama)과 메모리를 나눠 써야 해서 기본은 CPU다.
- 처음 쓸 때 한 번만 불러온다(약 1초, 모델 파일은 처음 한 번 내려받는다: small 약 480MB).
- 로봇 명령 단어를 힌트로 준다(힌트가 없으면 짧은 "실행"을 "시랭"으로, "취소"를 "치솔"로 알아들었다. 2026-10-21 시험).
- 말소리가 없는 구간은 음성 구간 검출(VAD)로 버린다. Whisper는 무음에 "감사합니다" 같은 문장을 지어내는 경향이 있다.

필요: pip install faster-whisper (lerobot_312에 새 패키지 5개만 추가됨, 기존 패키지 변경 없음).

원격 인식(Raspberry Pi): ARM_STT_URL을 주면 이 컴퓨터에서 Whisper를 돌리지 않고 녹음을 그 주소로 넘긴다.
  노트북: python scripts/stt_server.py --host 0.0.0.0 --token 비밀값        (Whisper만 도는 작은 서버)
  Pi:     ARM_STT_URL=http://노트북주소:8010/api/stt  ARM_STT_TOKEN=비밀값
브라우저는 계속 Pi의 /api/stt 하나만 부른다(Pi가 대신 넘긴다). Pi에는 faster-whisper가 필요 없다.
"""
import os
import re
import json
import tempfile
import threading
import time
import urllib.error
import urllib.request

MODEL = os.environ.get('ARM_STT_MODEL', 'small')
DEVICE = os.environ.get('ARM_STT_DEVICE', 'cpu')
COMPUTE = os.environ.get('ARM_STT_COMPUTE', 'int8' if DEVICE == 'cpu' else 'float16')
REMOTE_URL = os.environ.get('ARM_STT_URL', '').strip()      # 비어 있으면 이 컴퓨터에서 인식
REMOTE_TOKEN = os.environ.get('ARM_STT_TOKEN', '').strip()
REMOTE_TIMEOUT_S = float(os.environ.get('ARM_STT_TIMEOUT', '30'))
MAX_AUDIO_BYTES = 2 * 1024 * 1024          # 약 1분 분량(webm/opus). 명령 한 마디는 수십 KB
HINT = ('로봇팔 명령: 실행, 취소, 멈춰, 정지, 네, 흰 상자, 빨간 큐브, 빨간 정육면체, 줄자, 새 물체, '
        '1번 칸, 2번 칸, 3번 칸, 넣어줘, 다 정리해줘, 책상 정리해줘, 홈으로 돌아가, 작업대에 뭐 있어.')
# 무음·잡음에 Whisper가 지어내는 흔한 문장(유튜브 자막 학습 흔적). 이것만 나오면 못 알아들은 것으로 본다
HALLUCINATIONS = ('감사합니다', '시청해주셔서 감사합니다', '시청해 주셔서 감사합니다', '구독과 좋아요', '좋아요와 구독', 'mbc 뉴스', '다음 영상에서 만나요')


class SttUnavailable(Exception):
    """faster-whisper가 설치되어 있지 않거나 모델을 불러오지 못했다."""


_model = None
_lock = threading.Lock()


def describe():
    """서버 시작 안내용 한 줄."""
    return f'원격 Whisper({REMOTE_URL})' if REMOTE_URL else f'Whisper {MODEL} ({DEVICE})'


def load():
    """로컬이면 모델을 불러 두고, 원격이면 그 서버가 살아 있는지만 확인한다."""
    if REMOTE_URL:
        health = REMOTE_URL.rsplit('/api/stt', 1)[0] + '/health'
        try:
            with urllib.request.urlopen(health, timeout=3) as response:
                json.loads(response.read() or b'{}')
        except (urllib.error.URLError, OSError, ValueError) as exc:
            raise SttUnavailable(f'원격 음성 인식 서버({REMOTE_URL})에 연결하지 못했습니다: {exc}') from exc
        return None
    return load_local()


def load_local():
    global _model
    with _lock:
        if _model is None:
            try:
                from faster_whisper import WhisperModel
            except ImportError as exc:
                raise SttUnavailable('로컬 음성 인식을 쓰려면 서버에 pip install faster-whisper가 필요합니다.') from exc
            try:
                try:        # 받아 둔 모델이 있으면 인터넷을 전혀 확인하지 않는다(오프라인에서 연결을 기다리지 않게)
                    _model = WhisperModel(MODEL, device=DEVICE, compute_type=COMPUTE, local_files_only=True)
                except Exception:
                    _model = WhisperModel(MODEL, device=DEVICE, compute_type=COMPUTE)     # 처음 한 번: 내려받는다
            except Exception as exc:
                raise SttUnavailable(f'Whisper 모델({MODEL})을 불러오지 못했습니다: {type(exc).__name__}: {exc}') from exc
        return _model


def clean(text):
    """앞뒤 공백·문장부호를 정리하고, 지어낸 문장이면 빈 문자열."""
    text = re.sub(r'\s+', ' ', text).strip()
    text = text.strip(' .,!?~…')
    text = re.sub(r'\s*,\s*', ' ', text)            # "새 물체, 2번 칸에 넣어" → "새 물체 2번 칸에 넣어"
    if text.lower().rstrip('.!') in HALLUCINATIONS:
        return ''
    return text


CONTENT_TYPES = {'.webm': 'audio/webm', '.ogg': 'audio/ogg', '.wav': 'audio/wav', '.mp4': 'audio/mp4', '.mp3': 'audio/mpeg'}


def transcribe(audio, suffix='.webm'):
    """audio(bytes) → dict(text, seconds). 말소리가 없으면 text는 빈 문자열. ARM_STT_URL이 있으면 원격으로 넘긴다."""
    if not audio:
        return dict(text='', seconds=0.)
    return transcribe_remote(audio, suffix) if REMOTE_URL else transcribe_local(audio, suffix)


def transcribe_remote(audio, suffix='.webm'):
    started = time.monotonic()
    headers = {'Content-Type': CONTENT_TYPES.get(suffix, 'audio/webm')}
    if REMOTE_TOKEN:
        headers['Authorization'] = f'Bearer {REMOTE_TOKEN}'
    request = urllib.request.Request(REMOTE_URL, data=audio, method='POST', headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=REMOTE_TIMEOUT_S) as response:
            body = json.loads(response.read() or b'{}')
    except urllib.error.HTTPError as error:
        try:
            reason = json.loads(error.read() or b'{}').get('reason')
        except ValueError:
            reason = None
        if error.code in (401, 503):
            raise SttUnavailable(f'원격 음성 인식 서버 오류 {error.code}: {reason or "토큰(ARM_STT_TOKEN)이나 서버 상태를 확인하세요"}') from error
        raise RuntimeError(f'원격 음성 인식 서버 오류 {error.code}: {reason or ""}') from error
    except (urllib.error.URLError, OSError, ValueError) as exc:
        raise SttUnavailable(f'원격 음성 인식 서버({REMOTE_URL})에 연결하지 못했습니다: {exc}') from exc
    return dict(text=clean(str(body.get('text') or '')), seconds=round(time.monotonic() - started, 2), remote=True,
                remote_seconds=body.get('seconds'))


def transcribe_local(audio, suffix='.webm'):
    if not audio:
        return dict(text='', seconds=0.)
    model = load_local()
    started = time.monotonic()
    with tempfile.NamedTemporaryFile(suffix=suffix) as f:      # 컨테이너(webm/ogg/wav/mp3)는 av(ffmpeg)가 읽는다
        f.write(audio)
        f.flush()
        segments, _info = model.transcribe(f.name, language='ko', beam_size=1, initial_prompt=HINT,
                                           condition_on_previous_text=False, vad_filter=True,
                                           vad_parameters=dict(min_silence_duration_ms=500))
        text = ''.join(s.text for s in segments if s.no_speech_prob < .6)
    return dict(text=clean(text), seconds=round(time.monotonic() - started, 2))
