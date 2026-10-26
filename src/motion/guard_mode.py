"""속도 우선 모드(사용자 지시 2026-10-19 "추종·정지 가드만 없애줘"): 추종 오차·정지 안정·도착 허용 오차 가드를 사실상 끈다.

관절 한계, 책상·벽 충돌 높이, 통신 오류·서보 알람, 취소·시간 제한, 펌프 응답은 그대로다.
VACUUM_STRICT_GUARDS=1 이면 예전 엄격 모드(배율도 1).

가드 배율(사용자 지시 2026-10-26 "모든 안전 가드 1.5배"): 소프트웨어 허용 범위(도착·위치 오차, 정지·인계 확인 펄스,
HOME 시작·도착 판정, 한 번 이동량, 접근 보정 한도, 놓기 위치 확인)를 config/guard_scale.json의 scale배로 넓힌다.
판정 기준만 넓히고 경로를 나누는 계획 값은 그대로다(움직임은 같고 멈추는 경우만 줄어든다).
넓히지 않는 것: 관절 한계·제어기 여유(±16펄스), 책상·벽·트레이 충돌 높이와 여유, 집기 가능 거리, 수직 리셋 구간,
시간 제한, 통신·알람·토크·펌프 확인. 환경 변수 VACUUM_GUARD_SCALE이 있으면 그 값이 우선(테스트는 1로 고정).
"""
import json
import math
import os
from pathlib import Path

RELAXED_TRACKING_PULSES = 4096      # 사실상 무제한: 서보가 목표를 못 따라가도 이 가드로는 멈추지 않는다
RELAXED_POSE_TOLERANCE_M = .050     # 접근·하강이 아닌 구간의 도착 허용 오차(접근은 측정 오차 보정이 5mm 검사에 의존해 유지)
SCALE_FILE = Path(__file__).resolve().parents[2] / 'config' / 'guard_scale.json'
SCALE_RANGE = (1.0, 2.0)            # 배율은 넓히기만, 2배까지


def relaxed():
    return os.environ.get('VACUUM_STRICT_GUARDS') != '1'


def scale():
    """가드 배율. 엄격 모드는 1. 환경 변수 > 설정 파일 > 1. 범위 밖이면 거부한다(잘못 쓴 값으로 조용히 넓어지지 않게)."""
    if not relaxed():
        return 1.0
    text = os.environ.get('VACUUM_GUARD_SCALE')
    if text is None:
        try:
            value = json.loads(SCALE_FILE.read_text())['scale']
        except FileNotFoundError:
            return 1.0
    else:
        value = float(text)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not SCALE_RANGE[0] <= value <= SCALE_RANGE[1]:
        raise ValueError(f'guard scale must be {SCALE_RANGE[0]}-{SCALE_RANGE[1]}, got {value!r}')
    return float(value)


def length(value_m):
    """길이(m) 허용치를 배율만큼 넓힌다."""
    return value_m * scale()


def pulses(count):
    """펄스 허용치를 배율만큼 넓힌다(정수로 내림: 2→3, 1→1, 8→12, 1600→2400)."""
    return int(math.floor(count * scale() + 1e-9))
