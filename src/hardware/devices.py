"""장치 경로를 한곳에서 정한다. 하드웨어를 열지 않는다(경로 문자열만 돌려준다).

우선순위: 환경 변수 → config/devices.json(있으면) → 기본값(개발 노트북에서 쓰던 값).
Raspberry Pi 등 다른 컴퓨터로 옮기면 by-id 이름이나 OpenNI2 런타임 폴더가 달라질 수 있어 코드 수정 없이 바꿀 수 있게 했다.

  ARM_SERVO_PORT       서보 버스 어댑터  /dev/serial/by-id/...
  ARM_PUMP_PORT        ESP32 펌프 제어기 /dev/serial/by-id/...
  ARM_WRIST_CAMERA     손목 카메라       /dev/v4l/by-id/...-video-index0
  ARM_OPENNI2_RUNTIME  Astra용 OpenNI2 런타임 폴더(libOpenNI2.so가 있는 곳). Pi는 ARM64 빌드가 필요하다

기본값과 같은 장치(같은 USB 일련번호)면 by-id 이름은 어느 리눅스에서나 같아서 설정이 필요 없다.
직접 지정한 경로는 일련번호 이름 검사를 하지 않는다(다른 어댑터로 바꿨을 수 있다). 고정 이름인지는 계속 검사한다:
기본은 /dev/serial/by-id, 직접 지정하면 /dev/serial/by-path(꽂은 USB 포트 위치 이름)도 된다.
by-path는 같은 일련번호의 장치가 둘일 때 쓴다. 예: 팀 라이다와 펌프 ESP32가 둘 다 CP2102 '_0001'이라
by-id 이름이 하나만 생긴다(scripts/find_pump_port.py로 펌프의 by-path를 찾는다). 다른 포트로 옮겨 꽂으면 다시 정해야 한다.
"""
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'config' / 'devices.json'

DEFAULTS = {
    'servo_port': None,                        # None이면 config/servo_driver_candidate.json의 port
    'pump_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
    'wrist_camera': '/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0',
    'openni2_runtime': str(ROOT / 'calibration' / 'openni2_redist'),
}
ENV = {'servo_port': 'ARM_SERVO_PORT', 'pump_port': 'ARM_PUMP_PORT',
       'wrist_camera': 'ARM_WRIST_CAMERA', 'openni2_runtime': 'ARM_OPENNI2_RUNTIME'}


SERIAL_STABLE_DIRS = (Path('/dev/serial/by-id'), Path('/dev/serial/by-path'))


def stable_serial(path, configured):
    """재부팅해도 같은 장치를 가리키는 이름인가. by-id는 늘, by-path는 직접 지정했을 때만."""
    path = Path(path)
    allowed = SERIAL_STABLE_DIRS if configured else SERIAL_STABLE_DIRS[:1]
    return path.parent in allowed and path.is_symlink()


def _file_values():
    try:
        data = json.loads(CONFIG.read_text())
    except (OSError, ValueError):
        return {}
    return {k: v for k, v in data.items() if k in DEFAULTS and isinstance(v, str) and v.strip()}


def lookup(name):
    """(경로 문자열 또는 None, 출처 'env'|'file'|'default')."""
    if name not in DEFAULTS:
        raise KeyError(name)
    value = os.environ.get(ENV[name], '').strip()
    if value:
        return value, 'env'
    value = _file_values().get(name)
    if value:
        return value.strip(), 'file'
    return DEFAULTS[name], 'default'


def get(name, fallback=None):
    value, _ = lookup(name)
    return value if value is not None else fallback


def configured(name):
    """기본값이 아니라 사람이 직접 지정했는가(그러면 일련번호 이름 검사를 하지 않는다)."""
    return lookup(name)[1] != 'default'


def summary(servo_profile_port=None):
    """설치·점검용: 장치별 (경로, 출처, 존재 여부)."""
    rows = {}
    for name in DEFAULTS:
        value, source = lookup(name)
        if name == 'servo_port' and value is None:
            value, source = servo_profile_port, 'servo_driver_candidate.json'
        rows[name] = dict(path=value, source=source, exists=bool(value) and Path(value).exists())
    return rows
