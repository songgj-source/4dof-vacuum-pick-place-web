#!/usr/bin/env python3
"""설치 점검(읽기 전용): 장치 경로, Astra용 OpenNI2 런타임의 CPU 종류, 파이썬 패키지, 권한, 원격 LLM·음성 서버 연결.

로봇·펌프·카메라를 열지 않는다(파일과 장치 이름만 본다). Raspberry Pi 설치 뒤, 또는 장치를 바꾼 뒤 실행한다.
  python scripts/check_devices.py
"""
import grp
import importlib
import json
import os
import platform
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from hardware import devices      # noqa: E402

ELF_MACHINE = {0x3E: 'x86_64', 0xB7: 'aarch64', 0x28: 'arm'}
OK, WARN, BAD = '  OK  ', ' 주의 ', ' 문제 '


def elf_machine(path):
    try:
        head = Path(path).read_bytes()[:20]
    except OSError:
        return None
    if head[:4] != b'\x7fELF':
        return None
    return ELF_MACHINE.get(int.from_bytes(head[18:20], 'little'), hex(int.from_bytes(head[18:20], 'little')))


def reachable(url):
    try:
        with urllib.request.urlopen(url, timeout=3):
            return True
    except Exception:
        return False


def main():
    problems = 0
    machine = platform.machine()
    print(f'컴퓨터: {machine}, Python {platform.python_version()}')

    print('\n[장치 경로]  (환경 변수 → config/devices.json → 기본값)')
    profile_port = json.loads((ROOT / 'config/servo_driver_candidate.json').read_text()).get('port')
    for name, row in devices.summary(profile_port).items():
        mark = OK if row['exists'] else BAD
        problems += not row['exists']
        print(f"[{mark}] {name:<16} {row['path']}  ({row['source']})")
    for folder in ('/dev/serial/by-id', '/dev/v4l/by-id'):
        names = sorted(p.name for p in Path(folder).glob('*')) if Path(folder).exists() else []
        print(f'        {folder}: ' + (', '.join(names) if names else '(없음)'))

    print('\n[Astra OpenNI2 런타임]')
    runtime = Path(devices.get('openni2_runtime'))
    lib = runtime / 'libOpenNI2.so'
    arch = elf_machine(lib)
    if arch is None:
        problems += 1
        print(f'[{BAD}] {lib} 가 없습니다. Orbbec OpenNI2 SDK(이 컴퓨터용)의 Redist 폴더를 ARM_OPENNI2_RUNTIME으로 지정하세요.')
    elif arch != machine:
        problems += 1
        print(f'[{BAD}] {lib} 는 {arch}용인데 이 컴퓨터는 {machine}입니다. {machine}용 OpenNI2가 필요합니다.')
    else:
        print(f'[{OK}] {lib} ({arch})')

    print('\n[파이썬 패키지]')
    for module, package in (('numpy', 'numpy'), ('cv2', 'opencv-python-headless'), ('serial', 'pyserial'),
                            ('scservo_sdk', 'feetech-servo-sdk'), ('primesense', 'primesense')):
        try:
            importlib.import_module(module)
            print(f'[{OK}] {module}')
        except Exception as exc:
            problems += 1
            print(f'[{BAD}] {module} 없음 → pip install {package}  ({type(exc).__name__})')
    try:
        importlib.import_module('faster_whisper')
        print(f'[{OK}] faster_whisper (이 컴퓨터에서 음성 인식 가능)')
    except Exception:
        print(f'[{WARN}] faster_whisper 없음: ARM_STT_URL로 노트북 음성 인식을 쓰면 필요 없습니다.')

    print('\n[권한]')
    groups = {grp.getgrgid(g).gr_name for g in os.getgroups()}
    for group in ('dialout', 'video'):
        if group in groups:
            print(f'[{OK}] {group} 그룹')
        else:
            problems += 1
            print(f'[{BAD}] {group} 그룹이 아닙니다 → sudo usermod -aG {group} $USER 후 다시 로그인')

    print('\n[원격 서버]')
    ollama = os.environ.get('OLLAMA_HOST', '').strip()
    if ollama:
        url = (ollama if '://' in ollama else 'http://' + ollama).rstrip('/') + '/api/tags'
        print(f"[{OK if reachable(url) else WARN}] LLM(Ollama) {url}")
    else:
        print(f'[{WARN}] OLLAMA_HOST 미설정: 대화는 이 컴퓨터의 Ollama(127.0.0.1:11434)를 찾습니다.')
    stt = os.environ.get('ARM_STT_URL', '').strip()
    if stt:
        url = stt.rsplit('/api/stt', 1)[0] + '/health'
        print(f"[{OK if reachable(url) else WARN}] 음성 인식 {url}")
    else:
        print(f'[{WARN}] ARM_STT_URL 미설정: 음성 인식은 이 컴퓨터의 Whisper를 씁니다.')

    print('\n' + ('점검 통과' if not problems else f'문제 {problems}개: 위 [ 문제 ] 항목을 고친 뒤 다시 실행하세요.'))
    return 0 if not problems else 1


if __name__ == '__main__':
    raise SystemExit(main())
