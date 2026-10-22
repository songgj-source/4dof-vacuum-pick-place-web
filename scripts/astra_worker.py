"""위 카메라(Astra)를 열어 둔 채 사진을 바로 주는 작은 프로세스. arm_api.py가 띄우고 표준 입출력으로 부른다.

사진마다 카메라 프로그램을 새로 띄우면 파이썬 시작·카메라 열기(약 1.3초)·안정화·닫기(약 0.8초)로 수 초가 걸린다(Pi에서 상황 보기 약 3.5초).
카메라를 서버 프로세스 안에 두지 않고 따로 두는 것은 OpenNI가 죽어도 서버가 같이 죽지 않게 하려는 것이다.
로봇 작업(pick_auto 등)도 같은 카메라를 쓰므로, 서버는 작업을 시작하기 전에 이 프로세스를 닫는다.

  표준 입력 한 줄:  grab <저장 경로(.npz)>  → 한 줄 'ok' 또는 'error <이유>'
                    quit                  → 카메라를 닫고 끝낸다
  아무 명령 없이 IDLE_S가 지나면 스스로 닫는다(카메라와 USB 대역을 놓아 준다).
"""
import select
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from camera.astra_calibration_camera import AstraCalibrationCamera  # noqa: E402

IDLE_S = 120.
FLUSH_FRAMES = 2          # 쌓여 있던 오래된 프레임을 버리고 지금 장면을 준다


def main():
    camera = AstraCalibrationCamera(fixed_exposure=False)        # calibrate_robot_camera.py capture와 같은 설정
    try:
        print('ready', flush=True)
        while True:
            if not select.select([sys.stdin], [], [], IDLE_S)[0]:
                break
            line = sys.stdin.readline()
            if not line or line.strip() == 'quit':
                break
            command, _, path = line.strip().partition(' ')
            if command != 'grab' or not path:
                print('error bad command', flush=True)
                continue
            try:
                for _ in range(FLUSH_FRAMES):
                    camera.read()
                bgr, depth, _ = camera.read_synchronized()
                np.savez(path, bgr=bgr, depth=depth)
                print('ok', flush=True)
            except Exception as exc:
                print(f'error {type(exc).__name__}: {str(exc)[:160]}', flush=True)
    finally:
        camera.close()


if __name__ == '__main__':
    main()
