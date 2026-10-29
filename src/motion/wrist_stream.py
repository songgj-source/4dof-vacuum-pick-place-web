"""손목 카메라를 열어 둔 채 최신 사진을 계속 받아 둔다. 하드웨어는 카메라 열기뿐(팔·펌프 접근 없음).

집힘 확인용: 흡착하는 동안(0.8초) 카메라를 미리 열어 두면, 집기 직전 사진과 들어올린 직후 사진을 카메라를 다시 열지 않고(사진마다 약 1초) 받을 수 있다.
그래서 확인 때문에 동작이 느려지지 않는다. 한 카메라는 한 곳에서만 열 수 있어서, 이 스트림이 켜져 있는 동안 다른 곳은 열지 않는다.
"""
import threading
import time

import numpy as np

WARMUP_FRAMES = 5
FRAME_SHAPE = (480, 640, 3)


def open_wrist_camera(device):
    import cv2
    cam = cv2.VideoCapture(str(device), cv2.CAP_V4L2)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cam


class WristStream:
    def __init__(self, device, opener=open_wrist_camera, clock=time.monotonic):
        self.device, self.opener, self.clock = device, opener, clock
        self.frame, self.stamp, self.error = None, None, None
        self.stopping = threading.Event()
        self.thread = None
        self.lock = threading.Lock()

    def start(self):
        self.thread = threading.Thread(target=self._run, name='wrist-stream', daemon=True)
        self.thread.start()
        return self

    def _run(self):
        cam = None
        try:
            cam = self.opener(self.device)
            seen = 0
            while not self.stopping.is_set():
                ok, frame = cam.read()
                if not ok or not isinstance(frame, np.ndarray) or frame.shape != FRAME_SHAPE:
                    self.error = 'wrist frame resolution/read failed'
                    continue
                seen += 1
                if seen > WARMUP_FRAMES:                  # 처음 몇 장은 노출이 안 맞는다
                    with self.lock:
                        self.frame, self.stamp = frame, self.clock()
        except Exception as exc:
            self.error = f'{type(exc).__name__}: {exc}'
        finally:
            if cam is not None:
                try:
                    cam.release()
                except Exception:
                    pass

    def latest(self, after=None, timeout=1.0):
        """가장 최근 사진. after(시각)를 주면 그 뒤에 찍힌 사진을 기다린다(timeout 안에 없으면 있는 것 중 가장 최근, 없으면 None). 돌려주는 것: (사진, 찍은 시각)."""
        end = self.clock() + timeout
        while True:
            with self.lock:
                frame, stamp = self.frame, self.stamp
            if frame is not None and (after is None or stamp > after):
                return frame.copy(), stamp
            if self.clock() >= end or self.stopping.is_set():
                return (frame.copy(), stamp) if frame is not None else (None, None)
            time.sleep(.005)

    def stop(self):
        self.stopping.set()
        if self.thread is not None:
            self.thread.join(timeout=2.)
