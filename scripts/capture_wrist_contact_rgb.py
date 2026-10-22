#!/usr/bin/env python3
"""Disposable RGB capture process. Parent kills this on a stalled camera."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import sys

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from hardware import devices
DEVICE=Path(devices.get('wrist_camera'))   # ARM_WRIST_CAMERA / config/devices.json / 기본값


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if not DEVICE.is_symlink():raise OSError('wrist camera absent')
    cam=cv2.VideoCapture(str(DEVICE),cv2.CAP_V4L2)
    try:
        if not cam.isOpened():raise OSError('wrist camera cannot open')
        cam.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        cam.set(cv2.CAP_PROP_FRAME_WIDTH,640);cam.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        for _ in range(10):cam.read()
        args.output.mkdir(parents=True,exist_ok=False);records=[]
        for i in range(5):
            ok,frame=cam.read();stamp=time.monotonic_ns()
            if not ok or frame is None or frame.shape!=(480,640,3):raise OSError('wrist RGB read failed')
            file=args.output/f'{i:04d}.png'
            if not cv2.imwrite(str(file),frame):raise OSError('RGB save failed')
            records.append(dict(image=file.name,host_capture_ns=stamp,sha256=hashlib.sha256(file.read_bytes()).hexdigest()))
        (args.output/'capture.json').write_text(json.dumps(dict(device=str(DEVICE),frames=records),indent=2))
    finally:cam.release()


if __name__=='__main__':main()
