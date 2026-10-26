#!/usr/bin/env python3
"""물체 등록·삭제·목록. 위 카메라로 사진 한 장만 찍는다(로봇·펌프는 열지 않는다).

  python scripts/register_object.py --name 머그컵 --slot 2              # 책상(트레이 밖)에 그 물체 하나만 둔다
  python scripts/register_object.py --name 머그컵 --slot 2 --height-mm 38   # 자로 잰 높이(없으면 사진으로 잰 값)
  python scripts/register_object.py --list
  python scripts/register_object.py --remove 머그컵                     # 이름 또는 custom_N

등록하면 바로 집기·책상 정리하기 대상이 된다(흡착은 확인하지 않는다). 결과는 마지막 줄 JSON.
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from perception import object_registry


def emit(**data):
    print(json.dumps(data, ensure_ascii=False), flush=True)


def live_frame(frames=10):
    """Astra RGB+깊이 한 장(첫 몇 장은 노출이 자리 잡기 전이라 버린다)."""
    from camera.astra_calibration_camera import AstraCalibrationCamera
    camera = AstraCalibrationCamera(fixed_exposure=False)
    try:
        for _ in range(frames):
            bgr, depth, _stamp = camera.read()
    finally:
        camera.close()
    return bgr, depth


def live_tray_slots(bgr, depth):
    """지금 사진에서 트레이 칸(칸 안 물체를 후보에서 빼려고). 못 찾으면 None."""
    try:
        from tray.tray_detect import detect_tray
        tray = detect_tray(bgr, depth, ROOT, first_end='far_from_base')
        return tray['slots'] if tray.get('found') else None
    except Exception:
        return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--name', help='새로 등록할 물체 이름(한글 가능, 20자 이하)')
    action.add_argument('--remove', metavar='NAME', help='등록 물체 삭제(이름 또는 custom_N)')
    action.add_argument('--list', action='store_true', help='등록 물체 목록')
    parser.add_argument('--slot', type=int, choices=[1, 2, 3], help='넣을 기본 칸(등록할 때 필요)')
    parser.add_argument('--height-mm', type=float, help='자로 잰 높이(mm). 없으면 깊이로 잰 윗면 높이(약 ±3mm)')
    parser.add_argument('--from-files', nargs=2, metavar=('RGB', 'DEPTH'), help='카메라 대신 저장된 프레임(시험용)')
    args = parser.parse_args(argv)
    try:
        if args.list:
            entries = object_registry.load(ROOT)
            emit(ok=True, objects=entries, lines=[object_registry.describe(e) for e in entries])
            return 0
        if args.remove:
            gone = object_registry.remove(args.remove, ROOT)
            emit(ok=True, removed=gone, message=f"'{gone['name']}' 등록을 지웠습니다.")
            return 0
        if args.slot is None:
            parser.error('--name에는 --slot(1~3)이 필요합니다.')
        object_registry.check_name(args.name, ROOT)          # 사진을 찍기 전에 이름부터 확인한다
        if args.from_files:
            import cv2
            import numpy as np
            bgr, depth = cv2.imread(args.from_files[0]), np.load(args.from_files[1])
        else:
            bgr, depth = live_frame()
        entry = object_registry.register(bgr, depth, args.name, args.slot, args.height_mm, ROOT, live_tray_slots(bgr, depth))
    except (ValueError, OSError, RuntimeError) as exc:
        emit(ok=False, reason=str(exc))
        return 1
    emit(ok=True, object=entry, message='등록했습니다: ' + object_registry.describe(entry))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
