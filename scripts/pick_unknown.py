#!/usr/bin/env python3
"""등록 안 된 물체 하나를 unknown으로 자동 선택해 집는다. 기본은 계획만(--execute --supported로 실행).

기본(바로 집기, 2026-10-26 사용자): 지금 책상(트레이 밖)에 있는 등록 안 된 물체 하나를 사진으로 바로 찾는다.
  그 물체 자리를 책상 높이로 지운 가상 기준(unknown_objects.desk_baseline)과 비교해 '새 물체'로 보고, 기존 검사를 그대로 쓴다.
--wait-new: 예전 방식. 기준 장면을 찍은 뒤 사람이 새 물체를 놓을 때까지 기다린다(--timeout초).
"""
import argparse
import hashlib
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from motion.graceful_child import run_until_clean_exit
from perception.unknown_objects import locate_unknowns, StableUnknown, desk_baseline


def emit(**data):
    print(json.dumps(data, ensure_ascii=False), flush=True)


def median_observation(read, samples=7):
    """짧은 동기 프레임 묶음의 깊이 중간값. 실제 요철 허용치는 바꾸지 않는다."""
    import numpy as np
    depths = []
    first = previous = None
    for _ in range(samples):
        bgr, depth, stamp = read()
        if abs(stamp['color_us'] - stamp['depth_us']) > 30000:
            raise ValueError('카메라 RGB·깊이 동기화에 실패했습니다.')
        if previous is not None and (stamp['color_us'] <= previous['color_us'] or stamp['depth_us'] <= previous['depth_us']):
            raise ValueError('카메라의 새 프레임을 받지 못했습니다.')
        first = stamp if first is None else first
        previous = stamp
        depths.append(depth.copy())
    if stamp['color_us'] - first['color_us'] > 350000:
        raise ValueError('카메라 프레임이 지연됐습니다. 다시 실행하세요.')
    return bgr, np.median(np.stack(depths), axis=0).astype(np.uint16), dict(stamp, depth_temporal_samples=samples)


def wait_for_new_object(read, baseline, timeout, *, detector=locate_unknowns, now=time.monotonic, notify=emit):
    """대상 하나가 연속 세 관측에서 안정적인 경우에만 프레임을 반환한다."""
    stable = StableUnknown(samples=3)
    deadline = now() + timeout
    last_stamp, last_observed, last_sensor_us = None, None, None
    last_issue = '새 물체 하나가 작업 영역에 보이도록 놓으세요.'
    while now() < deadline:
        bgr, depth, stamp = read()
        key = (stamp['color_frame_index'], stamp['depth_frame_index'])
        if key == last_stamp:
            continue
        last_stamp = key
        if abs(stamp['color_us'] - stamp['depth_us']) > 30000:
            stable.update([])
            continue
        # 연속 프레임만 재사용하지 않고 최소 0.5초 간격으로 장면을 확인한다.
        if last_observed is not None and now() - last_observed < .5:
            continue
        if last_sensor_us is not None and stamp['color_us'] - last_sensor_us < 500000:
            continue
        last_observed = now()
        last_sensor_us = stamp['color_us']
        targets = detector(bgr, depth, baseline, ROOT)
        new = [t for t in targets if t.get('new_object')]
        if len(new) > 1:
            last_issue = '새 물체는 한 번에 하나씩 놓으세요.'
        elif len(new) == 1 and not new[0]['pickable']:
            last_issue = '; '.join(new[0]['problems'])
        if stable.update(targets):
            target = next(t for t in targets if t['new_object'])
            notify(phase='unknown_detected', label='unknown', height_mm=round(target['measured_height_mm'], 1))
            return bgr, depth, target
    raise TimeoutError('unknown 대기 시간 초과: ' + last_issue)


def live_tray_slots(bgr, depth):
    """지금 사진의 트레이 칸(칸 안 덩어리를 후보에서 빼려고). 못 찾으면 None(그때는 크기·거리 조건만으로 고른다)."""
    try:
        from tray.tray_detect import detect_tray
        tray = detect_tray(bgr, depth, ROOT, first_end='far_from_base')
        return tray['slots'] if tray.get('found') else None
    except Exception:
        return None


def verify_before_motion(job, root, manual_target_xy=None):
    """저장한 후보를 그대로 실행하지 않고, 모터를 열기 전에 새 관측으로 재확인."""
    import numpy as np
    from camera.astra_calibration_camera import AstraCalibrationCamera
    from tray.tray_capacity import fit_unknown_in_slot
    observation = job.get('unknown_observation')
    if not isinstance(observation, dict):
        raise ValueError('unknown의 최신 관측 기록이 없습니다. 자동 감지를 다시 실행하세요.')
    try:
        planned = datetime.fromisoformat(observation['planned_at'])
        age = (datetime.now(timezone.utc) - planned).total_seconds()
        baseline_path = Path(observation['baseline'])
        expected_xy = observation['target_xy']
    except (KeyError, ValueError, TypeError) as exc:
        raise ValueError('unknown 관측 기록이 올바르지 않습니다.') from exc
    if (not isinstance(expected_xy, (list, tuple)) or len(expected_xy) != 2
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in expected_xy)
            or manual_target_xy is None or len(manual_target_xy) != 2
            or any(not math.isfinite(v) for v in manual_target_xy)
            or math.dist(expected_xy, manual_target_xy) > .0001):
        raise ValueError('unknown 실행 좌표가 관측 좌표와 다릅니다. 자동 감지를 다시 실행하세요.')
    if not 0 <= age <= 15:
        raise ValueError('unknown 계획이 오래됐습니다. 자동 감지를 다시 실행하세요.')
    if hashlib.sha256(baseline_path.read_bytes()).hexdigest() != observation.get('baseline_sha256'):
        raise ValueError('unknown 기준 영상이 변경됐습니다. 다시 시작하세요.')
    baseline = np.load(baseline_path, allow_pickle=False)
    emit(phase='unknown_verify')
    camera = AstraCalibrationCamera(fixed_exposure=False)
    try:
        _, depth, target = wait_for_new_object(lambda: median_observation(camera.read_synchronized), baseline, 8., notify=lambda **kw: None)
        measured, expected = target['object_geometry'], job['object_geometry']
        # 같은 물체를 다시 재도 외곽(bbox) 가장자리 1~2픽셀 흔들림으로 크기·반경이 3~6mm 달라진다(10-20 실측: 61×90 → 67×94mm).
        # 줄어든 것은 안전하므로 커진 양만 보고, 칸 여유는 아래에서 새 측정값으로 다시 검사한다. 실제로 밀린 물체는 이보다 훨씬 크게 바뀐다.
        moved = float(np.linalg.norm(np.array(target['target_xy']) - expected_xy)) * 1000.
        dz = abs(measured['height_mm'] - expected['height_mm'])
        grown = max(a-b for a, b in zip(measured['footprint_mm'], expected['footprint_mm']))
        radius_grown = measured['footprint_radius_mm'] - expected['footprint_radius_mm']
        if moved > 5. or dz > 3. or grown > 5. or radius_grown > 4.:
            raise ValueError(f'unknown 위치·크기가 바뀌었습니다(위치 {moved:.1f}mm/허용 5, 높이 {dz:.1f}mm/3, '
                             f'외곽 증가 {grown:.1f}mm/5, 반경 증가 {radius_grown:.1f}mm/4). 자동 감지를 다시 실행하세요.')
        slot = dict(x_mm=job['slot']['x']*1000., y_mm=job['slot']['y']*1000.,
                    floor_height_mm=job.get('slot_floor_height_mm'), axis_xy=job.get('slot_axis_xy'),
                    center_mm=job.get('slot_center_mm'))
        fit_unknown_in_slot(depth, root, slot, measured, target['target_xy'], allow_shift=False)
    finally:
        camera.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--slot', type=int, choices=[1, 2, 3], default=3)
    parser.add_argument('--timeout', type=float, default=90., help='--wait-new에서 새 물체를 기다리는 시간(초)')
    parser.add_argument('--wait-new', action='store_true', help='예전 방식: 기준을 찍은 뒤 새 물체를 놓을 때까지 기다린다')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--supported', action='store_true')
    parser.add_argument('--attempts', type=int, choices=[1], default=1)
    args, extra = parser.parse_known_args(argv)
    if args.execute and not args.supported:
        parser.error('--execute requires --supported')
    if not 5 <= args.timeout <= 300:
        parser.error('--timeout must be 5-300 seconds')
    reserved = {'--object', '--command', '--unknown-baseline', '--from-files', '--manual-target-mm', '--place-json', '--output'}
    if any(token.split('=', 1)[0] in reserved for token in extra):
        parser.error('unknown 대상·관측 파일은 자동으로 결정됩니다.')
    # 장치를 여는 import/생성은 main에서만 한다. --help는 하드웨어를 건드리지 않는다.
    from camera.astra_calibration_camera import AstraCalibrationCamera
    import cv2
    import numpy as np
    out = ROOT / 'output/unknown_trials' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%f')
    out.mkdir(parents=True, exist_ok=False)
    try:
        camera = AstraCalibrationCamera(fixed_exposure=False)
        try:
            depths = []
            for _ in range(10):
                baseline_bgr, depth, _stamp = camera.read_synchronized()
                depths.append(depth.copy())
            baseline = np.median(np.stack(depths), axis=0).astype(np.uint16)
            if not cv2.imwrite(str(out / 'baseline_rgb.png'), baseline_bgr):
                raise OSError('기준 영상 저장 실패')
            if args.wait_new:
                wait_s = args.timeout
                emit(phase='unknown_wait')
            else:
                np.save(out / 'desk_depth.npy', baseline)          # 지우기 전 실제 깊이(확인용)
                baseline, desk = desk_baseline(baseline_bgr, baseline, ROOT, tray_slots=live_tray_slots(baseline_bgr, baseline))
                (out / 'desk_objects.json').write_text(json.dumps(desk, ensure_ascii=False, indent=2))
                if not desk:
                    raise ValueError('unknown not found: 책상(트레이 밖, 로봇에서 12~42cm)에 등록 안 된 물체가 없습니다.')
                if len(desk) > 1:
                    raise ValueError(f'책상에 등록 안 된 물체가 {len(desk)}개 있습니다. 집을 물체 하나만 남겨 주세요.')
                wait_s = 15.                                        # 같은 물체가 연속 3번 같게 보이면(약 2초) 바로 진행
                emit(phase='unknown_snapshot', count=len(desk))
            np.save(out / 'baseline_depth.npy', baseline)
            last_frame = []
            def read_observation():
                observation = median_observation(camera.read_synchronized)
                last_frame[:] = observation[:2]
                return observation
            try:
                bgr, depth, target = wait_for_new_object(read_observation, baseline, wait_s)
            except (Exception, KeyboardInterrupt):
                if last_frame:
                    cv2.imwrite(str(out / 'failed_frame_rgb.png'), last_frame[0])
                    np.save(out / 'failed_frame_depth.npy', last_frame[1])
                raise
        finally:
            camera.close()  # 기존 집기 프로세스가 카메라를 사용할 수 있도록 먼저 닫는다.
        if not cv2.imwrite(str(out / 'frame_rgb.png'), bgr):
            raise OSError('unknown 영상 저장 실패')
        np.save(out / 'frame_depth.npy', depth)
        (out / 'measurement.json').write_text(json.dumps(target, ensure_ascii=False, indent=2, allow_nan=False))
        command = [sys.executable, str(ROOT / 'scripts/run_sort.py'), '--object', 'unknown', '--slot', str(args.slot),
                   '--attempts', '1', '--unknown-baseline', str(out / 'baseline_depth.npy'),
                   '--from-files', str(out / 'frame_rgb.png'), str(out / 'frame_depth.npy'), *extra]
        if args.execute:
            command += ['--execute', '--supported']
        return run_until_clean_exit(command, ROOT)       # 정지 신호를 run_sort→pick_auto에 전달하고 정리가 끝날 때까지 기다린다
    except (Exception, KeyboardInterrupt) as exc:
        emit(ok=False, reason=str(exc) or '사용자 요청으로 중단했습니다.', output=str(out))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
