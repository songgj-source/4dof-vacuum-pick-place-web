#!/usr/bin/env python3
"""명령(물체, 선택적으로 칸) → place 작업 파일을 만들고 pick_auto.py 실행 명령을 **출력만** 한다.

카메라만 읽는다(로봇·펌프는 열지 않는다). 출력된 명령을 사용자가 확인한 뒤 직접 실행한다.

  python scripts/plan_sort.py --command "빨간 상자 3번 칸에 넣어줘"
  python scripts/plan_sort.py --object white_box --acknowledge-unverified-suction
  python scripts/plan_sort.py --object red_cube --empty-run      # 물체 없이 같은 경로만 시험(흡착·해제 없음)

칸을 지정하지 않으면 기본 규칙(흰 상자 1, 빨간 정육면체 2, 줄자 3). 칸 번호는 트레이에 쓴 숫자를 읽어 정한다.
"""
import argparse
import hashlib
import json
import shlex
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from perception.object_targets import load_catalog, locate_targets
from sorting.sort_job import build_job, parse_command
from tray.tray_detect import detect_tray
from tray.tray_sorting import SLOT_FOR_LABEL


def live_frames():
    """Astra RGB+깊이 한 장과, 숫자 읽기 다수결용 직전 15프레임(깜빡임으로 프레임마다 숫자가 가려진다)."""
    from camera.astra_calibration_camera import AstraCalibrationCamera
    camera = AstraCalibrationCamera(fixed_exposure=False)
    recent = []
    try:
        for _ in range(35):
            bgr, depth, _stamp = camera.read()
            recent.append(bgr.copy())
    finally:
        camera.close()
    return bgr, depth, recent[-16:-1]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--command', help='예: "빨간 상자 3번 칸에 넣어줘"')
    parser.add_argument('--object', choices=sorted([*SLOT_FOR_LABEL, 'unknown']))
    parser.add_argument('--slot', type=int, help='칸 지정(없으면 기본 규칙)')
    parser.add_argument('--empty-run', action='store_true', help='물체 없이 같은 경로만 시험(흡착·해제 없음)')
    parser.add_argument('--acknowledge-unverified-suction', action='store_true')
    parser.add_argument('--accept-uncertain-tray', action='store_true',
                        help='트레이 벽이 깊이에 안 보여 칸 중심이 최대 약 25mm 불확실해도 진행')
    parser.add_argument('--pick-radial-offset-mm', type=float, default=0.,
                        help='집을 위치를 로봇 밑동에서 바깥쪽으로 더 미는 보정(-10..10mm, 기본 5: "집는 위치가 짧다"는 요청 + 실물 접촉 기록 2회에서 추정, 0mm는 약 6mm 짧고 10mm는 약 6.5mm 넘침)')
    parser.add_argument('--tray-numbering', default='read_digits',
                        choices=['read_digits', 'larger_y', 'smaller_y', 'near_base', 'far_from_base'])
    parser.add_argument('--tray-frame', nargs=2, metavar=('RGB', 'DEPTH'),
                        help='칸 안에 물체가 들어 있어 칸 바닥을 못 찾을 때: 저장해 둔 트레이 프레임으로 칸 위치를 구한다. 지금 화면과 트레이 모서리가 같은 자리(3px 이내)일 때만 쓴다')
    parser.add_argument('--tray-fallback-live', action='store_true',
                        help='저장한 빈 트레이 프레임이 안 맞으면(트레이가 크게 움직임) 거부하지 않고 지금 카메라로 트레이를 다시 찾아 새 기준으로 저장한다')
    parser.add_argument('--from-files', nargs=2, metavar=('RGB', 'DEPTH'), help='카메라 대신 저장된 프레임(시험용)')
    parser.add_argument('--unknown-baseline', type=Path, help='unknown 추가 전 기준 깊이 프레임(.npy)')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if bool(args.command) == bool(args.object):
        parser.error('give exactly one of --command or --object')
    try:
        if args.command:
            label, commanded = parse_command(args.command)
            if args.slot is not None and commanded is not None and args.slot != commanded:
                parser.error(f'--slot {args.slot} conflicts with the slot in the command ({commanded})')
            commanded = commanded if commanded is not None else args.slot
        else:
            label, commanded = args.object, args.slot
    except ValueError as exc:
        parser.error(str(exc))
    if label == 'unknown' and args.unknown_baseline is None:
        parser.error('unknown은 scripts/pick_unknown.py 또는 --unknown-baseline이 필요합니다.')
    if args.from_files:
        import cv2
        import numpy as np
        bgr, depth, extra = cv2.imread(args.from_files[0]), np.load(args.from_files[1]), []
        extra = [bgr] * 5
    else:
        bgr, depth, extra = live_frames()
        _LAST_FRAMES[:] = [bgr, depth]
        if not args.tray_frame:
            # 깊이 벽 검출은 프레임마다 흔들린다(칸 안 물체가 있으면 특히). 트레이를 못 찾으면 새 프레임으로 다시 찍는다.
            # 2026-10-20: 같은 장면에서 3번 중 1번만 실패하는 것을 확인(손·그림자 등) → 재시도를 2회에서 5회로(프레임당 약 4초).
            for _ in range(5):
                if detect_tray(bgr, depth, ROOT, first_end=args.tray_numbering, extra_bgr=extra).get('found'):
                    break
                bgr, depth, extra = live_frames()
                _LAST_FRAMES[:] = [bgr, depth]
    tray_source, tray_offset = 'live camera', None
    stored_tray_ok = bool(args.tray_frame)
    fresh_tray = None
    if args.tray_frame:
        import cv2
        import numpy as np
        from tray.tray_frame_check import tray_offset_px, register_tray, move_slots
        stored_bgr, stored_depth = cv2.imread(args.tray_frame[0]), np.load(args.tray_frame[1])
        # 2026-10-21: 같은 장면에서 대조가 한 번은 실패(메뉴 4번, 흰 상자를 넣은 직후)하고 다시 하면 성공했다(7 px, 2.2°, 잔차 1.4 px).
        # 카메라로 찍는 경우에는 실패하면 새로 찍어 최대 3번 대조한다. 저장 파일(--from-files)은 같은 사진이라 다시 하지 않는다.
        tries = 1 if args.from_files else 3
        for attempt in range(tries):
            check = tray_offset_px(stored_bgr, bgr, depth=stored_depth, root=ROOT)     # 모서리는 저장 프레임 깊이에서 구한다(트레이가 어디에 있어도)
            registration = None
            if check['ok']:
                break
            # 조금 밀렸거나 돌았으면 밀린 만큼을 측정해서 저장해 둔 칸 위치를 옮겨 쓴다(모서리 2곳 이상이 정확히 맞아야 한다). 크게 움직였으면 거부한다.
            registration = register_tray(stored_bgr, bgr, depth=stored_depth, root=ROOT)
            if registration['ok'] or attempt + 1 == tries:
                break
            bgr, depth, extra = live_frames()
            _LAST_FRAMES[:] = [bgr, depth]
        if not check['ok']:
            if not registration['ok'] and args.tray_fallback_live:
                # 트레이가 저장한 위치에서 크게 움직였다: 지금 카메라로 트레이를 다시 찍는다(새 프레임으로 최대 5번). 물체 쪽 프레임은 그대로 쓴다.
                stored_tray_ok, registration = False, None
                fresh_tray = live_frames()
                for _ in range(5):
                    if detect_tray(fresh_tray[0], fresh_tray[1], ROOT, first_end=args.tray_numbering, extra_bgr=fresh_tray[2]).get('found'):
                        break
                    fresh_tray = live_frames()
            elif not registration['ok']:
                parser.error(f'the tray moved since the stored frame (corner offsets {[round(v, 1) for v in check["offsets_px"]]} px, scores '
                             f'{[round(v, 2) for v in check["scores"]]}; {registration.get("reason")}); detect it from the live camera instead')
        tray_frame_args = (stored_bgr, stored_depth, [stored_bgr] * 5) if stored_tray_ok else fresh_tray
        if not stored_tray_ok:
            tray_source = 'live camera (트레이가 저장한 위치에서 움직여 다시 찾음)'
        elif registration is None:
            tray_source, tray_offset = f'stored frame {args.tray_frame[0]} (tray unchanged: corner offset {check["max_offset_px"]:.1f} px)', check['max_offset_px']
        else:
            tray_source = (f'stored frame {args.tray_frame[0]} moved by the measured tray shift ({registration["shift_px"]:.0f} px, '
                           f'{registration["rotation_deg"]:.1f} deg, residual {registration["residual_px"]:.1f} px)')
            tray_offset = registration['shift_px']
    else:
        tray_frame_args = (bgr, depth, extra)
    try:
        tray = detect_tray(tray_frame_args[0], tray_frame_args[1], ROOT, first_end=args.tray_numbering, extra_bgr=tray_frame_args[2])
        if args.tray_frame and registration is not None and tray.get('found'):
            tray['slots'] = move_slots(tray['slots'], registration, ROOT)
            tray.setdefault('problems', []).append('slot positions were carried by the measured tray move (%.0f px, %.1f deg)' % (registration['shift_px'], registration['rotation_deg']))
        if label == 'unknown':
            import numpy as np
            from perception.unknown_objects import locate_unknowns
            targets = locate_unknowns(bgr, depth, np.load(args.unknown_baseline, allow_pickle=False), ROOT)
        else:
            targets = locate_targets(bgr, depth, ROOT)
            if tray.get('found'):                       # 이미 칸에 들어 있는 물체는 다시 꺼내지 않는다(없는 것으로 보고 건너뛰게)
                from tray.occupancy import outside_tray
                targets = outside_tray(targets, tray['slots'])
        result = build_job(targets, tray,
                           load_catalog(ROOT / 'config/object_catalog.json'), label, commanded,
                           acknowledge_unverified_suction=args.acknowledge_unverified_suction,
                           accept_uncertain_tray=args.accept_uncertain_tray,
                           pick_radial_offset_mm=args.pick_radial_offset_mm)
        if label == 'unknown':
            from tray.tray_capacity import fit_unknown_in_slot
            from tray.tray_capacity import slot_axis_xy
            slot = dict(result['job']['slot'], floor_height_mm=tray['slots'][result['slot_number']]['floor_height_mm'],
                        axis_xy=slot_axis_xy(tray['slots']))
            geometry = result['job']['object_geometry']
            pickup = next(t for t in targets if t.get('new_object'))
            try:
                fitted, clearance = fit_unknown_in_slot(depth, ROOT, slot, geometry, pickup['target_xy'])
            except ValueError as exc:
                # 4축 팔은 물체를 돌릴 수 없으므로, 책상에서 미리 돌려 놓을 각도와 들어가는 칸을 함께 알려 준다.
                from tray.tray_capacity import suggest_unknown_turn, turn_hint
                found = suggest_unknown_turn(depth, ROOT, {n: dict(s, axis_xy=slot['axis_xy']) for n, s in tray['slots'].items()},
                                             geometry, pickup['target_xy'])
                raise ValueError(f'{exc} {turn_hint(found, pickup["target_xy"], ROOT)}') from exc
            result['job']['slot'].update(x_mm=fitted['x_mm'], y_mm=fitted['y_mm'])
            result['job']['slot_clearance_mm'] = clearance
            if fitted.get('tight'):
                result['warnings'].append(f'칸 여유 {clearance:.0f}mm(권장 5mm 미만): 물체가 칸 벽에 닿을 수 있습니다. 사용자 허용으로 실행합니다.')
            result['job']['slot_floor_height_mm'] = slot['floor_height_mm']
            result['job']['slot_axis_xy'] = slot['axis_xy']
            result['job']['slot_center_mm'] = fitted.get('center_mm')    # 놓기 직전 재확인도 같은 칸 사각형을 쓴다
            result['job']['unknown_observation'] = dict(
                baseline=str(args.unknown_baseline.resolve()),
                baseline_sha256=hashlib.sha256(args.unknown_baseline.read_bytes()).hexdigest(),
                target_xy=list(next(t for t in targets if t.get('new_object'))['target_xy']),
                planned_at=datetime.now(timezone.utc).isoformat())
    except ValueError as exc:
        parser.error(str(exc))
    # 카메라로 찍었으면 저장 프레임을 썼더라도 트레이가 비어 있을 때(칸 바닥 3개) 새 기준으로 저장한다. 10-21: 기준이 10:46 것으로 남아
    # 그사이 트레이가 20~30px 밀린 것을 따라가지 못했다(1번 칸에 물체가 들어간 뒤라 다시 찾기도 실패). 저장 파일 입력은 다시 찾았을 때만.
    if not args.from_files or not stored_tray_ok:
        # 2026-10-20: 카메라로 트레이를 직접 찾았고 칸 바닥 3개가 모두 보이면(=비어 있음) 이 프레임을 빈 트레이 기준으로 자동 저장한다.
        # 물체가 들어간 다음 계획에서 --tray-frame으로 칸 위치를 알 수 있다(메뉴 9번을 따로 누르지 않아도 된다). 실패해도 계획에는 영향 없음.
        try:
            from save_tray_frame import save_if_empty
            save_if_empty(*(tray_frame_args[:2] if not stored_tray_ok and fresh_tray else (bgr, depth)))
        except Exception:
            pass
    out = args.output or ROOT / 'output/place_jobs' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    place_json = out / 'place.json'
    try:                                           # 계획에 쓴 프레임을 남긴다(칸 위치가 이상할 때 같은 입력으로 다시 확인하려고)
        import cv2
        import numpy as np
        cv2.imwrite(str(out / 'frame_rgb.png'), tray_frame_args[0])
        np.save(out / 'frame_depth_mm.npy', tray_frame_args[1])
    except Exception:
        pass
    place_json.write_text(json.dumps(result['job'], ensure_ascii=False, indent=2, allow_nan=False))
    command = [sys.executable, 'scripts/pick_auto.py', '--execute', '--supported',
               '--manual-target-mm', *(repr(round(v, 2)) for v in result['manual_target_mm']), '--place-json', str(place_json)]
    if result.get('pick_tilt_deg'):
        command += ['--pick-tilt-deg', repr(float(result['pick_tilt_deg']))]
    if args.empty_run:
        command.append('--place-empty-run')
    print(json.dumps(dict(label=label, slot_number=result['slot_number'], slot_source=result['slot_source'],
                          empty_run=args.empty_run, place_json=str(place_json), manual_target_mm=result['manual_target_mm'], camera_target_mm=result['camera_target_mm'],
                          pick_radial_offset_mm=result['pick_radial_offset_mm'], pick_tilt_radial_correction_mm=result.get('pick_tilt_radial_correction_mm', 0.), pick_tangential_correction_mm=result.get('pick_tangential_correction_mm', 0.), pick_tilt_deg=result.get('pick_tilt_deg') or 0.,
                          warnings=result['warnings'], tray_method=result['tray_method'],
                          tray_slots={str(k): dict(x_mm=round(v['x_mm']), y_mm=round(v['y_mm']), radius_m=round(v['radius_m'], 3)) for k, v in tray.get('slots', {}).items()}, tray_numbering=result['tray_numbering'], tray_source=tray_source, tray_frame_offset_px=tray_offset,
                          pick_auto_command=command, pick_auto_command_text=shlex.join(command),
                          executed=False, motor_writes=False, pump_writes=False, physical_accuracy_verified=False),
                     ensure_ascii=False, indent=2, default=float))
    return 0


_LAST_FRAMES = []      # 카메라로 찍은 마지막 사진(계획이 실패하면 원인 확인용으로 남긴다)


def save_failed_frames():
    """계획 실패 시 그때 사진을 output/place_jobs/<시각>_failed/에 남긴다(성공하면 계획 폴더에 이미 남는다)."""
    if len(_LAST_FRAMES) != 2:
        return
    try:
        import cv2
        import numpy as np
        out = ROOT / 'output/place_jobs' / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S') + '_failed')
        out.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out / 'frame_rgb.png'), _LAST_FRAMES[0])
        np.save(out / 'frame_depth_mm.npy', _LAST_FRAMES[1])
    except Exception:
        pass


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except SystemExit as exc:
        if exc.code not in (0, None):
            save_failed_frames()
        raise
