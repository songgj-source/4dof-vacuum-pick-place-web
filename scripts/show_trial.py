#!/usr/bin/env python3
"""지난 집기·놓기 실행 기록(output/pick_trials/<시각>/result.json)을 단계별로 자세히 보여 준다. 읽기만 한다(로봇·카메라 안 씀).

  python scripts/show_trial.py                 # 가장 최근 실행
  python scripts/show_trial.py 20261020T054449 # 그 실행(폴더 이름 또는 경로)
  python scripts/show_trial.py --list          # 실행 목록(최근 20개)

시간은 실행 시작부터 잰 초. 좌표는 로봇 밑동 기준 mm(x 앞, y 왼쪽, z 책상 위). 오차는 목표와 도착(관절값으로 계산한 공구 끝) 사이 거리.
"""
import argparse
import json
import math
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TRIALS = ROOT / 'output' / 'pick_trials'
sys.path.insert(0, str(Path(__file__).resolve().parent))
from sort_menu import bold, dim, green, red, yellow, cyan, width, phase_label   # noqa: E402  메뉴와 같은 색·한국어 단계 이름

NAMES = {'white_box': '흰 상자', 'red_cube': '빨간 정육면체', 'tape_measure': '줄자', 'unknown': '새 물체'}
LEGS = {'raise': '올리기', 'raise_clear': '바닥에서 올리기', 'raise_low': '낮게 올리기', 'pull_in': '끌어당기기', 'tilt': '기울이기',
        'carry': '운반', 'lower': '내리기', 'retreat': '후퇴', 'retire': '트레이에서 빠지기', 'untilt': '손목 세우기',
        'place_exit': '후퇴·빠지기·세우기 한 번에'}
# 메뉴 진행 표시(sort_menu.phase_label)는 나눠진 이동을 한 문구로 묶는다. 기록에서는 몇 가지를 더 나눠 보여 준다.
PHASE_EXTRA = (('approach_b', '물체에 접근(2차)'), ('lift_to_clear', '들어 올리기(70mm까지)'), ('lift20', '들어 올리기(20mm)'))


def korean(name):
    """단계 이름 → 한국어. _fix는 접근 보정."""
    base = name.replace('_fix2', '').replace('_fix', '')
    label = next((text for prefix, text in PHASE_EXTRA if base.startswith(prefix)), None) or phase_label(base)
    if name.endswith('_fix2'):
        label += ' (보정 2차)'
    elif name.endswith('_fix'):
        label += ' (보정)'
    return label


def pad(text, size):
    return str(text) + ' ' * max(0, size - width(str(text)))


def mm(xyz):
    return '(' + ', '.join(f'{v * 1000:6.1f}' for v in xyz) + ')' if xyz else '-'


def find(arg):
    if arg is None:
        runs = sorted(d for d in TRIALS.glob('2026*') if (d / 'result.json').exists())
        if not runs:
            raise SystemExit(f'실행 기록이 없습니다: {TRIALS}')
        return runs[-1]
    path = Path(arg)
    path = path if path.exists() else TRIALS / arg
    if path.is_file():
        path = path.parent
    if not (path / 'result.json').exists():
        raise SystemExit(f'실행 기록이 없습니다: {path}')
    return path


def started(result):
    try:
        return datetime.fromisoformat(result['started_at']).astimezone().strftime('%Y-%m-%d %H:%M:%S')
    except (KeyError, ValueError, TypeError):
        return '-'


def what(result):
    """(물체 이름, 칸 번호, 기본/명령)."""
    place = (result.get('plan') or {}).get('place') or {}
    label = place.get('label')
    if label and label.startswith('custom_'):
        try:
            from_registry = json.loads((ROOT / 'config/registered_objects.json').read_text())['objects']
            label = next((e['name'] for e in from_registry if e['label'] == label), label)
        except (OSError, ValueError, KeyError):
            pass
    return NAMES.get(label, label or '-'), place.get('slot_number'), {'command': '명령', 'default_rule': '기본'}.get(place.get('slot_source'), '-')


def outcome(result):
    if result.get('error'):
        return red('✘ 실패')
    if (result.get('place') or {}).get('empty_run'):
        return yellow('빈 실행(흡착 없음)')
    if result.get('motion_cycle_complete'):
        return green('✔ 동작 완료')
    return yellow('미완료')


def show_list():
    runs = sorted(d for d in TRIALS.glob('2026*') if (d / 'result.json').exists())[-20:]
    for run in runs:
        try:
            result = json.loads((run / 'result.json').read_text())
        except ValueError:
            print(f'{run.name}  (읽지 못함)')
            continue
        name, slot, _ = what(result)
        print(f"{run.name}  {started(result)}  {pad(name, 14)} → {slot or '-'}번 칸  {result.get('total_duration_s', 0):5.1f}초  {outcome(result)}")


def show(run):
    result = json.loads((run / 'result.json').read_text())
    name, slot, source = what(result)
    plan = result.get('plan') or {}
    print(bold(f'■ 실행 {run.name}') + dim(f'  ({started(result)})'))
    print(f'  물체 {name} → {slot or "-"}번 칸({source})   결과 {outcome(result)}   총 {result.get("total_duration_s", 0):.1f}초'
          + (f'   (들어 올리기까지 {result["time_to_lift_s"]:.1f}초)' if result.get('time_to_lift_s') else ''))
    if result.get('error'):
        print(red(f'  오류: {result["error"]}'))

    # ---- 계획 ----
    print('\n' + cyan('▌ 계획'))
    located = result.get('located') or {}
    if located:
        xy = located.get('target_xy') or [0, 0]
        print(f"  집기 목표   x {xy[0] * 1000:.1f}  y {xy[1] * 1000:.1f}  접촉 높이 {located.get('contact_z', 0) * 1000:.1f}mm"
              f"   로봇에서 {located.get('radius_m', math.hypot(*xy)) * 100:.1f}cm   ({located.get('target_source', '-')})")
    profile = plan.get('motion_profile') or {}
    if profile:
        print(f"  속도·게인   이동 {profile.get('travel_rate_pulses_s', '-')}펄스/초 · 하강 {profile.get('descent_rate_pulses_s', '-')}펄스/초 · "
              f"이동 P{profile.get('travel_p', '-')} · 접촉 P{profile.get('contact_p', '-')} · 밑동 P{profile.get('base_p', '-')}")
    place_cfg = plan.get('place') or {}
    if place_cfg:
        print(f"  놓기 설정   운반 {place_cfg.get('rate_pulses_s', '-')}펄스/초(틱당 {place_cfg.get('pulses_per_step', '-')}펄스) · 최대 기울기 "
              f"{place_cfg.get('max_tilt_deg', '-')}도 · 시간 한도 {place_cfg.get('max_seconds', '-')}초 · 들림 확인 {place_cfg.get('hold_check', '-')}")
    place_plan = result.get('place_plan') or {}
    if place_plan:
        legs = ' → '.join(LEGS.get(leg['name'], leg['name']) for leg in place_plan.get('legs', []))
        print(f"  놓기 경로   {legs}   운반 높이 {place_plan.get('carry_z', 0) * 1000:.0f}mm · 기울기 {place_plan.get('tilt_deg', 0):g}도"
              f" · 끌어당김 {place_plan.get('pull_in_mm', 0):g}mm · 칸까지 {place_plan.get('slot_radius_m', 0) * 100:.1f}cm"
              f" · 로봇 쪽 벽으로 {place_plan.get('near_wall_shift_mm', 0):g}mm")
        avoided = [NAMES.get(o, '등록 안 된 물체' if o == 'unregistered' else o) for o in place_plan.get('obstacles_checked') or []]
        print(f"  피한 물체   {', '.join(avoided) or '없음'}"
              f"   (든 물체 반경 {place_plan.get('held_radius_mm', 0):.0f}mm)")

    # ---- 단계 ----
    print('\n' + cyan('▌ 단계별 동작'))
    print(dim('  ' + pad('단계', 30) + '시작   이동   대기   목표 xyz(mm)                 도착 xyz(mm)                 오차  허용'))
    previous = 0.
    for phase in result.get('phases', []):
        receipt = phase.get('receipt') or {}
        start = phase.get('elapsed_s', 0) - phase.get('duration_s', 0)
        error = receipt.get('candidate_error_m')
        tolerance = receipt.get('pose_tolerance_m')
        ok = receipt.get('candidate_pose_in_tolerance')
        mark = '' if ok is None else (green(' ✔') if ok else red(' ✘'))
        line = (f"  {pad(korean(phase['phase']), 30)}{start:5.1f}  {phase.get('duration_s', 0):5.2f}  {start - previous:5.2f}   "
                f"{mm(receipt.get('requested_xyz'))}  {mm(receipt.get('candidate_xyz'))}  "
                f"{(error or 0) * 1000:4.1f}  {(tolerance or 0) * 1000:3.0f}{mark}")
        print(line)
        if phase.get('error'):
            print(red(f"    └ {phase['error']}"))
        previous = phase.get('elapsed_s', previous)

    # ---- 보정·판정 ----
    print('\n' + cyan('▌ 보정과 판정'))
    for fix in result.get('approach_fixes') or []:
        print(f"  접근 보정 {fix['attempt']}차   오차 {fix['error_mm']}mm → 다시 보냄 {mm(fix.get('correction_target_xyz'))}")
    for dx, dy, norm in result.get('descent_xy_correction_mm') or []:
        print(f'  하강 중 가로 보정   {dx:+.1f}, {dy:+.1f}mm (남은 가로 오차 {norm:.1f}mm 중)')
    if result.get('contact_candidate_xyz'):
        contact = result['contact_candidate_xyz']
        planned = located.get('contact_z')
        print(f'  접촉 위치   {mm(contact)}' + (f'   계획 높이와 차이 {(contact[2] - planned) * 1000:+.1f}mm (허용 ±3)' if planned is not None else ''))
    if result.get('candidate_lift_m') is not None:
        print(f"  들어 올림   {result['candidate_lift_m'] * 1000:.1f}mm  도착 {mm(result.get('lifted_candidate_xyz'))}")
    if result.get('hold_check'):
        hold = result['hold_check']
        print(f"  들림 확인   {'들림' if hold.get('held') else '안 들림'} ({hold.get('reason')}, {hold.get('mode')})")
    release = result.get('release') or {}
    if release.get('expected_release_xyz'):
        expected, actual = release['expected_release_xyz'], release['candidate_release_xyz']
        flat = math.hypot(actual[0] - expected[0], actual[1] - expected[1]) * 1000
        print(f"  놓은 위치   목표 {mm(expected)}  실제 {mm(actual)}   가로 {flat:.1f}mm(허용 5) · 높이 {(actual[2] - expected[2]) * 1000:+.1f}mm(허용 ±4)"
              f" · 놓은 뒤 대기 {release.get('release_settle_s', '-')}초")
    place = result.get('place') or {}
    if place:
        print(f"  놓기 전체   {place.get('duration_s', 0):.1f}초 · 단계 {len(place.get('steps') or [])}개"
              + (f" · 합친 이동 {', '.join(place['merged'])}" if place.get('merged') else '') + f" · 놓음 {'예' if place.get('released') else '아니오'}")

    # ---- 펌프·복귀 ----
    print('\n' + cyan('▌ 펌프 명령'))
    first = next((c['time'] for c in result.get('pump_commands') or [] if c.get('time') is not None), None)
    words = {'vent': '배기', 'stop': '정지', 'suction': '흡착', 'vent_place': '칸에서 배기', 'stop_place': '칸에서 정지',
             'vent_return': '제자리 배기', 'stop_return': '제자리 정지'}
    for command in result.get('pump_commands') or []:
        when = f"+{command['time'] - first:5.1f}초" if command.get('time') is not None and first is not None else '      '
        reply = ((command.get('controller_receipt') or {}).get('reply') or '').strip()
        print(f"  {when}  {pad(words.get(command['command'], command['command']), 12)}" + (dim(f'  응답: {reply}') if reply else ''))
    print(dim('  (압력 센서가 없어 실제로 붙었는지는 기록에 없습니다)'))
    home = result.get('home_return') or {}
    print('\n' + cyan('▌ 마무리'))
    tail = result.get('total_duration_s', 0) - previous
    if home:
        summary = home.get('summary') or {}
        print(f"  HOME 복귀   {'완료' if summary.get('home_return_complete') else '실패'} ({home.get('rate_pulses_s', '-')}펄스/초)"
              f" · 마지막 단계 뒤 끝까지 {tail:.1f}초(복귀와 마무리 포함)"
              + (red(f"  {home.get('error') or summary.get('error')}") if home.get('error') or summary.get('error') else ''))
    elif (plan.get('return_home') or {}).get('skip_after_place'):
        print('  HOME 복귀   건너뜀(연속 정리: 트레이 위에서 다음 물체로)')
    print(f"  팔 붙잡기 {'확인' if result.get('final_hold_confirmed') else '미확인'} · 게인 복구 {'확인' if result.get('restoration_verified') else '미확인'}"
          f" · 펌프 정지 {'보냄' if result.get('pump_stop_sent_on_close') else '-'}")
    print(dim(f'\n  원본: {run / "result.json"}'))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('run', nargs='?', help='실행 폴더 이름(예: 20261020T054449) 또는 경로. 없으면 가장 최근')
    parser.add_argument('--list', action='store_true', help='최근 실행 목록')
    args = parser.parse_args(argv)
    if args.list:
        show_list()
    else:
        show(find(args.run))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
