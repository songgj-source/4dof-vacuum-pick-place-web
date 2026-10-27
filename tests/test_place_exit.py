"""놓은 뒤 물러나기→빠지기→세우기를 한 번에(2026-10-26 사용자 지시). 실제 1번 칸 성공 실행(Pi 20261026T054614)의 계획·관절 값으로,
실제 팔 기구학 모델에서 경로가 모든 검사를 통과하는지 본다. 모터·펌프 없음."""
import contextlib
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from hardware.servo_mapping import MappingProfile  # noqa: E402
from motion import place_exit  # noqa: E402
from motion.place_exit import ExitPathRefused, run_place_exit_trial  # noqa: E402
from robot.planar_kinematics import NAMES, PlanarGeometry  # noqa: E402
from robot.robot_frame import SHOULDER_HEIGHT_M  # noqa: E402
from sorting.place_exec import build_exit_transition, run_place  # noqa: E402

TILT = -math.pi / 4                                   # 45도 기울여 1번 칸에 넣었다
LEGS = [dict(name='retreat', target=(.3061, -.1426, .105), pitch_rad=TILT),
        dict(name='retire', target=(.2335, -.1089, .105), pitch_rad=TILT,
             retire_pitches=[-.785, -1.047, -1.309]),
        dict(name='untilt', target=(.2335, -.1089, .105), pitch_rad=-math.pi / 2)]
RELEASE_RAW = {'base': 2384, 'shoulder': 3768, 'elbow': 1488, 'wrist_flex': 2408}


def arm():
    profile = MappingProfile.load(ROOT / 'config/servo_driver_candidate.json', ROOT)
    base = ROOT / 'output/ik_resume_20261012/multipose_review'
    review, fit = json.loads((base / 'review.json').read_text()), json.loads((base / 'fit_full.json').read_text())
    zeros = {j.name: j.zero for j in profile.joints}
    zeros.update(fit['new_zero_pulses'])
    geometry = PlanarGeometry(.027, SHOULDER_HEIGHT_M, tuple(review['eight_pose_geometry_m']))
    joints = {j.name: j for j in profile.joints}
    pulse = 2 * math.pi / 4096
    fk = geometry.forward(tuple(joints[n].direction * (RELEASE_RAW[n] - zeros[n]) * pulse for n in NAMES))
    driver = SimpleNamespace(state='hold_only', hold_confirmed=True, names=NAMES, lock=contextlib.nullcontext(), profile=profile,
                             _snapshot=lambda torque=1: {'joints': {n: {'position': RELEASE_RAW[n], 'goal': RELEASE_RAW[n]} for n in NAMES}})
    return driver, geometry, zeros, fk


def test_waypoints_rise_straight_then_retire_while_straightening():
    t = build_exit_transition(LEGS, (.30598, -.13958, .05908), TILT)
    rise = [p for p in t['waypoints'] if p['xyz'][2] < t['clear_z'] - 1e-9]
    assert rise and all(p['xyz'][:2] == (.30598, -.13958) and p['pitch_rad'] == TILT for p in rise)   # 칸 안: 수직, 기울기 그대로
    assert [p['pulses'] for p in rise if p['xyz'][2] < .070] and all(p['pulses'] == 2 for p in rise if p['xyz'][2] < .070)
    pitches = [p['pitch_rad'] for p in t['waypoints']]
    assert all(abs(b - a) <= math.radians(15) + 1e-3 for a, b in zip(pitches, pitches[1:]))         # 15도 이하씩
    assert t['end_pitch'] == pytest.approx(-math.pi / 2) and t['target'] == pytest.approx(LEGS[1]['target'])
    home = build_exit_transition(LEGS, (.30598, -.13958, .05908), TILT, include_untilt=False)
    assert home['end_pitch'] == pytest.approx(-1.309)                                                # HOME 복귀가 이어지면 세우지 않는다
    assert build_exit_transition([dict(name='retire', target=(.2, 0, .1))], (.3, 0, .06), TILT) is None


def test_real_slot1_exit_passes_every_check_on_the_arm_model():
    driver, geometry, zeros, fk = arm()
    t = build_exit_transition(LEGS, fk[:3], TILT)          # run_place는 계획 각도를 넘긴다(측정 각도 차이는 실행 함수가 검사)
    captured = {}
    with patch.object(place_exit, '_execute_path', side_effect=lambda *a, **k: captured.update(k) or {'reached': True}):
        assert run_place_exit_trial(driver, geometry, zeros, t, supported=True, rate=150.)['reached']
    assert len(captured['waypoints']) == len(t['waypoints'])
    assert set(captured['seg_pulses']) == {2, 4}


def test_moved_start_is_refused_before_moving():
    driver, geometry, zeros, fk = arm()
    t = build_exit_transition(LEGS, (fk[0] + .02, fk[1], fk[2]), TILT)
    with patch.object(place_exit, '_execute_path') as execute, pytest.raises(ExitPathRefused):
        run_place_exit_trial(driver, geometry, zeros, t, supported=True, rate=150.)
    execute.assert_not_called()


def place(exit_path):
    drop = (.3061, -.1426, .0606)
    plan = dict(start=(.3061, -.1426, .105), executed=False,
                legs=[dict(name='lower', target=drop, pitch_rad=TILT), *LEGS])
    pose, moves = [plan['start']], []
    def hover(name, target, **kw):
        moves.append(name); pose[0] = tuple(target)
    report = run_place(plan, hover=hover, fresh_xyz=lambda: pose[0], release=lambda p: None, cancelled=lambda: False,
                       exit_path=exit_path)
    return report, moves


def test_run_place_uses_one_exit_move_and_falls_back_when_refused():
    calls = []
    report, moves = place(lambda t: calls.append(t))
    assert len(calls) == 1 and 'place_exit' in report['steps']
    assert not any(m.startswith(('place_retreat', 'place_retire', 'place_untilt')) for m in moves)   # 단계별 이동 없음
    def refuse(t): raise ExitPathRefused('no solution')
    report, moves = place(refuse)
    assert any(m.startswith('place_retreat') for m in moves) and 'place_exit' not in report['steps']  # 예전처럼 단계별


def test_measured_sag_inside_the_slot_is_allowed_up_to_6_mm():
    # 2026-10-27 Pi: 1번 칸 60도에서 빠져나오기 시작할 때 어깨가 늦게 따라와 컵이 2.3mm 처졌고 2mm 검사에 걸렸다.
    # 놓은 물체는 칸 바닥 20mm 위에서 떨어져 컵 아래가 비어 있으므로 6mm까지는 받고, 그보다 깊으면 멈춘다
    driver, geometry, zeros, fk = arm()
    t = build_exit_transition(LEGS, fk[:3], TILT)
    captured = {}
    with patch.object(place_exit, '_execute_path', side_effect=lambda *a, **k: captured.update(args=a) or {'reached': True}):
        run_place_exit_trial(driver, geometry, zeros, t, supported=True, rate=150.)
    clear = captured['args'][4]
    joints = {j.name: j for j in driver.profile.joints}
    pulse = 2 * math.pi / 4096
    limits = {n: tuple(sorted(joints[n].direction * (r - zeros[n]) * pulse for r in (joints[n].raw_min + 16, joints[n].raw_max - 16))) for n in NAMES}

    def raw_at(dz):
        q = min(geometry.solve((fk[0], fk[1], fk[2] - dz), fk[3], limits),
                key=lambda q: sum((a - b) ** 2 for a, b in zip(q, (joints[n].direction * (RELEASE_RAW[n] - zeros[n]) * pulse for n in NAMES))))
        return {n: round(zeros[n] + q[i] / (joints[n].direction * pulse)) for i, n in enumerate(NAMES)}
    clear(raw_at(.004))                                   # 4mm 처짐: 계속 간다(예전 2mm 검사는 여기서 멈췄다)
    with pytest.raises(ValueError, match='dipped below the release point'):
        clear(raw_at(.009))


def test_lagging_cup_may_leave_the_slot_column_a_little_below_the_carry_height():
    # 2026-10-27 Pi: 60도 1번 칸에서 실제 컵이 운반 높이보다 8mm 아래일 때 명령이 옆으로 빠지기 시작해 4.7mm에서 멈췄다
    driver, geometry, zeros, fk = arm()
    t = build_exit_transition(LEGS, fk[:3], TILT)
    captured = {}
    with patch.object(place_exit, '_execute_path', side_effect=lambda *a, **k: captured.update(args=a, kwargs=k) or {'reached': True}):
        run_place_exit_trial(driver, geometry, zeros, t, supported=True, rate=150.)
    assert 'lead_s' not in captured['kwargs']                        # 놓은 뒤 빈 컵은 앞당기지 않는다
    clear = captured['args'][4]
    joints = {j.name: j for j in driver.profile.joints}
    pulse = 2 * math.pi / 4096
    limits = {n: tuple(sorted(joints[n].direction * (r - zeros[n]) * pulse for r in (joints[n].raw_min + 16, joints[n].raw_max - 16))) for n in NAMES}
    inward = (-fk[0] / math.hypot(*fk[:2]), -fk[1] / math.hypot(*fk[:2]))

    def raw_at(z, side):
        q = geometry.solve((fk[0] + inward[0] * side, fk[1] + inward[1] * side, z), fk[3], limits)[0]
        return {n: round(zeros[n] + q[i] / (joints[n].direction * pulse)) for i, n in enumerate(NAMES)}
    clear(raw_at(t['clear_z'] - .008, .006))                          # 운반 높이 8mm 아래에서 6mm 옆: 계속 간다
    with pytest.raises(ValueError, match='rise straight out of the slot'):
        clear(raw_at(t['clear_z'] - .030, .006))                      # 칸 벽 가까이(30mm 아래)에서 옆으로는 멈춘다
