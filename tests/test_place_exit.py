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
