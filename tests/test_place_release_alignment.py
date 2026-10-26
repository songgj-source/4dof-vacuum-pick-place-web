"""Replay the measured carry/lower drift without motors or pump I/O."""
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from sorting.place_exec import run_place
from motion.trial_phase import phase_pose_tolerance


DROP = (.3075730649, -.1473848337, .0598838172)
START = (.2567426528, -.0106397185, .100)
PLAN = dict(start=START, executed=False, legs=[
    dict(name='carry', target=(*DROP[:2], .100)),
    dict(name='lower', target=DROP),
])


def simulate(*, carry_error=(.0013622, .0030053), alignment_works=True, start=START,
             lower_error=(-.0004, .0025, -.0017)):
    pose, moves, released = [tuple(start)], [], []
    plan = {**PLAN, 'start': start}
    def hover(name, target, **kwargs):
        moves.append((name, tuple(target), kwargs))
        if name.startswith('place_carry'):
            pose[0] = (target[0] + carry_error[0], target[1] + carry_error[1], target[2])
        elif name.startswith('place_align') and not alignment_works:
            pass
        elif name.startswith('place_lower'):
            # Lowering in the failing run added about 2.5mm of lateral drift.
            pose[0] = tuple(t + e for t, e in zip(target, lower_error))
        else:
            pose[0] = tuple(target)
    return dict(plan=plan, hover=hover, fresh_xyz=lambda: pose[0],
                release=lambda point: released.append(tuple(point)), cancelled=lambda: False), moves, released


def test_measured_carry_error_is_removed_above_tray_before_lowering():
    args, moves, released = simulate()
    report = run_place(**args)
    assert report['released']
    assert released == [DROP]
    align = [m for m in moves if m[0].startswith('place_align')]
    assert align and all(m[1][2] >= .070 for m in align)
    assert all(m[1][:2] == DROP[:2] for m in moves if m[0].startswith('place_lower'))


def test_alignment_cannot_hide_large_error_or_release_at_wrong_point():
    args, moves, released = simulate(carry_error=(0., .012))
    with pytest.raises(ValueError):
        run_place(**args)
    assert not released
    assert not any(m[0].startswith('place_lower') for m in moves)


def test_nonconvergent_alignment_is_bounded_and_never_lowers():
    args, moves, released = simulate(alignment_works=False)
    with pytest.raises(ValueError):
        run_place(**args)
    assert not released
    assert not any(m[0].startswith('place_lower') for m in moves)
    assert len([m for m in moves if m[0].startswith('place_align')]) <= 2


def test_release_still_refuses_excessive_descent_drift():
    args, _, released = simulate(lower_error=(0., .006, 0.))
    with pytest.raises(ValueError, match='release refused'):
        run_place(**args)
    assert not released


def test_alignment_preserves_the_planned_tool_pitch():
    args, moves, released = simulate()
    pitch = -math.pi / 4
    args['plan'] = {**args['plan'], 'legs': [{**leg, 'pitch_rad': pitch} for leg in PLAN['legs']]}
    run_place(**args)
    assert released
    assert all(m[2]['pitch_rad'] == pitch for m in moves if m[0].startswith('place_align'))


def test_no_horizontal_alignment_near_the_drop_surface():
    moves, released = [], []
    start = (*DROP[:2], .065)
    plan = dict(start=start, executed=False, legs=[dict(name='lower', target=DROP)])
    with pytest.raises(ValueError, match='pre-lower alignment refused'):
        run_place(plan, hover=lambda *a, **k: moves.append(a),
                  fresh_xyz=lambda: (start[0], start[1] + .0035, start[2]),
                  release=lambda p: released.append(p), cancelled=lambda: False)
    assert not moves and not released


@pytest.mark.parametrize('strict', ['0', '1'])
def test_lower_phase_must_fit_release_tolerance_in_both_modes(monkeypatch, strict):
    monkeypatch.setenv('VACUUM_STRICT_GUARDS', strict)
    assert phase_pose_tolerance('place_lower_0') <= .004
    assert phase_pose_tolerance('place_lower_0', pick_tilt_deg=10.) <= .004


def test_constant_arrival_bias_is_fed_forward_and_kept_while_lowering():
    # 2026-10-26 Pi: 2번 칸 위 정렬 목표로 두 번 보내도 두 번 다 같은 쪽으로 4.0mm 어긋나 섰다(같은 목표 재전송으로는 줄지 않음)
    bias = (.0040, 0.)
    pose, moves, released = [tuple(START)], [], []
    def hover(name, target, **kwargs):
        moves.append((name, tuple(target)))
        pose[0] = (target[0] + bias[0], target[1] + bias[1], target[2])          # 어디로 보내든 같은 쪽으로 4mm
    report = run_place(plan=dict(PLAN), hover=hover, fresh_xyz=lambda: pose[0],
                       release=lambda point: released.append(tuple(point)), cancelled=lambda: False)
    assert report['released'] and released == [DROP]
    align = [m for m in moves if m[0].startswith('place_align')]
    assert len(align) == 1 and math.isclose(align[0][1][0], DROP[0] - bias[0], abs_tol=1e-9)   # 잰 오차만큼 밀어 보냄
    lower = [m for m in moves if m[0].startswith('place_lower')]
    assert lower and all(math.isclose(m[1][0], DROP[0] - bias[0], abs_tol=1e-9) for m in lower)   # 내릴 때도 같은 보정
