"""새 물체 칸 배정: 가장 멀리 보낼 수 있는 물체가 가장 먼 빈 칸, 나머지는 고정 시드 무작위."""
import sys
from pathlib import Path

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / 'src')]
from sorting import slot_assignment as sa

RADII = {1: 340., 2: 270., 3: 190.}


def test_farthest_capable_object_gets_the_farthest_free_slot():
    slots, left = sa.assign({'a': 345., 'b': 380., 'c': 300.}, RADII, [1, 2, 3])
    assert slots['b'] == 1 and sorted(slots.values()) == [1, 2, 3] and left == []


def test_object_that_cannot_reach_the_far_slot_is_not_sent_there():
    slots, left = sa.assign({'a': 300., 'b': 310.}, RADII, [1, 2, 3])
    assert 1 not in slots.values() and left == []                    # 먼 칸에 닿는 물체가 없다: 그 칸은 비워 둔다


def test_far_slot_is_the_farthest_free_one_when_slot_1_is_taken():
    slots, _ = sa.assign({'a': 300., 'b': 290.}, RADII, [2, 3])      # 1번 칸은 기존 물체가 쓴다
    assert slots['a'] == 2 and slots['b'] == 3


def test_same_objects_and_slots_always_give_the_same_assignment():
    reach = {'x': 400., 'y': 400., 'z': 400.}
    first = sa.assign(reach, RADII, [1, 2, 3])
    assert all(sa.assign(reach, RADII, [1, 2, 3]) == first for _ in range(20))


def test_different_objects_can_shuffle_differently_but_every_slot_is_used_once():
    for names in (('a', 'b', 'c'), ('b', 'c', 'd'), ('x', 'y', 'z')):
        slots, left = sa.assign({n: 400. for n in names}, RADII, [1, 2, 3])
        assert sorted(slots.values()) == [1, 2, 3] and left == []


def test_unreachable_object_is_left_and_reported():
    slots, left = sa.assign({'a': 400., 'b': None}, RADII, [1, 2, 3])
    assert slots == {'a': 1} and left == ['b']


def test_free_slots_exclude_tray_and_default_slots():
    assert sa.free_slots([1], [3]) == [2]
    assert sa.free_slots([], []) == [1, 2, 3]


def test_far_slot_is_the_farthest_one_somebody_can_reach():
    radii = {1: 344., 2: 253., 3: 168.}                                        # 실제 Pi 트레이의 칸 거리
    slots, left = sa.assign({'a': 265., 'b': 262.}, radii, [1, 2, 3])        # 수직 놓기만: 1번 칸(344)은 아무도 못 간다
    assert 1 not in slots.values() and slots['a'] == 2 and slots['b'] == 3 and left == []


def test_vertical_only_reach_is_about_26_cm_and_below_slot_1():
    from robot.robot_model import load_planar_model
    model = load_planar_model(Path(__file__).resolve().parents[1])
    for height in (20., 30., 45.):
        reach = sa.reach_limit_mm(model, 0., held_height_mm=height, max_tilt_deg=0.)
        assert 255. <= reach <= 300. < 340.                                     # 1번 칸(약 34cm)에는 닿지 않는다
        assert sa.reach_limit_mm(model, 0., held_height_mm=height) > reach       # 기울이면 더 멀리 간다


def test_planner_says_taller_held_object_reaches_farther():
    from robot.robot_model import load_planar_model
    model = load_planar_model(Path(__file__).resolve().parents[1])
    low = sa.reach_limit_mm(model, 0., held_height_mm=24., held_radius_mm=35.)
    high = sa.reach_limit_mm(model, 0., held_height_mm=45., held_radius_mm=40.)
    assert low is not None and high is not None and high > low
