"""집기 접촉 높이에서 정지한 팔도 홈 복귀의 첫 들어올리기를 시작할 수 있다(2026-10-28 Pi: 컵 끝 18mm에서 'table clearance failed')."""
import json
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]


def _floor():
    return float(re.search(r'^RAISE_MIN_CONTACT_Z=([0-9.]+)', (ROOT / 'scripts/return_home.py').read_text(), re.M).group(1))


def test_raise_floor_is_passed_to_the_lift_and_within_hover_trial_limits():
    source = (ROOT / 'scripts/return_home.py').read_text()
    assert 'lift_m=.030,min_contact_z=RAISE_MIN_CONTACT_Z' in source
    assert .005 <= _floor() <= .030                                         # run_hover_trial이 받는 범위


def test_floor_is_below_every_pick_contact_height():
    from perception.object_targets import extra_press_mm, load_catalog
    catalog = load_catalog(ROOT / 'config/object_catalog.json')
    lowest = min((float(v['height_mm']) + float(catalog['cup_contact_offset_mm']) - extra_press_mm(catalog, k)) / 1000.
                 for k, v in catalog['objects'].items())
    assert _floor() < lowest                                                # 어느 물체를 집다 멈춰도 그 자세에서 올릴 수 있다
    from perception.unknown_objects import MIN_HEIGHT_MM, CONTACT_PRESS_MM
    assert _floor() < (MIN_HEIGHT_MM - CONTACT_PRESS_MM) / 1000.            # 가장 낮은 새 물체(20mm)의 접촉 높이(15mm)보다도


def test_stopped_pose_from_the_failure_clears_the_new_floor_but_not_the_old_one():
    from hardware.servo_mapping import MappingProfile
    from robot.robot_model import load_planar_model
    model = load_planar_model(ROOT)
    joints = {j.name: j for j in MappingProfile.load(ROOT / 'config/servo_driver_candidate.json', ROOT).joints}
    raw = dict(base=2436, shoulder=3366, elbow=2387, wrist_flex=2420)       # 실제 정지 자세(Pi, 10/28 10:51)
    pulse = 2 * math.pi / 4096
    angles = tuple(joints[n].direction * (raw[n] - model['zeros'][n]) * pulse for n in ('base', 'shoulder', 'elbow', 'wrist_flex'))
    lowest = min(p[2] for p in model['geometry'].link_points(angles))
    assert .010 < lowest < .030 and lowest >= _floor()                      # 새 하한이면 통과, 옛 하한(30mm)이면 실패했다


def test_raise_speed_matches_pick_auto_reset_raise_not_the_old_slow_default():
    source = (ROOT / 'scripts/return_home.py').read_text()
    assert '**RAISE_PROFILE' in source
    rate = float(re.search(r'^RAISE_RATE_PPS=([0-9.]+)', source, re.M).group(1))
    pick_auto = (ROOT / 'scripts/pick_auto.py').read_text()
    assert rate == 600.                                                                   # 같은 기본 저속(500펄스/초, 2026-10-28)
    assert 'SLOW_RATE_PPS=600.' in pick_auto.replace(' ', '')
    assert 'tracking_limit=64' in source and 'pulses_per_step=8' in source
