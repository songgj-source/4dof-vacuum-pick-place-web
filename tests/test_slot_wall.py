"""먼 칸은 칸 중심 대신 로봇 쪽 벽 가까이 놓는다(2026-10-27 사용자 요청). 물체 방향 측정·당길 거리·계획 반영. 장치 없음."""
import json
import math
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src')]
from perception.object_classify import classify_objects  # noqa: E402
from robot.robot_model import load_planar_model  # noqa: E402
from sorting.place_job import load_place_job  # noqa: E402
from sorting.place_plan import plan_place  # noqa: E402
from sorting.slot_wall import inward_limit_mm, object_axis  # noqa: E402

# 2026-10-23 흰 상자 1번 칸(반경 37.1cm, 55도 기울여야 닿던 계획). test_place_runtime_envelope와 같은 장면
FAR_SLOT = {'x': .33454203194333564, 'y': -.15980347268216067, 'drop_z': .060239670130836274}
START = (.22307, .03802, .0375)
OBSTACLES = [{'label': 'red_cube', 'x': .22285, 'y': -.04107, 'height_mm': 40., 'radius_mm': 28.28},
             {'label': 'tape_measure', 'x': .1468, 'y': -.03361, 'height_mm': 30., 'radius_mm': 43.56}]


def rect(center, long_mm, short_mm, angle_deg):
    a = math.radians(angle_deg)
    u, v = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
    return [(center[0] + su * long_mm / 2 * u[0] + sv * short_mm / 2 * v[0], center[1] + su * long_mm / 2 * u[1] + sv * short_mm / 2 * v[1])
            for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]


def plan(inward, slot=FAR_SLOT):
    return plan_place(load_planar_model(ROOT), dict(slot, inward_max=inward), START, carry_z=.130, min_carry_z=.09,
                      held_height_mm=24., obstacles=OBSTACLES, held_radius_mm=43.01, max_tilt_deg=60)


def test_object_axis_finds_the_long_side_and_rejects_wrong_sizes():
    found = object_axis(rect((300, -100), 72, 48, 30), [50, 70])
    assert found['half_mm'] == [35., 25.]
    assert abs(abs(found['axis_xy'][0] * math.cos(math.radians(30)) + found['axis_xy'][1] * math.sin(math.radians(30))) - 1) < 1e-6
    assert object_axis(rect((300, -100), 110, 48, 30), [50, 70]) is None        # 등록 크기와 다르면 방향을 믿지 않는다
    square = object_axis(rect((300, -100), 64, 60, 10), [65, 58])                # 줄자처럼 거의 정사각형: 두 변 모두 긴 변으로
    assert square['half_mm'] == [32.5, 32.5]


def test_unknown_direction_falls_back_to_the_diagonal_circle():
    center = (334.5, -159.8)
    radial = (center[0] / math.hypot(*center), center[1] / math.hypot(*center))
    tray_axis = (-radial[1], radial[0])
    assert inward_limit_mm(center, tray_axis, (92, 100), center, radius_mm=43.01) == 0.     # 흰 상자 대각선 원은 92mm 칸에 여유가 없다
    long_across = dict(axis_xy=list(tray_axis), half_mm=[35., 25.])
    long_inward = dict(axis_xy=list(radial), half_mm=[35., 25.])
    across = inward_limit_mm(center, tray_axis, (92, 100), center, footprint=long_across)
    inward = inward_limit_mm(center, tray_axis, (92, 100), center, footprint=long_inward)
    assert 12 < across < 20 and 3 < inward < 10 and across > inward               # 짧은 변이 로봇 쪽을 보면 더 당길 수 있다


def test_base_turn_rotates_the_object_before_it_reaches_the_slot():
    center = (334.5, -159.8)
    radial = (center[0] / math.hypot(*center), center[1] / math.hypot(*center))
    tray_axis = (-radial[1], radial[0])
    desk = dict(axis_xy=list(tray_axis), half_mm=[35., 25.])
    same_angle = inward_limit_mm(center, tray_axis, (92, 100), center, footprint=desk)
    turned = math.atan2(center[1], center[0]) - math.pi / 2                        # 베이스가 90도 돌면 긴 변이 로봇 쪽을 본다
    pick = (250 * math.cos(turned), 250 * math.sin(turned))
    assert inward_limit_mm(center, tray_axis, (92, 100), pick, footprint=desk) < same_angle - 5


def test_far_slot_is_placed_toward_the_robot_wall_and_needs_less_tilt():
    center = plan(0.)
    near_wall = plan(.019)
    assert center['near_wall_shift_mm'] == 0. and near_wall['near_wall_shift_mm'] == 19.
    assert near_wall['tilt_deg'] < center['tilt_deg']
    assert abs(near_wall['target_radius_m'] - (center['target_radius_m'] - .019)) < 1e-9
    lower = next(leg for leg in near_wall['legs'] if leg['name'] == 'lower')['target']
    assert math.hypot(*lower[:2]) < math.hypot(*next(leg for leg in center['legs'] if leg['name'] == 'lower')['target'][:2])


def test_slot_reached_straight_down_keeps_the_centre():
    near = dict(FAR_SLOT, x=.24, y=-.02)
    planned = plan(.019, near)
    assert planned['tilt_deg'] == 0. and planned['near_wall_shift_mm'] == 0.
    assert next(leg for leg in planned['legs'] if leg['name'] == 'lower')['target'][:2] == (.24, -.02)


def test_place_job_reads_the_inward_limit_and_old_files_stay_centred():
    job = dict(label='white_box', rim_height_mm=48., obstacles=[], slot=dict(x_mm=334.5, y_mm=-159.8, drop_z_mm=60., inward_max_mm=15.5))
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / 'job.json'
        path.write_text(json.dumps(job))
        assert abs(load_place_job(path)['slot']['inward_max'] - .0155) < 1e-12
        del job['slot']['inward_max_mm']
        path.write_text(json.dumps(job))
        assert load_place_job(path)['slot']['inward_max'] == 0.
        job['slot']['inward_max_mm'] = 55.
        path.write_text(json.dumps(job))
        with pytest.raises(ValueError, match='inward_max_mm'):
            load_place_job(path)


def test_classifier_reports_the_rotated_top_face():
    depth = np.full((480, 640), 800, np.uint16)
    bgr = np.full((480, 640, 3), 200, np.uint8)
    box = cv2.boxPoints(((220., 260.), (70., 46.), 25.)).astype(np.int32)
    cv2.fillPoly(depth, [box], 770)
    cv2.fillPoly(bgr, [box], (40, 40, 220))
    found = [f for f in classify_objects(bgr, depth) if f['top_corners_px']]
    assert len(found) == 1
    corners = np.array(found[0]['top_corners_px'])
    sides = sorted(np.linalg.norm(corners - np.roll(corners, -1, 0), axis=1)[:2])
    assert abs(sides[0] - 46) < 6 and abs(sides[1] - 70) < 6


def test_build_job_writes_the_inward_limit_from_the_measured_direction():
    from sorting.sort_job import build_job
    # 트레이 길이 방향이 반경과 거의 수직(옆으로 놓인 트레이), 칸 간격 94mm
    slots = {1: dict(x_mm=334.5, y_mm=-159.8, floor_height_mm=10.), 2: dict(x_mm=375.0, y_mm=-75.0, floor_height_mm=10.),
             3: dict(x_mm=415.6, y_mm=9.8, floor_height_mm=10.)}
    tray = dict(found=True, slots=slots, rim_height_mm=48.)
    catalog = dict(cup_contact_offset_mm=-3.5, objects=dict(white_box=dict(height_mm=24, footprint_mm=[50, 70])))
    target = dict(label='white_box', pickable=True, suction_verified=True, problems=[], target_xy=[.25, -.02], contact_z=.0155)
    tray_axis = (np.array([415.6, 9.8]) - np.array([334.5, -159.8]))
    tray_axis /= np.linalg.norm(tray_axis)
    turn = math.atan2(-159.8, 334.5) - math.atan2(-.02, .25)          # 베이스가 도는 각도만큼 책상에서는 반대로 놓여 있다
    c, s = math.cos(-turn), math.sin(-turn)
    axis = [c * tray_axis[0] - s * tray_axis[1], s * tray_axis[0] + c * tray_axis[1]]   # 칸에 도착하면 긴 변이 트레이 길이 방향
    measured = build_job([dict(target, footprint_axis=dict(axis_xy=axis, half_mm=[35., 25.]))], tray, catalog, 'white_box', 1)['job']['slot']
    circle = build_job([dict(target, footprint_axis=None)], tray, catalog, 'white_box', 1)['job']['slot']
    assert measured['inward_basis'] == 'measured_axis' and 10 < measured['inward_max_mm'] < 20
    assert circle['inward_basis'] == 'diagonal_circle' and circle['inward_max_mm'] == 0.


def test_tray_at_40_cm_fits_the_cube_against_the_robot_wall_but_not_the_white_box():
    # 2026-10-27: 트레이를 40cm에 두려고 1번 칸을 정육면체로 바꿨다. 같은 방향 40cm 칸
    angle = math.atan2(FAR_SLOT['y'], FAR_SLOT['x'])
    slot = dict(x=.40 * math.cos(angle), y=.40 * math.sin(angle), drop_z=.0765)
    model = load_planar_model(ROOT)
    kwargs = dict(carry_z=.130, min_carry_z=(48 + 10 + 40 - 3.5) / 1000., obstacles=[], max_tilt_deg=60)
    with pytest.raises(ValueError):
        plan_place(model, dict(slot, inward_max=0.), START, held_height_mm=40., held_radius_mm=28.28, **kwargs)   # 칸 중심은 닿지 않는다
    cube = plan_place(model, dict(slot, inward_max=.018), START, held_height_mm=40., held_radius_mm=28.28, **kwargs)
    assert cube['near_wall_shift_mm'] == 18.
    with pytest.raises(ValueError):
        plan_place(model, dict(slot, drop_z=.0605, inward_max=.0155), START, held_height_mm=24., held_radius_mm=43.01,
                   **dict(kwargs, min_carry_z=(48 + 10 + 24 - 3.5) / 1000.))


def test_cube_at_40_4_cm_fits_with_the_36_cm_cup_limit():
    # 2026-10-27 11:07 Pi: 칸 40.43cm, 당김 13.6mm → 60도에서 컵 35.61cm가 필요해 35.5cm 상한에 막혔다(같은 날 12.6~14.2mm)
    angle = math.atan2(FAR_SLOT['y'], FAR_SLOT['x'])
    slot = dict(x=.4044 * math.cos(angle), y=.4044 * math.sin(angle), drop_z=.0765)
    plan = plan_place(load_planar_model(ROOT), dict(slot, inward_max=.0126), START, carry_z=.130, min_carry_z=(48 + 10 + 40 - 3.5) / 1000.,
                      held_height_mm=40., held_radius_mm=28.28, obstacles=[], max_tilt_deg=60)
    assert plan['near_wall_shift_mm'] == 12.6
    assert .355 < max(math.hypot(*leg['target'][:2]) for leg in plan['legs']) <= .360
