"""A measured pickup pose near 70 mm must produce executable place steps."""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from robot.robot_model import load_planar_model  # noqa: E402
from sorting.place_exec import LIFT_MIN_M, build_steps  # noqa: E402
from sorting.place_plan import plan_place  # noqa: E402


def test_measured_white_box_pose_near_clearance_has_full_minimum_rise():
    # Pi 2026-10-27 11:31: cup z=67.4535 mm, planner clearance z=70 mm.
    start = (.2635528848414098, -.018221779375901553, .0674535212698743)
    plan = plan_place(
        load_planar_model(ROOT),
        dict(x=.2547374224144358, y=-.16352746157021903,
             drop_z=.05984049564250377, inward_max=.0011),
        start,
        min_carry_z=.0855,
        held_height_mm=24.,
        held_radius_mm=43.01162633521314,
        obstacles=[
            dict(label='red_cube', x=.19880685085834116, y=.03901099413478894,
                 height_mm=40., radius_mm=28.284271247461902),
            dict(label='tape_measure', x=.17680499742220496, y=-.04927771111052542,
                 height_mm=30., radius_mm=43.55743334954437),
        ],
        start_tilt_deg=10.,
    )

    clear = next(leg for leg in plan['legs'] if leg['name'] == 'raise_clear')
    assert clear['target'][2] - start[2] >= LIFT_MIN_M - 1e-9
    assert clear['target'][2] >= .070
    assert math.isclose(plan['carry_z'], .100, abs_tol=.005)
    assert build_steps(plan, start)[0]['leg'] == 'raise_clear'
