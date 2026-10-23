"""A planned tray move must fit the executor's hover radius before pickup."""

import math
from pathlib import Path

from robot.robot_model import load_planar_model
from sorting.place_plan import plan_place


ROOT = Path(__file__).resolve().parents[1]


def test_far_tray_plan_uses_the_new_355_mm_limit():
    # 2026-10-23 white-box attempt: a 55-degree plan ended at 351.1 mm.
    # The requested 5 mm increase should allow it without a 60-degree tilt.
    plan = plan_place(
        load_planar_model(ROOT),
        {'x': .33454203194333564, 'y': -.15980347268216067,
         'drop_z': .060239670130836274},
        (.22307, .03802, .0375),
        carry_z=.130, min_carry_z=.09, held_height_mm=24.,
        obstacles=[
            {'label': 'red_cube', 'x': .22285, 'y': -.04107,
             'height_mm': 40., 'radius_mm': 28.28},
            {'label': 'tape_measure', 'x': .1468, 'y': -.03361,
             'height_mm': 30., 'radius_mm': 43.56},
        ],
        held_radius_mm=43.01, max_tilt_deg=60,
    )

    assert plan['tilt_deg'] == 55.
    radius = max(math.hypot(*leg['target'][:2]) for leg in plan['legs'])
    assert .351 < radius <= .355
