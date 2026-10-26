"""A stopped approach close to HOME can use the checked direct return."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from motion.return_corridor import interrupted_near_home_start
import math
import motion.return_corridor as corridor
import pytest


HOME = {'base': 2110, 'shoulder': 1823, 'elbow': 3018, 'wrist_flex': 3075}
STOPPED = {'base': 2074, 'shoulder': 2057, 'elbow': 2845, 'wrist_flex': 3032}


def test_stopped_approach_near_home_can_enter_direct_checked_return():
    assert interrupted_near_home_start((.0871, .004, .1374), STOPPED, HOME)


def test_other_outside_corridor_poses_still_need_explicit_recovery():
    assert not interrupted_near_home_start((.25, 0, .1374), STOPPED, HOME)
    assert not interrupted_near_home_start((.0871, .004, .07), STOPPED, HOME)
    too_far = dict(STOPPED, shoulder=HOME['shoulder'] + 301)
    assert not interrupted_near_home_start((.0871, .004, .1374), too_far, HOME)


def admitted(xyz, pitch=-.8498585976343079):
    return corridor.tilted_tray_start(xyz, pitch, radius_max_m=.355,
                                    max_forward_tilt_rad=math.radians(60))


def test_recorded_tray_edge_pose_uses_separate_lift_path():
    xyz = (.3037794671224337, -.1397053796445158, .04365193813923507)
    assert admitted(xyz)
    assert not corridor.start_in_corridor(xyz)
    assert not interrupted_near_home_start(xyz, STOPPED, HOME)


def test_exact_executor_tilt_and_radius_boundary_is_allowed():
    assert admitted((.355, 0, .135), -math.pi / 6)
    assert admitted((.355, 0, .030), -math.pi / 2)


@pytest.mark.parametrize('xyz,pitch', [
    ((.320, 0, .08), -.85), ((.355001, 0, .08), -.85),
    ((.34, 0, .02999), -.85), ((.34, 0, .135001), -.85),
    ((.34, 0, .08), -math.pi / 2 - .0001),
    ((.34, 0, .08), -math.pi / 6 + .0001),
    ((float('nan'), 0, .08), -.85), ((.34, 0, .08), float('nan')),
])
def test_tray_recovery_rejects_outside_existing_envelope(xyz, pitch):
    assert not admitted(xyz, pitch)
