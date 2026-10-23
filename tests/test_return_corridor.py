"""A stopped approach close to HOME can use the checked direct return."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from motion.return_corridor import interrupted_near_home_start


HOME = {'base': 2110, 'shoulder': 1823, 'elbow': 3018, 'wrist_flex': 3075}
STOPPED = {'base': 2074, 'shoulder': 2057, 'elbow': 2845, 'wrist_flex': 3032}


def test_stopped_approach_near_home_can_enter_direct_checked_return():
    assert interrupted_near_home_start((.0871, .004, .1374), STOPPED, HOME)


def test_other_outside_corridor_poses_still_need_explicit_recovery():
    assert not interrupted_near_home_start((.25, 0, .1374), STOPPED, HOME)
    assert not interrupted_near_home_start((.0871, .004, .07), STOPPED, HOME)
    too_far = dict(STOPPED, shoulder=HOME['shoulder'] + 301)
    assert not interrupted_near_home_start((.0871, .004, .1374), too_far, HOME)
