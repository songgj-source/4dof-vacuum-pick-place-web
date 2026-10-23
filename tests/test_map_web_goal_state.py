"""Execute the real goal-request handler without ROS or motor publishers."""
import ast
import math
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

MAP_WEB = Path(__file__).resolve().parents[1] / 'ros2_slam/test/map_web.py'


def handler():
    tree = ast.parse(MAP_WEB.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MapWeb')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_nav_tick')
    ns = {'time': SimpleNamespace(time=lambda: 1000.0), 'math': math}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(MAP_WEB), 'exec'), ns)
    return ns['_nav_tick']


def request(previous, bearing=0.0):
    target = {'id': 'desk', 'name': '책상'}
    node = SimpleNamespace(lock=threading.Lock(), nav_requests=[('goto', target, None)],
                           reloc_request=None, nav=dict(previous), nav_then=None, after_nav=None,
                           nav_seq=1, nav_client=SimpleNamespace(server_is_ready=lambda: True),
                           goal_handle=None, backup={'old': True}, backup_m=.3,
                           _goal_bearing=lambda _: bearing, _backup_tick=lambda: None,
                           _after_nav_tick=lambda: None, get_logger=lambda: Mock())
    seen = []
    node._send_goal = lambda *args: seen.append(dict(node.nav))
    handler()(node)
    return node, seen


def test_forward_goal_gets_new_timestamp_after_old_goal():
    node, seen = request({'state': 'succeeded', 'started': 10.0, 'backed': .3})
    assert len(seen) == 1
    assert seen[0]['started'] == 1000.0
    assert seen[0]['target']['id'] == 'desk'
    assert seen[0].get('backed', 0.0) == 0.0
    assert node.backup is None


def test_replacement_cancels_previous_goal_before_resetting_state():
    # Exercise cancellation of the previous active action, then the new forward goal.
    tree = ast.parse(MAP_WEB.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MapWeb')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_nav_tick')
    ns = {'time': SimpleNamespace(time=lambda: 1000.0), 'math': math}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(MAP_WEB), 'exec'), ns)
    target = {'id': 'desk'}
    old = Mock()
    node = SimpleNamespace(lock=threading.Lock(), nav_requests=[('goto', target, None)],
                           reloc_request=None, nav={'state': 'active', 'started': 10.0},
                           nav_then=None, after_nav=None, nav_seq=1,
                           nav_client=SimpleNamespace(server_is_ready=lambda: True),
                           goal_handle=old, backup=None, backup_m=.3,
                           _goal_bearing=lambda _: 0.0, _send_goal=Mock(),
                           _after_nav_tick=lambda: None, _backup_tick=lambda: None)
    ns['_nav_tick'](node)
    old.cancel_goal_async.assert_called_once()
    assert node.nav['started'] == 1000.0
