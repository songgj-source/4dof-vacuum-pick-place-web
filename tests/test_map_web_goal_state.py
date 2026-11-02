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
    node._send_or_turn = lambda label, note: node._send_goal(label, note)
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
    node._send_or_turn = lambda label, note: node._send_goal(label, note)
    ns['_nav_tick'](node)
    old.cancel_goal_async.assert_called_once()
    assert node.nav['started'] == 1000.0


def turn_node(bearing, now=1000.0):
    tree = ast.parse(MAP_WEB.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MapWeb')
    ns = {'time': SimpleNamespace(time=lambda: now), 'math': math}
    for name in ('_send_or_turn', '_turn_tick'):
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(MAP_WEB), 'exec'), ns)
    node = SimpleNamespace(turn_min=math.radians(25), turn_tol=math.radians(8), turn_w_max=.6, turn_w_min=.25,
                           _goal_bearing=lambda _: node.bearing, bearing=bearing, backup=None, nav={}, drive=Mock(),
                           _send_goal=Mock(), get_logger=lambda: Mock())
    node._send_or_turn = lambda *a: ns['_send_or_turn'](node, *a)
    node._turn_tick = lambda b: ns['_turn_tick'](node, b)
    return node


def test_small_angle_sends_the_goal_without_turning():
    node = turn_node(math.radians(20))
    node._send_or_turn({'id': 'a', 'name': 'a'}, '')
    node._send_goal.assert_called_once()
    assert node.backup is None


def test_large_angle_turns_in_place_toward_the_goal_before_sending():
    node = turn_node(math.radians(80))
    node._send_or_turn({'id': 'a', 'name': 'a'}, '')
    node._send_goal.assert_not_called()
    assert node.backup['mode'] == 'turn' and node.nav['state'] == 'backing'
    node._turn_tick(node.backup)
    linear, angular = node.drive.drive.call_args[0]
    assert linear == 0.0 and .25 <= angular <= .6                 # 목표가 왼쪽(+): 제자리에서 왼쪽으로
    node.bearing = -math.radians(80)
    node._turn_tick(node.backup)
    assert node.drive.drive.call_args[0][1] < 0


def test_turn_ends_inside_the_tolerance_and_then_sends_the_goal():
    node = turn_node(math.radians(80))
    node._send_or_turn({'id': 'a', 'name': 'a'}, '')
    node.bearing = math.radians(5)
    node._turn_tick(node.backup)
    node.drive.stop.assert_called_once()
    node._send_goal.assert_called_once()
    assert node.backup is None
