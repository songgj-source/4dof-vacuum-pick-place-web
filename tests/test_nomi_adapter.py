import sys
import threading
import time
import unittest
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'integrations' / 'nomi'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_arm_api import ApiBase, arm_api
from arm_rest_skill import ArmRestSkill, resolve_object
from tool_specs import arm_tool_specs


@dataclass(frozen=True)
class ArgumentSpec:                      # NOMI tool_registry.ArgumentSpec와 같은 필드만
    name: str
    description: str
    kind: str = 'string'
    required: bool = True
    max_length: int = 160
    minimum: float | None = None
    maximum: float | None = None
    choices: tuple = ()


@dataclass(frozen=True)
class ToolSpec:
    name: str
    description: str
    arguments: tuple
    executor: Callable[[dict], Any]
    timeout_seconds: float = 5.0
    category: str = 'read_only'
    requires_completion_evidence: bool = False


class NomiAdapterTests(ApiBase):
    """가짜 실행기를 단 실제 API 서버에 NOMI Skill 어댑터를 붙여 본다(로봇·카메라 없음)."""

    def setUp(self):
        super().setUp()
        self.server.shutdown(); self.server.server_close()
        self.start_server(token='t')
        self.skill = ArmRestSkill(base_url=self.base, token='t')
        self.skill.POLL_S = .01

    def run_async(self, fn, *args, **kwargs):
        out = {}
        thread = threading.Thread(target=lambda: out.setdefault('r', fn(*args, **kwargs)))
        thread.start()
        return thread, out

    def test_resolves_korean_names(self):
        self.assertEqual(resolve_object('흰 상자'), 'white_box')
        self.assertEqual(resolve_object('빨간큐브'), 'red_cube')
        self.assertEqual(resolve_object('tape_measure'), 'tape_measure')
        self.assertIsNone(resolve_object('컵'))

    def test_sort_default_slot_waits_and_verifies(self):
        thread, out = self.run_async(self.skill.sort_object, '줄자')
        time.sleep(.1); self.release.set(); thread.join(5)
        r = out['r']
        self.assertTrue(r['success'])
        self.assertTrue(r['data']['completion_verified'])
        self.assertEqual(r['data']['menu'], 3)
        self.assertIn('물체에 접근', r['data']['phases'])
        self.assertIn('--object', self.commands[0])

    def test_sort_with_slot_uses_menu5(self):
        self.release.set()
        r = self.skill.sort_object('red_cube', slot=1)
        self.assertTrue(r['success'])
        self.assertEqual(r['data']['menu'], 5)

    def test_failure_is_not_verified(self):
        self.outcome = (1, dict(ok=False, error='target not found'), 1.)
        self.release.set()
        r = self.skill.sort_object('white_box')
        self.assertFalse(r['success'])
        self.assertFalse(r['data']['completion_verified'])

    def test_unknown_needs_slot_and_bad_names_ask(self):
        self.assertTrue(self.skill.sort_object('unknown')['data']['needs_clarification'])
        self.assertTrue(self.skill.sort_object('컵')['data']['needs_clarification'])
        self.assertEqual(self.commands, [])

    def test_busy_is_reported(self):
        thread, out = self.run_async(self.skill.arm_return_home)
        time.sleep(.1)
        r = self.skill.sort_all_objects()
        self.assertFalse(r['success'])
        self.assertEqual(r['data']['error_code'], 'busy')
        self.release.set(); thread.join(5)

    def test_stop_cancels_wait_and_robot(self):
        thread, out = self.run_async(self.skill.sort_object, '흰 상자')
        time.sleep(.1)
        self.assertTrue(self.skill.stop())
        thread.join(5)
        self.assertEqual(out['r']['data']['error_code'], 'cancelled')
        self.assertTrue(self.interrupted.is_set())

    def test_timeout_sends_stop(self):
        r = self.skill._run(6, None, 'HOME 복귀', timeout=.2)
        self.assertEqual(r['data']['error_code'], 'timeout')
        self.assertTrue(self.interrupted.is_set())

    def test_wrong_token_and_no_server(self):
        self.assertFalse(ArmRestSkill(base_url=self.base, token='x').get_arm_status()['success'])
        r = ArmRestSkill(base_url='http://127.0.0.1:9', token='t').get_arm_status()
        self.assertEqual(r['data']['error_code'], 'connection_error')

    def test_look_at_table_drops_image_bytes(self):
        import numpy as np
        frame = (np.zeros((48, 64, 3), np.uint8), np.zeros((48, 64), np.uint16))
        scene = dict(objects=[dict(label='red_cube', name='빨간 정육면체', pixel=[1, 2], position_mm=[250, 0], pickable=True)],
                     available_menus=[2, 5, 6], note='n')
        with patch.object(arm_api, 'grab_astra', return_value=frame), patch.object(arm_api, 'describe_scene', return_value=scene):
            r = self.skill.look_at_table()
        self.assertTrue(r['success'])
        self.assertEqual(r['data']['available_menus'], [2, 5, 6])
        self.assertNotIn('image_jpeg_base64', r['data'])
        self.assertIn('빨간 정육면체', r['message'])

    def test_tool_specs_shape(self):
        specs = {s.name: s for s in arm_tool_specs(self.skill, ToolSpec, ArgumentSpec)}
        self.assertEqual(set(specs), {'look_at_table', 'get_arm_status', 'sort_object', 'sort_all_objects', 'arm_return_home'})
        self.assertTrue(specs['sort_object'].requires_completion_evidence)
        self.assertFalse(specs['look_at_table'].requires_completion_evidence)
        self.release.set()
        self.assertTrue(specs['sort_object'].executor({'object_name': 'tape_measure'})['success'])


if __name__ == '__main__':
    unittest.main()
