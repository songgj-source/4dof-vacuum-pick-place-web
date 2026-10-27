"""물체 등록: 사진 한 장으로 등록 → 분류·카탈로그·칸 규칙·명령 해석·메뉴·API에 들어가는지. 로봇·카메라 없음(합성 장면)."""
import json
import shutil
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from perception import object_registry
from perception.object_classify import classify_objects
from perception.object_targets import load_catalog, locate_targets
from sorting.sort_job import parse_command
from tray.tray_sorting import choose_slot, slot_for

LOC = SimpleNamespace(R=np.diag([1., -1., -1.]), t=np.array([0., 0., .5]), fx=500., fy=500., cx=50., cy=235., offset_xy=np.zeros(2))


def scene(color=(200, 70, 20), top_mm=35, second=False):
    """회색 책상(깊이 500mm) 위 물체 하나(파란색, 높이 35mm). second=True면 물체 하나 더."""
    rgb = np.full((480, 640, 3), 160, np.uint8)
    depth = np.full((480, 640), 500, np.uint16)
    depth[210:260, 240:300] = 500 - top_mm
    rgb[210:260, 240:300] = color
    if second:
        depth[300:340, 240:280] = 470
        rgb[300:340, 240:280] = (30, 200, 30)
    return rgb, depth


class RegistryBase(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.root = Path(self.dir.name)
        (self.root / 'config').mkdir()
        shutil.copy(ROOT / 'config/object_catalog.json', self.root / 'config/object_catalog.json')
        for target in (patch.object(object_registry, 'ROOT', self.root),
                       patch('perception.object_targets.default_locator', return_value=LOC)):
            target.start()
            self.addCleanup(target.stop)
        object_registry._cache.clear()

    def register(self, name='파란 컵', slot=2, **kwargs):
        rgb, depth = kwargs.pop('frame', None) or scene()
        return object_registry.register(rgb, depth, name, slot, root=self.root, **kwargs)


class RegisterTests(RegistryBase):
    def test_registered_object_is_recognised_and_sortable(self):
        entry = self.register(height_mm=34)
        self.assertEqual(entry['label'], 'custom_1')
        self.assertEqual((entry['height_mm'], entry['height_source']), (34.0, 'user'))
        self.assertFalse(entry['suction_verified'])                 # 흡착은 확인하지 않고 등록(바로 집는다)
        self.assertTrue((self.root / 'output/registered_objects/custom_1.png').exists())
        self.assertEqual([f['label'] for f in classify_objects(*scene())], ['custom_1'])
        self.assertEqual(object_registry.labels(self.root), ['red_cube', 'white_box', 'tape_measure', 'custom_1'])
        self.assertEqual(load_catalog(self.root / 'config/object_catalog.json')['objects']['custom_1']['height_mm'], 34.0)
        self.assertEqual((slot_for('custom_1'), choose_slot('custom_1'), choose_slot('custom_1', 3)), (2, 2, 3))
        self.assertEqual(parse_command('파란컵 넣어줘'), ('custom_1', None))
        self.assertEqual(parse_command('파란 컵 1번 칸에 넣어'), ('custom_1', 1))
        with patch('perception.object_targets.reachable', return_value=True):
            targets = locate_targets(*scene(), self.root, model=SimpleNamespace(), locator=LOC)
        self.assertEqual([(t['label'], t['slot']) for t in targets], [('custom_1', 2)])
        self.assertAlmostEqual(targets[0]['contact_z'] * 1000, 34 - 3.5 - 3, delta=.01)    # 자로 잰 높이 + 컵 보정 - 추가 누름

    def test_measured_height_is_used_without_a_ruler(self):
        entry = self.register()
        self.assertEqual((entry['height_mm'], entry['height_source']), (35.0, 'depth'))
        self.assertTrue(entry['top_flat'])

    def test_other_colours_and_heights_stay_unregistered(self):
        self.register()
        self.assertEqual([f['label'] for f in classify_objects(*scene(color=(30, 200, 30)))], ['unknown'])   # 초록색
        self.assertEqual([f['label'] for f in classify_objects(*scene(top_mm=25))], ['unknown'])              # 10mm 낮다

    def test_refusals(self):
        cases = [
            (dict(frame=scene(second=True)), '하나만'),
            (dict(name='줄자'), '이미 쓰는 이름'),
            (dict(name='큰 줄자 케이스'), '기본 물체 이름'),
            (dict(name=''), '이름'),
            (dict(slot=4), '칸은'),
            (dict(height_mm=80), '높이는 20~45'),
            (dict(frame=scene(color=(235, 235, 235), top_mm=24)), '흰 상자'),     # 흰 상자와 색·높이가 비슷
            (dict(frame=scene(color=(20, 20, 210), top_mm=40)), '빨간 정육면체'),
        ]
        for kwargs, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                self.register(**kwargs)
        self.assertEqual(object_registry.load(self.root), [])

    def test_same_object_twice_and_limit(self):
        self.register()
        with self.assertRaisesRegex(ValueError, '이미 등록된'):
            self.register(name='다른 이름')
        with self.assertRaisesRegex(ValueError, '이미 등록된 이름'):
            self.register(name='파란컵', frame=scene(color=(30, 200, 30)))
        self.register(name='초록 컵', frame=scene(color=(30, 200, 30)))
        self.register(name='보라 컵', frame=scene(color=(200, 30, 160)))
        with self.assertRaisesRegex(ValueError, '3개까지'):
            self.register(name='노랑 컵', frame=scene(color=(30, 220, 230)))

    def test_remove(self):
        self.register()
        with self.assertRaisesRegex(ValueError, '삭제할 수 없습니다'):
            object_registry.remove('white_box', self.root)
        self.assertEqual(object_registry.remove('파란 컵', self.root)['label'], 'custom_1')
        self.assertEqual(object_registry.labels(self.root), ['red_cube', 'white_box', 'tape_measure'])
        self.assertEqual([f['label'] for f in classify_objects(*scene())], ['unknown'])

    def test_sync_tables_edits_in_place(self):
        names, slots, order = {'white_box': '흰 상자', 'custom_9': '옛 물체'}, {'white_box': 1, 'custom_9': 1}, ['white_box', 'custom_9']
        self.register()
        object_registry.sync_tables(names, slots, order, self.root)
        self.assertEqual(names, {'white_box': '흰 상자', 'custom_1': '파란 컵'})
        self.assertEqual(slots, {'white_box': 1, 'custom_1': 2})
        self.assertEqual(order, ['white_box', 'custom_1'])


class MenuAndApiTests(RegistryBase):
    def test_menu_lists_registered_objects_in_tidy_order(self):
        import sort_menu
        self.register()
        with patch.object(sort_menu, 'ROOT', self.root):
            sort_menu.refresh_objects()
            self.assertEqual(sort_menu.OBJECTS['4'], ('custom_1', '파란 컵', 2))
            self.assertEqual(sort_menu.OBJECTS_DEFAULT['custom_1'], 2)
            self.assertIn('파란 컵', sort_menu.brief_message('custom_1 not found in the current camera view'))
        sort_menu.refresh_objects()

    def test_api_register_list_tidy_and_delete(self):
        import arm_api
        from test_arm_api import ApiBase
        api = ApiBase('run')
        api.setUp()
        self.addCleanup(api.doCleanups)
        for target in (patch.object(arm_api.menu, 'ROOT', self.root), patch.object(arm_api, 'tray_slots', return_value=None)):
            target.start()
            self.addCleanup(target.stop)
        api.arm_grab = lambda: scene()
        status, body = api.arm.register_object(dict(name='파란 컵', slot=2), grab=api.arm_grab)
        self.assertEqual((status, body['object']['label']), (201, 'custom_1'))
        status, body = api.call('GET', '/api/objects')
        self.assertEqual([(o['label'], o['registered']) for o in body['objects']][-1], ('custom_1', True))
        self.assertEqual(api.call('POST', '/api/objects', dict(name='줄자', slot=1))[0], 400)       # 사진을 찍기 전에 이름부터 거절
        api.release.set()
        status, body = api.call('POST', '/api/menu/4', {'objects': ['custom_1']})
        self.assertEqual(status, 202)
        task = api.wait_done(body['task_id'])
        self.assertTrue(task['result']['ok'])
        self.assertIn('custom_1', api.commands[0])                     # 등록 물체도 계획·집기 명령으로 간다
        status, body = api.call('POST', '/api/menu/5', {'object': 'custom_1', 'slot': 3})
        self.assertEqual(status, 202)
        api.wait_done(body['task_id'])
        sys.path.insert(0, str(ROOT / 'integrations' / 'nomi'))
        from arm_rest_skill import ArmRestSkill
        skill = ArmRestSkill(base_url=api.base, token='')
        skill.POLL_S = .01
        done = skill.sort_object('파란컵')                            # NOMI: 서버 목록(GET /api/objects)으로 이름을 알아듣는다
        self.assertTrue(done['success'], done['message'])
        self.assertEqual(done['data']['menu'], 5)
        self.assertIn('파란 컵 → 2번 칸', done['message'])
        self.assertTrue(skill.sort_object('없는 컵')['data']['needs_clarification'])
        self.assertEqual(api.call('POST', '/api/objects/delete', {'label': 'custom_1'})[0], 200)
        self.assertEqual(api.call('POST', '/api/menu/5', {'object': 'custom_1', 'slot': 3})[0], 400)


if __name__ == '__main__':
    unittest.main()
