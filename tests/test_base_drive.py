"""주행 연결(base_drive): 팀 map_web을 흉내 낸 가짜 서버로 위치 읽기, 라벨 찾기, 이동·벽 근접 결과, 정지를 시험한다. 차·팔 없음."""
import json
import sys
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src'))
import arm_api   # noqa: E402
import arm_llm   # noqa: E402
import base_drive  # noqa: E402

LABELS = [dict(id='59ef557c', name='책상', x=-3.58, y=0.315, yaw=-3.1, cost=82),
          dict(id='744e1dac', name='휴지통', x=-1.0, y=2.0, yaw=0.0, cost=0),
          dict(id='95e3077c', name='밥먹는곳', x=1.0, y=-1.0, yaw=0.0, cost=99)]


class FakeMapWeb:
    """GET /api/state가 steps(nav, wall)를 차례로 돌려준다. 받은 POST를 posts에 남긴다."""

    def __init__(self, steps):
        self.steps, self.posts, self.started = list(steps), [], None
        fake = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a): pass

            def reply(self, body, code=200):
                data = json.dumps(body).encode()
                self.send_response(code); self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)

            def do_GET(self):
                if self.path == '/api/drive':
                    return self.reply(dict(moving=False, driver_connected=True, odom=dict(linear=0, angular=0)))
                nav, wall = fake.steps.pop(0) if len(fake.steps) > 1 else fake.steps[0]
                if fake.started and nav.get('state') != 'idle':
                    nav = dict(nav, started=fake.started, label=dict(id='59ef557c', name='책상'))
                self.reply(dict(frame='map', pose=dict(x=-3.0, y=0.4, yaw=3.14), labels=LABELS, nav=nav, wall=wall))

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
                fake.posts.append((self.path, body))
                if self.path == '/api/nav/goto_wall':
                    fake.started = time.time()
                self.reply(dict(ok=True))

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = f'http://127.0.0.1:{self.server.server_address[1]}'

    def close(self):
        self.server.shutdown(); self.server.server_close()


IDLE = (dict(state='idle'), dict(state='idle'))


class BaseDriveTests(unittest.TestCase):
    def fake(self, steps):
        fake = FakeMapWeb(steps); self.addCleanup(fake.close)
        return fake

    def drive(self, fake, name='책상', **kw):
        phases = []
        result = base_drive.goto_label_wall(name, url=fake.url, on_phase=lambda raw, text: phases.append(raw), sleep=lambda s: None, **kw)
        return result, phases

    def test_state_sorts_places_by_distance_and_reads_the_pose(self):
        state = base_drive.base_state(self.fake([IDLE]).url)
        self.assertEqual([l['name'] for l in state['labels']], ['책상', '휴지통', '밥먹는곳'])
        self.assertFalse(state['labels'][2]['reachable'])                      # cost 99: 막힌 자리
        self.assertEqual(state['pose']['x'], -3.0)
        self.assertIn('갈 수 있는 장소: 책상, 휴지통, 밥먹는곳', base_drive.where_reply(state))

    def test_find_label_exact_partial_and_unknown(self):
        self.assertEqual(base_drive.find_label('책상', LABELS)['id'], '59ef557c')
        self.assertEqual(base_drive.find_label('휴지', LABELS)['name'], '휴지통')
        with self.assertRaises(base_drive.BaseError) as caught:
            base_drive.find_label('부엌', LABELS)
        self.assertEqual(caught.exception.code, 'label_not_found')
        self.assertIn('휴지통', str(caught.exception))                          # 있는 장소를 알려 준다

    def test_goto_then_wall_approach_succeeds(self):
        fake = self.fake([IDLE, IDLE,
                          (dict(state='active', distance=1.2), dict(state='idle')),
                          (dict(state='succeeded', then_state='pending'), dict(state='idle')),
                          (dict(state='succeeded', then_state='started'), dict(state='running')),
                          (dict(state='succeeded', then_state='started'), dict(state='done', message='벽 앞 3 cm'))])
        result, phases = self.drive(fake, gap=0.05)
        self.assertTrue(result['ok'], result)
        self.assertIn(('/api/nav/goto_wall', {'id': '59ef557c'}), fake.posts)
        self.assertIn(('/api/wall/config', {'gap': 0.05}), fake.posts)
        self.assertEqual([p for p in phases if p in ('drive_active', 'drive_arrived', 'drive_wall')], ['drive_active', 'drive_arrived', 'drive_wall'])

    def test_navigation_failure_and_wall_failure_are_reported(self):
        result, _ = self.drive(self.fake([IDLE, IDLE, (dict(state='aborted', message='경로 없음'), dict(state='idle'))]))
        self.assertEqual(result['reason_code'], 'drive_failed'); self.assertIn('경로 없음', result['reason'])
        result, _ = self.drive(self.fake([IDLE, IDLE, (dict(state='succeeded', then_state='failed'), dict(state='failed', message='정면 벽 없음'))]))
        self.assertEqual(result['reason_code'], 'wall_failed'); self.assertIn('정면 벽 없음', result['reason'])

    def test_stop_cancels_the_navigation(self):
        fake = self.fake([IDLE, IDLE, (dict(state='active', distance=2.0), dict(state='idle'))])
        result, _ = self.drive(fake, stopped=lambda: bool(fake.started))
        self.assertEqual(result['reason_code'], 'cancelled')
        self.assertTrue({'/api/nav/cancel', '/api/wall/stop', '/api/drive/stop'} <= {path for path, _ in fake.posts})

    def test_short_status(self):
        self.assertEqual(base_drive.short_status(dict(nav=dict(state='active', target='책상', distance=1.24))), '차: 책상으로 이동 중 (남은 1.2 m)')
        self.assertEqual(base_drive.short_status(dict(frame='map', pose=dict(x=0, y=0, yaw_deg=0), labels=[dict(name='휴지통', dist_m=0.42)], nav={})),
                         '차: 휴지통 근처 (0.4 m)')
        self.assertEqual(base_drive.short_status(dict(frame='odom', pose=None, labels=[], nav={})), '차: 지도 위 위치 모름')

    def test_unreachable_map_web_is_a_clear_error(self):
        with self.assertRaises(base_drive.BaseError) as caught:
            base_drive.base_state('http://127.0.0.1:9')
        self.assertEqual(caught.exception.code, 'base_unavailable')


class SimMapWeb:
    """map_web 흉내 + 아주 단순한 차 운동: /api/nav/goto는 라벨에서 offset만큼 어긋난 곳에 '도착'시키고,
    /api/drive 명령(0.5초 유효)대로 위치·방향을 적분한다. scan에 장애물 점을 넣을 수 있다."""

    def __init__(self, label, offset=(0.12, -0.05, 0.2), frame='map', scan=(), amcl_bias=None):
        import math as m
        self.m, self.label, self.offset, self.frame, self.scan = m, dict(label), offset, frame, list(scan)
        # amcl_bias: 지도 위치가 실제보다 (dx, dy) 어긋나 있다가 /api/amcl/settle을 받으면 새 추정으로 맞춰진다(None = 옛 map_web: amcl 정보 없음)
        self.bias, self.amcl = (list(amcl_bias), dict(received=0.0)) if amcl_bias is not None else ([0.0, 0.0], None)
        self.pose = [0.0, 0.0, 0.0]
        self.cmd, self.cmd_t, self.last = (0.0, 0.0), 0.0, time.monotonic()
        self.nav, self.gets, self.posts = dict(state='idle'), 0, []
        sim = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a): pass

            def reply(self, body):
                data = json.dumps(body).encode()
                self.send_response(200); self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)

            def do_GET(self):
                sim.step()
                if self.path == '/api/drive':
                    return self.reply(dict(moving=False, driver_connected=True, odom=dict(linear=0, angular=0)))
                if self.path == '/api/get_map_info':
                    return self.reply(dict(robot=dict(radius=0.26, front=0.16, back=0.16), nav=sim.nav))
                if sim.nav.get('state') == 'active':
                    sim.gets += 1
                    if sim.gets >= 2:                       # Nav2 도착: 라벨에서 어긋난 자리
                        lx, ly, lyaw = sim.label['x'], sim.label['y'], sim.label['yaw']
                        sim.pose = [lx + sim.offset[0], ly + sim.offset[1], lyaw + sim.offset[2]]
                        sim.nav = dict(sim.nav, state='succeeded')
                pose = None if sim.frame != 'map' else dict(x=sim.pose[0] + sim.bias[0], y=sim.pose[1] + sim.bias[1], yaw=sim.pose[2], age=0.05)
                state = dict(frame=sim.frame, pose=pose, labels=[sim.label], nav=sim.nav, wall=dict(state='idle'), scan=sim.scan)
                if sim.amcl is not None:
                    state['amcl'] = sim.amcl
                self.reply(state)

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
                sim.posts.append((self.path, body)); sim.step()
                if self.path == '/api/nav/goto':
                    sim.nav = dict(state='active', started=time.time(), label=dict(id=sim.label['id'], name=sim.label['name']), distance=2.0)
                elif self.path == '/api/drive':
                    sim.cmd, sim.cmd_t = (body.get('linear', 0.0), body.get('angular', 0.0)), time.monotonic()
                elif self.path in ('/api/drive/stop', '/api/nav/cancel'):
                    sim.cmd = (0.0, 0.0)
                elif self.path == '/api/amcl/settle' and sim.amcl is not None:
                    sim.bias, sim.amcl = [0.0, 0.0], dict(received=time.time() + 0.01)
                self.reply(dict(ok=True))

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = f'http://127.0.0.1:{self.server.server_address[1]}'

    def step(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        v, w = self.cmd if now - self.cmd_t < 0.5 else (0.0, 0.0)
        self.pose[2] += w * dt
        self.pose[0] += v * dt * self.m.cos(self.pose[2])
        self.pose[1] += v * dt * self.m.sin(self.pose[2])

    def close(self):
        self.server.shutdown(); self.server.server_close()


class PreciseArrivalTests(unittest.TestCase):
    """책상 라벨을 만든 그 자리로: Nav2가 비켜 서도(장애물 옆 라벨은 옮겨 감, 방향 ±14°) 마무리 정렬로 ±3 cm·±2° 안에 선다."""
    LABEL = dict(id='59ef557c', name='책상', x=1.0, y=0.5, yaw=0.3, cost=82)

    def sim(self, **kw):
        sim = SimMapWeb(self.LABEL, **kw); self.addCleanup(sim.close)
        return sim

    def go(self, sim):
        phases = []
        result = base_drive.drive_to('책상', url=sim.url, on_phase=lambda raw, text: phases.append(raw), sleep=time.sleep)
        return result, phases

    def test_lands_on_the_label_pose(self):
        import math
        sim = self.sim(offset=(0.12, -0.06, 0.22))                          # 13 cm, 13° 어긋난 도착
        result, phases = self.go(sim)
        self.assertTrue(result['ok'], result)
        off = math.hypot(sim.pose[0] - 1.0, sim.pose[1] - 0.5)
        self.assertLess(off, base_drive.XY_TOL * 1.5)
        self.assertLess(abs(math.degrees(sim.pose[2] - 0.3)), base_drive.YAW_TOL_DEG * 1.5)
        self.assertIn('위치 오차', result['note'])
        self.assertTrue({'fine_turn', 'fine_move', 'fine_yaw'} <= set(phases))
        self.assertIn(('/api/nav/goto', {'id': '59ef557c'}), sim.posts)    # 벽 붙기(goto_wall)는 쓰지 않는다
        self.assertEqual(sim.posts[-1][0], '/api/drive/stop')              # 끝나면 차를 세운다

    def test_settles_amcl_before_judging_so_a_stale_map_pose_is_corrected(self):
        import math
        # 2026-10-27: 지도 위치가 4 cm 어긋난 채 갱신되지 않으면 예전에는 그 위치로 '도착'이라고 했다
        sim = self.sim(offset=(0.10, -0.04, 0.15), amcl_bias=(0.04, 0.0))
        result, _ = self.go(sim)
        self.assertTrue(result['ok'], result)
        self.assertIn('/api/amcl/settle', [path for path, _ in sim.posts])
        self.assertLess(math.hypot(sim.pose[0] - 1.0, sim.pose[1] - 0.5), base_drive.XY_TOL * 1.5)   # 실제 위치가 라벨에

    def test_label_behind_backs_up_instead_of_turning_around(self):
        sim = self.sim(offset=(0.10 * 0.955, 0.10 * 0.296, 0.0))           # 라벨이 차 바로 뒤(라벨 방향으로 10 cm 지나침)
        result, _ = self.go(sim)
        self.assertTrue(result['ok'], result)
        self.assertTrue(any(path == '/api/drive' and body['linear'] < 0 for path, body in sim.posts))

    def test_stops_for_an_obstacle_in_the_way(self):
        import math
        lx, ly, yaw = self.LABEL['x'], self.LABEL['y'], self.LABEL['yaw']
        # 라벨 방향 그대로 15 cm 덜 와서 섰고, 라벨 쪽 차체 앞 2 cm에 물체(돌 필요 없이 바로 직진하다 멈춰야 한다)
        start = (lx - 0.15 * math.cos(yaw), ly - 0.15 * math.sin(yaw))
        ahead = [(start[0] + (0.18 + 0.005 * i) * math.cos(yaw), start[1] + (0.18 + 0.005 * i) * math.sin(yaw)) for i in range(3)]
        sim = self.sim(offset=(start[0] - lx, start[1] - ly, 0.0), scan=ahead)
        result, _ = self.go(sim)
        self.assertFalse(result['ok'])
        self.assertIn('물체가 있어 멈췄습니다', result['reason'])

    def test_too_far_from_the_label_is_not_forced(self):
        result, _ = self.go(self.sim(offset=(0.9, 0.0, 0.0)))
        self.assertEqual(result['reason_code'], 'drive_failed')
        self.assertIn('마무리 정렬 한도', result['reason'])

    def test_without_a_map_pose_it_says_so(self):
        with self.assertRaises(base_drive.BaseError):
            self.go(self.sim(frame='odom'))


class ArmDriveTests(unittest.TestCase):
    HOME = {n: (1, v) for n, v in {'base': 2110, 'shoulder': 1823, 'elbow': 3018, 'wrist_flex': 3075}.items()}

    def arm(self, pose, drive=None):
        calls = []
        def fake_drive(label, mode='precise', gap=None, on_phase=None, stopped=None):
            calls.append((label, gap)); self.modes.append(mode)
            on_phase('drive_active', f'{label}으로 이동 중'); return dict(ok=True, reason=None, reason_code=None)
        self.modes = []
        arm = arm_api.Arm(run=lambda *a, **k: (0, {}, 0.), read_state=lambda: 'ok', base_check=lambda: None,
                          drive=drive or fake_drive, read_pose=lambda: pose)
        return arm, calls

    def run_task(self, arm, params):
        with patch.object(arm_api.menu, 'live_view_running', return_value=False):
            status, body = arm.start('drive', params)
            self.assertEqual(status, 202, body)
            for _ in range(200):
                task = arm.task(body['task_id'])
                if task['state'] != 'running':
                    return task
                time.sleep(.01)
        self.fail('주행 작업이 끝나지 않았습니다')

    def test_drives_when_the_arm_is_folded(self):
        pose = dict(self.HOME, wrist_flex=(1, 3075 - 47))                      # 손목이 조금 덜 접혀도(120 펄스 안) 주행한다
        arm, calls = self.arm(pose)
        task = self.run_task(arm, {'label': '책상', 'gap': 0.05})
        self.assertEqual((task['state'], calls), ('done', [('책상', 0.05)]))
        self.assertEqual(task['phases'][0]['phase'], 'drive_active')
        self.assertIsNone(arm.arm_home)                                        # 주행은 팔 자세를 바꾸지 않는다
        self.assertEqual(self.modes, ['precise'])                              # 기본은 라벨 자리 그대로

    def test_refuses_with_the_arm_stretched_out_or_unknown(self):
        arm, calls = self.arm(dict(self.HOME, shoulder=(1, 3210)))
        task = self.run_task(arm, {'label': '책상'})
        self.assertEqual((task['result']['reason_code'], calls), ('arm_not_folded', []))
        self.assertIn('홈으로 돌아가', task['result']['reason'])
        arm, calls = self.arm(None)
        self.assertEqual(self.run_task(arm, {'label': '책상'})['result']['reason_code'], 'arm_not_folded')

    def tidy_arm(self, scene_objects, base_reason=None):
        arm, calls = self.arm(self.HOME)
        arm.base_check = lambda: base_reason
        arm.arrival_check = lambda: base_reason
        picked = []
        arm._pick_all = lambda task_id, objects=None: picked.append(objects) or dict(ok=True, reason=None, reason_code=None)
        scene = dict(objects=[dict(label=label, pickable=True) for label in scene_objects], in_tray=[])
        return arm, calls, picked, scene

    def test_drive_then_tidy_picks_what_the_photo_shows(self):
        arm, calls, picked, scene = self.tidy_arm(['tape_measure', 'red_cube'])
        with patch.object(arm_api.Arm, '_steady_scene', staticmethod(lambda grab: (None, scene))):
            task = self.run_task(arm, {'label': '책상', 'then': 'tidy'})
        self.assertEqual((task['state'], calls), ('done', [('책상', None)]))
        self.assertEqual(picked, [['red_cube', 'tape_measure']])                   # 정리 순서(흰 상자 → 정육면체 → 줄자)
        self.assertIn('책상에 도착해 정리했습니다', task['result']['note'])
        self.assertTrue(arm.arm_home)                                              # 정리는 HOME으로 끝난다

    def test_drive_then_tidy_with_nothing_on_the_desk(self):
        arm, _, picked, scene = self.tidy_arm([])
        with patch.object(arm_api.Arm, '_steady_scene', staticmethod(lambda grab: (None, scene))):
            task = self.run_task(arm, {'label': '책상', 'then': 'tidy'})
        self.assertEqual((task['state'], picked), ('done', []))
        self.assertIn('정리할 물체가 없습니다', task['result']['note'])

    def test_drive_then_tidy_waits_for_the_car_to_stop(self):
        arm, calls, picked, scene = self.tidy_arm(['red_cube'])
        moving = iter([None] + ['차가 움직이는 중'] * 3 + [None] * 20)            # 출발 전엔 멈춤, 도착 직후 잠깐 움직임
        arm.base_check = lambda: next(moving)
        arm.arrival_check = arm.base_check
        with patch.object(arm_api.Arm, '_steady_scene', staticmethod(lambda grab: (None, scene))), patch.object(arm_api.time, 'sleep'):
            task = self.run_task(arm, {'label': '책상', 'then': 'tidy'})
        self.assertEqual((task['state'], picked), ('done', [['red_cube']]))

    def test_bad_drive_requests(self):
        arm, _ = self.arm(self.HOME)
        self.assertEqual(arm.start('drive', {})[0], 400)
        self.assertEqual(arm.start('drive', {'label': '책상', 'gap': 2})[0], 400)
        self.assertEqual(arm.start('drive', {'label': '책상', 'then': 'dance'})[0], 400)
        self.assertEqual(arm.start('drive', {'label': '책상', 'mode': 'fly'})[0], 400)


class ChatDriveTests(unittest.TestCase):
    def setUp(self):
        state = dict(frame='map', pose=dict(x=-3.0, y=0.4, yaw_deg=180.0), labels=[dict(id=l['id'], name=l['name'], dist_m=1.0) for l in LABELS],
                     nav=dict(state='idle'), moving=False)
        patcher = patch.object(arm_llm.base_drive, 'base_state', return_value=state)
        patcher.start(); self.addCleanup(patcher.stop)
        self.arm = arm_api.Arm(run=lambda *a, **k: (0, {}, 0.), read_state=lambda: 'ok')

    def chat(self, text):
        class NoLlm:
            def __call__(self, *a, **k): raise AssertionError('LLM을 부르면 안 된다')
        return arm_llm.chat([], text, self.arm, client=NoLlm())

    def test_go_to_a_place_by_rule(self):
        for text in ('책상으로 이동해줘', '자비스, 책상 앞으로 가줘'):
            body = self.chat(text)
            self.assertEqual((body['proposal'], body['source']), ({'action': 'drive', 'label': '책상'}, 'rule'))

    def test_go_and_tidy_is_one_command_not_a_plain_tidy(self):
        for text in ('책상으로 가서 정리해줘', '자비스, 책상 가서 정리해'):
            body = self.chat(text)
            self.assertEqual(body['proposal'], {'action': 'drive', 'label': '책상', 'then': 'tidy'})
        self.assertIn('정리합니다', body['reply'])

    def test_status_includes_the_car(self):
        self.assertIn('차: 책상 근처', self.chat('상태 어때?')['reply'])

    def test_unknown_place_lists_the_known_ones(self):
        body = self.chat('부엌으로 가')
        self.assertIsNone(body['proposal'])
        self.assertIn('밥먹는곳', body['reply'])

    def test_where_am_i(self):
        body = self.chat('지금 어디야?')
        self.assertIn('갈 수 있는 장소', body['reply'])
        self.assertIsNone(body['proposal'])

    def test_arm_commands_are_not_taken_as_driving(self):
        self.assertEqual(self.chat('홈으로 돌아가')['proposal'], {'action': 'return_home'})


if __name__ == '__main__':
    unittest.main()
