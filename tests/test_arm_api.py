import json
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src'))
import arm_api
from http.server import ThreadingHTTPServer


class FakeProc:
    def __init__(self, interrupted):
        self.interrupted = interrupted
    def poll(self):
        return None
    def send_signal(self, _):
        self.interrupted.set()


class ApiBase(unittest.TestCase):
    """로봇·카메라 없이 가짜 실행기로 API 동작만 시험한다(실제 명령은 실행하지 않는다)."""

    def setUp(self):
        self.commands = []
        self.release = threading.Event()
        self.interrupted = threading.Event()
        self.state = 'ok'
        self.outcome = (0, dict(ok=True), 1.)
        for target in (patch.object(arm_api.menu, 'live_view_running', return_value=False),
                       patch.object(arm_api.menu, 'newest_result', return_value=None)):
            target.start(); self.addCleanup(target.stop)
        self.holds = 0
        self.hold_works = True
        self.base_reason = None                # 팀 map_web 대신(움직이면 이유 문장)
        self.arm = arm_api.Arm(run=self.fake_run, read_state=lambda: self.state, hold=self.fake_hold, base_check=lambda: self.base_reason)
        self.start_server()

    def fake_hold(self):                       # servo_driver.py hold 대신(실제 서보를 건드리지 않는다)
        self.holds += 1
        if self.hold_works:
            self.state = 'ok'

    def start_server(self, token=None, cors_origin=None):
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), arm_api.make_handler(self.arm, token, cors_origin))
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        self.base = f'http://127.0.0.1:{self.server.server_address[1]}'

    def fake_run(self, command, on_event=None, on_start=None, quiet=False):
        self.commands.append(command)
        if on_start:
            on_start(FakeProc(self.interrupted))
        on_event('phase', ('approach', '물체에 접근'))
        on_event('plan', ({}, ['칸 여유 4mm']))
        while not self.release.is_set() and not self.interrupted.is_set():
            time.sleep(.01)
        if self.interrupted.is_set():
            return 130, dict(ok=False, reason='cancelled'), .1
        return self.outcome

    def call(self, method, path, body=None, headers=None):
        request = urllib.request.Request(self.base + path, method=method, headers=headers or {},
                                         data=json.dumps(body).encode() if body is not None else (b'{}' if method == 'POST' else None))
        try:
            with urllib.request.urlopen(request, timeout=5) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read())

    def wait_done(self, task_id):
        for _ in range(300):
            status, task = self.call('GET', f'/tasks/{task_id}')
            if task['state'] != 'running':
                return task
            time.sleep(.01)
        self.fail('작업이 끝나지 않았습니다.')


class ApiTests(ApiBase):
    def test_rejects_anything_outside_the_allowed_list(self):
        for body in ({'object': 'banana'}, {'object': 'red_cube', 'slot': 4}, {'object': 'red_cube', 'slot': True}, {}):
            self.assertEqual(self.call('POST', '/pick_place', body)[0], 400, body)
        self.assertEqual(self.call('POST', '/format_disk', {})[0], 404)
        self.assertEqual(self.commands, [])

    def test_pick_place_runs_and_reports_phase_then_success(self):
        status, body = self.call('POST', '/pick_place', {'object': 'red_cube', 'slot': 2})
        self.assertEqual(status, 202)
        for _ in range(300):
            if self.call('GET', '/status')[1]['phase']:
                break
            time.sleep(.01)
        running = self.call('GET', '/status')[1]
        self.assertEqual((running['state'], running['phase_label']), ('busy', '물체에 접근'))
        self.release.set()
        task = self.wait_done(body['task_id'])
        self.assertEqual(task['state'], 'done')
        self.assertTrue(task['result']['ok'])
        self.assertEqual(task['result']['warnings'], ['칸 여유 4mm'])
        status = self.call('GET', '/status')[1]
        self.assertEqual((status['state'], status['arm_home']), ('idle', True))
        self.assertIn('red_cube', self.commands[0])

    def test_second_command_while_busy_is_refused(self):
        self.call('POST', '/pick_place', {'object': 'white_box'})
        status, body = self.call('POST', '/return_home')
        self.assertEqual((status, body['reason_code']), (409, 'busy'))
        self.release.set()

    def test_does_not_move_when_torque_is_off_away_from_home(self):
        self.state = 'off_elsewhere'
        _, body = self.call('POST', '/pick_place', {'object': 'red_cube'})
        task = self.wait_done(body['task_id'])
        self.assertEqual(task['result']['reason_code'], 'torque_off')
        self.assertIn('홈으로 돌아가', task['result']['reason'])               # 집기 같은 작업은 거부하고 복귀를 안내한다
        self.assertEqual((self.commands, self.holds), ([], 0))             # 토크를 켜지도 않는다

    def test_does_not_move_while_the_car_moves(self):
        self.state, self.base_reason = 'off_at_home', '차가 목적지로 이동하는 중입니다(Nav2 active).'
        _, body = self.call('POST', '/pick_place', {'object': 'red_cube'})
        task = self.wait_done(body['task_id'])
        self.assertEqual(task['result']['reason_code'], 'base_moving')
        self.assertEqual((self.commands, self.holds), ([], 0))             # 토크도 켜지 않는다

    def test_home_return_from_elsewhere_holds_the_arm_where_it_is_then_returns(self):
        self.state = 'off_elsewhere'
        self.outcome = (0, dict(ok=True, home_return_complete=True, final_hold_confirmed=True, restoration_verified=True), 1.)
        self.release.set()
        _, body = self.call('POST', '/return_home')
        task = self.wait_done(body['task_id'])
        self.assertEqual((task['state'], self.holds), ('done', 1))               # 현재 자세를 붙잡고
        self.assertEqual(self.commands, [arm_api.HOME_ANY_START_CMD])            # 복귀 명령 실행(사용자 요청: 시작 구간 검사만 건너뜀)
        self.assertEqual(arm_api.HOME_ANY_START_CMD, [*arm_api.HOME_CMD, '--any-start'])
        self.assertEqual(task['phases'][0]['phase'], 'torque_on')

    def test_home_return_from_elsewhere_stops_if_the_hold_fails(self):
        self.state, self.hold_works = 'off_elsewhere', False
        _, body = self.call('POST', '/return_home')
        task = self.wait_done(body['task_id'])
        self.assertEqual((task['result']['reason_code'], self.commands), ('torque_off', []))

    def test_mixed_torque_is_still_refused_even_for_home_return(self):
        self.state = 'mixed'
        _, body = self.call('POST', '/return_home')
        task = self.wait_done(body['task_id'])
        self.assertEqual((task['result']['reason_code'], self.commands, self.holds), ('torque_off', [], 0))

    def test_torque_is_turned_on_automatically_at_home(self):
        self.state = 'off_at_home'
        self.release.set()
        _, body = self.call('POST', '/pick_place', {'object': 'red_cube'})
        task = self.wait_done(body['task_id'])
        self.assertEqual((task['state'], self.holds), ('done', 1))
        self.assertEqual(task['phases'][0]['phase'], 'torque_on')
        self.assertIn('red_cube', self.commands[0])

    def test_failed_torque_on_does_not_move(self):
        self.state, self.hold_works = 'off_at_home', False
        _, body = self.call('POST', '/pick_place', {'object': 'red_cube'})
        task = self.wait_done(body['task_id'])
        self.assertEqual((task['result']['reason_code'], self.commands), ('torque_off', []))

    def test_near_fold_is_held_then_aligned_to_home_before_picking(self):
        self.state = 'off_near_fold'
        self.release.set()
        _, body = self.call('POST', '/pick_place', {'object': 'red_cube'})
        self.wait_done(body['task_id'])
        self.assertEqual(self.commands[0], arm_api.HOME_CMD)

    def test_failure_gets_a_stable_reason_code(self):
        self.outcome = (1, dict(ok=False, reason='unknown을 이 방향으로 놓을 여유가 부족합니다(외곽 여유 2mm, 필요 5mm).'), 1.)
        self.release.set()
        _, body = self.call('POST', '/pick_place', {'object': 'unknown', 'slot': 2})
        task = self.wait_done(body['task_id'])
        self.assertEqual((task['state'], task['result']['reason_code']), ('failed', 'slot_too_small'))
        self.assertIsNone(self.call('GET', '/status')[1]['arm_home'])

    def test_stop_interrupts_the_running_command(self):
        _, body = self.call('POST', '/pick_place', {'object': 'red_cube'})
        for _ in range(300):
            if self.commands:
                break
            time.sleep(.01)
        self.assertTrue(self.call('POST', '/stop')[1]['stopped'])
        task = self.wait_done(body['task_id'])
        self.assertTrue(self.interrupted.is_set())
        self.assertEqual(task['result']['reason_code'], 'cancelled')

    def test_stop_with_nothing_running_does_nothing(self):
        self.assertFalse(self.call('POST', '/stop')[1]['stopped'])

    def test_token_is_required_when_set(self):
        self.start_server(token='secret')
        self.assertEqual(self.call('GET', '/health')[0], 200)
        self.assertEqual(self.call('GET', '/status')[0], 401)
        self.assertEqual(self.call('POST', '/pick_place', {'object': 'red_cube'})[0], 401)
        self.assertEqual(self.call('GET', '/status', headers={'Authorization': 'Bearer secret'})[0], 200)
        self.assertEqual(self.commands, [])

    def test_refuses_to_open_to_the_network_without_a_token(self):
        with self.assertRaises(SystemExit):
            arm_api.main(['--host', '0.0.0.0'])

    def test_single_robot_arm_endpoint_takes_the_llm_tool_call_as_is(self):
        status, body = self.call('POST', '/robot_arm', {'action': 'pick_place', 'object': 'red_cube', 'slot': 2})
        self.assertEqual(status, 202)
        self.assertEqual(self.call('POST', '/robot_arm', {'action': 'status'})[1]['state'], 'busy')
        self.assertTrue(self.call('POST', '/robot_arm', {'action': 'stop'})[1]['stopped'])
        self.assertEqual(self.wait_done(body['task_id'])['result']['reason_code'], 'cancelled')
        self.assertEqual(self.call('POST', '/robot_arm', {'action': 'dance'})[0], 400)

    def test_cors_headers_only_when_configured(self):
        self.start_server(cors_origin='http://ui.local')
        request = urllib.request.Request(self.base + '/health')
        with urllib.request.urlopen(request, timeout=5) as response:
            self.assertEqual(response.headers['Access-Control-Allow-Origin'], 'http://ui.local')

    @staticmethod
    def label_of(command):
        return next(o for o in ('white_box', 'red_cube', 'tape_measure') if o in command)

    def flag_value(self, command, flag):
        return command[command.index(flag) + 1] if flag in command else None

    def run_with(self, outcomes):
        """명령마다 outcomes(차례로)를 돌려주는 가짜 실행기. 명령은 self.commands에 남는다."""
        outcomes = iter(outcomes)
        original = self.fake_run
        def run(command, **kw):
            original(command, **kw)
            return next(outcomes)
        self.arm.run = run

    def test_pick_all_plans_all_three_at_home_then_chains_without_home(self):
        self.release.set()
        _, body = self.call('POST', '/robot_arm', {'action': 'pick_all'})
        task = self.wait_done(body['task_id'])
        self.assertTrue(task['result']['ok'])
        plans, picks = self.commands[:3], self.commands[3:]
        self.assertEqual([self.label_of(c) for c in plans], ['red_cube', 'white_box', 'tape_measure'])
        self.assertTrue(all('--plan-out' in c and '--plan-in' not in c for c in plans))   # 1) 팔이 HOME일 때 세 물체를 미리 계획
        self.assertEqual([self.label_of(c) for c in picks], ['red_cube', 'white_box', 'tape_measure'])
        for plan, pick in zip(plans, picks):                                              # 2) 같은 계획 파일로 집는다(사진을 다시 찍지 않는다)
            self.assertEqual(self.flag_value(pick, '--plan-in'), self.flag_value(plan, '--plan-out'))
        self.assertEqual([('--skip-home-after-place' in c, '--start-above-tray' in c) for c in picks],
                         [(True, False), (True, True), (False, True)])                    # 마지막 물체만 HOME으로 돌아간다
        self.assertFalse(any('return_home.py' in ' '.join(c) for c in self.commands))
        self.assertEqual(task['phases'][0]['label'], '빨간 정육면체: 계획(사진)')
        self.assertEqual([i['object'] for i in task['items']], ['red_cube', 'white_box', 'tape_measure'])

    def test_pick_all_plan_failure_runs_the_planned_ones_and_ends_at_home(self):
        self.release.set()
        self.run_with([(0, dict(ok=True), 1.), (1, dict(ok=False, reason='planning failed: tray not found'), 1.), (0, dict(ok=True), 1.)])
        _, body = self.call('POST', '/pick_all')
        task = self.wait_done(body['task_id'])
        self.assertEqual(len(self.commands), 3)                   # 계획 정육면체, 계획 흰 상자(실패), 정육면체 집기. 줄자는 계획도 안 한다
        pick = self.commands[2]
        self.assertEqual(self.label_of(pick), 'red_cube')
        self.assertNotIn('--skip-home-after-place', pick)          # 계획된 마지막 물체라 놓은 뒤 HOME으로 돌아간다
        self.assertNotIn('--start-above-tray', pick)
        self.assertEqual(task['result']['reason_code'], 'tray_not_found')
        self.assertIn('흰 상자에서 멈췄습니다', task['result']['reason'])

    def test_pick_all_failure_after_a_chained_place_returns_home(self):
        self.release.set()
        self.run_with([(0, dict(ok=True), 1.)] * 3 + [(0, dict(ok=True), 1.), (1, dict(ok=False, reason='target not found'), 1.), (0, dict(home_return_complete=True, final_hold_confirmed=True, restoration_verified=True), 1.)])
        _, body = self.call('POST', '/pick_all')
        task = self.wait_done(body['task_id'])
        self.assertIn('return_home.py', ' '.join(self.commands[-1]))   # 정육면체를 놓고 트레이 위에 서 있었을 수 있다: HOME 복귀
        self.assertEqual(len(self.commands), 6)                        # 줄자는 실행하지 않았다
        self.assertIn('흰 상자에서 멈췄습니다', task['result']['reason'])
        self.assertEqual([i['ok'] for i in task['items']], [True, False])

    def test_pick_all_refused_chain_start_goes_home_then_retries_and_continues(self):
        self.release.set()
        refused = (1, dict(ok=False, attempts=[dict(attempt=1, exit_code=4)], reason='pick_auto exited with 4; not retrying'), 1.)
        self.run_with([(0, dict(ok=True), 1.)] * 3 + [(0, dict(ok=True), 1.), refused, (0, dict(home_return_complete=True, final_hold_confirmed=True, restoration_verified=True), 1.),
                                                        (0, dict(ok=True), 1.), (0, dict(ok=True), 1.)])
        _, body = self.call('POST', '/pick_all')
        task = self.wait_done(body['task_id'])
        self.assertTrue(task['result']['ok'])
        white_first, home, white_again, tape = self.commands[4:8]
        self.assertIn('--start-above-tray', white_first)
        self.assertIn('return_home.py', ' '.join(home))                       # 움직이기 전에 멈췄으니 검증된 HOME 복귀
        self.assertEqual((self.label_of(white_again), '--start-above-tray' in white_again, '--skip-home-after-place' in white_again),
                         ('white_box', False, True))                             # HOME에서 다시 하고, 놓은 뒤에는 다시 연속으로
        self.assertIn('--start-above-tray', tape)
        self.assertEqual([i['ok'] for i in task['items']], [True, True, True])

    def test_chain_refused_code_matches_pick_auto(self):
        from sorting.place_followup import CHAIN_REFUSED_EXIT_CODE
        self.assertEqual(arm_api.menu.CHAIN_REFUSED_EXIT_CODE, CHAIN_REFUSED_EXIT_CODE)

    def test_menu4_with_objects_chains_only_those(self):
        self.release.set()
        status, body = self.call('POST', '/api/menu/4', {'objects': ['tape_measure', 'red_cube']})
        task = self.wait_done(body['task_id'])
        self.assertTrue(task['result']['ok'])
        self.assertEqual([self.label_of(c) for c in self.commands], ['red_cube', 'tape_measure'] * 2)   # 순서는 메뉴 순서로
        picks = self.commands[2:]
        self.assertEqual([('--skip-home-after-place' in c, '--start-above-tray' in c) for c in picks], [(True, False), (False, True)])
        self.assertEqual(self.call('POST', '/api/menu/4', {'objects': ['cup']})[0], 400)
        self.assertEqual(self.call('POST', '/api/menu/4', {'objects': []})[0], 400)

    def test_pick_all_skips_objects_missing_from_the_desk(self):
        self.release.set()
        missing = (1, dict(ok=False, attempts=[], reason='planning failed: plan_sort.py: error: white_box not found in the current camera view'), 1.)
        self.run_with([(0, dict(ok=True), 1.), missing, (0, dict(ok=True), 1.), (0, dict(ok=True), 1.), (0, dict(ok=True), 1.)])
        _, body = self.call('POST', '/pick_all')
        task = self.wait_done(body['task_id'])
        self.assertTrue(task['result']['ok'])
        self.assertIn('흰 상자', task['result']['note'])
        picks = self.commands[3:]
        self.assertEqual([self.label_of(c) for c in picks], ['red_cube', 'tape_measure'])
        self.assertEqual([('--skip-home-after-place' in c, '--start-above-tray' in c) for c in picks], [(True, False), (False, True)])
        self.assertEqual(task['items'][-1], dict(object='white_box', ok=None, skipped=True, reason_code='object_not_found'))

    def test_pick_all_reports_unregistered_objects_left(self):
        self.release.set()
        planned = (0, dict(ok=True, unregistered_left=1), 1.)            # 계획이 등록 안 된 물체 1개를 장애물로 봤다
        missing = (1, dict(ok=False, attempts=[], reason='planning failed: white_box not found in the current camera view'), 1.)
        self.run_with([planned, missing, planned, (0, dict(ok=True), 1.), (0, dict(ok=True), 1.)])
        _, body = self.call('POST', '/pick_all')
        task = self.wait_done(body['task_id'])
        self.assertTrue(task['result']['ok'])
        self.assertEqual(task['result']['unregistered_left'], 1)
        self.assertIn('등록 안 된 물체 1개는 그대로 뒀습니다', task['result']['note'])
        self.assertIn('흰 상자', task['result']['note'])
        self.assertEqual([self.label_of(c) for c in self.commands[3:]], ['red_cube', 'tape_measure'])   # 등록 안 된 물체는 집지 않는다

    def test_pick_all_with_nothing_on_the_desk(self):
        self.release.set()
        missing = lambda label: (1, dict(ok=False, attempts=[], reason=f'planning failed: {label} not found in the current camera view'), 1.)
        self.run_with([missing('red_cube'), missing('white_box'), missing('tape_measure')])
        _, body = self.call('POST', '/pick_all')
        task = self.wait_done(body['task_id'])
        self.assertEqual((task['result']['ok'], task['result']['reason_code']), (False, 'object_not_found'))
        self.assertEqual(len(self.commands), 3)                       # 계획만 하고 아무것도 집지 않았다

    def test_pick_all_stop_does_not_move_home(self):
        _, body = self.call('POST', '/pick_all')
        time.sleep(.1)
        self.call('POST', '/stop')
        task = self.wait_done(body['task_id'])
        self.assertEqual(task['result']['reason_code'], 'cancelled')
        self.assertFalse(any('return_home.py' in ' '.join(c) for c in self.commands))

    def test_menu_endpoints_match_the_terminal_menu(self):
        self.release.set()
        expect = {1: 'white_box', 2: 'red_cube', 3: 'tape_measure'}
        for number, label in expect.items():
            status, body = self.call('POST', f'/api/menu/{number}')
            self.assertEqual(status, 202)
            self.wait_done(body['task_id'])
            self.assertIn(label, self.commands[-1])
        self.assertEqual(self.call('POST', '/api/menu/5', {'object': 'red_cube'})[0], 400)       # 칸 없음
        self.assertEqual(self.call('POST', '/api/menu/7', {})[0], 400)                           # 칸 기본값 없음
        self.assertEqual(self.call('POST', '/api/menu/8')[0], 404)
        status, body = self.call('POST', '/api/menu/5', {'object': 'red_cube', 'slot': 3})
        self.wait_done(body['task_id'])
        self.assertEqual(self.arm.task(body['task_id'])['params'], {'object': 'red_cube', 'slot': 3})
        status, body = self.call('POST', '/api/menu/6')
        self.assertEqual(self.wait_done(body['task_id'])['skill'], 'return_home')
        menu = self.call('GET', '/api/menu')[1]['menu']
        self.assertEqual([m['id'] for m in menu], list(range(1, 8)))
        self.assertTrue(all(m['description'] and m['path'] == f"/api/menu/{m['id']}" for m in menu))
        with urllib.request.urlopen(self.base + '/docs', timeout=5) as response:
            page = response.read().decode()
        self.assertIn('/api/menu/7', page)
        self.assertNotIn('\u2014', page)

    def test_get_image_returns_photo_objects_and_menus_without_moving(self):
        import numpy as np
        frame = (np.full((480, 640, 3), 200, np.uint8), np.full((480, 640), 600, np.uint16))
        with patch.object(arm_api, 'grab_astra', return_value=frame):
            status, body = self.call('POST', '/api/image')
        self.assertEqual(status, 200)
        self.assertTrue(body['image_jpeg_base64'])
        self.assertIn(6, body['available_menus'])                  # HOME 복귀는 항상
        self.assertEqual(self.commands, [])                         # 로봇 명령 없음
        with urllib.request.urlopen(self.base + '/api/image/latest.jpg', timeout=5) as response:
            self.assertEqual(response.headers['Content-Type'], 'image/jpeg')

    def test_get_image_is_refused_while_robot_runs(self):
        self.call('POST', '/api/menu/2')
        status, body = self.call('POST', '/api/image')
        self.assertEqual((status, body['reason_code']), (409, 'busy'))
        self.release.set()

    def test_reason_codes(self):
        for text, code in (('planning failed: tray not found', 'tray_not_found'), ('x not reachable', 'unreachable'),
                           ('unknown 대기 시간 초과', 'object_not_found'), ('???', 'failed')):
            self.assertEqual(arm_api.reason_code(text), code)


def call(name, **args):
    return {'function': {'name': name, 'arguments': args}}


def say(text='', *calls):
    return {'role': 'assistant', 'content': text, 'tool_calls': list(calls)}


class FakeLlm:
    """Ollama 응답을 흉내 낸다(네트워크 없음). replies: 차례로 돌려줄 message dict."""
    def __init__(self, replies):
        self.replies, self.requests = list(replies), []
    def __call__(self, messages, tools):
        self.requests.append(list(messages))
        return self.replies.pop(0)


class ChatTests(ApiBase):
    def chat(self, client, text='그거 2번 칸에 넣어줘', history=None):
        self.server.shutdown(); self.server.server_close()
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), arm_api.make_handler(self.arm, chat_client=client))
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.base = f'http://127.0.0.1:{self.server.server_address[1]}'
        return self.call('POST', '/chat', {'history': history or [], 'text': text})

    def test_motion_is_only_proposed_never_run(self):
        llm = FakeLlm([say('넣을게요.', call('sort_object', object_name='red_cube', slot=1)),
                       say('화면에서 실행을 눌러 주세요.')])
        status, body = self.chat(llm)
        self.assertEqual(status, 200)
        self.assertEqual(body['proposal'], {'action': 'pick_place', 'object': 'red_cube'})   # 기본 칸(1, 2026-10-27부터)은 기본 칸 제안과 같게 정리
        self.assertIn('바로 시작합니다', body['reply'])             # 제안 뒤 문장은 서버가 정한다(웹 화면이 바로 실행)
        self.assertEqual(body['source'], 'llm')
        self.assertEqual(self.commands, [])                       # 로봇 명령은 실행되지 않았다
        self.assertEqual(llm.requests[0][0]['role'], 'system')
        self.assertEqual(llm.requests[0][-1], {'role': 'user', 'content': '그거 2번 칸에 넣어줘'})

    def test_default_slot_all_and_home_become_proposals(self):
        for target in self.fake_scene(('white_box', True), ('red_cube', True), ('tape_measure', True)):     # 정리 제안은 사진을 찍는다
            target.start(); self.addCleanup(target.stop)
        for tool_call, expected in ((call('sort_object', object_name='tape_measure'), {'action': 'pick_place', 'object': 'tape_measure'}),
                                    (call('sort_all_objects'), {'action': 'pick_all', 'objects': ['red_cube', 'white_box', 'tape_measure']}),
                                    (call('arm_return_home'), {'action': 'return_home'})):
            status, body = self.chat(FakeLlm([say('', tool_call), say('확인해 주세요.')]))
            self.assertEqual(body['proposal'], expected)
        self.assertEqual(self.commands, [])

    def test_bad_input_is_rejected_and_only_first_motion_is_kept(self):
        llm = FakeLlm([say('', call('sort_object', object_name='banana'), call('sort_object', object_name='unknown')),
                       say('어느 칸에 넣을까요?')])
        status, body = self.chat(llm)
        self.assertIsNone(body['proposal'])
        tool_replies = [m['content'] for m in llm.requests[1] if m['role'] == 'tool']
        self.assertEqual(len(tool_replies), 2)
        self.assertTrue(all('"ok": false' in r for r in tool_replies))        # banana는 잘못된 물체, unknown은 칸 없음
        llm = FakeLlm([say('', call('sort_object', object_name='red_cube', slot=3), call('arm_return_home')), say('확인해 주세요.')])
        status, body = self.chat(llm)
        self.assertEqual(body['proposal'], {'action': 'pick_place', 'object': 'red_cube', 'slot': 3})
        self.assertIn('한 번에 하나', [m['content'] for m in llm.requests[1] if m['role'] == 'tool'][1])

    def test_status_is_answered_immediately(self):
        llm = FakeLlm([say('', call('get_arm_status')), say('대기 중입니다.')])
        status, body = self.chat(llm, '팔 어때?')                           # 규칙에 없는 말은 LLM이 도구로 확인한다
        self.assertEqual((body['reply'], body['proposal']), ('대기 중입니다.', None))
        self.assertIn('idle', [m['content'] for m in llm.requests[1] if m['role'] == 'tool'][0])

    def test_stop_runs_immediately(self):
        started = threading.Event()
        threading.Thread(target=lambda: self.arm.start('return_home', {}), daemon=True).start()
        time.sleep(.1)
        status, body = self.chat(FakeLlm([]), '멈춰')               # 규칙으로 바로 멈춘다(LLM을 부르지 않는다: 부르면 FakeLlm이 실패)
        self.assertEqual((body['executed'][0]['command'], body['source']), ('stop', 'rule'))
        self.assertEqual(body['reply'], '정지 명령을 보냈습니다.')
        self.assertTrue(self.interrupted.wait(2))

    def test_robot_name_is_stripped_and_answered_by_rule(self):
        status, body = self.chat(FakeLlm([]), '자비스야, 홈으로 돌아가')        # 이름을 떼고 규칙으로 처리한다(LLM을 부르지 않는다)
        self.assertEqual((body['proposal'], body['source']), ({'action': 'return_home'}, 'rule'))
        status, body = self.chat(FakeLlm([]), '자비스')
        self.assertIn('자비스', body['reply'])
        status, body = self.chat(FakeLlm([]), '너 이름이 뭐야?')
        self.assertEqual((body['reply'], body['proposal']), ('저는 자비스입니다. 물체를 칸에 넣거나 책상을 정리해 드려요.', None))

    def test_look_at_table_returns_scene_but_hides_image_from_model(self):
        import numpy as np
        frame = (np.zeros((48, 64, 3), np.uint8), np.zeros((48, 64), np.uint16))
        scene = dict(objects=[dict(label='red_cube', name='빨간 정육면체', pixel=[1, 2], position_mm=[250, 0], pickable=True)],
                     available_menus=[2, 5, 6], note='n')
        llm = FakeLlm([say('', call('look_at_table')), say('빨간 정육면체가 보입니다.')])
        with patch.object(arm_api, 'grab_astra', return_value=frame), patch.object(arm_api, 'describe_scene', return_value=scene):
            status, body = self.chat(llm, '책상 위 물건 알려줘')
        self.assertEqual(body['scene']['available_menus'], [2, 5, 6])
        self.assertIn('image_jpeg_base64', body['scene'])                      # 웹 화면에는 사진을 준다
        tool_reply = [m['content'] for m in llm.requests[1] if m['role'] == 'tool'][0]
        self.assertNotIn('image_jpeg_base64', tool_reply)                      # 모델에는 빼고 준다

    def test_again_repeats_the_last_task(self):
        status, body = self.chat(FakeLlm([]), '다시 해줘')
        self.assertEqual((body['proposal'], body['source']), (None, 'rule'))       # 아직 한 작업이 없다
        self.release.set()
        _, started = self.call('POST', '/pick_place', {'object': 'red_cube', 'slot': 3})
        self.wait_done(started['task_id'])
        status, body = self.chat(FakeLlm([]), '자비스, 다시 해줘')
        self.assertEqual(body['proposal'], {'action': 'pick_place', 'object': 'red_cube', 'slot': 3})

    def test_llm_claiming_an_action_without_a_tool_is_corrected(self):
        status, body = self.chat(FakeLlm([say('흰 상자, 빨간 정육면체, 줄자를 차례로 넣습니다. 바로 시작합니다.')]), '그거 해')
        self.assertIsNone(body['proposal'])
        self.assertIn('아무것도 실행하지 않았습니다', body['reply'])

    def test_status_and_table_questions_skip_the_llm(self):
        import numpy as np
        status, body = self.chat(FakeLlm([]), '자비스, 지금 상태 어때?')           # LLM을 부르면 FakeLlm이 실패한다
        self.assertEqual((body['source'], body['proposal']), ('rule', None))
        self.assertTrue(body['reply'].startswith('대기 중입니다.'))
        frame = (np.zeros((48, 64, 3), np.uint8), np.zeros((48, 64), np.uint16))
        scene = dict(objects=[dict(label='red_cube', name='빨간 정육면체', pixel=[1, 2], position_mm=[250, 0], pickable=True)],
                     available_menus=[2, 5, 6], note='n')
        with patch.object(arm_api, 'grab_astra', return_value=frame), patch.object(arm_api, 'describe_scene', return_value=scene):
            status, body = self.chat(FakeLlm([]), '작업대에 뭐 있어?')
        self.assertEqual(body['source'], 'rule')
        self.assertIn('빨간 정육면체', body['reply'])
        self.assertEqual(body['proposal']['action'], 'pick_all')                 # 정리는 제안만 하고 실행하지 않는다
        status, body = self.chat(FakeLlm([say('', call('sort_object', object_name='red_cube')), say('확인해 주세요.')]), '작업대에 있는 빨간 큐브 넣어줘')
        self.assertEqual(body['proposal'], {'action': 'pick_place', 'object': 'red_cube'})

    def test_history_is_sent_and_arguments_as_json_string_are_accepted(self):
        llm = FakeLlm([{'role': 'assistant', 'content': '', 'tool_calls': [{'function': {'name': 'sort_object', 'arguments': '{"object_name": "white_box"}'}}]},
                       say('확인해 주세요.')])
        status, body = self.chat(llm, '그것도 해줘', history=[{'role': 'user', 'content': '흰 상자 알려줘'}, {'role': 'assistant', 'content': '네'}])
        self.assertEqual(body['proposal'], {'action': 'pick_place', 'object': 'white_box'})
        self.assertEqual([m['role'] for m in llm.requests[0]], ['system', 'user', 'assistant', 'user'])

    def fake_scene(self, *objects):
        import numpy as np
        frame = (np.zeros((48, 64, 3), np.uint8), np.zeros((48, 64), np.uint16))
        scene = dict(objects=[dict(label=label, name=label, pixel=[1, 2], position_mm=[200, 0], pickable=pickable) for label, pickable in objects],
                     available_menus=[6], note='n')
        return patch.object(arm_api, 'grab_astra', return_value=frame), patch.object(arm_api, 'describe_scene', return_value=scene)

    def test_clear_commands_skip_the_llm(self):
        everything = self.fake_scene(('white_box', True), ('red_cube', True), ('tape_measure', True))
        for target in everything:
            target.start(); self.addCleanup(target.stop)
        all_three = {'action': 'pick_all', 'objects': ['red_cube', 'white_box', 'tape_measure']}
        cases = {'빨간 큐브 넣어줘': {'action': 'pick_place', 'object': 'red_cube'},
                 '줄자 1번 칸에 넣어': {'action': 'pick_place', 'object': 'tape_measure', 'slot': 1},
                 '빨간 큐브 1번 칸에 넣어': {'action': 'pick_place', 'object': 'red_cube'},
                 '새 물체 2번 칸에 넣어줘': {'action': 'pick_place', 'object': 'unknown', 'slot': 2},
                 '모르는 물체 3번 칸에 넣어줘': {'action': 'pick_place', 'object': 'unknown', 'slot': 3},
                 '흰 상자 다 넣어줘': {'action': 'pick_place', 'object': 'white_box'},
                 '다 정리해줘': all_three, '전부 넣어': all_three,
                 '정리해줘': all_three, '좀 치워줘': all_three, '분류해줘': all_three,
                 'HOME으로 돌아가': {'action': 'return_home'}, '팔 접어': {'action': 'return_home'}, '로봇팔 홈 자세로': {'action': 'return_home'}}
        for text, expected in cases.items():
            status, body = self.chat(FakeLlm([]), text)
            self.assertEqual((status, body['proposal'], body['source']), (200, expected, 'rule'), text)
            self.assertIn('바로 시작합니다', body['reply'])
        self.assertEqual(self.commands, [])

    def test_tidy_takes_a_photo_and_proposes_only_what_is_on_the_desk(self):
        self.arm.awaiting_desk_choice = True
        a, b = self.fake_scene(('red_cube', True), ('tape_measure', True), ('white_box', False), ('unknown', False))
        with a, b:
            status, body = self.chat(FakeLlm([]), '책상 정리해줘')
        self.assertEqual(body['proposal'], {'action': 'pick_all', 'objects': ['red_cube', 'tape_measure'], 'include_new': True})
        self.assertIn('빨간 정육면체, 줄자를 찾았습니다', body['reply'])
        self.assertIn('흰 상자는 보이지만 팔이 닿지 않는', body['reply'])
        self.assertIn('등록 안 된 물체 1개', body['reply'])
        self.assertIn('남는 빈 칸에 넣습니다', body['reply'])                  # 새 물체는 기존 물체를 넣고 남는 빈 칸에
        self.assertIn('image_jpeg_base64', body['scene'])                  # 화면에 사진을 띄운다
        self.assertEqual(self.commands, [])                                 # 사진만 찍고 로봇은 움직이지 않는다

    def test_tidy_with_an_empty_desk_proposes_nothing(self):
        a, b = self.fake_scene(('unknown', False))
        with a, b:
            status, body = self.chat(FakeLlm([]), '정리해줘')
        self.assertEqual(body['proposal'], {'action': 'pick_all', 'objects': [], 'include_new': True})      # 새 물체만 있어도 정리한다
        self.assertIn('새 물체', body['reply'])

    def test_tidy_with_nothing_on_the_desk_proposes_nothing(self):
        a, b = self.fake_scene()
        with a, b:
            status, body = self.chat(FakeLlm([]), '정리해줘')
        self.assertIsNone(body['proposal'])
        self.assertIn('정리할 물체가 없습니다', body['reply'])

    def test_tidy_without_the_camera_still_proposes_and_skips_missing_later(self):
        self.arm.awaiting_desk_choice = True
        with patch.object(self.arm, 'capture', return_value=(409, dict(ok=False, reason_code='busy', reason='작업 중'))):
            status, body = self.chat(FakeLlm([]), '책상 정리해줘')
        self.assertEqual(body['proposal'], {'action': 'pick_all'})
        self.assertIn('사진을 찍지 못했습니다', body['reply'])

    def test_llm_sort_all_also_takes_the_photo(self):
        a, b = self.fake_scene(('white_box', True))
        with a, b:
            status, body = self.chat(FakeLlm([say('', call('sort_all_objects')), say('확인해 주세요.')]), '저거 다 좀 해줘')
        self.assertEqual((body['proposal'], body['source']), ({'action': 'pick_all', 'objects': ['white_box']}, 'llm'))

    def situation(self):
        return self.call('POST', '/api/situation', {})

    def test_situation_photo_first_lists_actions_and_prepares_tidy(self):
        a, b = self.fake_scene(('white_box', True), ('tape_measure', True), ('red_cube', False), ('unknown', False))
        with a, b:
            status, body = self.situation()
        self.assertEqual(status, 200)
        self.assertIn('흰 상자, 줄자, 빨간 정육면체, 등록 안 된 물체 1개를 찾았습니다', body['reply'])
        self.assertEqual(body['proposal'], {'action': 'pick_all', 'objects': ['white_box', 'tape_measure'], 'include_new': True})   # 기본 행동: 책상 정리(새 물체 포함)
        joined = ' / '.join(body['actions'])
        for expected in ("'책상 정리해줘'", "'흰 상자 넣어줘'", "'줄자 넣어줘'", '빨간 정육면체는 팔이 닿지 않습니다', "새 물체: 책상 정리에 함께 들어갑니다", "'멈춰'"):
            self.assertIn(expected, joined)
        self.assertIn('image_jpeg_base64', body['scene'])
        self.assertEqual(self.commands, [])                                  # 사진만 찍고 로봇은 움직이지 않는다

    def test_situation_on_an_empty_desk_asks_what_to_do(self):
        a, b = self.fake_scene()
        with a, b:
            status, body = self.situation()
        self.assertIsNone(body['proposal'])
        self.assertIn('책상에 정리할 물체가 없습니다', body['reply'])

    def test_objects_already_in_the_tray_are_reported_not_sorted(self):
        import numpy as np
        frame = (np.zeros((48, 64, 3), np.uint8), np.zeros((48, 64), np.uint16))
        scene = dict(objects=[dict(label='red_cube', name='빨간 정육면체', pixel=[1, 2], position_mm=[230, 0], pickable=True)],
                     in_tray=[dict(label='tape_measure', name='줄자', slot=3)], available_menus=[2, 5, 6], note='n')
        with patch.object(arm_api, 'grab_astra', return_value=frame), patch.object(arm_api, 'describe_scene', return_value=scene):
            _, body = self.situation()
            _, tidy = self.chat(FakeLlm([]), '정리해줘')
        self.assertIn('트레이에는 줄자(3번 칸)가 이미 들어 있습니다', body['reply'])
        self.assertEqual(body['proposal'], {'action': 'pick_all', 'objects': ['red_cube']})
        self.assertEqual(tidy['proposal'], {'action': 'pick_all', 'objects': ['red_cube']})

    def test_situation_when_the_camera_is_busy(self):
        with patch.object(self.arm, 'capture', return_value=(409, dict(ok=False, reason_code='busy', reason='작업 중'))):
            status, body = self.situation()
        self.assertEqual((status, body['proposal']), (200, None))
        self.assertIn('사진을 찍지 못했습니다', body['reply'])

    def test_rules_ask_instead_of_guessing(self):
        for text in ('새 물체 넣어줘', '줄자 4번 칸에 넣어'):
            status, body = self.chat(FakeLlm([]), text)
            self.assertIsNone(body['proposal'], text)
            self.assertIn('칸', body['reply'])

    def test_questions_and_negations_go_to_the_llm(self):
        import arm_llm
        for text in ('작업대에 뭐 있어?', '줄자 넣을까?', '빨간 큐브 넣지 마', '컵 넣어줘', '빨간 큐브 넣고 흰 상자도 넣어줘', '안녕', '책상 정리하지 마', '책상 정리할까?'):
            self.assertIsNone(arm_llm.rule_command(text), text)

    def test_web_page_is_served(self):
        with urllib.request.urlopen(self.base + '/', timeout=5) as response:
            self.assertEqual(response.headers['Cache-Control'], 'no-store')
            self.assertIn('<title>자비스</title>', response.read().decode())

    def test_ollama_down_reports_setup_needed(self):
        import arm_llm
        def down(messages, tools):
            raise arm_llm.LlmUnavailable('Ollama에 연결하지 못했습니다')
        status, body = self.chat(down)
        self.assertEqual((status, body['reason_code']), (503, 'llm_unavailable'))

    def test_ollama_client_reports_missing_model_and_dead_server(self):
        import arm_llm
        with patch.object(arm_llm.urllib.request, 'urlopen', side_effect=arm_llm.urllib.error.URLError('refused')):
            with self.assertRaises(arm_llm.LlmUnavailable):
                arm_llm.OllamaClient(host='127.0.0.1:1')([], [])
        self.assertEqual(arm_llm.OllamaClient(host='127.0.0.1:11434').host, 'http://127.0.0.1:11434')


class TraySlotFallbackTests(unittest.TestCase):
    """칸에 넣은 물체가 트레이 모서리를 가려도 저장한 칸을 쓴다(화면 판단용). 다른 자리에 확실히 맞는 모서리가 있으면 쓰지 않는다."""

    def slots_with(self, scores, offsets):
        import numpy as np, tempfile
        tmp = tempfile.TemporaryDirectory(); self.addCleanup(tmp.cleanup)
        rgb, dep = Path(tmp.name) / 'rgb.png', Path(tmp.name) / 'depth.npy'
        rgb.write_bytes(b'x'); np.save(dep, np.zeros((2, 2)))
        stored = {1: {'x_mm': 340., 'y_mm': -170.}}
        detect = lambda bgr, depth, root, first_end=None, **kw: dict(found=False) if bgr is not None and bgr.shape == (480, 640, 3) and bgr.any() else dict(found=True, slots=stored)
        offset = dict(ok=False, offsets_px=offsets, scores=scores)
        with patch.object(arm_api.menu, 'TRAY_FRAME', (rgb, dep)), patch('cv2.imread', return_value=np.zeros((480, 640, 3), np.uint8)), \
             patch('tray.tray_detect.detect_tray', side_effect=detect), \
             patch('tray.tray_frame_check.tray_offset_px', return_value=offset), \
             patch('tray.tray_frame_check.register_tray', return_value=dict(ok=False)):
            return arm_api.tray_slots(np.ones((480, 640, 3), np.uint8), np.zeros((480, 640), np.uint16))

    def test_occluded_corners_keep_the_stored_slots(self):
        self.assertEqual(self.slots_with([.80, .61, .93], [25.5, 24.2, 2.3]), {1: {'x_mm': 340., 'y_mm': -170.}})

    def test_a_corner_matching_elsewhere_means_the_tray_moved(self):
        self.assertIsNone(self.slots_with([.95, .61, .93], [12., 24.2, 2.3]))


class CameraWorkerTests(unittest.TestCase):
    """열어 둔 카메라 프로세스: 사진을 받고, 작업 전에 닫고, 다시 부르면 새로 띄운다. 실패하면 None(예전 방식으로 찍게)."""
    FAKE = """
import sys, numpy as np
print('ready', flush=True)
for line in sys.stdin:
    cmd, _, path = line.strip().partition(' ')
    if cmd == 'quit': break
    if MODE == 'fail': print('error camera', flush=True); continue
    np.savez(path, bgr=np.full((4, 4, 3), 7, np.uint8), depth=np.full((4, 4), 500, np.uint16)); print('ok', flush=True)
"""

    def worker(self, mode='ok'):
        import tempfile
        folder = tempfile.TemporaryDirectory(); self.addCleanup(folder.cleanup)
        script = Path(folder.name) / 'fake_worker.py'
        script.write_text(f'MODE = {mode!r}\n' + self.FAKE)
        camera = arm_api.CameraWorker([sys.executable, str(script)])
        self.addCleanup(camera.release)
        return camera

    def test_grab_release_and_restart(self):
        camera = self.worker()
        bgr, depth = camera.grab()
        self.assertEqual((int(bgr[0, 0, 0]), int(depth[0, 0])), (7, 500))
        first = camera.proc.pid
        camera.grab()
        self.assertEqual(camera.proc.pid, first)                   # 같은 프로세스로 바로 찍는다
        camera.release()
        self.assertIsNone(camera.proc)
        self.assertIsNotNone(camera.grab())                        # 작업 뒤에는 새로 띄운다

    def test_camera_error_returns_none(self):
        camera = self.worker('fail')
        self.assertIsNone(camera.grab())
        self.assertIsNone(camera.proc)

    def test_task_releases_the_camera_first(self):
        released = []
        arm = arm_api.Arm(run=lambda *a, **k: (0, dict(home_return_complete=True, final_hold_confirmed=True, restoration_verified=True), 0.), read_state=lambda: 'ok')
        with patch.object(arm_api.CAMERA, 'release', side_effect=lambda: released.append(1)), \
             patch.object(arm_api.menu, 'live_view_running', return_value=False):
            arm._execute('t1', 'return_home', {})
        self.assertEqual(released, [1])


class SteadySceneTests(unittest.TestCase):
    """사진 한 장이 튀어도 두 번 나온 판단을 쓴다."""

    def run_with(self, scenes):
        import numpy as np
        frames = iter(range(10))
        grab = lambda: (np.full((2, 2, 3), next(frames), np.uint8), np.zeros((2, 2), np.uint16))
        with patch.object(arm_api, 'describe_scene', side_effect=[dict(objects=[dict(label=l) for l in s], in_tray=[]) for s in scenes]):
            return arm_api.Arm._steady_scene(grab)

    def test_two_matching_shots_are_enough(self):
        bgr, scene = self.run_with([['red_cube'], ['red_cube']])
        self.assertEqual((scene['shots'], int(bgr[0, 0, 0])), (2, 1))

    def test_an_odd_shot_is_outvoted(self):
        bgr, scene = self.run_with([['red_cube'], ['red_cube', 'unknown', 'unknown'], ['red_cube']])
        self.assertEqual(([o['label'] for o in scene['objects']], scene['shots']), (['red_cube'], 3))
        self.assertNotIn('unstable', scene)

    def test_all_different_is_marked_unstable(self):
        bgr, scene = self.run_with([['red_cube'], ['white_box'], []])
        self.assertTrue(scene['unstable'])


class BaseMovingTests(unittest.TestCase):
    """팀 map_web에 차가 움직이는지 묻는다(읽기만). 닿지 않으면 주행을 안 쓰는 중으로 본다."""

    def serve(self, drive, nav):
        from http.server import BaseHTTPRequestHandler
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a): pass
            def do_GET(self):
                body = json.dumps(drive if self.path == '/api/drive' else {'nav': nav}).encode()
                self.send_response(200); self.send_header('Content-Length', str(len(body))); self.end_headers(); self.wfile.write(body)
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close); self.addCleanup(server.shutdown)
        return f'http://127.0.0.1:{server.server_address[1]}'

    def test_still_car_allows(self):
        url = self.serve({'moving': False, 'odom': {'linear': 0.002, 'angular': 0.0}}, {'state': 'succeeded'})
        self.assertIsNone(arm_api.base_moving(url))

    def test_odom_speed_drive_command_or_nav_goal_refuses(self):
        self.assertIn('움직이는 중', arm_api.base_moving(self.serve({'moving': False, 'odom': {'linear': 0.1, 'angular': 0}}, {})))
        self.assertIn('수동 주행', arm_api.base_moving(self.serve({'moving': True, 'odom': None}, {})))
        self.assertIn('Nav2 active', arm_api.base_moving(self.serve({'moving': False, 'odom': {'linear': 0, 'angular': 0}},
                                                                    {'state': 'active', 'target': '책상'})))

    def test_no_map_web_or_disabled_allows(self):
        self.assertIsNone(arm_api.base_moving('http://127.0.0.1:9'))
        self.assertIsNone(arm_api.base_moving(''))


class LogTests(ApiBase):
    """웹 로그창: 실패의 원문 이유와 마지막 출력이 남고, since로 새 기록만 받는다."""

    def logs(self, since=0):
        status, body = self.call('GET', f'/api/logs?since={since}')
        self.assertEqual(status, 200)
        return body

    def test_failed_task_keeps_the_raw_reason_and_the_last_output(self):
        raw = 'existing fixed goal is not stationary; takeover refused'
        self.outcome = (1, dict(ok=False, reason=raw, _tail=['approach 끝', 'takeover 검사']), 1.)
        self.release.set()
        _, started = self.call('POST', '/pick_place', {'object': 'red_cube'})
        self.wait_done(started['task_id'])
        body = self.logs()
        errors = [e for e in body['entries'] if e['level'] == 'error']
        script = next(e for e in errors if e['message'] == raw)                  # 실행 프로그램이 알려 준 원문
        self.assertIn('종료 코드 1', script['detail'])
        self.assertIn('takeover 검사', script['detail'])                          # 마지막 출력
        summary = next(e for e in errors if e['source'] == '작업')                 # 화면에 보이는 문장
        self.assertIn('실패', summary['message'])
        self.assertIn(started['task_id'], summary['detail'])

    def test_success_and_refusals_are_logged_with_their_level(self):
        self.release.set()
        _, started = self.call('POST', '/pick_place', {'object': 'red_cube'})
        self.wait_done(started['task_id'])
        self.base_reason = '차가 움직이는 중입니다.'
        _, refused = self.call('POST', '/pick_place', {'object': 'red_cube'})
        self.wait_done(refused['task_id'])
        levels = [(e['level'], e['source']) for e in self.logs()['entries'] if e['source'] == '작업']
        self.assertIn(('ok', '작업'), levels)
        self.assertIn(('warn', '작업'), levels)                                    # base_moving은 바로 고칠 수 있는 거부라 주의

    def test_since_returns_only_newer_entries_and_boot_identifies_the_server(self):
        first = self.logs()
        self.arm.log.add('info', '시험', '하나')
        newer = self.logs(first['last'])
        self.assertEqual([e['message'] for e in newer['entries']], ['하나'])
        self.assertEqual(newer['boot'], first['boot'])
        self.assertEqual(self.logs(newer['last'])['entries'], [])

    def test_api_errors_are_logged_but_not_the_log_polling_itself(self):
        self.assertEqual(self.call('POST', '/pick_place', {'object': 'cup'})[0], 400)
        entries = self.logs()['entries']
        self.assertTrue(any(e['source'] == 'API' and e['level'] == 'warn' and '400' in e['message'] for e in entries))
        before = self.logs()['last']
        self.call('GET', '/api/logs?since=999999')
        self.assertEqual(self.logs()['last'], before)

    def test_ring_buffer_keeps_the_newest(self):
        log = arm_api.EventLog(size=3)
        for n in range(5):
            log.add('info', 's', str(n))
        entries, last = log.since(0)
        self.assertEqual(([e['message'] for e in entries], last), (['2', '3', '4'], 5))

    def test_logs_need_the_token(self):
        self.start_server(token='t')
        self.assertEqual(self.call('GET', '/api/logs')[0], 401)
        self.assertEqual(self.call('GET', '/api/logs', headers={'Authorization': 'Bearer t'})[0], 200)


class SttTests(ApiBase):
    """로컬 음성 인식 주소(/api/stt). 가짜 인식기로 서버 쪽 동작을, 실제 Whisper로 녹음 파일 하나를 확인한다."""

    def stt_server(self, transcriber, token='t'):
        self.server.shutdown(); self.server.server_close()
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), arm_api.make_handler(self.arm, token, transcriber=transcriber))
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.base = f'http://127.0.0.1:{self.server.server_address[1]}'

    def post_audio(self, data, kind='audio/webm;codecs=opus', token='t'):
        headers = {'Content-Type': kind}
        if token:
            headers['Authorization'] = f'Bearer {token}'
        request = urllib.request.Request(self.base + '/api/stt', data=data, method='POST', headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read())

    def test_returns_text_and_never_moves_the_robot(self):
        seen = []
        self.stt_server(lambda audio, suffix: seen.append((audio, suffix)) or dict(text='빨간 큐브 넣어줘', seconds=.8))
        status, body = self.post_audio(b'x' * 2000)
        self.assertEqual((status, body['text']), (200, '빨간 큐브 넣어줘'))
        self.assertEqual(seen, [(b'x' * 2000, '.webm')])
        self.assertEqual(self.commands, [])
        self.post_audio(b'x' * 2000, kind='audio/ogg;codecs=opus')
        self.assertEqual(seen[-1][1], '.ogg')

    def test_needs_token_and_sane_size(self):
        self.stt_server(lambda audio, suffix: dict(text='', seconds=0.))
        self.assertEqual(self.post_audio(b'x' * 2000, token=None)[0], 401)
        self.assertEqual(self.post_audio(b'')[0], 400)
        # 너무 큰 녹음은 본문을 읽기 전에 거절한다(Content-Length만 보고). 본문을 다 보내면 서버가 먼저 닫아 클라이언트가 Broken pipe를 볼 수 있어 헤더만 보낸다
        import http.client
        conn = http.client.HTTPConnection('127.0.0.1', self.server.server_address[1], timeout=5)
        conn.putrequest('POST', '/api/stt')
        for k, v in (('Authorization', 'Bearer t'), ('Content-Type', 'audio/webm'), ('Content-Length', str(arm_api.arm_stt.MAX_AUDIO_BYTES + 1))):
            conn.putheader(k, v)
        conn.endheaders()
        self.assertEqual(conn.getresponse().status, 413)
        conn.close()

    def test_missing_whisper_and_broken_audio_are_reported(self):
        def missing(audio, suffix):
            raise arm_api.arm_stt.SttUnavailable('pip install faster-whisper')
        self.stt_server(missing)
        status, body = self.post_audio(b'x' * 2000)
        self.assertEqual((status, body['reason_code']), (503, 'stt_unavailable'))
        def broken(audio, suffix):
            raise ValueError('invalid data')
        self.stt_server(broken)
        self.assertEqual(self.post_audio(b'x' * 2000)[1]['reason_code'], 'stt_error')

    def test_cleanup_drops_whisper_silence_phrases(self):
        clean = arm_api.arm_stt.clean
        self.assertEqual(clean(' 새 물체, 2번 칸에 넣어. '), '새 물체 2번 칸에 넣어')
        self.assertEqual(clean('멈춰!'), '멈춰')
        self.assertEqual(clean('시청해주셔서 감사합니다.'), '')

    @unittest.skipUnless(__import__('importlib').util.find_spec('faster_whisper'), 'faster-whisper가 없어 실제 인식 시험을 건너뛴다')
    def test_real_whisper_on_recorded_command(self):
        self.stt_server(None)
        data = ROOT / 'tests' / 'data'
        status, body = self.post_audio((data / 'stt_red_cube.webm').read_bytes())
        self.assertEqual((status, body['text']), (200, '빨간 큐브 넣어줘'))
        self.assertEqual(self.post_audio((data / 'stt_silence.webm').read_bytes())[1]['text'], '')   # 무음에 문장을 지어내지 않는다


class UnknownCommandTests(unittest.TestCase):
    def test_unknown_command_retakes_the_tray_when_it_moved(self):
        with patch.object(arm_api.menu.Path, 'exists', return_value=True):
            command = arm_api.menu.unknown_command(3)
        self.assertIn('--tray-fallback-live', command)
        self.assertIn('--tray-frame', command)


if __name__ == '__main__':
    unittest.main()
