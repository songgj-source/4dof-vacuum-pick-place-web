#!/usr/bin/env python3
"""로봇팔을 HTTP API로 부른다(다른 팀원의 LLM·주행 코드가 호출). 실행 = 로봇이 움직인다.

  python scripts/arm_api.py                       # 이 장비(127.0.0.1:8000)에서만 접근
  python scripts/arm_api.py --host 0.0.0.0 --token 비밀값   # 다른 장비에서 접근(토큰 필수)

명령은 허용 목록만 받는다. LLM이 서보·모터 값을 직접 만들지 못한다.
  POST /pick_place   {"object": "red_cube", "slot": 2}   → 202 {"task_id": ...}  (약 1분 걸리므로 바로 돌려주고 /status로 확인)
  POST /return_home                                      → 202
  POST /pick_all                                         → 202  세 물체 모두 넣기(메뉴 4번, 앞 물체가 실패하면 거기서 멈춘다)
  POST /stop                                             → 실행 중인 작업을 즉시 중단(팔은 그 자리에 멈춘다)
  GET  /status                                           → 상태·지금 단계(음성 안내용)
  GET  /tasks/<id>                                       → 작업 결과·단계 기록
  GET  /            → 웹 화면(web/index.html): 버튼·진행 상태·정지·채팅
  POST /chat         {"history": [...], "text": "빨간 상자 2번 칸에 넣어줘"} → 로컬 LLM(Ollama)이 명령을 제안(움직이는 명령은 화면에서 확인해야 실행)
  GET  /health
  POST /robot_arm    {"action": "pick_place"|"return_home"|"status"|"stop", "object": ..., "slot": ...}
                     → LLM 도구(function) 호출 내용을 그대로 넘기는 한 개짜리 주소(docs/ROBOT_API.md의 도구 정의와 같은 모양)

object: white_box(기본 1번 칸) · red_cube(2) · tape_measure(3) · unknown(새 물체, 시작 후 "물체를 놓으세요" 단계에서 놓는다, 기본 3번 칸)
한 번에 한 작업만 받는다. 실행 중에 다른 명령이 오면 409(busy). 토크가 꺼졌거나 팔 상태를 읽지 못하면 움직이지 않고 거절한다.
"""
import argparse
import collections
import hmac
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import urllib.parse
import traceback
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))   # 물체 인식(perception) 모듈
import sort_menu as menu     # 명령 만들기·실행·상태 판정은 메뉴와 같은 코드를 쓴다
import arm_stt               # 로컬 음성 인식(faster-whisper는 처음 쓸 때 불러온다)
import base_drive            # 팀 주행 서버(map_web)로 차를 움직인다: base_state, goto_label_wall
from hardware import devices

OBJECTS = {'white_box': 1, 'red_cube': 2, 'tape_measure': 3, 'unknown': 3}      # 물체 → 기본 칸
NAMES = {'white_box': '흰 상자', 'red_cube': '빨간 정육면체', 'tape_measure': '줄자', 'unknown': 'unknown'}
ALL_ORDER = ('white_box', 'red_cube', 'tape_measure')     # 메뉴 4번: 흰 상자 → 정육면체 → 줄자, 각자 기본 칸
HOME_CMD = [menu.PY, str(menu.ROOT / 'scripts' / 'return_home.py'), '--execute', '--supported', '--travel-rate', '150', '--tracking-limit', '32']
# 토크가 꺼진 팔이 HOME(또는 접힌 HOME 근처)에 놓여 있으면 작업 전에 토크를 켜서 그 자리에 붙잡는다(메뉴에서 Enter로 하던 일). ARM_AUTO_TORQUE=0이면 예전처럼 거부한다.
HOLD_CMD = [menu.PY, str(menu.ROOT / 'scripts' / 'servo_driver.py'), 'hold', '--arm', '--supported', '--duration', '1.5']
AUTO_TORQUE = os.environ.get('ARM_AUTO_TORQUE', '1') != '0'
# 팀 주행(ros2_slam test/map_web.py, 같은 Pi)에 차가 움직이는지 묻는다(읽기만). 움직이면 팔 작업을 시작하지 않는다.
# map_web이 꺼져 있으면(주행을 안 쓰는 중) 그냥 진행한다. 빈 값이면 확인하지 않는다.
BASE_URL = os.environ.get('ARM_BASE_URL', 'http://127.0.0.1:8082').rstrip('/')
BASE_MOVING_NAV = ('backing', 'sending', 'active', 'canceling')     # map_web nav.state: Go to 진행 중
BASE_LINEAR, BASE_ANGULAR = .01, .03                                 # odom 실제 속도가 이보다 크면 움직이는 중


def base_moving(url=BASE_URL, timeout_s=1.5):
    """차가 움직이는 중이면 그 이유 문장, 아니면 None. map_web에 닿지 않으면 None(주행을 안 쓰는 중으로 본다)."""
    if not url:
        return None
    import urllib.request
    def get(path):
        with urllib.request.urlopen(url + path, timeout=timeout_s) as response:
            return json.loads(response.read())
    try:
        drive = get('/api/drive')
    except Exception:
        return None
    odom = drive.get('odom') or {}
    if abs(odom.get('linear') or 0.) > BASE_LINEAR or abs(odom.get('angular') or 0.) > BASE_ANGULAR:
        return f"차가 움직이는 중입니다(속도 {odom.get('linear', 0):.2f} m/s, 회전 {odom.get('angular', 0):.2f} rad/s)."
    if drive.get('moving'):
        return '차에 주행 명령이 들어가는 중입니다(수동 주행).'
    try:
        nav = (get('/api/get_map_info').get('nav') or {})
    except Exception:
        return None
    if nav.get('state') in BASE_MOVING_NAV:
        return f"차가 목적지로 이동하는 중입니다(Nav2 {nav.get('state')}"+ (f", {nav['target']}" if nav.get('target') else '') + ').'
    return None


def base_stationary(url=BASE_URL):
    """Combined drive/arm task: unknown state never counts as a stopped base."""
    if not url:
        return '주행 서버 주소가 없어 정지 상태를 확인할 수 없습니다.'
    try:
        drive = base_drive._call('GET', '/api/drive', url=url, timeout_s=1.5)
        state = base_drive._call('GET', '/api/state', url=url, timeout_s=1.5)
    except base_drive.BaseError as exc:
        return str(exc)
    odom = drive.get('odom')
    if drive.get('driver_connected') is not True or not isinstance(odom, dict):
        return '차 드라이버 연결 또는 최신 속도 정보를 확인할 수 없습니다.'
    speeds = (odom.get('linear'), odom.get('angular'))
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in speeds):
        return '차의 속도 정보가 올바르지 않아 정지를 확인할 수 없습니다.'
    if drive.get('moving') is not False or abs(speeds[0]) > BASE_LINEAR or abs(speeds[1]) > BASE_ANGULAR:
        return '차가 아직 움직이거나 주행 명령이 들어가는 중입니다.'
    if (state.get('nav') or {}).get('state') in BASE_MOVING_NAV:
        return '목적지로 주행 중입니다.'
    if any((state.get(k) or {}).get('state') == 'running' for k in ('wall', 'approach')):
        return '책상 근접 이동이 진행 중입니다.'
    return None

# 실패 원문 → 고정 코드(호출한 쪽이 문장 대신 코드로 분기한다). 위에서부터 처음 맞는 것.
REASON_CODES = (
    ('cancelled', 'cancelled'),
    ('more than one unknown', 'multiple_new_objects'),
    ('unknown not found', 'object_not_found'), ('not found in the current camera view', 'object_not_found'), ('nothing to sort', 'object_not_found'),
    ('대기 시간 초과', 'object_not_found'),
    ('tray not found', 'tray_not_found'),
    ('the tray moved since the stored frame', 'tray_moved'),
    ('여유가 부족', 'slot_too_small'), ('보다 큽니다', 'slot_too_small'),
    ('not reachable', 'unreachable'), ('pick envelope', 'unreachable'), ('carry height', 'unreachable'),
    ('위치·크기가 바뀌었', 'object_moved'),
    ('not held', 'not_held'),
    ('unexpected torque', 'torque_off'),
    ('no devices found', 'device_absent'), ('device is absent', 'device_absent'),
)


def reason_code(text):
    text = str(text or '')
    return next((code for needle, code in REASON_CODES if needle in text), 'failed')


class EventLog:
    """웹 로그창용 최근 사건 기록(메모리, 500건). 화면에는 '알 수 없는 오류'로만 나오는 실패의 원문(실행 프로그램이 알려 준 이유, 마지막 출력)을 남긴다.
    level: info | ok | warn | error. boot는 서버를 다시 켤 때마다 바뀌어, 화면이 번호를 처음부터 다시 받게 한다."""

    def __init__(self, size=500):
        self.items = collections.deque(maxlen=size)
        self.lock = threading.Lock()
        self.counter = 0
        self.boot = uuid.uuid4().hex[:8]

    def add(self, level, source, message, detail=None):
        with self.lock:
            self.counter += 1
            self.items.append(dict(id=self.counter, t=round(time.time(), 3), time=time.strftime('%H:%M:%S'), level=level, source=source,
                                   message=str(message)[:500], detail=str(detail)[:2000] if detail else None))
            return self.counter

    def since(self, after=0, limit=200):
        """(after보다 새로운 기록 최대 limit건, 마지막 번호)."""
        with self.lock:
            rows = [dict(row) for row in self.items if row['id'] > after]
            return rows[-limit:], self.counter


class Arm:
    """한 번에 한 작업을 돌리는 상태. HTTP 핸들러와 작업 스레드가 함께 쓴다."""

    def __init__(self, run=menu.run_streaming, read_state=None, hold=None, base_check=None, drive=None, read_pose=None,
                 arrival_check=None):
        self.run = run
        self.log = EventLog()
        self.drive = drive or base_drive.drive_to
        self.read_pose = read_pose or menu.read_arm_state       # {관절: (토크, 위치)} 또는 None
        self.base_check = base_check or base_moving
        self.arrival_check = arrival_check or base_stationary
        self.read_state = read_state or (lambda: menu.classify_arm(menu.read_arm_state()))
        self.hold = hold or (lambda: subprocess.run(HOLD_CMD, cwd=menu.ROOT, capture_output=True, text=True, timeout=60))
        self.lock = threading.Lock()
        self.proc = None
        self.stop_requested = False
        self.capturing = False
        self.scene = None                     # 마지막 get image 결과(웹 화면 표시용 JPEG 포함)
        self.awaiting_desk_choice = False
        self.tasks = {}
        self.current = None
        self.arm_home = None                  # True/False/None(모름): 성공한 작업 뒤에는 HOME, 중단·실패 뒤에는 모름

    # ---- 조회 ----
    def status(self):
        with self.lock:
            task = self.tasks.get(self.current) if self.current else None
            last = max(self.tasks.values(), key=lambda t: t['started'], default=None)
            shown = task or last
            return dict(state='busy' if task else 'idle', arm_home=self.arm_home,
                        task_id=shown['id'] if shown else None, skill=shown['skill'] if shown else None,
                        phase=shown['phase'] if shown else None, phase_label=shown['phase_label'] if shown else None,
                        task_state=shown['state'] if shown else None,
                        elapsed_s=round((time.monotonic() - shown['started']) if task else shown['seconds'], 1) if shown else None)

    def task(self, task_id):
        with self.lock:
            task = self.tasks.get(task_id)
            return json.loads(json.dumps({k: v for k, v in task.items() if k != 'started'})) if task else None

    # ---- 명령 ----
    def start(self, skill, params):
        """작업을 시작한다. (HTTP 상태, 응답 dict)."""
        if skill == 'pick_place':
            label = params.get('object')
            if label not in OBJECTS:
                return 400, dict(ok=False, reason_code='bad_request', reason=f'object는 {sorted(OBJECTS)} 중 하나여야 합니다.')
            slot = params.get('slot', OBJECTS[label])
            if isinstance(slot, bool) or slot not in (1, 2, 3):
                return 400, dict(ok=False, reason_code='bad_request', reason='slot은 1, 2, 3 중 하나여야 합니다.')
            command = ('pick_place', dict(object=label, slot=slot))
        elif skill == 'return_home':
            command = ('return_home', {})
        elif skill == 'pick_all':
            objects = params.get('objects')
            if objects is None:
                command = ('pick_all', {})               # 세 물체(책상에 없는 것은 계획에서 건너뛴다)
            elif (not isinstance(objects, list) or not objects or len(set(objects)) != len(objects)
                  or any(o not in ALL_ORDER for o in objects)):
                return 400, dict(ok=False, reason_code='bad_request', reason=f'objects는 {list(ALL_ORDER)} 중 하나 이상(중복 없이)이어야 합니다.')
            else:
                command = ('pick_all', dict(objects=[o for o in ALL_ORDER if o in objects]))
        elif skill == 'drive':
            label, gap = params.get('label'), params.get('gap')
            if not isinstance(label, str) or not label.strip() or len(label) > 40:
                return 400, dict(ok=False, reason_code='bad_request', reason='label(갈 장소 이름)이 필요합니다.')
            if gap is not None and (isinstance(gap, bool) or not isinstance(gap, (int, float))
                                    or not base_drive.GAP_LIMITS[0] <= gap <= base_drive.GAP_LIMITS[1]):
                return 400, dict(ok=False, reason_code='bad_request', reason=f'gap은 {base_drive.GAP_LIMITS[0]}~{base_drive.GAP_LIMITS[1]} m여야 합니다.')
            then, mode = params.get('then'), params.get('mode', 'precise')
            if mode not in base_drive.MODES:
                return 400, dict(ok=False, reason_code='bad_request', reason=f'mode는 {base_drive.MODES} 중 하나입니다(precise: 라벨 자리 그대로, wall: 앞 벽에 붙기).')
            if then not in (None, 'tidy', 'inspect'):
                return 400, dict(ok=False, reason_code='bad_request', reason="then은 'inspect' 또는 'tidy'(촬영 후 정리) 중 하나입니다.")
            command = ('drive', dict(label=label.strip(), mode=mode, **({'gap': float(gap)} if gap is not None else {}), **({'then': then} if then else {})))
        else:
            return 404, dict(ok=False, reason_code='bad_request', reason='알 수 없는 명령입니다.')
        with self.lock:
            if self.current or self.capturing:
                return 409, dict(ok=False, reason_code='busy', reason='다른 작업을 실행 중입니다.', task_id=self.current)
            task_id = uuid.uuid4().hex[:8]
            self.tasks[task_id] = dict(id=task_id, skill=command[0], params=command[1], state='running', phase=None,
                                       phase_label=None, phases=[], warnings=[], result=None, item=None, items=[],
                                       started=time.monotonic(), seconds=0.)
            self.current = task_id
            self.stop_requested = False
            self.awaiting_desk_choice = False
        self.log.add('info', '작업', f'{command[0]} 시작 {json.dumps(command[1], ensure_ascii=False)}', f'task {task_id}')
        threading.Thread(target=self._work, args=(task_id, *command), daemon=True).start()
        return 202, dict(ok=True, task_id=task_id)

    def capture(self, grab=None):
        """get image: 위 카메라(Astra)로 한 장 찍고, 보이는 물체를 함께 돌려준다. 로봇은 움직이지 않는다.
        카메라는 로봇 작업과 같이 쓸 수 없어서(OpenNI 장치 하나) 작업 중이면 거절하고, 찍는 동안은 새 작업을 받지 않는다.
        (HTTP 상태, 응답)"""
        with self.lock:
            if self.current or self.capturing:
                return 409, dict(ok=False, reason_code='busy', reason='로봇 작업 중에는 카메라를 쓸 수 없습니다.')
            self.capturing = True
        try:
            if menu.live_view_running():
                return 409, dict(ok=False, reason_code='camera_busy', reason='카메라 창(live_view.py)이 켜져 있습니다.')
            bgr, scene = self._steady_scene(grab or grab_astra)
            return 200, self._remember_scene(bgr, scene)
        except Exception as exc:
            self.log.add('error', '카메라', f'{type(exc).__name__}: {str(exc)[:300]}', traceback.format_exc())
            return 500, dict(ok=False, reason_code='camera_error', reason=f'{type(exc).__name__}: {str(exc)[:200]}')
        finally:
            with self.lock:
                self.capturing = False

    def _remember_scene(self, bgr, scene):
        import base64
        import cv2
        ok, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise OSError('JPEG 변환 실패')
        scene = dict(scene, ok=True, captured_at=time.strftime('%Y-%m-%dT%H:%M:%S'),
                     image_url='/api/image/latest.jpg', image_jpeg_base64=base64.b64encode(jpeg.tobytes()).decode())
        with self.lock:
            self.scene = dict(scene, jpeg=jpeg.tobytes())
        return scene

    @staticmethod
    def _steady_scene(grab, shots_max=3):
        """사진 한 장이 가끔 튄다(10~30장에 한 번 물체가 더 잡히거나 책상 물체가 칸 안으로 보임, 2026-10-22).
        두 장의 판단(책상 물체·칸 안 물체)이 같으면 그것을, 다르면 한 장 더 찍어 두 번 나온 판단을 쓴다.
        셋 다 다르면 마지막 것을 쓰고 unstable로 알린다. 돌려주는 것: (그 판단의 사진, 장면)."""
        def key(scene):
            return (tuple(sorted(o['label'] for o in scene.get('objects', []))),
                    tuple(sorted((o['label'], o.get('slot')) for o in scene.get('in_tray', []))))
        shots = []
        for _ in range(shots_max):
            bgr, depth = grab()
            scene = describe_scene(bgr, depth)
            shots.append((key(scene), bgr, scene))
            if sum(k == shots[-1][0] for k, _, _ in shots) >= 2:
                return bgr, dict(scene, shots=len(shots))
        _, bgr, scene = shots[-1]
        return bgr, dict(scene, shots=len(shots), unstable=True)

    def stop(self):
        """실행 중인 작업에 SIGINT를 보낸다(메뉴의 Ctrl+C와 같다: pick_auto가 현재 동작을 멈추고 정리한다)."""
        with self.lock:
            proc = self.proc
            running = bool(self.current)
            self.stop_requested = running          # 아직 프로세스를 띄우기 전이면, 띄우자마자 멈추고 다음 단계로 가지 않는다
            self.awaiting_desk_choice = False
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
        with self.lock:
            driving = running and self.tasks.get(self.current, {}).get('skill') == 'drive'
        if driving:                                  # 주행: 이동 루프도 멈추지만 차는 바로 세운다
            threading.Thread(target=base_drive.cancel, daemon=True).start()
        return dict(ok=True, stopped=running)

    # ---- 작업 스레드 ----
    def _event(self, task_id, kind, data):
        with self.lock:
            task = self.tasks[task_id]
            if kind == 'phase':
                if task['item']:                            # 세 물체 모두 넣기: 어느 물체의 단계인지 붙인다
                    data = (data[0], f"{NAMES[task['item']]}: {data[1]}")
                task['phase'], task['phase_label'] = data
                task['phases'].append(dict(phase=data[0], label=data[1], at_s=round(time.monotonic() - task['started'], 1)))
            elif kind == 'plan':
                task['warnings'] = list(data[1])

    def _set_proc(self, proc):
        with self.lock:
            self.proc = proc
            if self.stop_requested:
                proc.send_signal(signal.SIGINT)

    def _run(self, task_id, command):
        if self.stop_requested:
            return 130, dict(ok=False, reason='cancelled'), 0.
        code, summary, seconds = self.run(command, on_event=lambda k, d: self._event(task_id, k, d), on_start=self._set_proc, quiet=True)
        self._log_run(command, code, summary)
        return code, summary, seconds

    def _log_run(self, command, code, summary):
        """실행 프로그램(pick_auto, return_home 등)이 실패하면 그 원문 이유와 마지막 출력을 로그에 남긴다."""
        script = Path(str(command[1])).name if len(command) > 1 else str(command[0])
        if code == 130:
            self.log.add('info', script, '정지 신호로 중단됨')
        elif code != 0 or (summary is not None and summary.get('ok') is False):
            raw = (summary or {}).get('error') or (summary or {}).get('reason') or f'종료 코드 {code}'
            tail = '\n'.join((summary or {}).get('_tail') or [])
            self.log.add('error', script, raw, f'종료 코드 {code}' + (f'\n--- 마지막 출력 ---\n{tail}' if tail else ''))

    def _work(self, task_id, skill, params):
        started = time.monotonic()
        result = None
        try:
            result = self._execute(task_id, skill, params)
        except Exception as exc:                                    # 서버가 죽지 않게 모든 실패를 결과로 돌려준다
            result = dict(ok=False, reason=f'{type(exc).__name__}: {exc}', reason_code='failed')
            self.log.add('error', '서버', f'{skill} 처리 중 예외: {type(exc).__name__}: {exc}', traceback.format_exc())
        finally:
            with self.lock:
                task = self.tasks[task_id]
                task['seconds'] = round(time.monotonic() - started, 1)
                task['result'] = dict(result, seconds=task['seconds'], warnings=task['warnings'])
                task['state'] = 'done' if result['ok'] else 'failed'
                self._log_result(skill, task, result)
                if skill != 'drive' or params.get('then') in ('tidy', 'inspect'):
                    self.arm_home = True if result['ok'] else None
                self.proc = None
                self.current = None

    QUIET_REASONS = ('base_moving', 'torque_off', 'object_not_found', 'unreachable', 'busy',     # 사용자가 바로 고칠 수 있는 거부
                     'label_not_found', 'arm_not_folded', 'base_unavailable', 'wall_failed')

    def _log_result(self, skill, task, result):
        for warning in task.get('warnings') or []:
            self.log.add('warn', '작업', f'{skill} 주의: {warning}', f"task {task['id']}")
        code = result.get('reason_code')
        if result['ok']:
            self.log.add('ok', '작업', f"{skill} 완료 ({task['seconds']}초)" + (f" - {result['note']}" if result.get('note') else ''), f"task {task['id']}")
        elif code == 'cancelled':
            self.log.add('info', '작업', f"{skill} 정지됨 ({task['seconds']}초)", f"task {task['id']}")
        else:
            self.log.add('warn' if code in self.QUIET_REASONS else 'error', '작업', f"{skill} 실패: {result.get('reason')}",
                         f"task {task['id']}, reason_code={code}, {task['seconds']}초")

    DRIVE_FOLD_PULSES = 120     # 주행 전 팔이 이만큼 안으로 HOME에 접혀 있어야 한다(관절마다, 펄스)

    def _drive(self, task_id, params):
        """팀 주행 서버로 라벨까지 이동하고 벽에 붙는다. 팔이 접혀 있을 때만(뻗은 채 달리면 부딪히거나 넘어진다)."""
        pose = self.read_pose()
        if not pose:
            return dict(ok=False, reason_code='arm_not_folded', reason='서보가 응답하지 않아 팔이 접혀 있는지 모릅니다. 서보 전원을 확인하세요.')
        far = {n: pose[n][1] - home for n, home in menu.HOME_RAW.items() if n in pose and abs(pose[n][1] - home) > self.DRIVE_FOLD_PULSES}
        if far:
            return dict(ok=False, reason_code='arm_not_folded',
                        reason='팔이 HOME으로 접혀 있지 않아 주행하지 않습니다(' + ', '.join(f'{n} {d:+d}' for n, d in far.items()) + '). 먼저 "홈으로 돌아가"를 시키세요.')
        self.log.add('info', '주행', f"{params['label']}으로 이동 시작 ({'라벨 자리 그대로' if params.get('mode', 'precise') == 'precise' else '앞 벽에 붙기'})" + (f" (벽 앞 {params['gap']} m)" if 'gap' in params else ''))
        try:
            result = self.drive(params['label'], mode=params.get('mode', 'precise'), gap=params.get('gap'),
                                on_phase=lambda raw, text: self._event(task_id, 'phase', (raw, text)), stopped=lambda: self.stop_requested)
        except base_drive.BaseError as exc:
            return dict(ok=False, reason=str(exc), reason_code=exc.code)
        if result.get('ok') and params.get('then') in ('tidy', 'inspect') and not self.stop_requested:
            return self._tidy_here(task_id, params['label'])
        return result

    def _tidy_here(self, task_id, place, ask=False):
        """도착한 곳에서 위 카메라로 찍어 팔이 닿는 등록 물체를 정리한다("책상으로 가서 정리해줘"의 뒷부분)."""
        self._event(task_id, 'phase', ('drive_settle', f'{place} 도착, 차가 완전히 멈췄는지 확인'))
        steady = 0
        for _ in range(8):                               # 0.5초 간격으로 3회 연속 정지 확인
            if self.stop_requested:
                return dict(ok=False, reason_code='cancelled', reason='cancelled')
            problem = self.arrival_check()
            steady = 0 if problem else steady + 1
            if steady >= 3:
                break
            time.sleep(.5)
        else:
            return dict(ok=False, reason_code='base_moving',
                        reason=f'{place}에 도착했지만 연속 정지를 확인하지 못해 사진과 팔 작업을 시작하지 않았습니다. {problem or "정지 확인 시간 부족"}')
        if self.stop_requested:
            return dict(ok=False, reason_code='cancelled', reason='cancelled')
        self._event(task_id, 'phase', ('tidy_look', f'{place} 정지 확인 완료, 책상 사진 촬영'))
        try:
            bgr, scene = self._steady_scene(grab_astra)
            if bgr is not None:
                self._remember_scene(bgr, scene)
        except Exception as exc:
            return dict(ok=False, reason_code='camera_error', reason=f'{place}에 도착했지만 사진을 찍지 못했습니다: {type(exc).__name__}: {str(exc)[:160]}')
        finally:
            CAMERA.release()                             # 집기 프로그램이 카메라를 직접 연다
        if self.stop_requested:
            return dict(ok=False, reason_code='cancelled', reason='cancelled')
        objects = [label for label in ALL_ORDER if any(o['label'] == label and o.get('pickable') for o in scene.get('objects') or [])]
        if not objects:
            self._event(task_id, 'phase', ('tidy_empty', f'{place}에 정리할 물체가 보이지 않습니다.'))
            return dict(ok=True, reason=None, reason_code=None, note=f'{place}에 도착했습니다. 팔이 닿는 곳에 정리할 물체가 없습니다.')
        announcement = f"{place}에 {', '.join(NAMES[label] for label in objects)}가 보입니다. 정리하겠습니다."
        self._event(task_id, 'phase', ('tidy_found', announcement))
        self.log.add('info', '작업', f"{place}에서 정리: {', '.join(NAMES[label] for label in objects)}")
        refused = self._ensure_torque(task_id, 'pick_all')
        if refused:
            return refused
        outcome = self._pick_all(task_id, objects)
        outcome['announcement'] = announcement
        if outcome['ok']:
            outcome['note'] = f'{place}에 도착해 정리했습니다.' + (f" {outcome['note']}" if outcome.get('note') else '')
        return outcome

    def _ensure_torque(self, task_id, skill):
        """움직여도 되면 None, 아니면 거부 결과. HOME에 놓인 채 토크만 꺼진 팔은 자동으로 토크를 켜서 붙잡는다(접힌 HOME 근처면 HOME에 맞춘다)."""
        state = self.read_state()
        if state == 'ok':
            return None
        # HOME이 아닌 곳에서 토크가 꺼진 팔은 'HOME 복귀'만 받는다: 현재 자세를 그대로 붙잡은 뒤(목표=현재 위치라 튀지 않는다) return_home이
        # 복귀 경로 전체를 검사하고 간다. 그 밖의 작업은 HOME에서 시작해야 한다.
        recover = state == 'off_elsewhere' and skill == 'return_home'
        if AUTO_TORQUE and (state in ('off_at_home', 'off_near_fold') or recover) and not self.stop_requested:
            self._event(task_id, 'phase', ('torque_on', '토크 켜기(팔 붙잡기)'))
            self.log.add('info', '팔', f'토크가 꺼진 팔을 붙잡습니다(상태 {state})' + (' - 현재 자세에서 HOME으로 복귀' if recover else ''))
            self.hold()
            after = self.read_state()
            if after != 'ok':
                return dict(ok=False, reason_code='torque_off', reason=f'토크를 켜지 못했습니다({after}). 서보 전원과 팔을 확인하세요.')
            if state == 'off_near_fold' and skill != 'return_home':          # 조금 어긋난 접힌 자세: 먼저 HOME에 맞춘다
                code, summary, _ = self._run(task_id, HOME_CMD)
                if not (summary and summary.get('home_return_complete')):
                    error = (summary or {}).get('error')
                    return dict(ok=False, reason=f'토크를 켠 뒤 HOME에 맞추지 못했습니다: {error or f"종료 코드 {code}"}', reason_code=reason_code(error))
            return None
        return dict(ok=False, reason_code='arm_state_unknown' if state == 'unknown' else 'torque_off',
                    reason={'unknown': '서보가 응답하지 않습니다. 서보 전원(어댑터 전원)과 서보 선을 확인하세요.',
                            'off_elsewhere': '토크가 꺼져 있고 팔이 HOME이 아닙니다. 먼저 "홈으로 돌아가"를 시키세요. 팔을 현재 자세에서 붙잡고 HOME으로 복귀합니다(팔이 걸리거나 처지지 않게 받쳐 주세요).'}.get(
                                state, f'팔 상태가 움직여도 되는 상태가 아닙니다({state}). 사람이 팔을 받치고 토크를 켜야 합니다.'))

    def _execute(self, task_id, skill, params):
        CAMERA.release()                    # 작업(pick_auto 등)이 카메라를 직접 연다
        moving = self.base_check()
        if moving:
            return dict(ok=False, reason_code='base_moving', reason=moving + ' 차가 멈춘 뒤 다시 시키세요.')
        if skill == 'drive':
            return self._drive(task_id, params)
        if menu.live_view_running():
            return dict(ok=False, reason='카메라 창(live_view.py)이 켜져 있습니다.', reason_code='camera_busy')
        refused = self._ensure_torque(task_id, skill)
        if refused:
            return refused
        if skill == 'return_home':
            code, summary, _ = self._run(task_id, HOME_CMD)
            done = bool(summary and summary.get('home_return_complete'))
            return dict(ok=done, reason=None if done else str((summary or {}).get('error') or f'종료 코드 {code}'),
                        reason_code=None if done else reason_code((summary or {}).get('error')))
        if skill == 'pick_all':
            return self._pick_all(task_id, params.get('objects'))
        return self._pick(task_id, params['object'], params['slot'])

    def _pick_all(self, task_id, objects=None):
        # 메뉴 4와 같다: HOME에서 세 물체를 미리 계획(사진)한 뒤, 놓은 자리에서 HOME을 거치지 않고 다음 물체로 간다.
        def on_item(label, stage):
            with self.lock:
                self.tasks[task_id]['item'] = label
            if stage == 'plan':
                self._event(task_id, 'phase', ('plan', '계획(사진)'))
        outcome = menu.run_all_chained(run=lambda cmd: self._run(task_id, cmd), on_item=on_item, stopped=lambda: self.stop_requested,
                                       labels=objects)
        with self.lock:
            self.tasks[task_id]['items'] = [dict(object=i['object'], ok=i['ok'], reason_code=None if i['ok'] else reason_code(i['reason']))
                                            for i in outcome['items']] + [dict(object=label, ok=None, skipped=True, reason_code='object_not_found')
                                                                          for label in outcome['skipped']]
        note = ('책상에 없어 건너뜀: ' + ', '.join(NAMES[label] for label in outcome['skipped'])) if outcome['skipped'] else None
        if outcome['ok']:
            return dict(ok=True, reason=None, reason_code=None, note=note)
        if outcome['failed_label'] is None:
            return dict(ok=False, reason='책상에서 등록된 물체를 찾지 못했습니다. 넣을 것이 없습니다.', reason_code='object_not_found')
        if outcome['reason'] == 'cancelled':
            return dict(ok=False, reason='cancelled', reason_code='cancelled')
        return dict(ok=False, reason=f"{NAMES[outcome['failed_label']]}에서 멈췄습니다: {menu.brief_message(outcome['reason'])}",
                    reason_code=reason_code(outcome['reason']))

    def _pick(self, task_id, label, slot):
        since = time.time()
        build = (lambda tray: menu.unknown_command(slot, execute=True)) if label == 'unknown' else \
            (lambda tray: menu.sort_command(label, slot=slot if slot != OBJECTS[label] else None, use_tray_frame=tray))
        code, result, _ = self._run(task_id, build(True))
        if code != 0 and 'the tray moved since the stored frame' in str((result or {}).get('reason', '')):
            code, result, _ = self._run(task_id, build(False))      # 저장한 빈 트레이가 안 맞으면 카메라로 다시 찾는다(메뉴와 같다)
        if code == 0 and result and result.get('ok'):
            return dict(ok=True, reason=None, reason_code=None)
        reason = str((result or {}).get('reason') or f'종료 코드 {code}')
        placed = menu.newest_result(since)
        if placed and placed.get('motion_cycle_complete') and not placed.get('error'):
            # 물체는 칸에 넣었는데 자동 HOME 복귀만 실패: 복귀를 다시 시도한다(메뉴와 같다).
            _, summary, _ = self._run(task_id, HOME_CMD)
            if summary and summary.get('home_return_complete'):
                return dict(ok=True, reason=None, reason_code=None, note='자동 HOME 복귀가 실패해 다시 복귀했습니다.')
        return dict(ok=False, reason=menu.brief_message(reason), reason_code=reason_code(reason))


WEB = Path(__file__).resolve().parents[1] / 'web' / 'index.html'


class CameraWorker:
    """scripts/astra_worker.py(카메라를 열어 둔 프로세스)를 띄워 두고 사진을 받는다. 처음 한 장만 카메라를 여는 시간이 들고 그 뒤로는 바로 나온다.
    로봇 작업도 같은 카메라를 쓰므로 작업을 시작하기 전에 release()로 닫는다. 실패하면 None을 돌려주고 호출한 쪽이 예전 방식으로 찍는다."""

    START_S, GRAB_S, CLOSE_S = 15., 5., 5.

    def __init__(self, command=None):
        self.command = command or [menu.PY, str(menu.ROOT / 'scripts' / 'astra_worker.py')]
        self.proc = None
        self.lock = threading.Lock()

    def _start(self):
        self.proc = subprocess.Popen(self.command, cwd=menu.ROOT,
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        if self._reply(self.START_S) != 'ready':
            self._kill()

    def _reply(self, timeout_s):
        import select
        if self.proc is None or not select.select([self.proc.stdout], [], [], timeout_s)[0]:
            return None
        return self.proc.stdout.readline().strip() or None

    def _kill(self):
        if self.proc is not None:
            self.proc.kill()
            self.proc.wait()
        self.proc = None

    def grab(self):
        import tempfile
        import numpy as np
        with self.lock:
            for _ in range(2):                                    # 쉬다가 스스로 닫혔으면 한 번 다시 띄운다
                if self.proc is None or self.proc.poll() is not None:
                    self.proc = None
                    self._start()
                    if self.proc is None:
                        return None
                with tempfile.TemporaryDirectory() as folder:
                    path = Path(folder) / 'shot.npz'
                    try:
                        self.proc.stdin.write(f'grab {path}\n')
                        self.proc.stdin.flush()
                    except (BrokenPipeError, OSError):
                        self.proc = None
                        continue
                    reply = self._reply(self.GRAB_S)
                    if reply == 'ok':
                        with np.load(path) as shot:
                            return shot['bgr'], shot['depth']
                self._kill()
                if reply is not None:                             # 카메라가 답은 했지만 실패: 다시 띄우지 않는다
                    return None
            return None

    def release(self):
        """카메라를 놓아 준다(로봇 작업 전·서버 종료 때). 닫는 데 1초쯤 걸린다."""
        with self.lock:
            if self.proc is None:
                return
            if self.proc.poll() is None:
                try:
                    self.proc.stdin.write('quit\n')
                    self.proc.stdin.flush()
                    self.proc.wait(self.CLOSE_S)
                except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
                    pass
            self._kill()


CAMERA = CameraWorker()


def grab_astra():
    """위 카메라 한 장(RGB, 깊이 mm). 열어 둔 카메라 프로세스(CameraWorker)에서 받고, 안 되면 예전처럼 촬영 명령을 따로 실행한다."""
    shot = CAMERA.grab()
    if shot is not None:
        return shot
    return grab_astra_once()


def grab_astra_once():
    """기존 촬영 명령을 별도 프로세스로 한 번 실행한다(카메라를 열고 한 장 찍고 닫는다)."""
    import tempfile
    import cv2
    import numpy as np
    with tempfile.TemporaryDirectory() as folder:
        out = Path(folder) / 'shot'
        done = subprocess.run([menu.PY, str(menu.ROOT / 'scripts' / 'calibrate_robot_camera.py'), 'capture', '--output', str(out)],
                              cwd=menu.ROOT, capture_output=True, text=True, timeout=30)
        if done.returncode != 0:
            raise OSError((done.stderr.strip().splitlines() or ['카메라 촬영 실패'])[-1][:200])
        return cv2.imread(str(out / '0000_rgb.png')), np.load(out / '0000_depth_mm.npy')


BASE_CLEAR_M = .12       # 이보다 로봇 밑동에 가까운 '등록 안 된 물체'는 로봇 자신(밑동·팔)으로 보고 뺀다
FAR_CLEAR_M = .42        # 이보다 먼 것은 작업 범위 밖(케이블·설비)으로 보고 뺀다
ARM_HEIGHT_MM = 100.     # 이보다 높은 '등록 안 된 물체'는 로봇팔 자신(등록 물체는 40mm 이하)


IN_TRAY_SCENE_MM = 70.       # 화면 판단에서 '칸 안'으로 보는 칸 중심 거리(칸 92×100mm의 모서리까지). 칸 안 물체 윗면은 칸 중심에서 50~70mm 떨어져 잡히기도 한다


def tray_slots(bgr, depth):
    """지금 트레이 칸 위치({번호: x_mm, y_mm ...}) 또는 None. 칸에 물체가 들어 있으면 실시간으로 칸 바닥을 못 찾으므로
    저장해 둔 빈 트레이 사진을 쓰고, 트레이가 조금 밀렸으면 밀린 만큼 옮긴다(plan_sort와 같은 방식)."""
    import cv2
    import numpy as np
    from tray.tray_detect import detect_tray
    root = menu.ROOT
    try:
        live = detect_tray(bgr, depth, root, first_end='far_from_base')
        if live.get('found'):
            return live['slots']
        rgb, dep = menu.TRAY_FRAME
        if not (rgb.exists() and dep.exists()):
            return None
        from tray.tray_frame_check import tray_offset_px, register_tray, move_slots, GOOD_SCORE
        stored_bgr, stored_depth = cv2.imread(str(rgb)), np.load(dep)
        stored = detect_tray(stored_bgr, stored_depth, root, first_end='far_from_base')
        if not stored.get('found'):
            return None
        offset = tray_offset_px(stored_bgr, bgr, depth=stored_depth, root=root)
        if offset['ok']:
            return stored['slots']
        moved = register_tray(stored_bgr, bgr, depth=stored_depth, root=root)
        if moved['ok']:
            return move_slots(stored['slots'], moved, root)
        # 화면 판단용(집기 계획에는 쓰지 않는다): 1번 칸에 넣은 물체가 트레이 끝 모서리 둘을 가리면 맞는 모서리가 하나뿐이다(2026-10-22).
        # 남은 모서리가 정확히 맞고, 나머지가 다른 자리에도 확실히 맞지 않으면(가려짐) 트레이가 그대로라고 보고 저장한 칸을 쓴다.
        # 나머지 모서리가 다른 자리에 확실히 맞으면 트레이가 움직인 것이다. 안 그러면 칸 안 물체가 책상 물체로 보인다.
        good = [i for i, (shift, score) in enumerate(zip(offset['offsets_px'], offset['scores'])) if shift <= 3. and score >= GOOD_SCORE]
        if good and all(i in good or score < GOOD_SCORE for i, score in enumerate(offset['scores'])):
            return stored['slots']
        return None
    except Exception:
        return None


def describe_scene(bgr, depth):
    """사진에서 보이는 물체와, 지금 쓸 수 있어 보이는 메뉴(판단 참고용). 등록 물체는 위치·집기 가능 여부까지 본다.
    이미 트레이 칸에 들어 있는 물체는 objects에서 빼고 in_tray로 따로 알린다(정리 대상이 아니다). 로봇 밑동 근처나
    작업 범위 밖의 '등록 안 된 물체'(로봇 자신, 케이블 등)도 뺀다."""
    from perception.object_classify import classify_objects
    from perception.object_targets import locate_targets, load_catalog, default_locator, top_center_xy
    from perception.object_classify import table_height_mm
    from perception.object_targets import DEPTH_MINUS_TOP_CENTER_PX
    from tray.occupancy import slot_containing, slot_contents
    from tray.tray_detect import _color_pixel
    seen = classify_objects(bgr, depth)
    try:
        targets = {t['label']: t for t in locate_targets(bgr, depth, menu.ROOT)}
    except Exception:
        targets = {}
    slots = tray_slots(bgr, depth)
    try:
        locator, catalog = default_locator(menu.ROOT), load_catalog(menu.ROOT / 'config/object_catalog.json')
    except Exception:
        locator = catalog = None
    objects, in_tray = [], []
    for item in seen:
        if item['label'] == 'unknown' and max(item.get('bbox_px', (0, 0, 0, 0))[2:]) > 150:
            continue        # 트레이·설비 같은 큰 덩어리(새 물체 인식 unknown_objects와 같은 기준)
        if item['label'] == 'unknown' and (item.get('height_mm') or 0) > ARM_HEIGHT_MM:
            continue        # 로봇팔 자신
        target = targets.get(item['label'])
        xy = None
        if target:
            xy = target['target_xy']
        elif locator is not None:
            try:
                top = catalog['objects'][item['label']]['height_mm'] if item['label'] in catalog['objects'] else item.get('height_mm') or 30.
                xy = top_center_xy(item, top, locator)
            except Exception:
                xy = None
        if xy is not None and item['label'] == 'unknown' and not BASE_CLEAR_M <= (float(xy[0]) ** 2 + float(xy[1]) ** 2) ** .5 <= FAR_CLEAR_M:
            continue        # 로봇 밑동 근처(로봇 자신)·작업 범위 밖(케이블 등)
        if slot_containing(xy, slots, radius_mm=IN_TRAY_SCENE_MM) is not None:
            continue        # 칸 안: 무엇이 들어 있는지는 아래에서 칸마다 높이로 따로 본다(칸막이·숫자가 '등록 안 된 물체'로 잡히기도 한다)
        objects.append(dict(label=item['label'], name=NAMES.get(item['label'], item['label']),
                            pixel=[int(v) for v in item.get('top_center_px', item.get('center_px', (0, 0)))],
                            position_mm=[round(v * 1000) for v in xy] if xy is not None else None,
                            pickable=bool(target and target['pickable'])))
    if slots and locator is not None and catalog is not None:
        try:
            for found in slot_contents(bgr, table_height_mm(depth), slots, lambda xy, h: _color_pixel(locator, xy, h), DEPTH_MINUS_TOP_CENTER_PX,
                                       {k: float(v['height_mm']) for k, v in catalog['objects'].items() if k in ALL_ORDER}):
                in_tray.append(dict(label=found['label'], name=NAMES.get(found['label'], '물체') if found['label'] != 'unknown' else '물체', slot=found['slot']))
        except Exception:
            pass
    labels = {o['label'] for o in objects if o['pickable']}
    available = [m['id'] for m in MENU[:3] if m['params']['object'] in labels]
    if {'white_box', 'red_cube', 'tape_measure'} <= labels:
        available.append(4)
    if labels & {'white_box', 'red_cube', 'tape_measure'}:
        available.append(5)
    available.append(6)
    if any(o['label'] == 'unknown' for o in objects):
        available.append(7)
    return dict(objects=objects, in_tray=sorted(in_tray, key=lambda o: o['slot']), tray_found=slots is not None,
                available_menus=sorted(available),
                note='available_menus는 사진에서 보이는 물체로 고른 참고값이다. 트레이 칸 안에 보이는 물체는 in_tray로 따로 알리고 정리 대상에서 뺀다.')

# 메뉴 1~7번을 그대로 REST API로 둔다(터미널 메뉴 sort_menu.py와 같은 번호·동작). 웹 화면의 버튼도 이 주소를 부른다.
# GET /api/menu가 이 목록을 그대로 돌려주고, /docs가 이 목록으로 설명 페이지를 만든다.
MENU = [
    dict(id=1, label='흰 상자 → 1번 칸', path='/api/menu/1', body=None, skill='pick_place', params=dict(object='white_box', slot=1),
         description='흰 상자를 찾아 집은 뒤 1번 칸에 넣고 HOME으로 돌아온다.'),
    dict(id=2, label='빨간 정육면체 → 2번 칸', path='/api/menu/2', body=None, skill='pick_place', params=dict(object='red_cube', slot=2),
         description='빨간 정육면체를 찾아 집은 뒤 2번 칸에 넣고 HOME으로 돌아온다.'),
    dict(id=3, label='줄자 → 3번 칸', path='/api/menu/3', body=None, skill='pick_place', params=dict(object='tape_measure', slot=3),
         description='줄자를 찾아 집은 뒤 3번 칸에 넣고 HOME으로 돌아온다.'),
    dict(id=4, label='세 물체 모두 넣기', path='/api/menu/4', body={'objects': '(선택) [white_box, red_cube, tape_measure] 중 일부'}, skill='pick_all', params={},
         description='흰 상자 → 빨간 정육면체 → 줄자 순서로 각자 기본 칸에 넣는다(책상 정리). HOME에서 미리 계획하고, 놓은 뒤 HOME을 거치지 않고 다음 물체로 간다. '
                     '책상에 없는 물체는 건너뛴다. objects를 주면 그 물체만. 앞 물체가 실패하면 나머지는 실행하지 않는다.'),
    dict(id=5, label='물체와 칸을 골라서 넣기', path='/api/menu/5', body={'object': 'white_box | red_cube | tape_measure', 'slot': '1 | 2 | 3'},
         skill='pick_place', params=None, description='고른 물체를 고른 칸에 넣는다. object와 slot이 둘 다 필요하다.'),
    dict(id=6, label='HOME으로 복귀', path='/api/menu/6', body=None, skill='return_home', params={},
         description='팔을 접힌 HOME 자세로 돌려놓는다.'),
    dict(id=7, label='unknown 자동 집기', path='/api/menu/7', body={'slot': '1 | 2 | 3'}, skill='pick_place', params=None,
         description='시작 후 새 물체 하나를 작업 영역에 놓으면(진행 단계 unknown_wait, 90초 대기) 자동으로 찾아 고른 칸에 넣는다. slot이 필요하다.'),
]
# 메뉴 말고 공통으로 쓰는 주소(설명 페이지용)
COMMON_API = [
    dict(method='GET', path='/api/menu', body=None, description='메뉴 1~7번 목록과 각 주소·설명.'),
    dict(method='POST', path='/api/image', body=None,
         description='get image: 위 카메라로 지금 장면을 한 장 찍는다(약 4초, 로봇은 안 움직임). 응답: image_jpeg_base64(사진), objects(보이는 물체·위치·집기 가능), available_menus(쓸 수 있어 보이는 메뉴 번호). 로봇 작업 중이면 409.'),
    dict(method='GET', path='/api/image/latest.jpg', body=None, description='마지막으로 찍은 사진(JPEG).'),
    dict(method='POST', path='/api/situation', body=None,
         description='상황 파악: 사진을 찍고 책상 상황(reply·speech), 지금 쓸 수 있는 기능(actions, 예시 문장 포함), 기본 제안(proposal: 정리할 물체가 있으면 책상 정리)을 돌려준다. 로봇은 움직이지 않는다. 웹 화면은 열 때와 작업이 끝날 때마다 부른다.'),
    dict(method='POST', path='/api/stt', body='녹음 파일(webm·ogg·wav, 2MB 이하, Content-Type: audio/webm 등)',
         description='로컬 음성 인식(Whisper, 인터넷 없이 이 PC에서). 응답: text(알아들은 문장, 말소리가 없으면 빈 문자열), seconds. 로봇은 움직이지 않는다. 문장은 /chat으로 보내 명령으로 쓴다.'),
    dict(method='GET', path='/api/base', body=None,
         description='주행 상태(base_state, 팀 map_web의 /api/state·/api/drive): 지도 위 위치·방향, 장소 라벨(가까운 순, 거리), 이동·벽 근접 상태, 드라이버 연결. 차는 움직이지 않는다. map_web이 꺼져 있으면 503.'),
    dict(method='POST', path='/robot_arm', body={'action': 'drive', 'label': '책상', 'mode': 'precise', 'then': 'tidy'},
         description='주행: mode=precise(기본)는 라벨을 만든 그 자리(위치 ±3 cm·방향 ±2°)로 간다(Nav2로 근처까지 → 지도 위치를 보며 마무리 정렬). mode=wall은 라벨로 간 뒤 정면 벽과 수직으로 맞추고 gap(m, 0.02~0.5) 앞까지 붙는다. 차가 움직인다. 팔이 HOME으로 접혀 있을 때만 받는다. then=\'tidy\'면 도착한 뒤 사진을 찍어 팔이 닿는 물체를 정리한다("책상으로 가서 정리해줘"). 진행은 /status의 phase_label, 정지는 /stop.'),
    dict(method='GET', path='/api/logs?since=<번호>', body=None,
         description='최근 사건 기록(최대 500건 중 번호가 since보다 큰 것). 작업 시작·완료·실패의 원문 이유와 실행 프로그램의 마지막 출력, 카메라·API 오류. 응답: boot(서버를 다시 켜면 바뀜), last(마지막 번호), entries[{id, time, level(info/ok/warn/error), source, message, detail}]. 웹 화면의 로그창이 쓴다.'),
    dict(method='GET', path='/status', body=None, description='지금 상태. state(idle/busy), phase_label(지금 단계), task_id, arm_home.'),
    dict(method='GET', path='/tasks/<task_id>', body=None, description='작업 하나의 단계 기록(phases)과 결과(result: ok, reason, reason_code, warnings).'),
    dict(method='POST', path='/stop', body=None, description='실행 중인 동작을 즉시 멈춘다. 팔은 그 자리에 선다(이후 메뉴 6번으로 복귀).'),
    dict(method='POST', path='/robot_arm', body={'action': 'pick_place | pick_all | return_home | status | stop', 'object': '...', 'slot': '...'},
         description='LLM 도구 호출 내용을 그대로 넘기는 한 개짜리 주소(메뉴와 같은 동작).'),
]


def menu_start(arm, number, params):
    """메뉴 번호 → 작업 시작. (HTTP 상태, 응답)."""
    item = next((m for m in MENU if m['id'] == number), None)
    if item is None:
        return 404, dict(ok=False, reason_code='not_found', reason='메뉴는 1~7번입니다.')
    if item['id'] == 5:
        if params.get('object') not in ('white_box', 'red_cube', 'tape_measure') or params.get('slot') not in (1, 2, 3):
            return 400, dict(ok=False, reason_code='bad_request', reason='object(white_box, red_cube, tape_measure)와 slot(1~3)이 필요합니다.')
        chosen = dict(object=params['object'], slot=params['slot'])
    elif item['id'] == 7:
        if isinstance(params.get('slot'), bool) or params.get('slot') not in (1, 2, 3):
            return 400, dict(ok=False, reason_code='bad_request', reason='놓을 칸 slot(1~3)이 필요합니다.')
        chosen = dict(object='unknown', slot=params['slot'])
    elif item['id'] == 4 and 'objects' in params:
        chosen = dict(objects=params['objects'])          # 사진으로 고른 물체만(책상 정리)
    else:
        chosen = dict(item['params'])
    return arm.start(item['skill'], chosen)


def docs_html(base=''):
    """API 설명 페이지(메뉴 목록에서 만든다)."""
    import html as h
    def row(method, path, body, desc, label=None):
        body_txt = '-' if not body else h.escape(json.dumps(body, ensure_ascii=False))
        title = f'<b>{h.escape(label)}</b><br>' if label else ''
        return (f'<tr><td><code>{method}</code></td><td><code>{h.escape(path)}</code></td>'
                f'<td><code>{body_txt}</code></td><td>{title}{h.escape(desc)}</td></tr>')
    menu_rows = ''.join(row('POST', m['path'], m['body'], m['description'], f"{m['id']}. {m['label']}") for m in MENU)
    common_rows = ''.join(row(c['method'], c['path'], c['body'], c['description']) for c in COMMON_API)
    return f'''<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>로봇팔 API</title><style>
:root {{ --bg:#f3f4f5; --s:#fff; --ink:#16191d; --sub:#5b636e; --line:#e3e6ea; --a:#0f766e; }}
@media (prefers-color-scheme: dark) {{ :root {{ --bg:#0e1012; --s:#171a1d; --ink:#e9ecef; --sub:#a1a9b3; --line:#2a2f35; --a:#2bb5a6; }} }}
body {{ margin:0; background:var(--bg); color:var(--ink); font:15px/1.6 "Pretendard Variable",Pretendard,"Noto Sans KR",system-ui,sans-serif; }}
main {{ max-width:1100px; margin:0 auto; padding:28px 20px 48px; }} h1 {{ font-size:22px; margin:0 0 4px; }} h2 {{ font-size:16px; margin:28px 0 10px; }}
p {{ color:var(--sub); margin:0 0 6px; }} .card {{ background:var(--s); border:1px solid var(--line); border-radius:16px; overflow-x:auto; }}
table {{ border-collapse:collapse; width:100%; min-width:720px; }} th,td {{ text-align:left; padding:12px 14px; vertical-align:top; font-size:14px; }}
tr+tr td {{ border-top:1px solid var(--line); }} th {{ color:var(--sub); font-weight:600; border-bottom:1px solid var(--line); }}
th {{ white-space:nowrap; }} td:nth-child(1), td:nth-child(2) {{ white-space:nowrap; }} td:nth-child(1) code, td:nth-child(2) code {{ word-break:normal; }} td:nth-child(3) {{ max-width:260px; }}
code {{ font-family:"JetBrains Mono",ui-monospace,monospace; font-size:13px; color:var(--a); word-break:break-all; }}
pre {{ background:var(--s); border:1px solid var(--line); border-radius:12px; padding:14px; overflow-x:auto; font-size:13px; }} a {{ color:var(--a); }}
</style></head><body><main>
<h1>로봇팔 REST API</h1>
<p>모든 요청과 응답은 JSON입니다. 서버를 <code>--token</code>으로 켰으면 헤더에 <code>Authorization: Bearer 토큰</code>을 붙입니다.</p>
<p>움직이는 명령은 바로 <code>202 {{"ok": true, "task_id": "..."}}</code>를 돌려줍니다. 진행은 <code>GET /status</code>, 결과는 <code>GET /tasks/&lt;task_id&gt;</code>로 확인합니다. 실행 중에 다른 명령을 보내면 <code>409 busy</code>입니다.</p>
<h2>메뉴 (1~7번)</h2><div class="card"><table><tr><th>방식</th><th>주소</th><th>보낼 내용</th><th>설명</th></tr>{menu_rows}</table></div>
<h2>공통</h2><div class="card"><table><tr><th>방식</th><th>주소</th><th>보낼 내용</th><th>설명</th></tr>{common_rows}</table></div>
<h2>예시</h2>
<pre>curl -X POST -H "Authorization: Bearer 토큰" {h.escape(base)}/api/menu/2
curl -X POST -H "Authorization: Bearer 토큰" -H "Content-Type: application/json" -d '{{"object":"red_cube","slot":3}}' {h.escape(base)}/api/menu/5
curl -H "Authorization: Bearer 토큰" {h.escape(base)}/status</pre>
<h2>실패 코드 (result.reason_code)</h2>
<p><code>object_not_found</code> 물체를 찾지 못함 · <code>slot_too_small</code> 칸 여유 부족 · <code>unreachable</code> 팔이 닿지 않음 · <code>tray_not_found</code> 트레이 인식 실패 ·
<code>tray_moved</code> 트레이 위치 변경 · <code>torque_off</code> 서보 토크 꺼짐 · <code>device_absent</code> 장치 연결 확인 · <code>cancelled</code> 정지함 · <code>busy</code> 다른 작업 중 · <code>failed</code> 그 밖의 실패</p>
<p><a href="/">← 조작 화면으로</a></p>
</main></body></html>'''


def make_handler(arm, token=None, cors_origin=None, chat_client=None, transcriber=None):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, status, body):
            log = getattr(arm, 'log', None)
            if log is not None and status >= 400 and status != 401 and isinstance(body, dict) and not self.path.startswith(('/api/logs', '/favicon')):
                log.add('error' if status >= 500 else 'warn', 'API', f"{status} {self.command} {self.path.split('?')[0]}: {body.get('reason') or body.get('reason_code') or ''}",
                        body.get('reason_code'))
            data = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self._cors()
            self.end_headers()
            self.wfile.write(data)

        def _cors(self):
            # 웹 화면(브라우저)이 직접 부를 때만 필요하다. 웹 서버가 대신 부르면 설정하지 않아도 된다.
            if cors_origin:
                self.send_header('Access-Control-Allow-Origin', cors_origin)
                self.send_header('Access-Control-Allow-Headers', 'Content-Type, Authorization')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')

        def do_OPTIONS(self):
            self.send_response(204)
            self._cors()
            self.end_headers()

        def _authorized(self):
            if not token:
                return True
            given = self.headers.get('Authorization', '')
            return hmac.compare_digest(given, f'Bearer {token}')

        def _body(self):
            length = int(self.headers.get('Content-Length') or 0)
            if length > (65536 if self.path == '/chat' else 4096):
                raise ValueError('요청이 너무 큽니다.')
            data = json.loads(self.rfile.read(length) or b'{}')
            if not isinstance(data, dict):
                raise ValueError('JSON 객체가 필요합니다.')
            return data

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                # 화면 자체는 토큰 없이 연다(화면이 API를 부를 때 토큰을 붙인다).
                data = WEB.read_bytes()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                return self.wfile.write(data)
            if self.path == '/health':
                return self._send(200, dict(ok=True))
            if self.path == '/docs':
                data = docs_html(f"http://{self.headers.get('Host', '127.0.0.1:8000')}").encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                return self.wfile.write(data)
            if self.path == '/api/menu':
                return self._send(200, dict(ok=True, menu=[{k: v for k, v in m.items() if k not in ('skill', 'params')} | {'method': 'POST'} for m in MENU]))
            if not self._authorized():
                return self._send(401, dict(ok=False, reason_code='unauthorized'))
            if self.path == '/status':
                return self._send(200, arm.status())
            if self.path == '/api/base':
                try:
                    return self._send(200, base_drive.base_state())
                except base_drive.BaseError as exc:
                    return self._send(503, dict(ok=False, reason_code=exc.code, reason=str(exc)))
            if self.path.split('?')[0] == '/api/logs':
                query = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
                try:
                    after = max(0, int(query.get('since', ['0'])[0]))
                except ValueError:
                    after = 0
                entries, last = arm.log.since(after)
                return self._send(200, dict(ok=True, boot=arm.log.boot, last=last, entries=entries))
            if self.path == '/api/image/latest.jpg':
                scene = arm.scene
                if not scene:
                    return self._send(404, dict(ok=False, reason_code='not_found', reason='아직 찍은 사진이 없습니다. POST /api/image를 먼저 부르세요.'))
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(scene['jpeg'])))
                self.send_header('Cache-Control', 'no-store')
                self._cors()
                self.end_headers()
                return self.wfile.write(scene['jpeg'])
            match = re.fullmatch(r'/tasks/([0-9a-f]{8})', self.path)
            task = arm.task(match[1]) if match else None
            if task:
                return self._send(200, task)
            self._send(404, dict(ok=False, reason_code='not_found'))

        def do_POST(self):
            if not self._authorized():
                return self._send(401, dict(ok=False, reason_code='unauthorized'))
            if self.path == '/api/stt':                  # 본문이 JSON이 아니라 녹음 파일이다
                try:
                    length = int(self.headers.get('Content-Length') or 0)
                except ValueError:
                    length = -1
                if not 0 < length <= arm_stt.MAX_AUDIO_BYTES:
                    return self._send(413 if length > 0 else 400, dict(ok=False, reason_code='bad_request',
                                                                      reason=f'녹음 파일은 1바이트~{arm_stt.MAX_AUDIO_BYTES // 1024 // 1024}MB여야 합니다.'))
                audio = self.rfile.read(length)
                kind = (self.headers.get('Content-Type') or '').lower()
                suffix = '.ogg' if 'ogg' in kind else '.wav' if 'wav' in kind else '.mp4' if 'mp4' in kind else '.mp3' if 'mpeg' in kind else '.webm'
                try:
                    return self._send(200, dict(ok=True, **(transcriber or arm_stt.transcribe)(audio, suffix=suffix)))
                except arm_stt.SttUnavailable as exc:
                    return self._send(503, dict(ok=False, reason_code='stt_unavailable', reason=str(exc)))
                except Exception as exc:                   # 깨진 녹음 등
                    return self._send(400, dict(ok=False, reason_code='stt_error', reason=f'{type(exc).__name__}: {str(exc)[:200]}'))
            try:
                params = self._body()
            except (ValueError, UnicodeDecodeError) as exc:
                return self._send(400, dict(ok=False, reason_code='bad_request', reason=str(exc)))
            if self.path == '/stop':
                return self._send(200, arm.stop())
            if self.path == '/chat':
                text = params.get('text')
                history = params.get('history', [])
                if not isinstance(text, str) or not text.strip() or len(text) > 500 or not isinstance(history, list):
                    return self._send(400, dict(ok=False, reason_code='bad_request', reason='text(500자 이하)가 필요합니다.'))
                try:
                    import arm_llm
                    return self._send(200, dict(ok=True, **arm_llm.chat(history, text.strip(), arm, client=chat_client)))
                except arm_llm.LlmUnavailable as exc:          # Ollama 꺼짐·모델 없음: 버튼 조작은 계속 가능해야 한다
                    return self._send(503, dict(ok=False, reason_code='llm_unavailable', reason=str(exc)))
                except Exception as exc:
                    return self._send(502, dict(ok=False, reason_code='llm_error', reason=f'{type(exc).__name__}: {str(exc)[:200]}'))
            if self.path == '/robot_arm':
                action = params.get('action')
                if action == 'status':
                    return self._send(200, arm.status())
                if action == 'stop':
                    return self._send(200, arm.stop())
                if action not in ('pick_place', 'pick_all', 'return_home', 'drive'):
                    return self._send(400, dict(ok=False, reason_code='bad_request', reason='action은 pick_place, pick_all, return_home, drive, status, stop 중 하나입니다.'))
                status, body = arm.start(action, {k: v for k, v in params.items() if k in ('object', 'slot', 'objects', 'label', 'gap', 'then', 'mode')})
                return self._send(status, body)
            if self.path == '/api/image':
                status, body = arm.capture()
                return self._send(status, body)
            if self.path == '/api/situation':          # 사진 → 상황 설명·쓸 수 있는 기능·기본 제안(책상 정리). 로봇은 안 움직인다
                import arm_llm
                return self._send(200, dict(ok=True, **arm_llm.situation(arm)))
            match = re.fullmatch(r'/api/menu/(\d+)', self.path)
            if match:
                status, body = menu_start(arm, int(match[1]), params)
                return self._send(status, body)
            skill = self.path.strip('/')
            status, body = arm.start(skill, params)
            self._send(status, body)

    return Handler


def shutdown_arm(arm, wait_s=25.):
    """서버를 끌 때 실행 중인 작업이 있으면 정지 신호를 보내고 끝날 때까지 기다린다(집기 프로세스가 게인 복구·팔 고정·펌프 정지를 마치도록).
    서비스(systemd)는 프로세스 그룹 전체에 신호를 보내지만, 백그라운드로 켠 경우에는 이 서버만 신호를 받으므로 여기서 직접 전달한다."""
    if arm.status()['state'] != 'busy':
        return True
    print('실행 중인 작업에 정지를 보내고 정리가 끝나길 기다립니다…', flush=True)
    arm.stop()
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        if arm.status()['state'] != 'busy':
            return True
        time.sleep(.2)
    print('정리가 끝나지 않았습니다. 팔 상태를 확인하세요.', flush=True)
    return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=int(os.environ.get('ARM_API_PORT') or 8000))
    parser.add_argument('--cors-origin', help='웹 화면이 브라우저에서 직접 부를 때 허용할 주소(예: http://192.168.0.10:3000)')
    parser.add_argument('--token', default=os.environ.get('ARM_API_TOKEN') or None,
                        help='Authorization: Bearer <값>. 127.0.0.1이 아닌 주소로 열려면 필수(없으면 ARM_API_TOKEN 환경 변수)')
    args = parser.parse_args(argv)
    if args.host not in ('127.0.0.1', 'localhost', '::1') and not args.token:
        parser.error('다른 장비에서 접근하게 열려면 --token이 필요합니다(로봇을 움직이는 API입니다).')
    arm = Arm()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(arm, args.token, args.cors_origin))
    print(f'로봇팔 API: http://{args.host}:{args.port}  (Ctrl+C로 종료)', flush=True)
    arm.log.add('info', '서버', f'시작 http://{args.host}:{args.port}', f'펌프 {devices.get("pump_port")} / 서보 {devices.get("servo_port", "(프로필)")}')

    def warm_stt():                                  # 첫 음성 명령이 느리지 않게 Whisper를 미리 불러 둔다(없으면 조용히 넘어간다)
        try:
            arm_stt.load()
            print(f'음성 인식 준비됨: {arm_stt.describe()}', flush=True)
            arm.log.add('info', '음성', f'음성 인식 준비됨: {arm_stt.describe()}')
        except arm_stt.SttUnavailable as exc:
            print(f'로컬 음성 인식 꺼짐: {exc}', flush=True)
            arm.log.add('warn', '음성', f'음성 인식 꺼짐: {exc}')
    threading.Thread(target=warm_stt, daemon=True).start()
    # 백그라운드(nohup)로 켜면 SIGINT가 무시된 채로 시작한다. 직접 받아서 아래 정리를 거치게 한다. SIGTERM(kill)도 같게 취급한다.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        shutdown_arm(arm)
        CAMERA.release()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
