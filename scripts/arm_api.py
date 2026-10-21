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
import hmac
import json
import re
import signal
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))   # 물체 인식(perception) 모듈
import sort_menu as menu     # 명령 만들기·실행·상태 판정은 메뉴와 같은 코드를 쓴다
import arm_stt               # 로컬 음성 인식(faster-whisper는 처음 쓸 때 불러온다)

OBJECTS = {'white_box': 1, 'red_cube': 2, 'tape_measure': 3, 'unknown': 3}      # 물체 → 기본 칸
NAMES = {'white_box': '흰 상자', 'red_cube': '빨간 정육면체', 'tape_measure': '줄자', 'unknown': 'unknown'}
ALL_ORDER = ('white_box', 'red_cube', 'tape_measure')     # 메뉴 4번: 흰 상자 → 정육면체 → 줄자, 각자 기본 칸
HOME_CMD = [menu.PY, str(menu.ROOT / 'scripts' / 'return_home.py'), '--execute', '--supported', '--travel-rate', '150', '--tracking-limit', '32']

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


class Arm:
    """한 번에 한 작업을 돌리는 상태. HTTP 핸들러와 작업 스레드가 함께 쓴다."""

    def __init__(self, run=menu.run_streaming, read_state=None):
        self.run = run
        self.read_state = read_state or (lambda: menu.classify_arm(menu.read_arm_state()))
        self.lock = threading.Lock()
        self.proc = None
        self.stop_requested = False
        self.capturing = False
        self.scene = None                     # 마지막 get image 결과(웹 화면 표시용 JPEG 포함)
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
            bgr, depth = (grab or grab_astra)()
            scene = describe_scene(bgr, depth)
            import base64
            import cv2
            ok, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise OSError('JPEG 변환 실패')
            scene.update(ok=True, captured_at=time.strftime('%Y-%m-%dT%H:%M:%S'), image_url='/api/image/latest.jpg',
                         image_jpeg_base64=base64.b64encode(jpeg.tobytes()).decode())
            with self.lock:
                self.scene = dict(scene, jpeg=jpeg.tobytes())
            return 200, scene
        except Exception as exc:
            return 500, dict(ok=False, reason_code='camera_error', reason=f'{type(exc).__name__}: {str(exc)[:200]}')
        finally:
            with self.lock:
                self.capturing = False

    def stop(self):
        """실행 중인 작업에 SIGINT를 보낸다(메뉴의 Ctrl+C와 같다: pick_auto가 현재 동작을 멈추고 정리한다)."""
        with self.lock:
            proc = self.proc
            running = bool(self.current)
            self.stop_requested = running          # 아직 프로세스를 띄우기 전이면, 띄우자마자 멈추고 다음 단계로 가지 않는다
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
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
        return self.run(command, on_event=lambda k, d: self._event(task_id, k, d), on_start=self._set_proc, quiet=True)

    def _work(self, task_id, skill, params):
        started = time.monotonic()
        result = None
        try:
            result = self._execute(task_id, skill, params)
        except Exception as exc:                                    # 서버가 죽지 않게 모든 실패를 결과로 돌려준다
            result = dict(ok=False, reason=f'{type(exc).__name__}: {exc}', reason_code='failed')
        finally:
            with self.lock:
                task = self.tasks[task_id]
                task['seconds'] = round(time.monotonic() - started, 1)
                task['result'] = dict(result, seconds=task['seconds'], warnings=task['warnings'])
                task['state'] = 'done' if result['ok'] else 'failed'
                self.arm_home = True if result['ok'] else None
                self.proc = None
                self.current = None

    def _execute(self, task_id, skill, params):
        if menu.live_view_running():
            return dict(ok=False, reason='카메라 창(live_view.py)이 켜져 있습니다.', reason_code='camera_busy')
        state = self.read_state()
        if state != 'ok':
            return dict(ok=False, reason_code='arm_state_unknown' if state == 'unknown' else 'torque_off',
                        reason=f'팔 상태가 움직여도 되는 상태가 아닙니다({state}). 사람이 팔을 받치고 토크를 켜야 합니다.')
        if skill == 'return_home':
            code, summary, _ = self._run(task_id, HOME_CMD)
            done = bool(summary and summary.get('home_return_complete'))
            return dict(ok=done, reason=None if done else str((summary or {}).get('error') or f'종료 코드 {code}'),
                        reason_code=None if done else reason_code((summary or {}).get('error')))
        if skill == 'pick_all':
            # 메뉴 4와 같다: HOME에서 세 물체를 미리 계획(사진)한 뒤, 놓은 자리에서 HOME을 거치지 않고 다음 물체로 간다.
            def on_item(label, stage):
                with self.lock:
                    self.tasks[task_id]['item'] = label
                if stage == 'plan':
                    self._event(task_id, 'phase', ('plan', '계획(사진)'))
            outcome = menu.run_all_chained(run=lambda cmd: self._run(task_id, cmd), on_item=on_item, stopped=lambda: self.stop_requested,
                                           labels=params.get('objects'))
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
        return self._pick(task_id, params['object'], params['slot'])

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


def grab_astra():
    """위 카메라 한 장(RGB, 깊이 mm). 기존 촬영 명령을 별도 프로세스로 실행한다(카메라 드라이버를 서버 프로세스에 남기지 않는다)."""
    import subprocess
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
        from tray.tray_frame_check import tray_offset_px, register_tray, move_slots
        stored_bgr, stored_depth = cv2.imread(str(rgb)), np.load(dep)
        stored = detect_tray(stored_bgr, stored_depth, root, first_end='far_from_base')
        if not stored.get('found'):
            return None
        if tray_offset_px(stored_bgr, bgr, depth=stored_depth, root=root)['ok']:
            return stored['slots']
        moved = register_tray(stored_bgr, bgr, depth=stored_depth, root=root)
        return move_slots(stored['slots'], moved, root) if moved['ok'] else None
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
        if slot_containing(xy, slots) is not None:
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
                if action not in ('pick_place', 'pick_all', 'return_home'):
                    return self._send(400, dict(ok=False, reason_code='bad_request', reason='action은 pick_place, pick_all, return_home, status, stop 중 하나입니다.'))
                status, body = arm.start(action, {k: v for k, v in params.items() if k in ('object', 'slot', 'objects')})
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--cors-origin', help='웹 화면이 브라우저에서 직접 부를 때 허용할 주소(예: http://192.168.0.10:3000)')
    parser.add_argument('--token', help='Authorization: Bearer <값>. 127.0.0.1이 아닌 주소로 열려면 필수')
    args = parser.parse_args(argv)
    if args.host not in ('127.0.0.1', 'localhost', '::1') and not args.token:
        parser.error('다른 장비에서 접근하게 열려면 --token이 필요합니다(로봇을 움직이는 API입니다).')
    server = ThreadingHTTPServer((args.host, args.port), make_handler(Arm(), args.token, args.cors_origin))
    print(f'로봇팔 API: http://{args.host}:{args.port}  (Ctrl+C로 종료)', flush=True)

    def warm_stt():                                  # 첫 음성 명령이 느리지 않게 Whisper를 미리 불러 둔다(없으면 조용히 넘어간다)
        try:
            arm_stt.load()
            print(f'로컬 음성 인식 준비됨: Whisper {arm_stt.MODEL} ({arm_stt.DEVICE})', flush=True)
        except arm_stt.SttUnavailable as exc:
            print(f'로컬 음성 인식 꺼짐: {exc}', flush=True)
    threading.Thread(target=warm_stt, daemon=True).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
