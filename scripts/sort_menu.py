#!/usr/bin/env python3
"""번호 메뉴로 물체 넣기·복귀·종료를 실행한다(실행 = 로봇이 움직인다).

  /home/song/miniconda3/envs/lerobot/bin/python scripts/sort_menu.py

목록에서 번호를 입력하면 그에 맞게 scripts/run_sort.py 등을 실행한다. 로봇이 움직이는 항목은 한 번 더 확인한다(Enter=실행, n=취소).
사진은 매번 새로 찍어서 물체·트레이 좌표를 계산한다(run_sort.py가 한다). 이 파일은 명령을 고르고 부르기만 한다.
"""
import json
import os
import re
import subprocess
import unicodedata
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from perception import object_registry      # noqa: E402  등록 물체(config/registered_objects.json)
from motion import guard_mode               # noqa: E402  가드 배율(HOME 판정 펄스)
from sorting.place_followup import home_return_succeeded, home_failure_reason

PY = sys.executable
BUILTIN_OBJECTS = (('red_cube', '빨간 정육면체', 1), ('white_box', '흰 상자', 2), ('tape_measure', '줄자', 3))
OBJECTS = {}            # 번호 → (label, 이름, 기본 칸). 기본 세 물체 + 등록 물체(refresh_objects가 채운다)
OBJECTS_DEFAULT = {}    # label → 기본 칸


def registered():
    """등록 물체 목록. 등록 파일이 깨졌으면 알리고 기본 세 물체만 쓴다(메뉴가 멈추지 않게)."""
    try:
        return object_registry.load(ROOT)
    except ValueError as exc:
        print(f'   등록 물체 파일을 읽지 못해 기본 세 물체만 씁니다: {exc}')
        return []


def refresh_objects():
    """OBJECTS·OBJECTS_DEFAULT를 지금 등록 상태로 그 자리에서 고친다(arm_api도 같은 dict를 본다)."""
    rows = list(BUILTIN_OBJECTS) + [(e['label'], e['name'], int(e['slot'])) for e in registered()]
    OBJECTS.clear()
    OBJECTS.update({str(number): row for number, row in enumerate(rows, 1)})
    OBJECTS_DEFAULT.clear()
    OBJECTS_DEFAULT.update({label: slot for label, _, slot in rows})


refresh_objects()
COMMON = ['--acknowledge-unverified-suction', '--accept-uncertain-tray', '--tray-numbering', 'far_from_base',
          '--attempts', '1', '--approach-tracking-limit', '100',
          '--place-rate', '300', '--place-pulses-per-step', '4', '--home-rate', '300']   # 속도 우선(사용자 2026-10-19): 운반·후퇴 300, 틱당 4펄스(13.4ms), 자동 HOME 복귀 300
COLOR = sys.stdout.isatty() and not os.environ.get('NO_COLOR')


def style(text, *codes):
    return f"\033[{';'.join(codes)}m{text}\033[0m" if COLOR and codes else str(text)


def bold(t): return style(t, '1')
def dim(t): return style(t, '2')
def green(t): return style(t, '1', '32')
def red(t): return style(t, '1', '31')
def yellow(t): return style(t, '33')
def cyan(t): return style(t, '1', '36')


def width(text):
    """터미널 칸 너비(한글 등 전각 문자는 2칸)."""
    return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in plain(text))


def box(lines, title, inner=58):
    """각진 네모 상자. 글자 너비를 맞춘 뒤 줄마다 오른쪽 테두리를 붙인다. 테두리만 청록색."""
    head = '┌─ ' + title + ' '
    out = [cyan(head[:3]) + bold(title) + cyan(' ' + '─' * (inner + 3 - width(head)) + '┐')]
    out += [cyan('│') + ' ' + line + ' ' * (inner - width(plain(line))) + ' ' + cyan('│') for line in lines]
    out.append(cyan('└' + '─' * (inner + 2) + '┘'))
    return '\n'.join(out)


def plain(text):
    """색 코드를 뺀 글자(너비 계산용)."""
    return re.sub(r'\033\[[0-9;]*m', '', text)


TRAY_FRAME = (ROOT / 'output' / 'tray_frames' / 'latest_rgb.png', ROOT / 'output' / 'tray_frames' / 'latest_depth.npy')
HOME_COMMAND = [PY, str(ROOT / 'scripts' / 'return_home.py'), '--execute', '--supported', '--travel-rate', '150', '--tracking-limit', '32']
ITEMS = [
    ('1', '책상 정리하기 (정육면체 → 흰 상자 → 줄자 → 등록 물체 순서)'),
    ('2', '물체와 칸을 골라서 넣기'),
    ('3', 'HOME으로 복귀'),
    ('4', 'unknown 집기 (책상의 등록 안 된 물체 하나를 바로 찍어 집기)'),
    ('5', '물체 등록·삭제'),
    ('0', '나가기'),
]



def brief_message(message):
    """화면 문구만 번역·요약하고 내부 판정용 원문은 유지한다."""
    text = str(message or '')
    names = {'white_box': ('흰 상자가', '흰 상자'), 'red_cube': ('빨간 정육면체가', '빨간 정육면체'),
             'tape_measure': ('줄자가', '줄자'), 'unknown': ('unknown이', 'unknown'),
             **{label: (f'{name}이(가)', name) for label, name, _ in OBJECTS.values() if label.startswith('custom_')}}
    subject, name = next((pair for label, pair in names.items() if label in text), ('물체가', '물체'))
    # 경계 수치를 원문에서 읽어 실제 적용 중인 제한과 같은 값을 안내한다.
    radius = re.search(r'radius ([0-9.]+) m outside (?:the )?([0-9.]+)-([0-9.]+) m pick envelope', text)
    if radius:
        distance, lower, upper = (float(value) * 100 for value in radius.groups())
        if distance < lower:
            return f'{subject} 너무 가깝습니다({distance:g}cm, 최소 {lower:g}cm). 조금 멀리 놓으세요.'
        if distance > upper:
            return f'{subject} 너무 멉니다({distance:g}cm, 최대 {upper:g}cm). 로봇 쪽으로 옮기세요.'
    translations = (
        ('more than one unknown', '새 물체는 한 번에 하나씩 놓으세요.'),
        ('unknown not found', '등록 안 된 물체를 찾지 못했습니다. 책상(트레이 밖, 팔이 닿는 곳)에 하나를 두고 다시 실행하세요.'),
        ('the tray moved since the stored frame', '트레이 위치 변경: 빈 트레이로 계획을 한 번 실행해 다시 저장하세요.'),
        ('could collide with unregistered', '운반 경로에 등록 안 된 물체가 있어 부딪힐 수 있습니다. 그 물체를 팔·트레이 사이에서 치워 주세요.'),
        ('could collide with', f'운반 경로에 다른 물체가 있어 부딪힐 수 있습니다. {name} 주변을 비워 주세요.'),
        ('not reachable', '도달 불가: 물체·트레이 위치를 조정하세요.'),
        ('tray not found', '트레이 인식 실패: 트레이 전체가 보이게 하세요.'),
        ('not found in the current camera view', f'{name} 인식 실패: 카메라 화면을 확인하세요.'),
        ('not held', '집힘 확인 실패: 물체와 흡착컵을 확인하세요.'),
        ('hold unconfirmed', '정지 유지 확인 실패: 팔 상태를 확인하세요.'),
        ('hold not confirmed', '정지 유지 확인 실패: 팔 상태를 확인하세요.'),
        ('hold is not settled', '팔이 안정되지 않았습니다. 팔 상태를 확인하세요.'),
        ('hold changed', '정지 중 위치 변화 감지. 팔 상태를 확인하세요.'),
        ('unexpected torque', '모터 토크 상태 이상: 전원과 팔 상태를 확인하세요.'),
        ('pose outside phase tolerance', '목표 위치 오차 초과. 팔 상태를 확인하세요.'),
        ('no devices found', '카메라 연결을 확인하세요.'),
        ('device is absent', '장치 연결을 확인하세요.'),
        ('cancelled', '사용자 요청으로 중단했습니다.'),
    )
    for needle, korean in translations:
        if needle in text:
            return korean
    if 'is not pickable' in text:
        detail = text.split('is not pickable:', 1)[-1].split(' / ', 1)[0].strip()
        if any('가' <= char <= '힣' for char in detail):
            return detail[:140]
        return f'{name} 집기 조건 미충족. 물체 위치·윗면·흡착 상태를 확인하세요.'
    if '여유가 부족' in text:
        # 칸 여유 부족은 돌려 놓을 각도 안내가 뒤에 붙으므로 자르지 않는다.
        return text.split('error: ', 1)[-1].replace(' 다른 칸을 선택하세요.', '').strip()
    uncertainty = re.search(r'uncertain by up to about ([0-9.]+) mm', text)
    if uncertainty:
        return f'칸 위치 오차 최대 약 {uncertainty[1]}mm'
    if ' / ' in text:
        detail = text.rsplit(' / ', 1)[-1]
        if any('가' <= char <= '힣' for char in detail):
            return detail[:140]
    if 'pick_auto exited' in text and ' / ' in text:
        # 원인 문구가 영어여도 숨기지 않는다(예: 운반 높이 한도). 일반 안내만 보이면 원인을 알 수 없다.
        detail = text.rsplit(' / ', 1)[-1].split('error: ', 1)[-1].strip()
        if detail:
            return f'집기·놓기 중단: {detail[:160]}'
    for needle, korean in (
        ('suction not verified', '흡착 성능 미확인'),
        ('top surface is not flat', '윗면이 고르지 않아 흡착 불안정'),
        ('pick_auto exited', '집기·놓기 중단. 팔 상태와 실행 기록을 확인하세요.'),
    ):
        if needle in text:
            return korean
    # 미등록 원인은 임의로 단정하지 않고 원문을 짧게 남긴다.
    text = text.split(' / ', 1)[0]
    return text if len(text) <= 140 else text[:137] + '…'


def phase_label(name):
    """분할 이동 단계는 같은 한국어 진행 문구로 묶는다."""
    labels = (
        ('unknown_verify', 'unknown 위치·크기 재확인'),
        ('unknown_wait', '기준 저장 완료: 새 물체 하나를 놓고 손을 빼세요(90초 대기)'),
        ('unknown_snapshot', '책상의 등록 안 된 물체 확인'),
        ('unknown_detected', 'unknown 감지 완료'),
        # 놓기(place_exec 단계 이름). raise_clear·raise_low는 물체를 든 직후(놓기 전) 단계다. 더 긴 이름을 먼저 둔다.
        ('place_raise_clear', '든 채 바닥에서 올리기'), ('place_raise_low', '든 채 낮게 올리기'), ('place_raise', '운반 높이로 상승'),
        ('place_pull_in', '로봇 쪽으로 끌어오기'), ('place_carry', '트레이로 운반'), ('place_align', '놓기 전 위치 정렬'), ('place_lower', '칸으로 내리기'),
        ('place_exit', '놓은 뒤 한 번에 빠져나오기'), ('place_retire', '트레이에서 후퇴'), ('place_retreat', '놓은 뒤 후퇴'),
        ('place_untilt', '손목 다시 세우기'), ('place_tilt', '손목 각도 조정'),
        # 집기(pick_auto 단계 이름)
        ('reset_raise', '시작 전 올리기'), ('chain_raise', '트레이 위에서 올리기'), ('approach', '물체에 접근'),
        ('descent', '흡착 위치로 하강'), ('lift_to_carry', '들어 올리며 운반 자세 만들기'), ('lift', '물체 들어 올리기'),
        ('rise_', '컵 중심 다시 맞추기'), ('recenter', '컵 중심 다시 맞추기'), ('redescend', '컵 중심 다시 맞추기'),
        ('probe', '시험 이동'), ('home', 'HOME 복귀'), ('return', '복귀'),
    )
    return next((label for prefix, label in labels if name.startswith(prefix)), name)


def sort_command(label, slot=None, execute=True, use_tray_frame=True):
    cmd = [PY, str(ROOT / 'scripts' / 'run_sort.py')]
    if execute:
        cmd += ['--execute', '--supported']
    cmd += ['--object', label, *COMMON]
    if slot is not None:
        cmd += ['--slot', str(slot)]
    if use_tray_frame and all(p.exists() for p in TRAY_FRAME):      # 저장한 빈 트레이 위치(밀렸으면 밀린 만큼 옮겨 쓴다). 칸에 물체가 들어가도 칸 위치를 안다
        cmd += ['--tray-frame', str(TRAY_FRAME[0]), str(TRAY_FRAME[1]), '--tray-fallback-live']
    return cmd


def unknown_command(slot=3, execute=True, use_tray_frame=True):
    cmd = [PY, str(ROOT / 'scripts/pick_unknown.py'), '--slot', str(slot), *COMMON]
    if execute:
        cmd += ['--execute', '--supported']
    if use_tray_frame and all(p.exists() for p in TRAY_FRAME):
        cmd += ['--tray-frame', *(str(p) for p in TRAY_FRAME), '--tray-fallback-live']     # 트레이가 움직였으면 지금 카메라로 다시 찾는다(물체를 다시 놓게 하지 않는다)
    return cmd


def live_view_running():
    done = subprocess.run(['pgrep', '-f', 'live_view.py'], capture_output=True, text=True)
    return bool(done.stdout.split())


def confirm(text):
    if live_view_running():
        print('카메라 창(live_view.py)이 켜져 있습니다. 닫은 뒤 다시 실행하세요(카메라 충돌).')
        return False
    answer = input(f'{text}  [Enter=실행 / n=취소] ').strip().lower()
    return answer not in ('n', 'no', 'ㅜ')


HOME_RAW_FALLBACK = {'base': 2110, 'shoulder': 1823, 'elbow': 3018, 'wrist_flex': 3075}     # 파일을 못 읽을 때만(2026-10-19 기록)


def load_home_raw(root=ROOT):
    """return_home.py가 쓰는 HOME 기록(original_current_hold.json)을 그대로 읽는다. 예전엔 옛 HOME 값이 코드에 박혀 있어서
    HOME을 다시 등록한 뒤(손목 2956 → 3075)에는 HOME에 둔 팔을 'HOME이 아니다'라고 판정해 메뉴가 아무것도 실행하지 않았다(2026-10-19)."""
    try:
        data = json.loads((Path(root) / 'output/ik_resume_20261012/original_current_hold.json').read_text())
        home = {n: int(row['position']) for n, row in data['joints'].items()}
        if set(home) == set(HOME_RAW_FALLBACK):
            return home
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return dict(HOME_RAW_FALLBACK)


HOME_RAW = load_home_raw()
FOLD_NEAR_PULSES = 200       # 접힌 HOME 근처(모든 관절이 이 안): 토크를 켠 뒤 return_home이 접힌 채 HOME에 맞춘다
HOME_NEAR_PULSES = 40         # 토크가 꺼진 팔이 HOME에 놓여 있다고 볼 허용 범위(펄스)


def read_arm_state():
    """servo_driver.py inspect(읽기 전용)로 관절별 (토크, 위치)를 읽는다. 실패하면 None."""
    done = subprocess.run([PY, str(ROOT / 'scripts' / 'servo_driver.py'), 'inspect'], cwd=ROOT, capture_output=True, text=True)
    return parse_inspect(done.stdout)


def parse_inspect(text):
    try:
        data, _ = json.JSONDecoder().raw_decode(text[text.index('{'):])
        return {name: (int(row['torque']), int(row['position'])) for name, row in data['joints'].items()}
    except (ValueError, KeyError, TypeError):
        return None


def classify_arm(state):
    """'ok'(전부 토크 켜짐) / 'off_at_home'(전부 꺼짐, HOME 근처) / 'off_near_fold'(전부 꺼짐, 접힌 HOME 근처) / 'off_elsewhere'(전부 꺼짐, HOME 아님) / 'mixed' / 'unknown'."""
    if not state or set(state) != set(HOME_RAW):
        return 'unknown'
    torques = {t for t, _ in state.values()}
    if torques == {1}:
        return 'ok'
    if torques == {0}:
        near = all(abs(state[n][1] - HOME_RAW[n]) <= guard_mode.pulses(HOME_NEAR_PULSES) for n in HOME_RAW)
        if near:
            return 'off_at_home'
        return 'off_near_fold' if all(abs(state[n][1] - HOME_RAW[n]) <= guard_mode.pulses(FOLD_NEAR_PULSES) for n in HOME_RAW) else 'off_elsewhere'
    return 'mixed'


def ensure_torque():
    """로봇을 움직이기 전에 서보 토크를 확인한다. 꺼져 있고 팔이 HOME에 있으면 붙잡을지 물어 처리한다. 움직여도 되면 True."""
    state = classify_arm(read_arm_state())
    if state == 'ok':
        return True
    if state == 'unknown':
        print('   서보 상태를 읽지 못했습니다(USB 연결과 전원을 확인하세요).')
        return False
    if state == 'off_at_home':
        print('   서보 토크가 꺼져 있습니다(전원이 한 번 끊겼을 수 있습니다). 팔은 HOME 위치에 놓여 있습니다.')
        if input('   토크를 켜서 팔을 붙잡을까요? 팔을 손으로 받쳐 주세요.  [Enter=붙잡기 / n=취소] ').strip().lower() in ('n', 'no'):
            return False
        subprocess.run([PY, str(ROOT / 'scripts' / 'servo_driver.py'), 'hold', '--arm', '--supported'], cwd=ROOT, capture_output=True, text=True)
        if classify_arm(read_arm_state()) != 'ok':
            print('   토크를 켜지 못했습니다. 팔을 확인하세요.')
            return False
        print('   토크를 켜서 팔을 붙잡았습니다.')
        return True
    if state == 'off_near_fold':
        print('   서보 토크가 꺼져 있고 팔이 접힌 HOME 근처에 있지만 조금 어긋나 있습니다. 토크를 켜서 붙잡은 뒤 HOME 자세로 맞춥니다.')
        if input('   토크를 켜서 팔을 붙잡을까요? 팔을 손으로 받쳐 주세요.  [Enter=붙잡기 / n=취소] ').strip().lower() in ('n', 'no'):
            return False
        subprocess.run([PY, str(ROOT / 'scripts' / 'servo_driver.py'), 'hold', '--arm', '--supported'], cwd=ROOT, capture_output=True, text=True)
        if classify_arm(read_arm_state()) != 'ok':
            print('   토크를 켜지 못했습니다. 팔을 확인하세요.')
            return False
        print('   토크를 켜서 팔을 붙잡았습니다. HOME 자세로 맞춥니다.')
        code, summary, _ = run_streaming([PY, str(ROOT / 'scripts' / 'return_home.py'), '--execute', '--supported', '--travel-rate', '150', '--tracking-limit', '32'])
        done = home_return_succeeded(code, summary)
        print('   → HOME 자세로 맞췄습니다.' if done else f'   → HOME에 맞추지 못했습니다: {brief_message(home_failure_reason(code, summary))} (팔 위치를 확인하세요)')
        return done
    if state == 'off_elsewhere':
        print('   서보 토크가 꺼져 있고 팔이 HOME 위치가 아닙니다. 팔을 손으로 받쳐 HOME 자세로 옮긴 뒤 다시 실행하세요.')
    else:
        print('   일부 서보만 토크가 켜져 있습니다. 팔을 받치고 servo_driver.py disarm으로 토크를 모두 끈 뒤 HOME에서 다시 시작하세요.')
    return False


def run_streaming(cmd, on_event=None, on_start=None, quiet=False):
    """명령을 실행하고 진행 단계만 한 줄씩 보여 준다. 마지막 JSON 줄(결과)을 돌려준다.

    arm_api.py가 같은 실행 경로를 쓰도록 훅을 둔다(메뉴는 기본값 그대로 화면에 출력):
    on_event('phase', (원래 이름, 한국어 문구)) / on_event('plan', (계획 dict, 경고 목록)), on_start(proc), quiet=True면 출력 안 함.
    """
    started = time.monotonic()
    proc = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    if on_start:
        on_start(proc)
    echo = (lambda *a, **k: None) if quiet else print
    last_json = None
    last_error = None
    previous_phase = None
    step = 0
    tail = []                                   # JSON이 아닌 마지막 출력 줄(실패했을 때 원인을 보려고 결과에 붙인다)
    for line in proc.stdout:
        line = line.strip()
        if line and not line.startswith('{'):
            tail = (tail + [line[:300]])[-12:]
        if not line.startswith('{') and 'error:' in line:
            last_error = line.split('error:', 1)[1].strip()
        if line.startswith('{"phase"'):
            try:
                raw = json.loads(line)['phase']
                phase = phase_label(raw)
                if phase != previous_phase:
                    step += 1
                    echo(f'   {dim(f"{step:>2}")} {cyan("›")} {phase}', flush=True)
                    previous_phase = phase
                    if on_event:
                        on_event('phase', (raw, phase))
            except (ValueError, KeyError):
                pass
        elif line.startswith('{'):
            try:
                last_json = json.loads(line)
            except ValueError:
                pass
            if line.startswith('{"plan"'):
                plan = last_json.get('plan', {})
                name = next((name for label, name, _ in OBJECTS.values() if label == plan.get('label')), plan.get('label'))
                target = [round(v) for v in plan.get('manual_target_mm', [])]
                echo(f"   {bold('계획')} {name} {cyan('→')} {plan.get('slot_number')}번 칸  {dim(f'목표 {target} mm')}", flush=True)
                slots = plan.get('tray_slots') or {}
                if slots:
                    echo(dim('   트레이 칸 위치(로봇 기준 거리): ') + ', '.join(
                        f"{k}번 {v['radius_m']:.3f}m" for k, v in sorted(slots.items())
                    ), flush=True)
                hidden_warnings = (
                    'suction not verified for this object',
                    'top surface is not flat;',
                    'tray slot positions are uncertain by up to about ',
                )
                warnings = list(dict.fromkeys(
                    brief_message(w) for w in plan.get('warnings', [])
                    if not any(str(w).startswith(prefix) for prefix in hidden_warnings)
                ))
                if warnings:
                    echo(yellow('   ⚠ 주의: ' + ' / '.join(warnings)), flush=True)
                if on_event:
                    on_event('plan', (plan, warnings))
    proc.wait()
    if last_json is not None and last_error and not last_json.get('ok', True):
        last_json['reason'] = f"{last_json.get('reason')} / {last_error}"
    if last_json is not None and tail and (proc.returncode != 0 or not last_json.get('ok', True)):
        last_json['_tail'] = tail
    return proc.returncode, last_json, time.monotonic() - started


def report(code, result, seconds):
    ok = bool(result and result.get('ok')) and code == 0
    print('   ' + (green('✔ 성공') if ok else red('✘ 실패')) + dim(f'  ({seconds:.0f}초)'))
    if not ok:
        reason = (result or {}).get('reason')
        print(red('   이유:'), brief_message(reason) if reason else f'종료 코드 {code}')
    return ok


def newest_result(since):
    """이번 실행이 만든 가장 최근 pick_trials 결과(result.json). 없으면 None."""
    folders = [d for d in (ROOT / 'output' / 'pick_trials').glob('2026*') if (d / 'result.json').exists() and d.stat().st_mtime >= since - 1]
    if not folders:
        return None
    try:
        return json.loads((max(folders, key=lambda d: d.stat().st_mtime) / 'result.json').read_text())
    except ValueError:
        return None


def recover_home_if_placed(since):
    """물체는 칸에 넣었는데(motion_cycle_complete) 자동 HOME 복귀만 실패한 경우: 복귀를 다시 시도하고 성공으로 본다."""
    result = newest_result(since)
    if not result or not result.get('motion_cycle_complete') or result.get('error'):
        return False
    print('   물체는 칸에 넣었지만 자동 HOME 복귀가 실패했습니다. HOME 복귀를 다시 시도합니다.')
    code, summary, _ = run_streaming([PY, str(ROOT / 'scripts' / 'return_home.py'), '--execute', '--supported', '--travel-rate', '150', '--tracking-limit', '32'])
    done = home_return_succeeded(code, summary)
    print('   → HOME 복귀 완료' if done else f'   → HOME 복귀 실패: {brief_message(home_failure_reason(code, summary))} (팔 위치를 확인하세요)')
    return done


CHAIN_PLAN_DIR = ROOT / 'output' / 'chain_plans'


def chain_flags(index, count):
    """책상 정리하기(메뉴 1): 마지막이 아니면 놓은 뒤 HOME을 건너뛰고(트레이 위에서 공구를 세운 채 선다),
    첫 물체가 아니면 그 자리에서 바로 다음 물체로 간다. 조건이 안 맞으면 pick_auto가 기존 HOME 리셋을 쓴다."""
    flags = []
    if index < count - 1:
        flags.append('--skip-home-after-place')
    if index > 0:
        flags.append('--start-above-tray')
    return flags


def plan_command(label, path, use_tray_frame=True):
    """계획만(카메라, 로봇은 안 움직임) 하고 path에 저장하는 명령. 연속 정리는 팔이 HOME일 때 세 물체를 미리 계획한다
    (트레이 위에 선 팔이 위 카메라를 가리지 않게)."""
    return sort_command(label, use_tray_frame=use_tray_frame) + ['--plan-out', str(path)]


def chained_command(label, path, index, count, from_home=False):
    flags = [f for f in chain_flags(index, count) if not (from_home and f == '--start-above-tray')]
    return sort_command(label) + ['--plan-in', str(path), *flags]


CHAIN_REFUSED_EXIT_CODE = 4     # pick_auto: 트레이 위에서 바로 갈 수 없어 움직이기 전에 멈춤(sorting.place_followup과 같은 값)


def chain_refused(result):
    attempts = (result or {}).get('attempts') or []
    return bool(attempts) and attempts[-1].get('exit_code') == CHAIN_REFUSED_EXIT_CODE


def run_all_chained(run=None, on_item=None, stopped=lambda: False, labels=None):
    """메뉴 1(책상 정리하기): 정육면체 → 흰 상자 → 줄자(labels를 주면 그 물체만, 같은 순서). (1) HOME에서 차례로 계획(사진)
    (2) 놓은 뒤 HOME을 거치지 않고 다음 물체. 계획에서 물체가 안 보이면 멈추지 않고 건너뛴다(책상에 있는 것만 정리).
    등록 안 된 물체는 집지 않고 장애물로만 피한다(계획마다 센 수 중 가장 큰 값을 unregistered_left로 알린다).
    run(cmd) → (code, result, seconds). on_item(label, stage)은 진행 표시용.
    돌려주는 것: dict(ok, failed_label, reason, items, skipped, unregistered_left)."""
    left = [0]

    def done(**outcome):
        return dict(outcome, items=items, skipped=skipped, unregistered_left=left[0])

    run = run or run_streaming
    stamp = time.strftime('%Y%m%dT%H%M%S')
    refresh_objects()
    order = [label for label, _, _ in OBJECTS.values()]          # 기본 세 물체 → 등록 물체(등록 순서)
    wanted = [label for label in order if labels is None or label in labels]
    planned, items, skipped = [], [], []
    for label in wanted:
        if stopped():
            return done(ok=False, failed_label=label, reason='cancelled')
        if on_item:
            on_item(label, 'plan')
        path = CHAIN_PLAN_DIR / f'{stamp}_{label}.json'
        code, result, _ = run(plan_command(label, path))
        if code != 0 and 'the tray moved since the stored frame' in str((result or {}).get('reason', '')) and all(p.exists() for p in TRAY_FRAME):
            code, result, _ = run(plan_command(label, path, use_tray_frame=False))
        left[0] = max(left[0], int((result or {}).get('unregistered_left') or 0))
        if code != 0 or not (result or {}).get('ok'):
            reason = str((result or {}).get('reason') or f'종료 코드 {code}')
            if 'not found in the current camera view' in reason:
                skipped.append(label)                 # 책상에 없다: 건너뛰고 다음 물체
                continue
            # 다른 계획 실패(트레이 못 찾음 등)는 그 물체부터 하지 않는다. 앞에서 계획된 물체는 넣고(마지막은 HOME 복귀), 이 물체에서 멈췄다고 알린다.
            failed = dict(label=label, reason=reason)
            break
        planned.append((label, path))
    else:
        failed = None
    if not planned and failed is None:
        return done(ok=False, failed_label=None, reason='nothing to sort: no registered object found on the desk')
    for index, (label, path) in enumerate(planned):
        if stopped():
            return done(ok=False, failed_label=label, reason='cancelled')
        if on_item:
            on_item(label, 'pick')
        since = time.time()
        code, result, _ = run(chained_command(label, path, index, len(planned)))
        if chain_refused(result) and not stopped():
            # 트레이 위에서 바로 갈 수 없었다(움직이기 전에 멈춤): 검증된 return_home.py로 HOME에 간 뒤 같은 물체를 평소처럼 한다
            if on_item:
                on_item(label, 'recover_home')
            home_code, home, _ = run(HOME_COMMAND)
            if stopped():
                items.append(dict(object=label, ok=False, reason='cancelled'))
                return done(ok=False, failed_label=label, reason='cancelled')
            if not home_return_succeeded(home_code, home):
                reason = '연속 집기 경로 복구 중 HOME 복귀 실패: ' + home_failure_reason(home_code, home)
                items.append(dict(object=label, ok=False, reason=reason))
                return done(ok=False, failed_label=label, reason=reason)
            if on_item:
                on_item(label, 'retry_pick')
            code, result, _ = run(chained_command(label, path, index, len(planned), from_home=True))
        ok = code == 0 and bool((result or {}).get('ok'))
        items.append(dict(object=label, ok=ok, reason=None if ok else str((result or {}).get('reason') or f'종료 코드 {code}')))
        if not ok:
            if not stopped() and (index > 0 or newest_placed_without_error(since)):
                run(HOME_COMMAND)        # 앞 물체를 놓고 트레이 위에 서 있었거나, 넣은 뒤 복귀만 실패했을 수 있다: 기존 HOME 복귀(이미 HOME이면 움직이지 않는다)
            return done(ok=False, failed_label=label, reason=items[-1]['reason'])
    if failed:
        return done(ok=False, failed_label=failed['label'], reason=failed['reason'])
    return done(ok=True, failed_label=None, reason=None)


def unregistered_note(count):
    """책상 정리하기 뒤 남겨 둔 등록 안 된 물체 안내. 없으면 ''."""
    return f'등록 안 된 물체 {count}개는 그대로 뒀습니다(4번 unknown 자동 집기로 따로 넣을 수 있습니다).' if count else ''


def newest_placed_without_error(since):
    found = newest_result(since)
    return bool(found and found.get('motion_cycle_complete') and not found.get('error'))


def run_one(label, slot=None):
    since = time.time()
    code, result, seconds = run_streaming(sort_command(label, slot=slot))
    if (code != 0 and 'the tray moved since the stored frame' in str((result or {}).get('reason', ''))
            and any('--tray-frame' in part for part in sort_command(label, slot=slot))):
        print('   트레이 위치 변경: 카메라로 다시 찾습니다. 실패하면 빈 트레이로 계획을 한 번 실행해 다시 저장하세요.')
        code, result, seconds = run_streaming(sort_command(label, slot=slot, use_tray_frame=False))
    if report(code, result, seconds):
        found = newest_result(since)
        if found and found.get('placed_despite_retreat_error'):
            print('   주의: 물체를 놓고 HOME 복귀했으나 후퇴 중 오류가 있었습니다.')
        return True
    return recover_home_if_placed(since)


def put_one(label, name, slot):
    print('\n' + bold(f'▌ {name} → {slot}번 칸'))
    if not confirm(f'{name}을(를) {slot}번 칸에 넣습니다. 팔 주변을 비웠는지 확인하세요.'):
        print('취소했습니다.')
        return False
    if not ensure_torque():
        return False
    return run_one(label, slot=slot if slot != OBJECTS_DEFAULT[label] else None)



def choose_object():
    refresh_objects()
    for key, (label, name, slot) in OBJECTS.items():
        print(f'   {cyan(key)}  {name}')
    pick = input(bold('물체 번호 ') + cyan('▸ ')).strip()
    return OBJECTS.get(pick)


def last_json(text):
    """출력에서 마지막 JSON 줄(스크립트 결과). 없으면 None."""
    for line in reversed(text.strip().splitlines()):
        if line.startswith('{'):
            try:
                return json.loads(line)
            except ValueError:
                return None
    return None


def register_menu():
    """메뉴 5: 물체 등록·삭제. 위 카메라로 사진 한 장만 찍는다(로봇은 움직이지 않는다)."""
    entries = registered()
    print('\n' + bold('▌ 물체 등록·삭제'))
    for entry in entries:
        print('   ' + object_registry.describe(entry))
    if not entries:
        print(dim('   등록한 물체가 없습니다.'))
    left = object_registry.MAX_REGISTERED - len(entries)
    pick = input(f'   a=새로 등록(남은 자리 {left}개) / d=삭제 / Enter=돌아가기 ' + cyan('▸ ')).strip().lower()
    if pick in ('a', 'ㅁ'):
        name = input('   물체 이름: ').strip()
        try:
            object_registry.check_name(name, ROOT)
        except ValueError as exc:
            print(red('   ' + str(exc)))
            return
        slot = input('   넣을 칸(1~3): ').strip()
        if slot not in ('1', '2', '3'):
            print('   칸은 1~3 중 선택하세요.')
            return
        height = input('   자로 잰 높이 mm(Enter=사진으로 잼, 약 ±3mm): ').strip()
        try:
            height = float(height) if height else None
        except ValueError:
            print('   높이는 숫자(mm)로 입력하세요.')
            return
        if live_view_running():
            print('카메라 창(live_view.py)이 켜져 있습니다. 닫은 뒤 다시 실행하세요(카메라 충돌).')
            return
        input(f'   책상(트레이 밖)에 {name} 하나만 두고 손을 뺀 뒤 Enter를 누르세요. 로봇은 움직이지 않습니다. ')
        cmd = [PY, str(ROOT / 'scripts' / 'register_object.py'), '--name', name, '--slot', slot]
        if height is not None:
            cmd += ['--height-mm', repr(height)]
    elif pick in ('d', 'ㅇ') and entries:
        for number, entry in enumerate(entries, 1):
            print(f'   {cyan(number)}  {entry["name"]}')
        chosen = input('   지울 번호: ').strip()
        if not chosen.isdigit() or not 1 <= int(chosen) <= len(entries):
            print('   목록에 있는 번호를 입력하세요.')
            return
        entry = entries[int(chosen) - 1]
        if input(f"   '{entry['name']}' 등록을 지울까요? [y=지우기 / Enter=취소] ").strip().lower() not in ('y', 'yes', 'ㅛ'):
            print('   취소했습니다.')
            return
        cmd = [PY, str(ROOT / 'scripts' / 'register_object.py'), '--remove', entry['label']]
    else:
        return
    done = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    result = last_json(done.stdout) or {}
    if result.get('ok'):
        print('   ' + green(result.get('message', '완료')))
        if result.get('object'):
            print(dim('   이제 2번(골라서 넣기)과 1번(책상 정리하기)에서 집습니다. 흡착은 확인하지 않았으니 처음 몇 번은 지켜봐 주세요.'))
    else:
        print('   ' + red('실패: ' + (result.get('reason') or (done.stderr.strip().splitlines() or [f'종료 코드 {done.returncode}'])[-1])))
    refresh_objects()


def menu():
    rows = [f'{cyan(f"{key:>2}") if key != "0" else dim(f"{key:>2}")}  {text if key != "0" else dim(text)}' for key, text in ITEMS]
    print('\n' + box(rows, '물체 분류기'))
    return input(bold('번호 입력 ') + cyan('▸ ')).strip()


def main():
    while True:
        choice = menu()
        if choice == '0':
            print(dim('종료합니다.'))
            return
        if choice == '1':
            refresh_objects()
            order = ' → '.join(name for _, name, _ in OBJECTS.values())
            if not confirm(f'책상을 정리합니다({order} 순서, 책상에 없는 것은 건너뜀). 팔 주변을 비웠는지 확인하세요.'):
                print('취소했습니다.')
                continue
            if not ensure_torque():
                continue
            names = {label: (name, slot) for label, name, slot in OBJECTS.values()}

            def show(label, stage):
                name, slot = names[label]
                print('\n' + bold(f'▌ {name} ' + ('계획(사진)' if stage == 'plan' else f'→ {slot}번 칸')))

            outcome = run_all_chained(on_item=show)
            missing = ', '.join(names[label][0] for label in outcome['skipped'])
            if outcome['ok']:
                print('\n' + green('✔ 책상의 물체를 모두 넣었습니다.' + (f' (책상에 없어 건너뜀: {missing})' if missing else '')))
            elif outcome['failed_label'] is None:
                print('책상에서 등록된 물체를 찾지 못했습니다. 넣을 것이 없습니다.')
            else:
                print(f"{names[outcome['failed_label']][0]}에서 멈췄습니다: {brief_message(outcome['reason'])}. 나머지는 실행하지 않았습니다. 원인을 확인한 뒤 다시 실행하세요.")
            if unregistered_note(outcome['unregistered_left']):
                print('   ' + yellow(unregistered_note(outcome['unregistered_left'])))
        elif choice == '2':
            picked = choose_object()
            slot = input('칸 번호(1~3): ').strip()
            if picked is None or slot not in ('1', '2', '3'):
                print('잘못된 입력입니다.')
                continue
            label, name, _ = picked
            put_one(label, name, int(slot))
        elif choice == '4':
            slot = input('unknown을 놓을 칸(1~3): ').strip()
            if slot not in ('1', '2', '3'):
                print('칸은 1~3 중 선택하세요.')
                continue
            execute = True
            if execute and (not confirm('책상의 등록 안 된 물체 하나를 바로 찍어서 집습니다. 집을 물체 하나만 두고 손을 빼세요.') or not ensure_torque()):
                continue
            if not execute and live_view_running():
                print('카메라 창을 닫은 뒤 다시 실행하세요.')
                continue
            code, result, seconds = run_streaming(unknown_command(int(slot), execute=execute))
            if execute:
                report(code, result, seconds)
            else:
                print('   계획 확인 완료(이동 없음).' if code == 0 else '   실패: ' + brief_message((result or {}).get('reason') or f'종료 코드 {code}'))
        elif choice == '3':
            if confirm('HOME으로 복귀합니다.') and ensure_torque():
                code, result, seconds = run_streaming([PY, str(ROOT / 'scripts' / 'return_home.py'), '--execute', '--supported', '--travel-rate', '150', '--tracking-limit', '32'])
                done = home_return_succeeded(code, result)
                print('   → 복귀 완료' if done else f'   → 복귀 결과: {brief_message(home_failure_reason(code, result))}')
        elif choice == '5':
            register_menu()
        else:
            print('목록에 있는 번호를 입력하세요.')


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\n종료합니다.')
        os._exit(0)
