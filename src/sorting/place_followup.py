"""place가 끝난 뒤 자동 HOME 복귀: 판단과 실행(기존 scripts/return_home.py를 그대로 부른다). 장치를 직접 열지 않는다.

pick_auto.py가 로봇 버스·펌프를 모두 닫은 뒤(게인 복원 후) 별도 프로세스로 return_home.py를 실행한다. 실패하거나
물체를 해제하지 못한 채 끝났으면(들고 있을 수 있음) 자동으로 움직이지 않는다.
"""
import json
import math
import subprocess
import sys
from pathlib import Path

EMPTY_RUN_DONE = 'empty place run complete'
TIMEOUT_S = 300


def home_return_succeeded(code, summary):
    """Use the same HOME completion contract for API, menu and automatic return."""
    return (code == 0 and isinstance(summary, dict)
            and all(summary.get(k) is True for k in ('home_return_complete', 'final_hold_confirmed', 'restoration_verified'))
            and not any(summary.get(k) for k in ('error', 'final_hold_error', 'recorder_error', 'recorder_close_error')))


def home_failure_reason(code, summary):
    summary = summary if isinstance(summary, dict) else {}
    for key in ('error', 'final_hold_error', 'recorder_error', 'recorder_close_error'):
        if summary.get(key):
            return str(summary[key])
    for key in ('home_return_complete', 'final_hold_confirmed', 'restoration_verified'):
        if summary.get(key) is not True:
            return f'HOME confirmation missing or failed: {key}'
    return f'return_home exited with {code}'


def should_return_home(result, *, enabled):
    if not enabled or not result.get('motion_cycle_complete'):
        return False
    place = result.get('place')
    if not isinstance(place, dict):
        return False
    error = result.get('error')
    if error and not str(error).startswith(EMPTY_RUN_DONE):
        return False
    return bool(place.get('released')) or bool(place.get('empty_run'))


def home_return_command(root, rate, output):
    if isinstance(rate, bool) or not isinstance(rate, (int, float)) or not math.isfinite(rate) or not 50. <= rate <= 300.:
        raise ValueError('home return rate must be 50-300 pulses/s')
    command = [sys.executable, str(Path(root) / 'scripts' / 'return_home.py'), '--execute', '--supported',
               '--travel-rate', str(float(rate)), '--output', str(output)]
    if rate > 150.:                 # 150을 넘으면 어깨 추종이 16을 넘는다(300에서 17~29 실측): 추종 한도를 32로 둔다(사용자: 속도 우선, 2026-10-19)
        command += ['--tracking-limit', '32']
    return command


def run_home_return(root, rate, output, *, runner=subprocess.run):
    command = home_return_command(root, rate, output)
    done = runner(command, cwd=str(root), text=True, capture_output=True, timeout=TIMEOUT_S)
    summary = None
    for line in reversed((done.stdout or '').splitlines()):
        try:
            candidate = json.loads(line)
            if isinstance(candidate, dict) and 'home_return_complete' in candidate:
                summary = candidate
                break
        except ValueError:
            continue
    ok = home_return_succeeded(done.returncode, summary)
    stderr = (done.stderr or '').strip().splitlines()
    error = None if ok else (stderr[-1] if summary is None and stderr else home_failure_reason(done.returncode, summary))
    return dict(ok=bool(ok), rate_pulses_s=float(rate), returncode=done.returncode, summary=summary,
                error=error if not isinstance(error, list) else (error[0] if error else f'return_home exited with {done.returncode}'))


NOT_HELD_EXIT_CODE = 3        # pick_auto.py가 '들어 올렸는데 물체가 안 붙었다'로 place를 하지 않고 끝낼 때의 종료 코드
CHAIN_REFUSED_EXIT_CODE = 4   # --start-above-tray: 트레이 위에서 바로 갈 수 없어 움직이기 전에 멈춤. 호출자가 return_home.py로 HOME에 간 뒤 같은 물체를 다시 한다


def should_return_home_after_failed_pick(result, *, enabled):
    """집힘 확인에서 안 붙었다고 멈춘 경우: 들고 있는 물체가 없으므로 자동으로 HOME에 복귀해 다시 촬영·집기를 할 수 있게 한다."""
    return bool(enabled) and bool(result.get('pick_failed')) and not result.get('motion_cycle_complete')


def _released(result):
    release = result.get('release')
    return isinstance(release, dict) and release.get('vent_command_sent') is True and release.get('stop_command_sent') is True


def should_return_home_after_release_failure(result, *, enabled):
    """물체를 이미 해제(진공 해제·정지 명령까지 보냄)한 뒤 후퇴 단계에서 실패해 팔이 도중에 멈춘 경우: 들고 있는 물체가 없으므로
    팔이 붙잡혀 있고(마지막 hold 확인) 게인이 복원됐으면 자동으로 HOME에 복귀한다(2026-10-19: 후퇴 역기구학 오류)."""
    return bool(enabled) and _released(result) and bool(result.get('error')) and not result.get('motion_cycle_complete') \
        and result.get('restoration_verified') is True and result.get('final_hold_confirmed') is True


def placed_despite_retreat_error(result, home_return):
    """해제까지 끝난 뒤 후퇴만 실패했고 팔이 HOME에 돌아왔으면 물체는 놓은 것으로 본다(오류는 기록에 그대로 남긴다)."""
    return bool(_released(result) and result.get('error') and not result.get('motion_cycle_complete')
                and isinstance(home_return, dict) and home_return.get('ok') is True)
