"""물체 옮기기 재시도 흐름: 계획(카메라) → pick_auto 실행 → 안 붙었으면(종료 코드 3) 다시 계획·집기. 하드웨어를 직접 열지 않는다.

pick_auto.py가 집힘 확인에서 '안 붙음'으로 place 없이 끝나면 HOME으로 복귀해 둔다. 그다음 카메라로 물체를 다시 찾아 새로 집는다.
다른 실패(가드 중단 등)는 재시도하지 않고 멈춘다.
"""
from sorting.place_followup import NOT_HELD_EXIT_CODE, CHAIN_REFUSED_EXIT_CODE


def run_sort(plan_once, pick_once, *, max_attempts=3):
    if isinstance(max_attempts, bool) or not isinstance(max_attempts, int) or not 1 <= max_attempts <= 5:
        raise ValueError('attempts must be an integer 1-5')
    attempts = []
    for number in range(1, max_attempts + 1):
        try:
            plan = plan_once()
        except ValueError as exc:
            return dict(ok=False, attempts=attempts, reason=f'planning failed: {exc}')
        code = pick_once(plan)
        attempts.append(dict(attempt=number, exit_code=code))
        if code == 0:
            return dict(ok=True, attempts=attempts, reason=None)
        if code == CHAIN_REFUSED_EXIT_CODE:
            return dict(ok=False, attempts=attempts, reason_code='chain_reset_required',
                        reason='트레이에서 바로 이동할 수 없습니다. HOME 복귀 후 같은 물체를 재시도해야 합니다.')
        if code != NOT_HELD_EXIT_CODE:
            return dict(ok=False, attempts=attempts, reason=f'pick_auto exited with {code}; not retrying')
    return dict(ok=False, attempts=attempts, reason=f'object not held after {max_attempts} attempts')
