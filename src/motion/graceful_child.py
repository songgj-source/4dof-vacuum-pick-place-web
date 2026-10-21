"""정지 신호를 안쪽 프로세스(pick_auto)에 그대로 전달하고, 안쪽이 정리를 끝낼 때까지 기다린다. 하드웨어 접근 없음.

subprocess.run은 KeyboardInterrupt를 받으면 0.25초 뒤 안쪽 프로세스를 SIGKILL한다. pick_auto의 정리(임시 게인 복구, 팔 고정, 펌프 닫기)는
그보다 오래 걸려서, API의 /stop이나 "멈춰"로 끊으면 어깨·팔꿈치 게인이 임시값(P32, 어깨 I1)으로 남았고 HOME 복귀가
'gain baseline changed'로 막혔다(2026-10-21 실물).
"""
import signal
import subprocess


def run_until_clean_exit(command, cwd):
    """command를 실행해 종료 코드를 돌려준다. SIGINT/SIGTERM을 받으면 안쪽에 전달하고, 안쪽이 스스로 끝날 때까지 기다린 뒤
    KeyboardInterrupt를 올린다(바깥 프로세스도 예전처럼 멈추고, 재시도 루프가 정지 뒤에 다시 움직이지 않는다)."""
    proc = subprocess.Popen(command, cwd=cwd)
    received = []

    def forward(signum, _frame):
        received.append(signum)
        try:
            proc.send_signal(signum)
        except ProcessLookupError:
            pass

    previous = {sig: signal.signal(sig, forward) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        code = proc.wait()
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    if received:
        raise KeyboardInterrupt
    return code
