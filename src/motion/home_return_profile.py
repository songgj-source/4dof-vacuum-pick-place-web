"""Existing guarded HOME travel profile with explicit requested rate."""
import math

from motion import guard_mode


def return_profile(rate=300.):
    if not math.isfinite(rate) or not 50<=rate<=300:
        raise ValueError('HOME travel rate must be50..300pulse/s')
    # 틱이 길수록(4펄스/26.7ms) 접히는 구간에서 어깨·밑동이 약 8Hz로 떨렸다(2026-10-16 기록: 떨림 rms 1.1~1.7펄스).
    # 속도 150 이하는 2펄스/약 13.3ms 틱으로 같은 속도를 낸다(place 운반과 같은 방식, 접근에서 떨림이 가장 작았던 틱).
    pulses = 2 if rate <= 150 else 4
    return dict(pulses_per_step=pulses,fast_travel=True,tick_s=pulses/rate,
                tracking_limit=16,smooth_motion=True,native=False,
                batch_slow_joints=False)


SHOULDER_FAR_RAW = 3150      # 이보다 크게 뻗은 자세(어깨 raw)에서 출발하면 P32로는 시작 때 어깨가 가드(16펄스)를 넘게 뒤처진다


def choose_pitch_p(shoulder_raw, requested=None):
    """복귀 중 어깨·팔꿈치 P 게인. 접히는 구간 떨림은 P32가 없애지만(0.4~0.6 대 P48 2~3펄스), 멀리 뻗은 자세에서 출발하면
    시작 때 어깨가 17펄스 뒤처져 가드(16)에 걸렸다(2026-10-16 실물). 그래서 멀리 뻗은 출발은 P48을 쓴다."""
    if requested is not None:
        return requested
    return 48 if shoulder_raw > SHOULDER_FAR_RAW else 32


# 한 번의 이동은 1600펄스 이하(hover_trial의 trial envelope). 먼 칸(0.35m)에 놓은 뒤에는 어깨가 HOME에서 1629펄스라 한 번에 못 돌아온다(2026-10-19).
# 한도는 그대로 두고, 이 값 넘게 떨어져 있으면 중간 지점을 한 번 거친다(경로의 모든 점은 기존 링크·책상 높이 검사를 그대로 받는다).
MID_STAGE_FROM_PULSES = 1500


def intermediate_home_target(current, home, *, force=False):
    """current(관절별 raw)가 HOME에서 MID_STAGE_FROM_PULSES보다 멀면(force면 항상), 둘의 중간 raw(정수)를 돌려준다. 아니면 None."""
    if not force and max(abs(current[n] - home[n]) for n in home) <= MID_STAGE_FROM_PULSES:
        return None
    return {n: int(round((current[n] + home[n]) / 2)) for n in home}


def continuous_home_waypoints(current, home):
    """기존 두 구간 복귀 범위 안에서, 정지 없이 통과할 경유점만 만든다."""
    span = max(abs(current[n] - home[n]) for n in home)
    if span > 3200:
        raise ValueError('continuous HOME exceeds two 1600-pulse segments')
    if span <= 1600:
        return None
    mid = {n: int(round((current[n] + home[n]) / 2)) for n in home}
    return [mid, dict(home)]


# HOME 도착 판정. 예전엔 기록된 HOME에서 8펄스를 넘으면 실패였다. 그런데 다음 사이클은 HOME에서 32펄스까지(링크가 책상 위 6cm 이상이면)
# 시작을 허용한다(pick_auto near_home_takeover). 그 안에서 안정적으로 멈췄으면(마지막 hold 확인됨) 복귀는 성공이다(2026-10-19).
HOME_EXACT_PULSES = 8
HOME_ACCEPT_PULSES = 40         # 2026-10-19: 로봇을 올린 뒤 wrist가 31~35펄스 모자라게 서는 일이 반복됐다. 다음 사이클 시작 허용선(48) 안이면 성공
MID_ACCEPT_PULSES = 56          # 중간 경유점(mid 단계)은 지나가는 점이라 더 느슨하다


def home_arrival(observed, target):
    """(등급, 최대 편차). 등급: exact(8 이내) / near(24 이내, 다음 사이클 시작 허용선 32보다 안쪽) / far(실패)."""
    deviation = max(abs(observed[n] - target[n]) for n in target)
    if deviation <= guard_mode.pulses(HOME_EXACT_PULSES):          # 가드 배율(guard_mode)만큼 넓힌다
        return 'exact', deviation
    return ('near' if deviation <= guard_mode.pulses(HOME_ACCEPT_PULSES) else 'far'), deviation


# 접힌 HOME 근처에서 관절이 조금 어긋난 채 멈춘 경우(전원이 끊겼거나 손으로 놓은 자세): 수직 복귀 구간 밖이라 예전엔 "손으로 옮기라"고 했다.
# 모든 관절이 HOME에서 이 값 안이면 접힌 자세를 벗어나지 않고 관절 값으로 HOME에 바로 맞춘다(링크·책상 높이 검사는 이동 중 그대로).
FOLD_NEAR_PULSES = 200


def near_fold(observed, home):
    return max(abs(observed[n] - home[n]) for n in home) <= guard_mode.pulses(FOLD_NEAR_PULSES)
