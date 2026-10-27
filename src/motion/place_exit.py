"""놓은 뒤 물러나기(retreat) → 로봇 쪽으로 빠지기(retire) → 손목 세우기(untilt)를 멈추지 않는 한 번의 경로로 실행한다.

사용자 지시(2026-10-26): 세 단계를 합쳐 한 번에. 예전에는 단계·손목 각도마다 따로 이동하고 매번 도착·정지 확인을 해서
놓은 뒤 끝까지 약 28초 걸렸다(1번 칸, 기울여 넣은 경우).

경로 점(place_exec.build_exit_transition이 계획에서 만든 높이·손목 각도)의 자세를 이동 전에 모두 풀고, 점 사이 보간까지
관절 한계·링크/책상 높이·칸 벽을 넘기 전까지 수직 유지·넘은 뒤 높이 유지를 검사한 다음 하나의 고정 틱 경로로 실행한다.
준비에서 거부되면 ExitPathRefused(움직이기 전)를 올리고, 호출자는 예전처럼 단계별로 간다. 장치 접근은 실행 함수에서만.
"""
import math

from robot.planar_kinematics import NAMES, finite_vector
from motion import guard_mode
from motion.hover_trial import _execute_path, _hold_offset, HOVER_RADIUS_MAX_M, MAX_TILT_RAD

VERTICAL = -math.pi / 2
MAX_PITCH_STEP_RAD = math.radians(15)      # 점 사이 손목 각도 변화(예전 제자리 세우기와 같은 15도)
SLOT_COLUMN_M = .015                       # 놓은 자리에서 이만큼 안쪽이고 벽 높이 아래면 '칸 안'으로 본다
# 놓은 높이 아래로 처져도 되는 양(실측 위치 기준). 2026-10-27 Pi: 1번 칸 60도(컵 35.7cm, 팔을 거의 다 뻗음)에서 위로 빠지기 시작하면
# 어깨가 7펄스·손목이 4펄스 늦게 따라와 계획은 +1.4mm 올라가는데 컵은 2.3mm 처져 2mm 검사에 두 번 걸렸다(그 뒤 HOME 경유).
# 물체는 놓는 높이를 칸 바닥 20mm 위로 잡아(sort_job.DROP_RAISE_MM) 놓은 뒤 떨어지므로 컵 아래 약 20mm가 비어 있다. 칸 안 수직·기울기 검사는 그대로다
RELEASE_DIP_M = .006
# 칸 위(clear_z)는 물체를 든 운반 높이라 놓은 뒤의 빈 컵이 피해야 할 트레이 벽(약 48mm)·칸 안 물체(최대 약 50mm)보다 50mm 넘게 높다.
# 실제 컵은 명령보다 늦게 올라와(2026-10-27 Pi: 60도에서 13~20mm, 15도 끝에서 8mm 처짐) 운반 높이 바로 아래에서 옆으로 움직이거나
# 처지면 2mm·5mm 기준에 걸려 멈췄다. 칸 기둥(수직·기울기 유지) 검사는 운반 높이보다 15mm 아래까지, 빠지는 중 높이는 20mm 아래까지 본다
COLUMN_TOP_BELOW_M = .015
EXIT_DROP_BELOW_M = .020


class ExitPathRefused(ValueError):
    """Preparation failed before the motion executor was called."""


def run_place_exit_trial(driver, geometry, zeros, transition, *, supported=False, rate=150.,
                         min_contact_z=.010, tracking_limit=32, record=lambda event: None, cancelled=lambda: False):
    """transition: dict(start, start_pitch, clear_z, target, waypoints=[dict(xyz, pitch_rad, pulses)]).
    clear_z 아래(칸 안)에서는 놓은 자리 바로 위로 수직, 기울기 유지. 위로 올라온 뒤에는 clear_z 아래로 내려가지 않는다."""
    if not supported or driver.state != 'hold_only' or not driver.hold_confirmed:
        raise ExitPathRefused('place exit requires supported confirmed hold')
    if (not math.isfinite(rate) or not 13 <= rate <= 150
            or not math.isfinite(min_contact_z) or not .005 <= min_contact_z <= .030
            or tuple(driver.names) != NAMES or set(zeros) != set(NAMES)):
        raise ExitPathRefused('invalid place exit rate, floor or joint mapping')
    joints = {j.name: j for j in driver.profile.joints}
    pulse = 2 * math.pi / 4096
    limits = {n: tuple(sorted(joints[n].direction * (r - zeros[n]) * pulse
                        for r in (joints[n].raw_min + 16, joints[n].raw_max - 16))) for n in NAMES}

    def angles(raw):
        return tuple(joints[n].direction * (raw[n] - zeros[n]) * pulse for n in NAMES)

    with driver.lock:
        snapshot = driver._snapshot(torque=1)
        origin = {n: snapshot['joints'][n]['position'] for n in NAMES}
        initial = geometry.forward(angles(origin))
        start = finite_vector(transition['start'], 3, 'place exit start')
        start_pitch = float(transition['start_pitch'])
        clear_z = float(transition['clear_z'])
        target = finite_vector(transition['target'], 3, 'place exit target')
        if not math.isfinite(clear_z) or not start[2] < clear_z <= .150:
            raise ExitPathRefused('invalid place exit clearance height')
        if math.dist(initial[:3], start) > guard_mode.length(.005) or abs(initial[3] - start_pitch) > math.radians(3) * guard_mode.scale():
            raise ExitPathRefused('place exit start pose changed')
        def clear(raw):
            # 상태 없이 그 자세만으로 판단한다(이동 전 계산·실행 전 경로 검사·실행 중 매 틱에 같은 기준으로 불린다)
            if any(not joints[n].raw_min + 16 <= raw[n] <= joints[n].raw_max - 16 for n in NAMES):
                raise ValueError('place exit controller margin')
            q = angles(raw)
            links = geometry.link_points(q)
            xyz_pitch = geometry.forward(q)
            if min(p[2] for p in links) < min_contact_z or max(p[2] for p in links) > .450:
                raise ValueError('place exit link/table clearance')
            if xyz_pitch[2] < start[2] - RELEASE_DIP_M:
                raise ValueError('place exit dipped below the release point')
            if xyz_pitch[2] < clear_z - COLUMN_TOP_BELOW_M:
                away = math.hypot(xyz_pitch[0] - start[0], xyz_pitch[1] - start[1])
                if away <= SLOT_COLUMN_M:
                    # 칸 벽 아래(놓은 자리 기둥 안): 바로 위로, 넣을 때의 기울기 그대로 올라가야 벽에 닿지 않는다
                    if away > guard_mode.length(.003):
                        raise ValueError('place exit must rise straight out of the slot')
                    if abs(xyz_pitch[3] - start_pitch) > math.radians(3) * guard_mode.scale():
                        raise ValueError('place exit tool pitch changed inside the slot')
                elif xyz_pitch[2] < clear_z - guard_mode.length(EXIT_DROP_BELOW_M):
                    raise ValueError('place exit dropped below the tray clearance height')     # 빠지는 중에는 벽 높이 위
            return links[-1]

        try:
            clear(origin)
            if not guard_mode.relaxed() and any(abs(snapshot['joints'][n]['goal'] - origin[n] + _hold_offset(driver, n)) > 2 for n in NAMES):
                raise ValueError('place exit current hold not settled')
            goals, seg_pulses, previous, previous_raw, steps = [], [], angles(origin), origin, 0
            for point in transition['waypoints']:
                xyz = finite_vector(point['xyz'], 3, 'place exit waypoint')
                pitch = float(point['pitch_rad'])
                if (not min_contact_z <= xyz[2] <= .150 or not .1 <= math.hypot(*xyz[:2]) <= HOVER_RADIUS_MAX_M
                        or not VERTICAL - 1e-9 <= pitch <= VERTICAL + MAX_TILT_RAD + 1e-9 or point.get('pulses') not in (2, 4)):
                    raise ValueError('invalid place exit waypoint')
                options = geometry.solve(xyz, pitch, limits)
                if not options:
                    raise ValueError('place exit has no direct-branch solution')
                q = min(options, key=lambda q: sum((a - b) ** 2 for a, b in zip(q, previous)))
                raw = {n: round(zeros[n] + q[i] / (joints[n].direction * pulse)) for i, n in enumerate(NAMES)}
                span = max(abs(raw[n] - previous_raw[n]) for n in NAMES)
                steps += span
                if steps > guard_mode.pulses(1600):
                    raise ValueError('place exit exceeds trial envelope')
                for index in range(span + 1):
                    clear({n: round(previous_raw[n] + (raw[n] - previous_raw[n]) * index / max(1, span)) for n in NAMES})
                goals.append(raw)
                seg_pulses.append(int(point['pulses']))
                previous, previous_raw = q, raw
            if not goals:
                raise ValueError('empty place exit path')
        except ValueError as exc:
            raise ExitPathRefused(str(exc)) from exc
        record(dict(phase='place_exit_preflight', points=len(goals), steps=steps, target=target,
                    start_pitch=start_pitch, clear_z=clear_z, intermediate_holds=0, physically_validated=False))
        # 칸 안(2펄스/틱 = rate)과 벽 위(4펄스/틱 = 2×rate)를 한 경로로. 서보 속도 레지스터는 _execute_path가 경로 속도에 맞춘다
        return _execute_path(driver, geometry, joints, angles, clear, origin, goals[-1], target, record, cancelled,
                             pulses_per_step=2, period_s=2 / rate, tracking_limit=tracking_limit,
                             waypoints=goals, seg_pulses=seg_pulses)      # 놓은 뒤 빈 컵: 앞당기기 없음(넣은 뒤 끝에서 넘쳐 흔들렸다)
