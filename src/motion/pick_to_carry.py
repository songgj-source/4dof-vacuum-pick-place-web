"""Checked, fixed-rate pickup rise and carry tilt. No I/O until explicit trial."""
import math

from robot.planar_kinematics import NAMES, finite_vector
from motion import guard_mode
from motion.hover_trial import _execute_path, _hold_offset, HOVER_RADIUS_MAX_M, MAX_TILT_RAD, LIFT_MAX_M

VERTICAL = -math.pi / 2
CLEAR_Z = .070


class LiftPathRefused(ValueError):
    """Preparation failed before the motion executor was called."""


def build_lift_transition(plan, start, *, start_pitch=VERTICAL, tilt_start_z=CLEAR_Z):
    """Consume only vertical raise/tilt prefixes; other plans keep their old path."""
    start = finite_vector(start, 3, 'pick-to-carry start')
    clear_z = float(tilt_start_z)
    if not math.isfinite(clear_z) or not CLEAR_Z <= clear_z <= .150:
        return None
    if abs(start_pitch - VERTICAL) > 1e-9 or plan.get('executed') is not False:
        return None
    legs = plan.get('legs') or []
    index = next((i for i, leg in enumerate(legs) if leg['name'] == 'carry'), None)
    if index is None or any(leg['name'] not in ('raise_clear', 'raise_low', 'raise', 'tilt') for leg in legs[:index]):
        return None
    prefix = legs[:index]
    if not prefix or any(math.hypot(leg['target'][0] - start[0], leg['target'][1] - start[1]) > .003 for leg in prefix):
        return None
    height = float(legs[index]['target'][2])
    pitch = float(legs[index].get('pitch_rad', VERTICAL))
    if not .005 <= start[2] < clear_z <= height <= .150 or not VERTICAL <= pitch <= VERTICAL + MAX_TILT_RAD:
        return None
    if height <= clear_z + 1e-9 and pitch != VERTICAL:
        return None
    points = []
    count = math.ceil((clear_z - start[2]) / LIFT_MAX_M)
    for i in range(1, count + 1):
        z = clear_z if i == count else start[2] + (clear_z - start[2]) * i / count
        points.append(dict(xyz=(*start[:2], z), pitch_rad=VERTICAL))
    count = max(math.ceil((height - clear_z) / .010 - 1e-9), math.ceil((pitch - VERTICAL) / math.radians(15) - 1e-9))
    for i in range(1, count + 1):
        points.append(dict(xyz=(*start[:2], height if i == count else clear_z + (height - clear_z) * i / count),
                           pitch_rad=pitch if i == count else VERTICAL + (pitch - VERTICAL) * i / count))
    target = (*start[:2], height)
    remaining = {**plan, 'start': target, 'legs': [dict(leg) for leg in legs[index:]]}
    return dict(start=start, target=target, pitch_rad=pitch, tilt_start_z=clear_z, waypoints=points, remaining_plan=remaining)


def run_pick_to_carry_trial(driver, geometry, zeros, transition, *, supported=False, rate=150.,
                           min_contact_z=.010, tracking_limit=32, record=lambda event: None, cancelled=lambda: False):
    """Solve and sample the whole rise before one ramped waypoint execution."""
    if not supported or driver.state != 'hold_only' or not driver.hold_confirmed:
        raise LiftPathRefused('pick-to-carry requires supported confirmed hold')
    if (not math.isfinite(rate) or not 13 <= rate <= 150
            or not math.isfinite(min_contact_z) or not .005 <= min_contact_z <= .030
            or tuple(driver.names) != NAMES or set(zeros) != set(NAMES)):
        raise LiftPathRefused('invalid pick-to-carry rate, floor or joint mapping')
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
        start = finite_vector(transition['start'], 3, 'transition start')
        target = finite_vector(transition['target'], 3, 'transition target')
        clear_z = float(transition['tilt_start_z'])
        if not math.isfinite(clear_z) or not CLEAR_Z <= clear_z <= target[2]:
            raise LiftPathRefused('invalid tilt clearance height')
        if math.dist(initial[:3], start) > guard_mode.length(.003) or abs(initial[3] - VERTICAL) > math.radians(2) * guard_mode.scale():
            raise LiftPathRefused('pick-to-carry contact pose changed')
        def clear(raw):
            if any(not joints[n].raw_min + 16 <= raw[n] <= joints[n].raw_max - 16 for n in NAMES):
                raise ValueError('pick-to-carry controller margin')
            q = angles(raw)
            links = geometry.link_points(q)
            xyz_pitch = geometry.forward(q)
            if min(p[2] for p in links) < min_contact_z or max(p[2] for p in links) > .450:
                raise ValueError('pick-to-carry link/table clearance')
            # 실제 컵이 집은 자리 바로 위에 있는지(가드 배율 적용). 2026-10-26 Pi: 물체를 든 어깨가 25펄스 뒤처져 3.2mm 벗어나 멈췄다
            if math.hypot(xyz_pitch[0] - start[0], xyz_pitch[1] - start[1]) > guard_mode.length(.003):
                raise ValueError('pick-to-carry must stay above the pickup point')
            if xyz_pitch[2] < start[2] - .002:
                raise ValueError('pick-to-carry dipped below contact')
            if xyz_pitch[2] < clear_z and abs(xyz_pitch[3] - VERTICAL) > math.radians(2) * guard_mode.scale():
                raise ValueError('pick-to-carry tilt below clearance height')
            return links[-1]
        try:
            clear(origin)
            if not guard_mode.relaxed() and any(abs(snapshot['joints'][n]['goal'] - origin[n] + _hold_offset(driver, n)) > 2 for n in NAMES):
                raise ValueError('pick-to-carry current hold not settled')
            goals, previous, steps = [], angles(origin), 0
            previous_raw = origin
            for point in transition['waypoints']:
                xyz = finite_vector(point['xyz'], 3, 'lift waypoint')
                pitch = float(point['pitch_rad'])
                if (not min_contact_z <= xyz[2] <= .150 or not .1 <= math.hypot(*xyz[:2]) <= HOVER_RADIUS_MAX_M
                        or not VERTICAL <= pitch <= VERTICAL + MAX_TILT_RAD
                        or xyz[2] < clear_z and abs(pitch - VERTICAL) > 1e-9):
                    raise ValueError('invalid pick-to-carry waypoint')
                options = geometry.solve(xyz, pitch, limits)
                if not options:
                    raise ValueError('pick-to-carry has no direct-branch solution')
                q = min(options, key=lambda q: sum((a - b) ** 2 for a, b in zip(q, previous)))
                raw = {n: round(zeros[n] + q[i] / (joints[n].direction * pulse)) for i, n in enumerate(NAMES)}
                span = max(abs(raw[n] - previous_raw[n]) for n in NAMES)
                steps += span
                if steps > guard_mode.pulses(1600):
                    raise ValueError('pick-to-carry exceeds trial envelope')
                for index in range(span + 1):
                    clear({n: round(previous_raw[n] + (raw[n] - previous_raw[n]) * index / max(1, span)) for n in NAMES})
                goals.append(raw)
                previous, previous_raw = q, raw
            if not goals:
                raise ValueError('empty pick-to-carry path')
        except ValueError as exc:
            raise LiftPathRefused(str(exc)) from exc
        record(dict(phase='lift_to_carry_preflight', points=len(goals), steps=steps, target=target,
                    pitch_rad=transition['pitch_rad'], intermediate_holds=0, physically_validated=False))
        # 서보 속도 레지스터(이 팔의 설정에서 약 펄스/초)를 경로 속도에 맞춘다. 40이면 관절이 초당 약 40펄스에 묶여
        # 물체를 든 어깨가 경로(53~65펄스/초)를 못 따라가 계속 뒤처지고 컵이 집은 자리에서 옆으로 밀렸다(2026-10-26 Pi 두 번).
        servo_speed = max(40, int(math.ceil(rate)))
        return _execute_path(driver, geometry, joints, angles, clear, origin, goals[-1], target, record, cancelled,
                             pulses_per_step=2, speed=servo_speed, period_s=2 / rate, tracking_limit=tracking_limit,
                             waypoints=goals, seg_pulses=[2] * len(goals))
