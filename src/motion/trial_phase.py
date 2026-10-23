"""Experimental pose receipts and release ordering, independent of task verdict.

A candidate FK tolerance never validates the geometry or the production arm.
Only the specific endpoint residual may be assessed; tracking/cancel/alarm
faults always propagate. Physical pickup is a separate observed outcome.
"""
import math
from robot.planar_kinematics import NAMES, finite_vector
from motion.hover_trial import _hold_offset
from motion import guard_mode


def phase_pose_tolerance(name, *, pick_tilt_deg=0.):
    """Keep the existing tilted-pick exception; low gain never relaxes arrival."""
    if pick_tilt_deg and name.startswith(('approach', 'descent', 'lift20', 'lift_to_clear')):return .008
    if guard_mode.relaxed() and not name.startswith(('approach', 'descent')):return guard_mode.RELAXED_POSE_TOLERANCE_M   # 접근·하강은 5mm 보정이 이 검사에 의존한다
    return .005


def approach_feedforward_target(target, radial_m, up_m):
    """접근(P32) 끝점은 매번 같은 쪽으로 모자란다(2026-10-20 15건: 앞쪽 3~4.6mm, 높이 2~4.6mm 낮게).
    처음부터 반경 바깥쪽으로 radial_m, 위로 up_m 더 보낸다. 위로만 밀므로 상자 쪽으로 낮아지지 않는다."""
    target = finite_vector(target, 3, 'approach target')
    if not (0. <= radial_m <= .006 and 0. <= up_m <= .006):
        raise ValueError('approach feed-forward must be 0-6 mm')
    r = math.hypot(target[0], target[1])
    k = (r + radial_m) / r if r > 1e-9 else 1.
    return (target[0] * k, target[1] * k, target[2] + up_m)


def approach_correction_target(target, actual, commanded=None):
    """One measured feed-forward correction, bounded to20mm and never downward.

    commanded: the goal sent last (a previous correction); the new error is added to it.
    The total shift from target also stays within20mm (2026-10-20: up to two corrections)."""
    target = finite_vector(target, 3, 'approach target')
    actual = finite_vector(actual, 3, 'observed approach')
    commanded = target if commanded is None else finite_vector(commanded, 3, 'commanded approach')
    if math.dist(target, actual) > .020:
        raise ValueError('approach error exceeds20mm correction envelope')
    error = [t - a for t, a in zip(target, actual)]
    fix = (commanded[0] + error[0], commanded[1] + error[1], max(target[2], commanded[2] + error[2]))
    if math.dist(fix, target) > .020:
        raise ValueError('approach correction exceeds20mm total envelope')
    return fix


class PoseToleranceError(ValueError):
    def __init__(self, receipt):
        super().__init__('candidate pose outside phase tolerance')
        self.receipt = receipt


def stationary_snapshot(driver,settle_attempts=1):
    """Three consecutive still, on-goal samples. Rewriting the hold goal to the
    current position can let a loaded joint sag a pulse or two once; with
    settle_attempts>1 that transient is waited out, never accepted as stationary."""
    for attempt in range(settle_attempts):
        previous=None;last=None;changed=False
        for _ in range(3):
            last=driver._snapshot(torque=1)
            if any(abs(last['joints'][n]['goal']-last['joints'][n]['position']+_hold_offset(driver,n))>2 for n in NAMES) and not guard_mode.relaxed():
                raise ValueError('stationary hold goal error')
            if previous is not None and any(abs(last['joints'][n]['position']-previous['joints'][n]['position'])>1 for n in NAMES):
                changed=True;break
            previous=last;driver.sleep(.05)
        if not changed:return last
        if attempt+1<settle_attempts:driver.sleep(.3)
    if guard_mode.relaxed():return last
    raise ValueError('stationary hold changed')


def candidate_xyz(driver,geometry,zeros,snapshot):
    joints={j.name:j for j in driver.profile.joints}
    q=[joints[n].direction*(snapshot['joints'][n]['position']-zeros[n])*2*math.pi/4096 for n in NAMES]
    return geometry.forward(q)[:3]


def finish_pose_phase(driver,geometry,zeros,operation,target,tolerance_m=.005,settle_attempts=1):
    raw_met=True;error=None;recovered=False
    try:operation()
    except ValueError as exc:
        if str(exc)!='hover endpoint not reached':raise
        raw_met=False;error=str(exc)
    if not driver.hold_confirmed:
        # 도착 못 한 채 멈춘 직후 하중 받는 어깨가 1~2펄스 더 움직이는 중일 수 있다: 같은 기준으로 움직임이 끝난 뒤 다시 확인한다(2026-10-16).
        recovered=True;driver.sleep(.15);driver.hold(attempts=4,settle_s=.4)
    if not driver.hold_confirmed:raise ValueError('phase physical hold not confirmed')
    snapshot=stationary_snapshot(driver,settle_attempts);actual=candidate_xyz(driver,geometry,zeros,snapshot)
    residual=math.dist(actual,target)
    receipt = {'raw_endpoint_met':raw_met,'raw_error':error,'hold_recovery_attempted':recovered,
            'candidate_pose_in_tolerance':residual<=tolerance_m,'candidate_error_m':residual,'candidate_xyz':actual,
            'requested_xyz':list(target),'pose_tolerance_m':tolerance_m,
            'snapshot':snapshot,'normal_execution_enabled':False,'physical_pick_success':None}
    if residual>tolerance_m:raise PoseToleranceError(receipt)
    return receipt


def release_after_return(driver,geometry,zeros,contact,pump):
    snapshot=stationary_snapshot(driver);actual=candidate_xyz(driver,geometry,zeros,snapshot)
    # Below the reference within the existing30mm model floor permits cup
    # compression; above it by >3mm would release an object above the surface.
    if (actual[2]<.030 or actual[2]>contact[2]+.003
            or math.hypot(actual[0]-contact[0],actual[1]-contact[1])>.005):
        pump.stop();raise ValueError('return pose unsuitable for vent release')
    try:
        pump.vent();driver.sleep(.3)
    finally:pump.stop()
    return {'vent_command_sent':True,'stop_command_sent':True,
            'pressure_verified':False,'candidate_return_xyz':actual}


RELEASE_SETTLE_S=.5


def release_at_slot(driver,geometry,zeros,expected,pump):
    """칸의 놓는 지점에서 해제: 팔이 멈춰 있고 expected(x, y, z)에서 xy 5mm, z 4mm 안일 때만 vent한다."""
    snapshot=stationary_snapshot(driver);actual=candidate_xyz(driver,geometry,zeros,snapshot)
    if math.hypot(actual[0]-expected[0],actual[1]-expected[1])>.005 or abs(actual[2]-expected[2])>.004:
        pump.stop();raise ValueError('release pose is not at the drop point; not venting')
    try:
        pump.vent();driver.sleep(.3)
    finally:pump.stop()
    # 펌프 배기는 300ms(펌웨어)라 끝나는 순간 바로 올라가면 물체가 컵에서 떨어질 여유가 없다: 가만히 둔 채 떨어지길 기다린다(2026-10-19).
    driver.sleep(RELEASE_SETTLE_S)
    return {'vent_command_sent':True,'stop_command_sent':True,'release_settle_s':RELEASE_SETTLE_S,
            'pressure_verified':False,'candidate_release_xyz':actual,'expected_release_xyz':list(expected)}
