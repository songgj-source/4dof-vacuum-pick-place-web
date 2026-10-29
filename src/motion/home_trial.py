"""Reverse a recorded supported HOME pose with the experimental path guards.

This does not use example ROS home angles or validate general execution.
"""
import math
from robot.planar_kinematics import NAMES,finite_vector
from motion.hover_trial import _hold_offset,_execute_path,SLOW_JOINT_BATCH_PULSES,TRAVEL_GOAL_SPEED,TRAVEL_PERIOD_S,SLOW_TRAVEL_PERIOD_S
from motion.home_return_profile import continuous_home_waypoints
from motion.tick_pulses import ALLOWED_STEPS

SETTLE_TRIES=10;SETTLE_WAIT_S=.1      # HOME 시작 전 정지 확인: 0.1초씩 최대 10번


def run_recorded_home_trial(driver,geometry,zeros,home_raw,*,supported=False,
                            record=lambda event:None,cancelled=lambda:False,pulses_per_step=1,batch_slow_joints=False,fast_travel=False,tick_s=None,tracking_limit=16,native=False,smooth_motion=False,continuous=False):
    if type(pulses_per_step) is not int or pulses_per_step not in ALLOWED_STEPS:
        raise ValueError('pulse step must be 1, 2, 4, 5, 6, 7 or 8')
    if not supported or driver.state!='hold_only' or not driver.hold_confirmed:
        raise ValueError('supported confirmed hold required for HOME trial')
    if type(continuous) is not bool or continuous and (not fast_travel or native or batch_slow_joints):
        raise ValueError('continuous HOME requires fixed-rate synchronous travel')
    if set(zeros)!=set(NAMES) or set(home_raw)!=set(NAMES) or tuple(driver.names)!=NAMES:
        raise ValueError('four original joints required')
    zeros={n:finite_vector((zeros[n],),1,'zero')[0] for n in NAMES}
    joints={j.name:j for j in driver.profile.joints};pulse=2*math.pi/4096
    def angles(raw):return tuple(joints[n].direction*(raw[n]-zeros[n])*pulse for n in NAMES)
    def clear(raw):
        points=geometry.link_points(angles(raw))
        if min(p[2] for p in points)<.060 or max(p[2] for p in points)>.450:
            raise ValueError('HOME sampled table clearance failed')
        return points[-1]
    with driver.lock:
        # 앞 단계 직후에는 몇 펄스 흔들리다 선다(2026-10-26 Pi: 바로 검사해 'HOME current hold not settled'로 멈춤). 최대 1초 기다려 선 뒤에 본다
        for _ in range(SETTLE_TRIES):
            snapshot=driver._snapshot(torque=1);origin={n:snapshot['joints'][n]['position'] for n in NAMES}
            if all(abs(snapshot['joints'][n]['goal']-origin[n]+_hold_offset(driver,n))<=2 for n in NAMES):break
            driver.sleep(SETTLE_WAIT_S)
        if geometry.forward(angles(origin))[2]<.080:
            raise ValueError('raise cup above 80mm before folding HOME')
        for n in NAMES:
            j=joints[n]
            if (type(home_raw[n]) is not int or not j.raw_min+16<=home_raw[n]<=j.raw_max-16
                    or not j.raw_min+16<=origin[n]<=j.raw_max-16):
                raise ValueError('HOME controller margin/range invalid')
            if abs(snapshot['joints'][n]['goal']-origin[n]+_hold_offset(driver,n))>2:
                raise ValueError('HOME current hold not settled')
        goal=dict(home_raw);target=geometry.forward(angles(goal))[:3]
        waypoints=continuous_home_waypoints(origin,goal) if continuous else None
        return _execute_path(driver,geometry,joints,angles,clear,origin,goal,target,record,cancelled,pulses_per_step,SLOW_JOINT_BATCH_PULSES if batch_slow_joints else 0,
                             TRAVEL_GOAL_SPEED if fast_travel else 40,(tick_s or TRAVEL_PERIOD_S) if fast_travel else SLOW_TRAVEL_PERIOD_S if smooth_motion else None,tracking_limit,
                             waypoints=waypoints,seg_pulses=[pulses_per_step]*len(waypoints) if waypoints else None,
                             native=native and fast_travel,continuous_segments=bool(waypoints))
