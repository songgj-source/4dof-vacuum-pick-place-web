"""Explicit experimental high-hover trial, separate from normal ROS execution.

Uses candidate geometry for a conservative sampled table-clearance check.
This is not collision certification. Low vertical descent requires an explicit
current measured box gap and is limited to gap plus 2mm. It cannot operate
vacuum, change calibration, or enable the normal trajectory execution gate.
"""
import math
from robot.planar_kinematics import NAMES,finite_vector
from motion import guard_mode
from motion.tick_pulses import ALLOWED_STEPS


def _hold_offset(driver,name):
    """확인된 보상 hold의 (위치-목표) 펄스. 보상이 없거나 hold가 확인되지 않았거나 가짜 드라이버면 0."""
    getter=getattr(driver,'hold_offset',None)
    value=getter(name) if callable(getter) else 0
    return value if type(value) is int else 0


# A joint that barely moves during a long travel would creep one pulse every
# dozen ticks and stick-slip across its deadband (the visible shaking). Such
# joints are commanded in bursts of this many pulses instead; endpoints stay exact.
# Opt-in (batch_slow_joints=True); the default keeps the strict per-step pulse contract.
SLOW_JOINT_BATCH_PULSES=6
# Clear-space travel (no contact/lift): faster servo speed register and a fixed command rate.
TRAVEL_GOAL_SPEED=600
PARKED_ERROR_PULSES=6
ENDPOINT_IDLE_CHECKS=3   # 목표에서 떨어져(8~11펄스) 멈춘 채 변화가 없으면 이만큼 읽고 바로 넘어간다. 전엔 8번(약 0.7초)이었고 위치는 2번째 읽기부터 그대로였다(2026-10-30 Pi). 뒤의 5mm 자세 검사와 보상 hold는 그대로
HELD_LEAD_S=.25   # 물체를 든 들기·놓은 뒤 빠져나오기의 앞당김(2026-10-27 Pi: 서보 지연 약 0.3초, 조금 덜 앞당긴다)
TRAVEL_GOAL_ACCELERATION=20
TRAVEL_PULSES_PER_STEP=4
MAX_TILT_RAD=math.radians(60.)   # place only: the tool may tilt forward from straight down by at most this much (slots beyond 0.30 m need about 43 degrees; user instruction 2026-10-15; 50->60 2026-10-21: 1번 칸 36.7cm)
HOVER_RADIUS_MAX_M=.360  # 2026-10-23: user-requested 5 mm increase; 2026-10-27 .355->.360 (user: 1번 칸 40.4cm 정육면체가 컵 35.61~35.72cm 필요). planner and via points share this bound
TRAVEL_PERIOD_S=.020
QUICK_PERIOD_S=.040
# Commanded acceleration (path pulses/s^2). The servo acceleration register limits what it
# can follow; a constant-rate start outruns it and trips the tracking guard.
TRAVEL_ACCEL=300.
# Candidate clear-space profile; physical validation is still required.
APPROACH_PERIOD_S=TRAVEL_PERIOD_S
APPROACH_TRACKING_LIMIT=32
SMOOTH_WINDOW_S=.25
# Native servo moves: one goal per leg; the servo's own speed/acceleration profile makes the
# motion (no per-tick goal updates, which excite a ~7 Hz oscillation). Monitoring stays per-tick.
NATIVE_POLL_S=.030
NATIVE_LEAD_INDICES=8
NATIVE_DEVIATION_LIMIT=30
SLOW_TRAVEL_PERIOD_S=.060
# Old contact/lift cycles measured ~73ms including serial I/O. Keep that
# conservative rate while replacing abrupt steps with the ramped profile.
CONTACT_PERIOD_S=.075
# 사용자 요청(2026-10-15): 집은 직후 들어올리기(lift20, raise_clear)가 합쳐서 약 17초라 너무 느림 → 들어올리기 틱만 38ms로 줄여
# 각 구간을 약 3초로 맞춘다(1펄스씩 약 26펄스/초). 접촉 직전 내리기(gap)는 그대로 75ms.
LIFT_MAX_M=.036  # 1회 들어올리기 상한: 30mm였으나 흡착 후 raise_clear를 한 번에 가려고 사용자 승인으로 36mm로 올림(2026-10-16)
LIFT_PERIOD_S=.038


def run_hover_trial(driver,geometry,zeros,target,measured_radius_m,*,supported=False,
                    cancelled=lambda:False,record=lambda event:None,measured_gap_m=None,lift_m=None,pulses_per_step=1,box_top_m=None,
                    batch_slow_joints=False,fast_travel=False,quick_ticks=False,tick_s=None,tracking_limit=16,
                    via=(),via_pulses=2,tail=None,native=False,smooth_motion=False,pitch_rad=-math.pi/2,min_contact_z=.030,
                    continuous_lift=False):
    target=finite_vector(target,3,'hover target')
    # Contact/lift height floor: default 30 mm for other callers; pick_auto passes a lower one (user instruction 2026-10-15: remove the 30 mm floor,
    # keep only a minimum that stops the cup reaching the table) so low objects such as the white box (top 24 mm) can be picked.
    if isinstance(min_contact_z,bool) or not isinstance(min_contact_z,(int,float)) or not math.isfinite(min_contact_z) or not .005<=min_contact_z<=.030:
        raise ValueError('contact floor must be 5-30 mm')
    # Tool pitch: default straight down. place may tilt the tool forward (up to MAX_TILT_RAD) to reach more height.
    if isinstance(pitch_rad,bool) or not isinstance(pitch_rad,(int,float)) or not math.isfinite(pitch_rad) or not -math.pi/2-1e-9<=pitch_rad<=-math.pi/2+MAX_TILT_RAD+1e-9:
        raise ValueError('tool pitch must be finite, straight down or tilted forward by at most %.0f degrees'%math.degrees(MAX_TILT_RAD))
    # A known plane permits 70mm hover with at least30mm nominal top clearance.
    measured=finite_vector((measured_radius_m,),1,'measured radius')[0]
    if not supported or driver.state!='hold_only' or not driver.hold_confirmed:
        raise ValueError('explicit supported, confirmed existing hold required')
    gap=None if measured_gap_m is None else finite_vector((measured_gap_m,),1,'measured gap')[0]
    if gap is not None and not .005<=gap<=.080:
        raise ValueError('measured descent gap outside trial range')
    lift=None if lift_m is None else finite_vector((lift_m,),1,'lift')[0]
    if type(continuous_lift) is not bool:
        raise ValueError('continuous lift flag must be boolean')
    if lift is not None and (gap is not None or not .005<=lift<=guard_mode.length(2*LIFT_MAX_M if continuous_lift else LIFT_MAX_M)):
        raise ValueError('small lift only, without a descent gap')
    if continuous_lift and (lift is None or len(via)!=1 or box_top_m is not None or fast_travel or native
                            or batch_slow_joints or tail is not None or not quick_ticks or pulses_per_step<2 or pulses_per_step not in ALLOWED_STEPS
                            or via_pulses!=pulses_per_step or tick_s is None or target[2]>.080):
        raise ValueError('continuous lift needs two bounded low vertical segments at the held rate')
    if type(pulses_per_step) is not int or pulses_per_step not in ALLOWED_STEPS:
        raise ValueError('pulse step must be 1, 2, 4, 5, 6, 7 or 8')
    # 접촉 주변과 상승에서는 틱당 최대 8펄스만 허용한다(2026-10-28 사용자 지시: 저속 동작 300·500펄스/초 = 4·7펄스 x 13.3~14ms 틱). 호출자가 속도를 설정한다.
    if pulses_per_step not in ALLOWED_STEPS and (gap is not None or lift is not None):
        raise ValueError('fast mode is forbidden near contact or during lift')
    box_top=None if box_top_m is None else finite_vector((box_top_m,),1,'box top')[0]
    if via and not continuous_lift and (box_top is None or not (fast_travel or quick_ticks) or via_pulses not in ALLOWED_STEPS or gap is not None or lift is not None):
        raise ValueError('via points need a box plane, the fixed-rate profile and no gap/lift')
    if tail is not None and (not (fast_travel or quick_ticks) or len(tail)!=2 or tail[0]<1 or not 0<tail[1]<=40):
        raise ValueError('invalid slow tail')
    if box_top is not None and (gap is not None or lift is not None or not .030<=box_top<=.100):
        raise ValueError('explicit box plane requires a separate hover and valid height')
    if box_top is not None and target[2]<.080 and pulses_per_step!=1 and not fast_travel:
        raise ValueError('fast mode forbidden for low box hover')
    minimum_height=max(.070,box_top+.030) if box_top is not None else min_contact_z if gap is not None or lift is not None else .080
    if not minimum_height<=target[2]<=.250 or not .1<=math.hypot(*target[:2])<=HOVER_RADIUS_MAX_M:
        raise ValueError('measured descent target outside safety envelope' if gap is not None else 'high hover only; contact/low target prohibited')
    if abs(math.hypot(*target[:2])-measured)>guard_mode.length(.030):
        raise ValueError('camera target and independent radial measurement disagree')
    if set(zeros)!=set(NAMES) or tuple(driver.names)!=NAMES:
        raise ValueError('four explicit zeros and original joint order required')
    zeros={n:finite_vector((zeros[n],),1,'zero')[0] for n in NAMES}
    pulse=2*math.pi/4096
    joints={j.name:j for j in driver.profile.joints}
    def angles(raw):return tuple(joints[n].direction*(raw[n]-zeros[n])*pulse for n in NAMES)
    def clear(raw):
        points=geometry.link_points(angles(raw))
        if min(p[2] for p in points)<(min_contact_z if gap is not None or lift is not None else .060) or max(p[2] for p in points)>.450:
            raise ValueError('sampled link endpoint/table clearance failed')
        if box_top is not None and points[-1][2]<box_top+.025:
            raise ValueError('known box hover tip clearance failed')
        return points[-1]
    with driver.lock:
        snapshot=driver._snapshot(torque=1)
        origin={n:snapshot['joints'][n]['position'] for n in NAMES}
        if lift is not None:
            initial_xyz=geometry.forward(angles(origin))[:3]
            if (abs(target[2]-initial_xyz[2]-lift)>.002
                    or math.hypot(initial_xyz[0]-target[0],initial_xyz[1]-target[1])>.003):
                raise ValueError('small lift must be vertical and match declared distance')
            if continuous_lift:
                midpoint=finite_vector(via[0],3,'continuous lift midpoint')
                if (math.hypot(midpoint[0]-initial_xyz[0],midpoint[1]-initial_xyz[1])>guard_mode.length(.003)
                        or not .005<=midpoint[2]-initial_xyz[2]<=guard_mode.length(LIFT_MAX_M)
                        or not .005<=target[2]-midpoint[2]<=guard_mode.length(LIFT_MAX_M)):
                    raise ValueError('continuous lift segment must be vertical and 5-36 mm')
        if gap is not None:
            initial_xyz=geometry.forward(angles(origin))[:3]
            drop=initial_xyz[2]-target[2]
            if (not 0<drop<=min(guard_mode.length(.055),gap+guard_mode.length(.002))+1e-9
                    or math.hypot(initial_xyz[0]-target[0],initial_xyz[1]-target[1])>guard_mode.length(.003)):
                raise ValueError('measured descent must be vertical and within gap plus 2mm')
        for n in NAMES:
            if not joints[n].raw_min+16<=origin[n]<=joints[n].raw_max-16:
                raise ValueError('current endpoint margin insufficient')
            if abs(snapshot['joints'][n]['goal']-origin[n]+_hold_offset(driver,n))>2 and not guard_mode.relaxed():
                raise ValueError('current hold is not settled')
        limits={n:tuple(sorted(joints[n].direction*(r-zeros[n])*pulse for r in (joints[n].raw_min+16,joints[n].raw_max-16))) for n in NAMES}
        q_prev=angles(origin);goals=[]
        for point in [finite_vector(v,3,'via point') for v in via]+[target]:
            if not minimum_height<=point[2]<=.250 or not .1<=math.hypot(*point[:2])<=HOVER_RADIUS_MAX_M:
                raise ValueError('via point outside safety envelope')
            options=geometry.solve(point,pitch_rad,limits)
            if not options:raise ValueError('hover has no direct-branch solution')
            q=min(options,key=lambda q:sum((a-b)**2 for a,b in zip(q,q_prev)));q_prev=q
            goals.append({n:round(zeros[n]+q[i]/(joints[n].direction*pulse)) for i,n in enumerate(NAMES)})
        goal=goals[-1]
        # Travel only: near contact/lift every joint must follow the vertical line closely.
        batch=SLOW_JOINT_BATCH_PULSES if batch_slow_joints and gap is None and lift is None else 0
        travel=fast_travel and gap is None and lift is None
        travel_ok=(fast_travel or quick_ticks) and gap is None and lift is None
        period=(tick_s or (TRAVEL_PERIOD_S if travel else QUICK_PERIOD_S)) if travel or quick_ticks else None
        if smooth_motion and period is None:
            period=CONTACT_PERIOD_S if gap is not None else LIFT_PERIOD_S if lift is not None else SLOW_TRAVEL_PERIOD_S
        return _execute_path(driver,geometry,joints,angles,clear,origin,goal,target,record,cancelled,pulses_per_step,batch,
                             TRAVEL_GOAL_SPEED if travel else 40,period,tracking_limit,
                             goals if via else None,[via_pulses]*len(via)+[pulses_per_step] if via else None,tail,
                             native and travel_ok,continuous_segments=continuous_lift,catch_up=travel)


def ramped_indices(steps,pulses_per_step,period_s,accel,smooth_s=SMOOTH_WINDOW_S,caps=None):
    """Per-tick path indices that start and end at rest, never exceed a per-index rate cap
    (default pulses_per_step/period_s) and never accelerate faster than ``accel`` (path
    pulses/s^2); the rate is then moving-averaged so acceleration changes gradually."""
    import numpy as np
    vmax=pulses_per_step/period_s
    caps=[vmax]*(steps+1) if caps is None else list(caps)
    w=[0.]*(steps+1)
    for i in range(steps):w[i+1]=min(caps[i+1]**2,w[i]+2*accel)
    w[steps]=0.
    for i in range(steps,0,-1):w[i-1]=min(w[i-1],w[i]+2*accel)
    v=[x**.5 for x in w]
    t=[0.]
    for i in range(steps):t.append(t[-1]+2./max(v[i]+v[i+1],1e-9))
    ticks=int(math.ceil(t[-1]/period_s))+1
    position=np.interp(np.arange(ticks+1)*period_s,t,np.arange(steps+1,dtype=float))
    rates=np.diff(position)/period_s
    width=max(1,int(round(smooth_s/period_s)))
    if width>1 and len(rates)>1:rates=np.convolve(rates,np.ones(width)/width,'full')
    out=[];total=0.
    for rate in rates:
        total=min(float(steps),total+rate*period_s);out.append(max(out[-1] if out else 0,int(round(total))))
    out.append(steps)
    return out


def _build_path(origin,waypoints,names):
    path=[dict(origin)];ends=[];current=dict(origin)
    for goal in waypoints:
        n=max(abs(goal[k]-current[k]) for k in names)
        for i in range(1,n+1):path.append({k:round(current[k]+(goal[k]-current[k])*i/n) for k in names})
        ends.append(len(path)-1);current=dict(goal)
    return path,ends


def _native_motion(driver,joints,clear,path,seg_ends,goals,seg_rates,record,cancelled,deviation_limit,poll_s):
    """Big joints: one goal per leg with scaled speed/acceleration, so the servo's own profile
    makes the motion (per-tick goal steps excite a ~7 Hz oscillation). Small joints cannot be
    slowed that way (the servo ignores speeds below about 40), so they follow the checked path
    index. Every poll the real position is read: stay near the checked joint-space path, keep
    clearance, and abort (the caller holds) on any deviation or timeout."""
    NAMES=tuple(driver.names)
    def dist(a,b):return max(abs(a[n]-b[n]) for n in NAMES)
    followed={}
    def command(leg,origin):
        goal=goals[leg];delta={n:abs(goal[n]-origin[n]) for n in NAMES};biggest=max(delta.values()) or 1
        big={n for n in NAMES if delta[n]>=.5*biggest}
        for n in big:
            driver.bus.goal(n,goal[n],max(1,min(TRAVEL_GOAL_SPEED,round(seg_rates[leg]*delta[n]/biggest))),2)
            followed.pop(n,None)
        return big
    leg=0;nearest=0;big=command(0,path[0])
    expected=sum(max(abs(goals[i][n]-(path[0] if i==0 else goals[i-1])[n]) for n in NAMES)/seg_rates[i] for i in range(len(goals)))
    begin=driver.clock();final=goals[-1];period=0
    while True:
        if cancelled():raise InterruptedError('hover cancelled')
        if driver.clock()-begin>expected*2+8:raise TimeoutError('native move took too long')
        feedback=driver._snapshot(torque=1)
        actual={n:feedback['joints'][n]['position'] for n in NAMES}
        # Progress is the path point nearest to the BIG joints (they set the pace); the small
        # joints follow it, so using them too would feed back and run ahead. The deviation check
        # still compares all four joints at that point.
        best=min(range(nearest,len(path)),key=lambda i:max(abs(path[i][n]-actual[n]) for n in big))
        if dist(path[best],actual)>deviation_limit:raise ValueError('hover tracking error')
        nearest=max(nearest,best)
        record({'phase':'moving','index':nearest,'previous_goal':path[nearest],'actual':actual,
                'feedback_goals':{n:feedback['joints'][n]['goal'] for n in NAMES},
                'read_retry_count':getattr(driver.bus,'read_retry_count',0)})
        clear(actual)
        if leg+1<len(goals) and nearest>=seg_ends[leg]-NATIVE_LEAD_INDICES:
            leg+=1;big=command(leg,goals[leg-1])
        # small joints: goal = checked path a little ahead of the measured progress
        ahead=path[min(len(path)-1,nearest+12)]
        for n in NAMES:
            if n not in big and followed.get(n)!=ahead[n]:
                driver.bus.goal(n,ahead[n]);followed[n]=ahead[n]
        if leg==len(goals)-1 and dist(final,actual)<=6:
            for n in NAMES:driver.bus.goal(n,final[n])
            return
        period+=1;driver.sleep(max(0.,begin+period*poll_s-driver.clock()))


def _execute_path(driver,geometry,joints,angles,clear,origin,goal,target,record,cancelled,pulses_per_step=1,batch_pulses=0,
                  speed=40,period_s=None,tracking_limit=16,waypoints=None,seg_pulses=None,tail=None,native=False,continuous_segments=False,
                  catch_up=False,lead_s=0.):
    """Caller holds driver.lock and has admitted the explicit experiment."""
    NAMES=tuple(driver.names)
    if native and period_s is None:raise ValueError('native move needs the travel profile')
    if waypoints is None:
        steps=max(abs(goal[n]-origin[n]) for n in NAMES)
        path=[{n:round(origin[n]+(goal[n]-origin[n])*i/max(1,steps)) for n in NAMES} for i in range(steps+1)]
    else:
        if period_s is None:raise ValueError('via points need the fixed-rate profile')
        path,seg_ends=_build_path(origin,waypoints,NAMES);steps=len(path)-1
    if native and waypoints is None:
        waypoints=[goal];seg_pulses=[pulses_per_step];path,seg_ends=_build_path(origin,waypoints,NAMES)
    if continuous_segments:
        # HOME 또는 검증된 수직 상승의 두 구간을 하나의 스트림으로 잇는다.
        # 각 구간 1600펄스, 전체 3200펄스 이내이며 전 경로를 사전·실시간 검사한다.
        if (native or period_s is None or batch_pulses or waypoints is None
                or len(waypoints)!=2 or waypoints[-1]!=goal or seg_pulses!=[pulses_per_step]*2
                or steps>guard_mode.pulses(3200) or any(not 0<end-begin<=guard_mode.pulses(1600) for begin,end in zip([0]+seg_ends,seg_ends))):
            raise ValueError('invalid continuous HOME segment envelope')
    elif steps>guard_mode.pulses(1600):raise ValueError('hover change exceeds trial envelope')
    slow={n for n in NAMES if batch_pulses and abs(goal[n]-origin[n])*4<steps}
    for raw in path:
        if any(not joints[n].raw_min+16<=raw[n]<=joints[n].raw_max-16 for n in NAMES):
            raise ValueError('path outside controller margin')
        clear(raw)
    record({'phase':'prepared','origin':origin,'goal':goal,'steps':steps,'target':target,'pulses_per_step':pulses_per_step,
            'continuous_segments':continuous_segments})
    if period_s is not None and speed==40 and not native:
        # 고정 틱 느린 이동(하강·들기·칸 안 내리기·트레이 위 올리기): 서보 속도 레지스터 40은 관절을 초당 약 56~60펄스에
        # 묶어 경로(150펄스/초)를 못 따라가고 관절마다 뒤처짐이 달라 수직 경로가 휘었다(2026-10-26 Pi 실측, 뒤처짐 최대 93펄스).
        # 레지스터를 경로 속도(틱당 펄스/틱 시간)에 맞춘다. 경로 자체(가속 제한·틱)는 그대로다
        speed=max(40,int(math.ceil((max(seg_pulses) if seg_pulses else pulses_per_step)/period_s)))
    last=dict(origin);start=driver.clock();next_tick=None;streaming=False;light=False
    # 보상된 hold는 목표를 위치보다 offset만큼 덜 써 둔다: 아직 안 쓴 관절의 목표 읽기 기준은 origin-offset.
    goal_read={n:origin[n]-_hold_offset(driver,n) for n in NAMES}
    stream_capable=all(callable(getattr(driver.bus,n,None)) for n in
                       ("begin_position_stream","sync_goal_positions","end_position_stream"))
    def end_stream():
        nonlocal streaming,light
        if streaming or light:
            driver.bus.end_position_stream();streaming=False;light=False
    initial_retries=getattr(driver.bus,'read_retry_count',0)
    try:
        if native and steps:
            _native_motion(driver,joints,clear,path,seg_ends,waypoints,[p/period_s for p in seg_pulses],record,cancelled,
                           guard_mode.pulses(max(tracking_limit,NATIVE_DEVIATION_LIMIT)),NATIVE_POLL_S)
            indices=[]
        elif period_s is not None and steps:
            caps=None
            if waypoints is not None or tail is not None:
                rate=pulses_per_step/period_s;caps=[rate]*(steps+1)
                if waypoints is not None:
                    begin=0
                    for end,pulses in zip(seg_ends,seg_pulses):
                        for i in range(begin,end+1):caps[i]=pulses/period_s
                        begin=end+1
                if tail is not None:
                    for i in range(max(0,steps-tail[0]),steps+1):caps[i]=min(caps[i],tail[1])
            indices=ramped_indices(steps,max(seg_pulses) if seg_pulses else pulses_per_step,period_s,TRAVEL_ACCEL if speed==TRAVEL_GOAL_SPEED else 100.,caps=caps)
        else:
            indices=list(range(pulses_per_step,steps+1,pulses_per_step))
            if steps and (not indices or indices[-1]!=steps):indices.append(steps)
        if indices and speed==TRAVEL_GOAL_SPEED and not native and stream_capable:
            driver.bus.begin_position_stream(origin,speed=speed,acceleration=TRAVEL_GOAL_ACCELERATION)
            streaming=True
        elif indices and period_s is not None and not native and callable(getattr(driver.bus,'begin_light_reads',None)):
            # 느린 고정 틱 이동: 틱마다 읽기를 가볍게 해서 틱이 밀리지 않게 한다(쓰기·속도 설정은 그대로)
            driver.bus.begin_light_reads();light=True
        # 앞당겨 명령하기(lead_s): 서보는 목표를 약 0.3초 늦게 따라가고(속도에 비례) 늦는 정도가 관절마다 달라 컵이 경로에서 벗어난다
        # (2026-10-27 Pi: 정육면체를 든 들기에서 어깨 36·손목 50펄스 → 옆 4.6mm). 그만큼 앞 틱의 경로 점을 보내 실제 위치가 경로 위에 있게 한다.
        # 검사(clear·추종 오차)는 그대로 실제 위치로 한다. 경로의 모든 점은 이미 검사했다
        lead=max(0,int(round(lead_s/period_s))) if lead_s and period_s else 0
        for tick,index in enumerate(indices):
            if next_tick is not None:driver.sleep(max(0.,next_tick-driver.clock()))
            tick_started=driver.clock()
            raw=path[indices[min(tick+lead,len(indices)-1)]]
            if cancelled():raise InterruptedError('hover cancelled')
            if driver.clock()-start>180:raise TimeoutError('hover duration exceeded')
            feedback=driver._snapshot(torque=1,fault_on_error=not streaming)
            actual={n:feedback['joints'][n]['position'] for n in NAMES}
            record({'phase':'moving','index':index,'previous_goal':last,'actual':actual,
                    'feedback_goals':{n:feedback['joints'][n]['goal'] for n in NAMES},
                    'read_retry_count':getattr(driver.bus,'read_retry_count',0)})
            if getattr(driver.bus,'read_retry_count',0)-initial_retries>4:
                raise ValueError('repeated read failures during hover')
            if any(abs(feedback['joints'][n]['goal']-goal_read[n])>2 for n in NAMES):
                raise ValueError('unexpected hover goal readback')
            # Match the original trajectory tracking envelope; endpoint remains 2.
            if any(abs(actual[n]-last[n])>(guard_mode.RELAXED_TRACKING_PULSES if guard_mode.relaxed() else guard_mode.pulses(tracking_limit)) for n in NAMES):
                raise ValueError('hover tracking error')
            clear(actual)
            if index!=indices[-1]:
                raw={n:(last[n] if n in slow and abs(raw[n]-last[n])<batch_pulses else raw[n]) for n in NAMES}
            if streaming:
                if raw!=last:driver.bus.sync_goal_positions(raw);goal_read=dict(raw)
            else:
                for n in NAMES:
                    if raw[n]!=last[n]:
                        if speed==40:driver.bus.goal(n,raw[n])
                        else:driver.bus.goal(n,raw[n],speed)
                        goal_read[n]=raw[n]
            last=dict(raw)
            if period_s is None:driver.sleep(.060) # 1/2/4 pulses per60ms; native speed register unchanged
            else:
                finished=driver.clock()
                deadline=tick_started+period_s
                # 접근·운반(catch_up)은 통신이 틱을 넘기면 바로 다음 틱을 보낸다. 예전처럼 한 틱을 더 쉬면 Pi에서 서보 읽기·쓰기(약 13~14ms)가
                # 틱보다 길 때 틱이 약 27ms로 늘어 설정 속도의 절반이 됐다(2026-10-27 사용자: 300→400). HOME 복귀(home_trial)와 느린 이동은
                # 그대로 쉰다: HOME 300은 어깨 추종이 17~29로 32 가드에 가까워 더 빨라지면 멈출 수 있다
                next_tick=deadline if finished<=deadline else finished if streaming and catch_up else finished+period_s
        stable=0;previous=None;idle=0
        for _ in range(40):
            if cancelled():raise InterruptedError('hover cancelled')
            feedback=driver._snapshot(torque=1,fault_on_error=not streaming)
            actual={n:feedback['joints'][n]['position'] for n in NAMES}
            record({'phase':'endpoint','goal':goal,'actual':actual})
            clear(actual)
            settled=all(abs(actual[n]-goal[n])<=guard_mode.pulses(2) for n in NAMES)
            still=previous is not None and all(abs(actual[n]-previous[n])<=1 for n in NAMES)
            stable=stable+1 if settled and still else 0
            # Parked off the 2-pulse contract (stiction): waiting longer will not change it.
            idle=idle+1 if still and not settled else 0
            # 목표에서 확실히 떨어져(PARKED_ERROR_PULSES 이상) 멈춘 경우만 짧게 기다린다. 가까운 꼬리(서보가 천천히 수렴하는 중)는 예전처럼 길게 본다
            parked=max(abs(actual[n]-goal[n]) for n in NAMES)>=guard_mode.pulses(PARKED_ERROR_PULSES)
            if idle>=guard_mode.pulses(ENDPOINT_IDLE_CHECKS if parked else 8):raise ValueError('hover endpoint not reached')
            if stable>=3:
                end_stream()
                driver.hold() # retain position, do not switch torque
                return {'reached':True,'requested_raw':goal,'actual_raw':actual,
                        'candidate_fk':geometry.forward(angles(actual)),
                        'pump_writes':False,'torque_writes':False,'normal_execution_enabled':False}
            previous=actual;driver.sleep(.05)
        raise ValueError('hover endpoint not reached')
    except BaseException as exc:
        end_stream()
        if driver.state!='fault':driver._fault(exc)
        record({'phase':'fault','error':str(exc),'hold_confirmed':driver.hold_confirmed})
        raise

    finally:
        end_stream()
