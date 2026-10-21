"""User-requested HOME return from the post-depth-pick release pose.

Adapted from output/ik_resume_20261012/return_home_only_repeat.py. Differences:
temporary pitch gains (P48, shoulder I1) are raised BEFORE the existing-torque takeover, as in
scripts/pick_auto.py, because the extended pose sags at baseline gains and the hold cannot settle;
start corridor widened to the pick_auto reset corridor (z .030-.135 m, radius .14-.32 m) so it can also return from a 70 mm hover;
Optional Astra recording; no pump commands. HOME folding uses one ramped synchronous stream,
without an intermediate stop or gain switch. The selected starting P is retained until HOME,
then the original gains are restored. Initial clearance lifts still finish at a confirmed hold.
Each checked HOME segment remains within1600pulses (two segments maximum).
"""
from pathlib import Path
import argparse,signal
from datetime import datetime,timezone
import sys,json,time,math
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from hardware.servo_mapping import MappingProfile
from hardware.servo_transport import ServoTransport
from hardware.servo_driver import ServoDriver
from robot.planar_kinematics import PlanarGeometry
from motion.hover_trial import run_hover_trial
from motion.home_trial import run_recorded_home_trial
from motion.trial_gains import TemporaryPitchGains
from motion.home_return_profile import choose_pitch_p,home_arrival
from motion.home_return_profile import return_profile,near_fold
from motion.return_corridor import start_in_corridor
from optional.video_recording.astra_recorder import AstraRecorder
from robot.robot_frame import SHOULDER_HEIGHT_M
class AlreadyAtHome(Exception):pass
parser=argparse.ArgumentParser(description='Explicit guarded HOME return, default300pulse/s. No pump commands.')
parser.add_argument('--execute',action='store_true');parser.add_argument('--supported',action='store_true')
parser.add_argument('--travel-rate',type=float,default=300.);parser.add_argument('--pitch-p',type=int,choices=(32,48,64),default=None,help='temporary shoulder/elbow P gain during the return (default: automatic, 32 unless the arm starts far extended: at 48 the shoulder hunted at about 8 Hz, rms 2-3 pulses, where the arm folds (shoulder raw 2000-2340); at 32 it measured 0.4-0.6 but a far start lagged 17 pulses over the 16-pulse guard, 2026-10-16)');parser.add_argument('--shoulder-i',type=int,choices=(0,1),default=1,help='temporary shoulder I gain during the return (default 1)');parser.add_argument('--damping',type=int,choices=(32,48,64,96),default=None,help='temporary D gain on shoulder and elbow during the return (default: unchanged 32)');parser.add_argument('--record-astra',action='store_true')
parser.add_argument('--tracking-limit',type=int,choices=range(16,33),metavar='16-32',default=16,help='HOME stage follow-error guard in pulses (default 16, unchanged). The follow lag grows with speed (about 70 ms: 17 pulses at 240 pulses/s, so ~21 at the 300 pulses/s peak, measured 2026-10-19) and trips 16 on the fold; 32 is the project approach default. Not for the 100 approach limit.')
parser.add_argument('--output',type=Path)
parser.add_argument('--any-start',action='store_true',help='skip only the start-corridor check (path/desk/link checks and tracking guards stay); for a stop that left the arm outside the corridor, 2026-10-21')
args=parser.parse_args()
try: travel=return_profile(args.travel_rate)
except ValueError as exc: parser.error(str(exc))
travel=dict(travel,tracking_limit=args.tracking_limit)
if args.execute and not args.supported: parser.error('--execute requires --supported')
if not args.execute:
    print(json.dumps(dict(execute=False,motor_writes=False,pump_writes=False,travel_rate_pulse_s=args.travel_rate,profile=travel,path_checked=False)))
    raise SystemExit(0)
OUT=args.output or ROOT/'output/home_returns'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
OUT.mkdir(parents=True,exist_ok=False)
cancelled=[False]
def cancel(*_):cancelled[0]=True
signal.signal(signal.SIGINT,cancel);signal.signal(signal.SIGTERM,cancel)
recorder=None
result={'home_return_complete':False,'physical_pick_success':None,'normal_execution_enabled':False,'candidate_applied_permanently':False,'phases':[]};driver=None;gains=None
candidate=ROOT/'output/ik_resume_20261012/multipose_review'
fit=json.loads((candidate/'fit_full.json').read_text());review=json.loads((candidate/'review.json').read_text())
profile=MappingProfile.load(ROOT/'config/servo_driver_candidate.json',ROOT)
zeros={j.name:j.zero for j in profile.joints};zeros.update(fit['new_zero_pulses']);g=PlanarGeometry(.027,SHOULDER_HEIGHT_M,tuple(review['eight_pose_geometry_m']))
stopfile=OUT/'stop'
def stopped():return cancelled[0] or stopfile.exists()
def tcp(snapshot):
    return g.forward([-(snapshot['joints'][n]['position']-zeros[n])*2*math.pi/4096 for n in driver.names])
def home_driver(bus,profile):
    """HOME 복귀용: 뻗은 자세에서 어깨가 무게로 10펄스 넘게 처져도(10-21 실물 3719/3709) 멈춰 있으면 인계·보정한다."""
    d=ServoDriver(bus,profile);d.max_pending_pulses=d.max_load_offset=math.inf;return d
with ServoTransport(profile) as bus:
    try:
        if args.record_astra:recorder=AstraRecorder(OUT)
        driver=home_driver(bus,profile)
        first=driver._snapshot(torque=1);driver.sleep(.05);second=driver._snapshot(torque=1)
        for joint in profile.joints:
            n=joint.name;a=first['joints'][n];b=second['joints'][n]
            # 2026-10-21 사용자: HOME 복귀는 목표–위치 차이(하중 처짐) 검사를 뺀다. 움직이는 중인지·관절 한계 근처인지는 그대로 본다.
            if abs(a['position']-b['position'])>1 or a['goal']!=b['goal'] or not joint.raw_min+16<=b['position']<=joint.raw_max-16:
                raise ValueError('pose is not stationary before temporary gains')
        result['pre_gain_stationary_snapshot']=second
        pitch_p=choose_pitch_p(second['joints']['shoulder']['position'],args.pitch_p);result['pitch_p_used']=pitch_p
        gains=TemporaryPitchGains(bus,OUT,target_p={2:pitch_p,3:pitch_p},shoulder_i=args.shoulder_i,target_d=None if args.damping is None else {2:args.damping,3:args.damping})
        with gains:
            driver.sleep(.8)   # 사용자 요청(2026-10-15)으로 2초→0.8초. 바로 뒤에서 정지·목표 일치 확인(hold_existing_torque)을 그대로 한다
            driver.hold_existing_torque(supported=True,attempts=4,settle_s=.4,takeover_attempts=4)  # 하중 관절이 3~6펄스 밀려 서면 같은 기준으로 목표를 보정한다(2026-10-19)
            initial=driver._snapshot(torque=1);contact_xyz=tcp(initial)[:3]
            home_source=json.loads((ROOT/'output/ik_resume_20261012/original_current_hold.json').read_text())
            home_raw={n:home_source['joints'][n]['position'] for n in driver.names}
            # 이미 HOME이면 움직이지 않는다. HOME 자세는 접혀 있어서 수직 복귀 구간(높이 3~13.5cm) 밖이라, 예전엔 HOME에서 누르면 오류가 났다(2026-10-19).
            grade,deviation=home_arrival({n:initial['joints'][n]['position'] for n in driver.names},home_raw)
            if grade in ('exact','near'):
                result.update(start_snapshot=initial,start_candidate_xyz=contact_xyz,travel_rate_pulse_s=args.travel_rate,already_at_home=True,home_deviation_pulses=deviation,
                              home_return_complete=True,home_requested_raw=home_raw,home_actual_snapshot=initial)
                raise AlreadyAtHome(deviation)
            # Same vertical reset corridor as scripts/pick_auto.py (contact up to the 70-120 mm hover over a box).
            folded=near_fold({n:initial['joints'][n]['position'] for n in driver.names},home_raw)
            if folded:result['fold_near_home_start']=True      # 접힌 HOME 근처: 접힌 채 관절 값으로 바로 HOME에 맞춘다(수직 복귀 구간·올리기·중간 지점 생략)
            elif not args.any_start and not start_in_corridor(contact_xyz):raise ValueError('not at HOME and outside the vertical reset corridor')
            if args.any_start and not folded and not start_in_corridor(contact_xyz):result['start_outside_corridor']=True
            result.update(start_snapshot=initial,start_candidate_xyz=contact_xyz,travel_rate_pulse_s=args.travel_rate,home_travel_profile=travel)
            if recorder:recorder.snapshot('start')
            def stage(name,operation,expected_xyz=None,expected_raw=None):
                global driver
                with (OUT/(name+'_events.jsonl')).open('w',buffering=1) as stream:
                    record=lambda e:stream.write(json.dumps({**e,'host_ns':time.monotonic_ns()})+'\n')
                    try:
                        receipt=operation(record);result['phases'].append({'phase':name,'result':receipt})
                    except BaseException as exc:
                        result['phases'].append({'phase':name,'error':str(exc),'hold_confirmed':driver.hold_confirmed})
                        if str(exc)!='hover endpoint not reached':raise
                        if not driver.hold_confirmed:
                            time.sleep(.15);driver.hold(attempts=4,settle_s=.4)
                    if not driver.hold_confirmed:raise ValueError('HOME stage hold not confirmed')
                    observed=driver._snapshot(torque=1);xyz=tcp(observed)[:3]
                    if expected_xyz and math.dist(xyz,expected_xyz)>.005:raise ValueError('HOME raise pose exceeds5mm candidate tolerance')
                    if expected_raw:
                        arrived={n:observed['joints'][n]['position'] for n in driver.names}
                        grade,deviation=home_arrival(arrived,expected_raw)
                        if grade=='far':raise ValueError(f'HOME exceeds the recorded-pose tolerance by {deviation} pulses')
                        if grade=='near':result['home_deviation_pulses']=deviation   # 8 넘게 어긋나도 다음 사이클 시작 허용선(32) 안에서 멈췄으면 성공으로 기록
                    result['phases'][-1].update(observed=observed,candidate_xyz=xyz)
                driver=home_driver(bus,profile);driver.hold_existing_torque(supported=True,keep_goal=True,load_offset=True,settle_s=.3,takeover_attempts=4)  # 보상한 목표를 현재 위치로 덮어쓰지 않는다
            for index in range(0 if folded else 3):
                current=driver._snapshot(torque=1);xyz=tcp(current)[:3]
                if xyz[2]>=.080:break
                target=(*xyz[:2],xyz[2]+.030)
                stage('raise'+str(index+1),lambda record:run_hover_trial(driver,g,zeros,target,math.hypot(*target[:2]),supported=True,lift_m=.030,record=record,cancelled=stopped),expected_xyz=target)
            # 접히는 도중 P 전환·정지 인계를 하지 않는다. 중간 경유점도
            # 한 번의 가감속 스트림으로 통과하고 HOME에서만 정지를 확인한다.
            result.update(continuous_home_return=True,pitch_p_home=pitch_p)
            stage('home',lambda record:run_recorded_home_trial(driver,g,zeros,home_raw,supported=True,record=record,cancelled=stopped,continuous=True,**travel),expected_raw=home_raw)
            result['home_return_complete']=True
            result['home_requested_raw']=home_raw
            result['home_actual_snapshot']=driver._snapshot(torque=1)
    except AlreadyAtHome:pass          # 이동 없이 성공: 게인은 with 종료로 복원되고 아래 finally가 hold를 확인한다
    except BaseException as exc:result['error']=str(exc)
    finally:
        if driver is not None:
            try:driver.hold(attempts=4,settle_s=.4);result['final_hold_confirmed']=driver.hold_confirmed
            except BaseException as exc:result['final_hold_error']=str(exc)
        result['restoration_verified']=bool(gains is not None and gains.restoration_verified) if gains is not None else None
        try:result['final_snapshot']=bus.read()
        except BaseException as exc:result['final_read_error']=str(exc)
        if recorder:
            try:recorder.snapshot('final')
            except BaseException as exc:result['recorder_error']=str(exc)
            finally:
                try:recorder.close()
                except BaseException as exc:result['recorder_close_error']=str(exc)
        (OUT/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({k:result.get(k) for k in ('home_return_complete','error','final_hold_confirmed','restoration_verified','home_requested_raw')},ensure_ascii=False),flush=True)
raise SystemExit(0 if all(result.get(k) for k in ('home_return_complete','final_hold_confirmed','restoration_verified')) and not result.get('error') and not result.get('recorder_error') and not result.get('recorder_close_error') else 1)
