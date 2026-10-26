#!/usr/bin/env python3
"""Automatic red-box pick: locate the box from the HOME camera image, then pick.

Computes the box position every run (no stored grip position). Without
--execute it only locates the box and checks reachability; the camera is read
but no motor or pump is touched. The candidate profile is not physically
validated and the camera estimate has about 1 cm error; general ROS execution
stays blocked. Physical task success requires observed evidence.
"""
from pathlib import Path
import argparse,json,sys,time,math,signal,subprocess,hashlib
from datetime import datetime,timezone
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
WRIST_CAMERA=None   # 아래 import 뒤에 정한다(ARM_WRIST_CAMERA / config/devices.json / 기본값)
from hardware.servo_mapping import MappingProfile
from hardware.servo_transport import ServoTransport
from hardware.servo_driver import ServoDriver
from robot.planar_kinematics import PlanarGeometry,NAMES
from motion.hover_trial import run_hover_trial,TRAVEL_PERIOD_S,TRAVEL_PULSES_PER_STEP,TRAVEL_GOAL_SPEED,TRAVEL_GOAL_ACCELERATION,TRAVEL_ACCEL,LIFT_MAX_M
from motion.home_trial import run_recorded_home_trial
from motion import guard_mode
from motion.trial_phase import finish_pose_phase,candidate_xyz,release_after_return,release_at_slot,stationary_snapshot,phase_pose_tolerance,approach_correction_target,approach_feedforward_target,PoseToleranceError
from sorting.place_exec import run_place,DEFAULT_MAX_CARRY_S
from sorting.place_plan import OBSTACLE_MARGIN_MM
from sorting.carry_path import polar_path
from sorting.place_job import load_place_job,build_plan
from sorting.place_followup import should_return_home,should_return_home_after_failed_pick,should_return_home_after_release_failure,placed_despite_retreat_error,run_home_return,NOT_HELD_EXIT_CODE,CHAIN_REFUSED_EXIT_CODE
from motion.hold_check import check_hold,hold_decision
from motion.pick_to_carry import build_lift_transition, run_pick_to_carry_trial, LiftPathRefused
from motion.place_exit import run_place_exit_trial
from robot.robot_model import load_planar_model
from perception.object_targets import load_catalog
from motion.trial_gains import TemporaryPitchGains
from hardware.vacuum_serial import SerialVacuum
from perception.box_target import BoxLocator,BoxNotFound,annotate
from robot.robot_frame import SHOULDER_HEIGHT_M
from camera.wrist_center import red_centroid,offset_mm,correction_xy,recenter_allowed,radial_offset_xy
from optional.wrist_centering.wrist_hover_center import run_center_once
from camera.wrist_contact_rgb import record_observation,outcome_context,camera_worker,unknown as unknown_contact_rgb

BASE=ROOT/'output/ik_resume_20261012'
SLOW_RATE_PPS=150.   # 천천히 움직이는 동작의 속도(펄스/초), 2026-10-23 사용자 요청: 100→150
# 세 물체 연속 정리(2026-10-21 사용자): 앞 물체를 놓은 트레이 위에서 HOME을 거치지 않고 다음 물체로 간다.
CHAIN_START_Z=(.070,.135)      # 놓기가 끝나는 운반 높이(70~130mm). 이 밖이면 기존 HOME 리셋
CHAIN_START_R=(.14,.32)        # HOME 복귀 시작 구간과 같은 반경(먼 칸은 놓은 뒤 이 안으로 물러나 선다)
CHAIN_PITCH_TOL=math.radians(5.)   # 공구가 수직(놓은 뒤 세우기 완료)이어야 한다
CHAIN_TRANSIT_Z=.120           # 트레이에서 물체로 갈 때의 최소 높이: 접근 경유점(120mm)과 같다. 물체는 40mm 이하, 트레이 벽은 운반 높이로 이미 넘는다
from hardware import devices
PUMP=devices.get('pump_port')      # ARM_PUMP_PORT / config/devices.json / 기본값
WRIST_CAMERA=Path(devices.get('wrist_camera'))


def post_pick_rise(hover, fresh_xyz, contact, *, placing):
    """After suction, stream two checked lift segments to 70 mm for a place job."""
    if placing:
        start=tuple(fresh_xyz())
        if math.hypot(start[0]-contact[0],start[1]-contact[1])>guard_mode.length(.003):
            raise ValueError('post-pick lift start moved more than 3 mm sideways')
        rise=.070-start[2]
        if .010<=rise<=2*LIFT_MAX_M:
            midpoint=(start[0],start[1],start[2]+rise/2)
            hover('lift_to_clear',(start[0],start[1],.070),lift_m=rise,via=(midpoint,),
                  via_pulses=2,continuous_lift=True)
            return 'lift_to_clear'
    hover('lift20',(*contact[:2],contact[2]+.020),lift_m=.020)
    return 'lift20'


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--supported',action='store_true')
    parser.add_argument('--image',type=Path,help='plan only: locate the box in this saved RGB image instead of the live camera')
    parser.add_argument('--depth-measurement',type=Path,help='experimental: match this selected depth target to a fresh synchronized HOME capture')
    parser.add_argument('--contact-offset-mm',type=float,default=0.,help='depth trial only: user-observed downward contact correction, -5 to 0 mm')
    parser.add_argument('--radial-offset-mm',type=float,default=0.,help='explicit user correction along the base-to-box ray, -10 to10 mm')
    parser.add_argument('--radius-cm',type=float,help='independent measured horizontal base-to-box distance; abort if the camera disagrees by >3 cm')
    space=parser.add_mutually_exclusive_group()
    space.add_argument('--fast-space',dest='fast_space',action='store_true',help='faster HOME/approach with ramped acceleration (default); contact and lift stay slow')
    space.add_argument('--slow-space',dest='fast_space',action='store_false',help='conservative 1-pulse travel with ramped acceleration')
    parser.set_defaults(fast_space=True)
    parser.add_argument('--center-cup',action='store_true',help='opt in to wrist-camera centering; default performs one descent then suction directly')
    parser.add_argument('--hover-photo-only',action='store_true',help='stop at the 70 mm hover over the box, take one wrist photo, hold and restore gains; no descent, no suction')
    parser.add_argument('--hover-center',action='store_true',help='with --hover-photo-only: image-based lateral centering, max5mm and one slow move, then verify a new photo')
    parser.add_argument('--hover-probe-tangent-mm',type=float,default=0.,help='with --hover-photo-only: after the first photo move the cup this far along the base tangent (+ = counter-clockwise/left) at the same 70 mm hover and photograph again; |value|<=6 mm, no contact')
    parser.add_argument('--contact-photo-only',action='store_true',help='calibration: descend to contact, take one wrist photo, lift 20 mm and stop; no suction, no place')
    parser.add_argument('--evidence-photos',action='store_true',help='also photograph the lifted and released states (adds about 10 s)')
    parser.add_argument('--no-contact-rgb-log',action='store_true',help='disable passive before-suction RGB observation; verdicts never block motion')
    parser.add_argument('--contact-rgb-reference',type=Path,default=BASE/'pickup_twoaxis_p64_trial/before_suction_wrist.png',help='historical contact-view reference for observational RGB comparison')
    rate=parser.add_mutually_exclusive_group()
    rate.add_argument('--travel-rate',type=float,help='candidate clear-space path rate, 50-300 pulses/s (default300); contact/lift unchanged')
    rate.add_argument('--approach-speed',type=float,help='legacy multiplier relative to50 pulses/s, 1-6; use --travel-rate instead')
    parser.add_argument('--pitch-p',type=int,choices=[32,48,64],default=64,help='temporary shoulder/elbow P gain for contact/lift (default64)')
    parser.add_argument('--travel-p',type=int,choices=[32,48,64],default=32,help='temporary P gain for HOME/approach (default 32). At 48 the approach shook in about 3 of 4 runs at one target (shoulder rms 5, base 3, measured 2026-10-16); at 32 it never shook (0.6-1.1) but the elbow stops a few mm short, so a failed 5 mm arrival check at P<48 is corrected once with a slow move by the measured error (up to 20 mm). The follow error reaches about 36: use --approach-tracking-limit 100. contact/lift use --pitch-p')
    parser.add_argument('--base-p',type=int,choices=[16,32],default=16,help='temporary base P gain (default 16 = servo baseline: with 32 the base oscillated at ~6 Hz while carrying to the tray, rms 2.5 pulses vs 0.7 at 16, measured 2026-10-15); restored on exit')
    parser.add_argument('--base-d',type=int,choices=[32,48,64],help='temporary base D (damping) gain (default: servo baseline 32); restored on exit. Added to reduce the ~6 Hz base oscillation seen while carrying')
    parser.add_argument('--damping',type=int,choices=[32,48,64,96],help='temporary shoulder/elbow D gain (default: servo baseline 32)')
    parser.add_argument('--batch-base',action='store_true',help='send barely-moving joints in 6-pulse bursts (off by default: motion stays continuous)')
    parser.add_argument('--native-moves',action='store_true',help='HOME/approach legs: send each goal once and let the servo run its own speed/acceleration profile (no per-tick goal steps)')
    parser.add_argument('--pick-tilt-deg',type=float,default=0.,help='tilt the tool forward by this many degrees (0-20) for the approach, descent and lift of the pick. Needed for far objects that a vertical cup cannot reach (user instruction 2026-10-15). Default 0 = vertical as before')
    parser.add_argument('--approach-tracking-limit',type=int,default=32,help='approach/approach_b only: follow-error guard in pulses (default 32). User instruction 2026-10-15: the 300 pulses/s approach leaves the shoulder at 33-34 and the 32 guard stopped most runs; raise it (e.g. 100) to let the approach finish. Place carry and HOME keep their own guard')
    parser.add_argument('--record-astra',action='store_true',help='record the top-view Astra video for the whole run (shake analysis)')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--manual-target-mm',type=float,nargs=3,metavar=('X','Y','CONTACT_Z'),help='robot-base target x y contact_z in mm instead of locating the red box; same reachability/envelope checks, validated before any device is opened')
    parser.add_argument('--place-json',type=Path,help='after lifting, carry the object to the tray slot in the job file and release it there (vacuum stays on in this process); needs --manual-target-mm; slot number is the commanded one, else the default rule white_box=1 red_cube=2 tape_measure=3')
    parser.add_argument('--place-empty-run',action='store_true',help='with --place-json: test run of the same carry path from the 70 mm hover over the target with NO suction and NO release (no object)')
    parser.add_argument('--place-max-tilt-deg',type=float,default=60.,help='place only: the tool may tilt forward from straight down by up to this many degrees (0 = stay vertical; max 60); only the smallest tilt that works is used (slots beyond 0.30 m need about 43) to lift the held object clear of the tray rim without dragging it low')
    parser.add_argument('--place-rate',type=float,default=150.,help='place only: clear-space carry speed in pulses/s, 50-300 (default 150 with 2 pulses/tick = the 13.3 ms tick that keeps the arm steady; at 300 the wrist follow error reached 33 and tripped the 32-pulse guard on the carry). Applies to horizontal carry moves and vertical rises above 70 mm; lifting off the table, lowering into the slot and tool tilt changes stay slow')
    parser.add_argument('--no-return-home',action='store_true',help='place only: do not return to HOME automatically after a completed place')
    parser.add_argument('--skip-home-after-place',action='store_true',help='place only: after a completed place stay above the tray with the tool vertical (the next object follows with --start-above-tray); failed picks/releases still return HOME')
    parser.add_argument('--start-above-tray',action='store_true',help='start from where the previous --skip-home-after-place cycle ended (above the tray, tool vertical) and go straight to the object without HOME; other start poses use the existing HOME reset')
    parser.add_argument('--home-rate',type=float,default=150.,help='place only: automatic HOME return speed in pulses/s, 50-300 (default 150, the speed that passes the return_home.py 16-pulse follow-error guard: at 200 and 300 the shoulder lags 17 and the guard stops the return (measured 2026-10-15))')
    parser.add_argument('--held-rate',type=float,default=150.,help='pulses/s while the object hangs on the cup: post-pick rise, lifts/lowers in place and the table return, 0 = old slow profile (26-33), else 40-150 (default 150)')
    parser.add_argument('--approach-feedforward-mm',type=float,nargs=2,default=[3.5,3.0],metavar=('RADIAL','UP'),help='send the approach/approach_b hover this far outward and up (P32 stops short the same way: 2026-10-20, 15 runs); 0 0 disables, each 0-6 mm')
    parser.add_argument('--descent-rate',type=float,default=150.,help='pulses/s while lowering from above (pick descent; place lower uses --held-rate), 13-150 (default 150)')
    parser.add_argument('--place-pulses-per-step',type=int,choices=[2,4],default=2,help='place carry: pulses per tick (tick = pulses/--place-rate). Past records show the arm shakes unless the tick is about 13.4 ms; 2 pulses at 150 pulses/s gives that tick at a lower speed than 4 at 300')
    parser.add_argument('--hold-check',choices=['enforce','record','off'],default='off',help='optional diagnostic after the post-pick rise: compare a wrist photo with the contact photo; enforce = stop if it appears unheld, record = log only, off = no photo check (default)')
    parser.add_argument('--min-contact-z-mm',type=float,default=10.,help='lowest cup height for contact/lift moves, 5-30 mm (default 10). The old 30 mm floor was removed on user instruction so low objects (white box top 24 mm) can be picked; this minimum only keeps the cup off the table')
    parser.add_argument('--place-max-s',type=float,default=DEFAULT_MAX_CARRY_S,help='stop (without releasing) if the place sequence takes longer than this many seconds')
    args=parser.parse_args(argv)
    if args.hover_center and not args.hover_photo_only:
        parser.error('hover-center requires --hover-photo-only until pick integration is verified')
    if args.hover_center and args.hover_probe_tangent_mm:
        parser.error('hover-center conflicts with manual probe')
    if args.hover_center and args.center_cup:
        parser.error('hover-center cannot use contact centering')
    travel_rate=args.travel_rate if args.travel_rate is not None else 50*args.approach_speed if args.approach_speed is not None else 300.
    if not math.isfinite(travel_rate) or not 50<=travel_rate<=300:parser.error('travel rate must be finite and within50-300 pulses/s')
    approach_pulses=TRAVEL_PULSES_PER_STEP
    approach_period=approach_pulses/travel_rate
    approach_limit=32 if args.fast_space else 16
    if not math.isfinite(args.pick_tilt_deg) or not 0.<=args.pick_tilt_deg<=20.:parser.error('--pick-tilt-deg must be 0-20')
    if not math.isfinite(args.descent_rate) or not 13.<=args.descent_rate<=150.:parser.error('--descent-rate must be 13-150 pulses/s')
    if not all(math.isfinite(v) and 0.<=v<=6. for v in args.approach_feedforward_mm):parser.error('--approach-feedforward-mm must be 0-6 mm each')
    if not math.isfinite(args.held_rate) or not (args.held_rate==0 or 40.<=args.held_rate<=150.):parser.error('--held-rate must be 0 or 40-150 pulses/s')
    if not math.isfinite(args.min_contact_z_mm) or not 5.<=args.min_contact_z_mm<=30.:parser.error('--min-contact-z-mm must be 5-30 mm')
    if not math.isfinite(args.radial_offset_mm) or abs(args.radial_offset_mm)>10:parser.error('radial offset must be finite within -10 to10 mm')
    if not math.isfinite(args.contact_offset_mm) or not -5<=args.contact_offset_mm<=0:
        parser.error('contact offset must be finite within -5 to 0 mm')
    if args.contact_offset_mm and not args.depth_measurement:
        parser.error('contact offset requires --depth-measurement')
    if args.execute and not args.supported:parser.error('--execute requires --supported')
    manual=None
    if args.manual_target_mm is not None:
        for flag,value in (('--depth-measurement',args.depth_measurement),('--center-cup',args.center_cup),('--hover-center',args.hover_center),
                           ('--image',args.image),('--radial-offset-mm',args.radial_offset_mm)):
            if value:parser.error(f'manual target conflicts with {flag}')
        mx,my,mz=args.manual_target_mm
        if not all(math.isfinite(v) for v in (mx,my,mz)):parser.error('manual target must be finite')
        if not 15.<=mz<=45. or mz<args.min_contact_z_mm:parser.error('manual contact z must be within 15-45 mm and not below --min-contact-z-mm')
        # 기울여 집으면 접촉이 약간 높게 끝났다(사용자 확인 2026-10-15). 기울여 집을 때만 접촉 높이를 3mm 더 낮춘다(최소 높이는 지킨다).
        if args.pick_tilt_deg:mz=max(mz-3.,args.min_contact_z_mm)
        manual=dict(target_xy=(mx/1000.,my/1000.),contact_z=mz/1000.,radius_m=math.hypot(mx,my)/1000.,
                    camera_target_xy=[mx/1000.,my/1000.],user_radial_offset_mm=0.,
                    target_source='manual user-provided robot-base target',
                    physical_accuracy_verified=False,usable_for_motion=False)
    profile=MappingProfile.load(ROOT/'config/servo_driver_candidate.json',ROOT)
    review=json.loads((BASE/'multipose_review/review.json').read_text());fit=json.loads((BASE/'multipose_review/fit_full.json').read_text())
    objects_transform=BASE/'multipose_review/camera_transform_objects.json'   # 물체 위치용 Pi 재보정(2026-10-26). 없으면 예전 변환
    locator=BoxLocator(objects_transform if objects_transform.exists() else BASE/'multipose_review/camera_transform_candidate.json',BASE/'moved_box_target.json')
    home_source=json.loads((BASE/'original_current_hold.json').read_text())
    home={n:home_source['joints'][n]['position'] for n in profile.data['expected_registers']}
    zeros={j.name:j.zero for j in profile.joints};zeros.update(fit['new_zero_pulses'])
    geometry=PlanarGeometry(.027,SHOULDER_HEIGHT_M,tuple(review['eight_pose_geometry_m']))
    joint_map={j.name:j for j in profile.joints}
    pulse=2*math.pi/4096
    limits={n:tuple(sorted(joint_map[n].direction*(r-zeros[n])*pulse for r in (joint_map[n].raw_min+16,joint_map[n].raw_max-16))) for n in NAMES}
    def check_target(located):
        """Raise unless the camera target is inside the envelope the hover code accepts."""
        x,y=located['target_xy'];radius=located['radius_m'];zc=located['contact_z']
        if not .16<=radius<=.30:raise ValueError(f'box radius {radius:.3f} m outside .16-.30 m pick envelope')
        if args.radius_cm is not None and abs(radius*100-args.radius_cm)>3:
            raise ValueError(f'camera radius {radius*100:.1f} cm disagrees with measured {args.radius_cm} cm')
        inner=max(.12,radius-.040)
        for label,point in (('inner approach',(x*inner/radius,y*inner/radius,.120)),('approach 70 mm',(x,y,.070)),('contact',(x,y,zc))):
            if not geometry.solve(point,-math.pi/2+math.radians(args.pick_tilt_deg),limits):raise ValueError(f'target not reachable ({label}, tilt {args.pick_tilt_deg:g} deg): {tuple(round(v,3) for v in point)}')
    def locate_image(path):
        import cv2
        image=cv2.imread(str(path))
        if image is None:raise OSError('image unreadable: '+str(path))
        if args.depth_measurement:
            from optional.depth_pick.depth_pick_target import locate_depth_box
            located=locate_depth_box(path,args.depth_measurement)
            located['depth_contact_z_m']=located['contact_z']
            located['contact_offset_mm']=args.contact_offset_mm
            located['contact_z']+=args.contact_offset_mm/1000.
        else:
            located=locator.locate(image)
        located['camera_target_xy']=list(located['target_xy'])
        located['user_radial_offset_mm']=args.radial_offset_mm
        located['target_xy']=radial_offset_xy(located['target_xy'],args.radial_offset_mm)
        located['radius_m']=math.hypot(*located['target_xy'])
        check_target(located)
        return image,located
    if manual is not None:
        try:check_target(manual)
        except ValueError as exc:parser.error(str(exc))
    place_job=place_model=place_catalog=None
    if args.place_empty_run and not args.place_json:parser.error('--place-empty-run requires --place-json')
    if args.place_json:
        if manual is None:parser.error('place requires --manual-target-mm (the pick target must come from the same detection)')
        for flag,value in (('--contact-photo-only',args.contact_photo_only),('--hover-photo-only',args.hover_photo_only),('--center-cup',args.center_cup),('--depth-measurement',args.depth_measurement),('--hover-center',args.hover_center)):
            if value:parser.error(f'place conflicts with {flag}')
        if not math.isfinite(args.place_max_s) or not 1.<=args.place_max_s<=1800.:parser.error('--place-max-s must be 1-1800 seconds')
        if not math.isfinite(args.place_rate) or not 50.<=args.place_rate<=300.:parser.error('--place-rate must be 50-300 pulses/s')
        if not math.isfinite(args.home_rate) or not 50.<=args.home_rate<=300.:parser.error('--home-rate must be 50-300 pulses/s')
        if not math.isfinite(args.place_max_tilt_deg) or not 0.<=args.place_max_tilt_deg<=60.:parser.error('--place-max-tilt-deg must be 0-60 degrees')
        try:
            place_job=load_place_job(args.place_json)
            if place_job['label']=='unknown':
                object_geometry=place_job['object_geometry']   # 'geometry'는 팔 기구학 모델 이름이라 덮어쓰면 안 된다
                args.place_max_tilt_deg=0.  # 외곽 적합 검사는 수직 놓기를 전제로 한다.
                from perception.unknown_objects import CONTACT_PRESS_MM
                if args.pick_tilt_deg or abs(manual['contact_z']*1000.-(object_geometry['height_mm']-CONTACT_PRESS_MM))>.1:
                    raise ValueError('unknown 목표 높이가 관측값과 다르거나 수직 집기가 아닙니다.')
            place_model=load_planar_model(ROOT);place_catalog=load_catalog(ROOT/'config/object_catalog.json')
            preview_start=(*manual['target_xy'],.070 if args.place_empty_run or args.held_rate and manual['contact_z']<=.060 else manual['contact_z']+.020)
            preview=build_plan(place_job,place_model,place_catalog,preview_start,args.place_max_tilt_deg,args.pick_tilt_deg)
        except ValueError as exc:parser.error(str(exc))
    plan=dict(home_raw=home,contact_rgb_mode='disabled' if args.no_contact_rgb_log else 'record_only',contact_rgb_can_block_motion=False,target_source=manual['target_source'] if manual is not None else 'fresh HOME RGB-depth matched to selected measurement' if args.depth_measurement else 'camera at HOME, computed this run',hover_heights_m=[.120,.070],lift_m=(round(.070-manual['contact_z'],4) if place_job is not None and args.held_rate and manual['contact_z']<=.060 else .020),normal_execution_enabled=False,space_pulses_per_step=approach_pulses if args.fast_space else 1,
              motion_profile=dict(center_cup=args.center_cup,hover_center=args.hover_center,fast_space=args.fast_space,travel_rate_pulses_s=travel_rate,clear_period_s=approach_period,
                                  goal_speed=TRAVEL_GOAL_SPEED,goal_acceleration=TRAVEL_GOAL_ACCELERATION,path_acceleration=TRAVEL_ACCEL,
                                  physically_validated=False,smooth_motion=True,native_moves=args.native_moves,descent_rate_pulses_s=args.descent_rate,held_rate_pulses_s=args.held_rate,slow_rate_pulses_s=SLOW_RATE_PPS,min_contact_z_mm=args.min_contact_z_mm,
                                  travel_p=args.travel_p,contact_p=args.pitch_p,base_p=args.base_p,base_d=args.base_d,
                                  approach_pose_tolerance_m=phase_pose_tolerance('approach',pick_tilt_deg=args.pick_tilt_deg)))
    if place_job is not None:
        plan['place']=dict(label=place_job['label'],slot_number=place_job['slot_number'],slot_source=place_job['slot_source'],empty_run=args.place_empty_run,
                           max_seconds=args.place_max_s,hold_check=args.hold_check,max_tilt_deg=args.place_max_tilt_deg,rate_pulses_s=args.place_rate,pulses_per_step=args.place_pulses_per_step,tick_s=args.place_pulses_per_step/args.place_rate,
                           return_home=dict(enabled=not (args.no_return_home or args.skip_home_after_place),rate_pulses_s=args.home_rate,skip_after_place=args.skip_home_after_place),rim_height_mm=place_job['rim_height_mm'],
                           start_assumed_xyz=list(preview_start),plan=preview,
                           note=('empty run: no suction and no release; starts from the 70 mm hover' if args.place_empty_run else
                                 '70 mm preview start; eligible pickup rise and carry tilt are planned from the measured contact pose and executed in one stream, otherwise the place plan is recomputed after the vertical rise'),
                           physically_validated=False)
        if place_job['label']=='unknown':
            plan['place']['object_geometry']=place_job['object_geometry']
    if not args.execute:
        if manual is not None:
            plan['located']=dict(manual)
            print(json.dumps(dict(execute=False,motor_writes=False,pump_writes=False,plan=plan),ensure_ascii=False,indent=2,default=float));return 0
        import cv2
        source=args.image
        if source is None:
            plan_dir=ROOT/'output/pick_trials'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'_plan')
            plan_dir.mkdir(parents=True,exist_ok=False)
            subprocess.run([sys.executable,str(ROOT/'scripts/calibrate_robot_camera.py'),'capture','--output',str(plan_dir/'astra')],cwd=ROOT,check=True,timeout=15)
            source=plan_dir/'astra/0000_rgb.png'
        image,located=locate_image(source)
        cv2.imwrite(str(Path(source).with_name('target_annotation.png')),annotate(image,located))
        plan['located']=located
        print(json.dumps(dict(execute=False,motor_writes=False,pump_writes=False,plan=plan),ensure_ascii=False,indent=2,default=float));return 0
    if place_job is not None and place_job['label']=='unknown':
        from pick_unknown import verify_before_motion
        try:verify_before_motion(place_job,ROOT,manual['target_xy'])
        except Exception as exc:parser.error(str(exc))
    out=args.output or ROOT/'output/pick_trials'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
    out.mkdir(parents=True,exist_ok=False)
    result=dict(plan=plan,motion_cycle_complete=False,physical_pick_success=None,phases=[],pump_commands=[],normal_execution_enabled=False)
    cycle_start=time.monotonic()
    result['started_at']=datetime.now(timezone.utc).isoformat()
    cancelled=[False]
    def stop_signal(*_):cancelled[0]=True
    signal.signal(signal.SIGINT,stop_signal);signal.signal(signal.SIGTERM,stop_signal)
    def is_cancelled():return cancelled[0] or (out/'stop').exists()
    driver=pump=gains=None
    recorder=[None]
    def capture(label,astra=True,wrist=True):
        import cv2
        if astra and recorder[0] is not None:
            recorder[0].snapshot(label)
        elif astra:
            command_args=[sys.executable,str(ROOT/'scripts/calibrate_robot_camera.py'),'capture','--output',str(out/(label+'_astra'))]
            if label=='home' and args.depth_measurement:command_args.extend(['--burst-count','12'])
            subprocess.run(command_args,cwd=ROOT,check=True,timeout=20)
        if not wrist:return
        if not WRIST_CAMERA.is_symlink():raise OSError('stable wrist camera by-id device is absent')
        cam=cv2.VideoCapture(str(WRIST_CAMERA),cv2.CAP_V4L2);cam.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'));cam.set(cv2.CAP_PROP_FRAME_WIDTH,640);cam.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        try:
            for _ in range(10):ok,frame=cam.read()
            if not ok or frame is None or frame.shape!=(480,640,3):raise OSError('wrist frame resolution/read failed')
            if not cv2.imwrite(str(out/(label+'_wrist.png')),frame):raise OSError('wrist image save failed')
        finally:cam.release()
    def save():
        (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    def command(name):
        receipt=pump.suction(require_ack=True) if name=='suction' else getattr(pump,name)()
        result['pump_commands'].append(dict(command=name,time=time.monotonic(),pressure_verified=False,controller_receipt=receipt));save()
    def contact_rgb_observation():
        import cv2
        evidence=[]
        def frames():
            if is_cancelled():raise InterruptedError('cancelled before RGB observation')
            folder=out/'contact_rgb_frames'
            camera_worker([sys.executable,str(ROOT/'scripts/capture_wrist_contact_rgb.py'),'--output',str(folder)])
            if is_cancelled():raise InterruptedError('cancelled during RGB observation')
            records=json.loads((folder/'capture.json').read_text())['frames']
            images=[]
            for row in records:
                file=folder/row['image']
                if hashlib.sha256(file.read_bytes()).hexdigest()!=row['sha256']:raise OSError('RGB evidence hash mismatch')
                evidence.append(dict(row,image=str(file.relative_to(out))))
                images.append(cv2.imread(str(file)))
            return images
        reference=None;reference_hash=None
        try:
            reference=cv2.imread(str(args.contact_rgb_reference))
            reference_hash=hashlib.sha256(args.contact_rgb_reference.read_bytes()).hexdigest()
        except Exception:pass
        def store(report):
            report.update(reference_path=str(args.contact_rgb_reference),reference_sha256=reference_hash,
                          reference_status='historical_contact_view_not_independently_validated',
                          evidence=list(evidence),candidate_contact_xyz=result.get('contact_candidate_xyz'))
            result['contact_rgb']=dict(report)
            (out/'contact_rgb.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
            save()
        report=record_observation(frames,reference,store)
        result['contact_rgb']=dict(report)
        print(json.dumps(dict(contact_rgb=report['display'],mode='record_only',can_block_motion=False,record_error=report.get('record_error')),ensure_ascii=False),flush=True)
    with ServoTransport(profile) as bus:
        try:
            if args.record_astra:
                from optional.video_recording.astra_recorder import AstraRecorder
                recorder[0]=AstraRecorder(out)
            driver=ServoDriver(bus,profile)
            # Extended poses sag at the baseline gains, so the hold cannot settle.
            # Check the arm is stationary, raise the temporary pitch gains first
            # (restored on exit), then take over the existing torque hold.
            first=driver._snapshot(torque=1);driver.sleep(.05);second=driver._snapshot(torque=1)
            for joint in profile.joints:
                n=joint.name;a=first['joints'][n];b=second['joints'][n]
                if abs(a['position']-b['position'])>1 or a['goal']!=b['goal'] or abs(b['goal']-b['position'])>8 or not joint.raw_min+16<=b['position']<=joint.raw_max-16:
                    raise ValueError('pose is not stationary before temporary gains')
            result['pre_gain_stationary_snapshot']=second
            gains=TemporaryPitchGains(bus,out,target_p={2:args.travel_p,3:args.travel_p},shoulder_i=1,target_d=None if args.damping is None else {2:args.damping,3:args.damping},base_p=args.base_p,base_d=args.base_d)
            integral=[1]
            def set_pitch(value):
                if all(gains.target_p[m]==value for m in (2,3)):return
                # No gain change during motion or while the object is suspended.
                # Called only before clear-space HOME/approach and before contact.
                stationary_snapshot(driver,settle_attempts=3)
                gains.set_pitch_p(value)
                driver.hold(attempts=4,settle_s=.4)   # 게인을 바꾸면 하중 받는 어깨가 새 평형으로 움직인다: 다 움직인 뒤 같은 기준으로 다시 확인(2026-10-16)
                stationary_snapshot(driver,settle_attempts=3)
                result.setdefault('pitch_changes',[]).append(dict(value=value,time=time.monotonic()));save()
            def set_integral(value):
                # I=1 holds extended poses. P64/I1 hunted on the old HOME travel;
                # tested P48/I1 keeps the hold while reducing travel vibration.
                # Switch only between phases, never during motion.
                if integral[0]!=value:
                    gains.set_shoulder_integral(value);integral[0]=value;result.setdefault('integral_changes',[]).append(dict(value=value,time=time.monotonic()));save()
            with gains:
                driver.sleep(.8)   # 2초→0.8초(return_home.py와 같음, 2026-10-16). 바로 뒤 hold_existing_torque가 정지·목표 일치를 그대로 확인한다
                driver.hold_existing_torque(supported=True,attempts=4,settle_s=.4)   # 하중 관절이 3~6펄스 밀려 서면 hold(attempts>1)처럼 목표를 보정한다(2026-10-19)
                pump=SerialVacuum(PUMP).open();command('vent');driver.sleep(.3);command('stop')
                def fresh_xyz():return candidate_xyz(driver,geometry,zeros,driver._snapshot(torque=1))
                def stage(name,target,operation,home_goal=None):
                    nonlocal driver
                    if is_cancelled():raise InterruptedError('cycle cancelled')
                    phase_start=time.monotonic()
                    with (out/(name+'_events.jsonl')).open('w',buffering=1) as stream:
                        record=lambda e:stream.write(json.dumps({**e,'host_ns':time.monotonic_ns()})+'\n')
                        try:
                            receipt=finish_pose_phase(driver,geometry,zeros,lambda:operation(record),target,settle_attempts=3,
                                                      tolerance_m=phase_pose_tolerance(name,pick_tilt_deg=args.pick_tilt_deg))
                        except PoseToleranceError as exc:
                            result['phases'].append(dict(phase=name,receipt=exc.receipt,error=str(exc),duration_s=time.monotonic()-phase_start,elapsed_s=time.monotonic()-cycle_start))
                            record(dict(phase='pose_rejected',receipt=exc.receipt));save()
                            raise
                    if home_goal and max(abs(receipt['snapshot']['joints'][n]['position']-home_goal[n]) for n in home)>guard_mode.pulses(8):
                        raise ValueError('recorded HOME exceeds8pulse pose tolerance')
                    result['phases'].append(dict(phase=name,receipt=receipt,duration_s=time.monotonic()-phase_start,elapsed_s=time.monotonic()-cycle_start));save()
                    print(json.dumps(dict(phase=name,raw_endpoint_met=receipt['raw_endpoint_met'],candidate_xyz=receipt['candidate_xyz']),ensure_ascii=False),flush=True)
                    # 하중으로 3~6펄스 일정하게 처져 선 관절은 목표를 보정해 붙잡는다(흡착 뒤·HOME 복귀와 같은 방식). 2026-10-26 Pi:
                    # 트레이 위에서 올라간 뒤 손목이 목표보다 4펄스 아래에 고정돼 한 번 확인으로는 'physical hold unconfirmed'로 멈췄다
                    driver=ServoDriver(bus,profile);driver.hold_existing_torque(supported=True,attempts=4,settle_s=.4,takeover_attempts=4)
                    return receipt
                def hover(name,target,**kwargs):
                    nonlocal driver
                    if args.pick_tilt_deg and name.startswith(('approach','descent','lift20','lift_to_clear','return','probe','rise','recenter','redescend')):
                        kwargs.setdefault('pitch_rad',-math.pi/2+math.radians(args.pick_tilt_deg))   # far object: pick with the tool tilted
                    kwargs.setdefault('min_contact_z',args.min_contact_z_mm/1000.)  # only matters for contact/lift moves
                    fast_place=kwargs.pop('fast',False) and name.startswith('place')   # place moves that may use the clear-space profile
                    travel=args.fast_space and (target[2]>=.080 or kwargs.get('box_top_m') is not None and target[2]>=max(.070,kwargs['box_top_m']+.030)) and not ('lift_m' in kwargs or 'measured_gap_m' in kwargs)
                    if name.startswith(('probe','place')):travel=False  # a few-millimetre hover probe is always the slow 1-pulse profile
                    kwargs['pulses_per_step']=approach_pulses if travel else 1;kwargs['fast_travel']=travel
                    if travel:kwargs.update(tick_s=approach_period,tracking_limit=approach_limit)
                    # The low moves use their own bounded pulse rate and tracking guard.
                    kwargs['quick_ticks']=args.fast_space and not name.startswith(('contact','lift20','return','probe','place'))
                    if fast_place:
                        # place carry speed (--place-rate): same fixed-rate profile as the approach, only for moves place_exec marked fast
                        kwargs.update(pulses_per_step=args.place_pulses_per_step,fast_travel=True,quick_ticks=True,tick_s=args.place_pulses_per_step/args.place_rate,tracking_limit=4095)  # 사용자 지시 2026-10-19: place 운반 추종 가드 해제(300에서 손목 33이 32 가드에 걸림). 4095=서보 전 범위라 사실상 무제한
                        if kwargs.get('via'):kwargs['via_pulses']=args.place_pulses_per_step  # continuous through the via points, no stop (like the approach)
                    if args.fast_space and name in ('approach','approach_b'):
                        # HOME->over the box in one continuous move: 4 pulses/tick until the 4 cm short
                        # point and the explicit70mm box hover; no stop at the via point.
                        kwargs['fast_travel']=True;kwargs['quick_ticks']=True;kwargs['pulses_per_step']=approach_pulses
                        kwargs['tick_s']=approach_period;kwargs['tracking_limit']=args.approach_tracking_limit if args.fast_space else approach_limit;kwargs['native']=args.native_moves
                    if args.fast_space and name.startswith('descent'):
                        kwargs['tick_s']=1./args.descent_rate
                    if name.startswith('place_lower'):
                        kwargs.update(quick_ticks=True,tick_s=1./args.descent_rate)
                    if args.held_rate and ('lift_m' in kwargs or 'measured_gap_m' in kwargs) and name.startswith(('lift20','lift_to_clear','place','return')):
                        # 흡착한 채 움직일 때의 속도는 --held-rate를 적용한다.
                        kwargs.update(quick_ticks=True,pulses_per_step=2,tick_s=2./args.held_rate,tracking_limit=max(kwargs.get('tracking_limit',16),32))
                        if name.startswith('place'):kwargs.pop('tail',None)
                        elif name.startswith('return'):kwargs['tail']=(14,13.)
                    if not kwargs.get('fast_travel'):
                        # 기본 저속 동작은 150펄스/초. 명시한 하강·흡착 운반 속도는 유지한다.
                        kwargs.pop('tail',None)
                        pps=kwargs.get('pulses_per_step',1)
                        # 틱당 2펄스. 낮은 상자 위 호버만 hover_trial의 1펄스 규칙을 유지한다.
                        if pps==1 and not (kwargs.get('box_top_m') is not None and target[2]<.080):pps=2
                        held_move=('lift_m' in kwargs or 'measured_gap_m' in kwargs) and name.startswith(('lift20','lift_to_clear','place','return'))
                        rate=(args.held_rate if held_move and args.held_rate else
                              args.descent_rate if name.startswith('descent') or name.startswith('place_lower') else
                              SLOW_RATE_PPS)
                        if held_move and not args.held_rate:
                            pass  # explicit legacy slow profile
                        elif not kwargs.get('quick_ticks') or pps!=kwargs.get('pulses_per_step',1) or pps/kwargs.get('tick_s',1.)<rate:
                            kwargs.update(quick_ticks=True,pulses_per_step=pps,tick_s=pps/rate)
                        kwargs['tracking_limit']=max(kwargs.get('tracking_limit',16),32)
                    def move(record,goal,**extra):
                        return run_hover_trial(driver,geometry,zeros,goal,math.hypot(*goal[:2]),supported=True,record=record,cancelled=is_cancelled,batch_slow_joints=args.batch_base,smooth_motion=True,**{**kwargs,**extra})
                    try:
                        sent=target
                        if name in ('approach','approach_b') and args.travel_p<48 and any(args.approach_feedforward_mm):
                            sent=approach_feedforward_target(target,args.approach_feedforward_mm[0]/1000.,args.approach_feedforward_mm[1]/1000.)   # 판정은 원래 목표로
                        return stage(name,target,lambda record:move(record,sent))
                    except ValueError as exc:
                        # P 게인을 낮춰(--travel-p 32) 접근하면 떨림은 없지만 하중 받는 팔꿈치가 목표보다 짧게 멈춘다.
                        # 작은 오차(20mm 이하)는 측정한 오차만큼 한 번 더 보내 고친다(느린 1펄스 이동, 허용치는 그대로 5mm).
                        if str(exc)!='candidate pose outside phase tolerance' or args.travel_p>=48 or name not in ('approach','approach_b'):raise
                        # 최대 2회(2026-10-20 사용자): 1회 고침 뒤 5.4mm로 0.4mm 넘겨 멈춘 기록(09:30). 두 번째는 직전 명령에 새 오차를 더한다.
                        commanded=sent   # 미리 밀어 보낸 목표에서 이어서 고친다
                        for attempt in range(2):
                            driver=ServoDriver(bus,profile)
                            # Endpoint fault already established a verified hold.
                            # Read-check its fixed goals; do not recenter and sag again.
                            driver.hold_existing_torque(supported=True,keep_goal=True,load_offset=True,settle_s=.3,takeover_attempts=4)   # finish_pose_phase의 보상 hold(목표 3~6펄스 덜 씀)를 그대로 받는다(2026-10-19)
                            actual=fresh_xyz();error=[t-a for t,a in zip(target,actual)]
                            fix=approach_correction_target(target,actual,commanded)
                            correction=dict(name=name,attempt=attempt+1,error_mm=[round(v*1000,1) for v in error],before_candidate_xyz=list(actual),
                                            original_target_xyz=list(target),correction_target_xyz=list(fix),before_error_m=math.dist(target,actual))
                            result.setdefault('approach_fixes',[]).append(correction);save()
                            try:
                                receipt=stage(name+('_fix' if attempt==0 else '_fix2'),target,lambda record:move(record,fix,via=(),fast_travel=False,quick_ticks=False,pulses_per_step=1,tick_s=None,tail=None,native=False))
                            except ValueError as again:
                                if str(again)!='candidate pose outside phase tolerance' or attempt==1:raise
                                commanded=fix;continue
                            correction.update(after_error_m=receipt['candidate_error_m'],after_candidate_xyz=list(receipt['candidate_xyz']));save()
                            return receipt
                def run_place_job(empty,prepared=None):
                    start=fresh_xyz()
                    try:plan_now=prepared if prepared is not None else build_plan(place_job,place_model,place_catalog,start,args.place_max_tilt_deg,args.pick_tilt_deg)
                    except ValueError as exc:raise ValueError('place plan refused from the measured pose: '+str(exc))
                    result['place_plan']=plan_now;save()
                    print(json.dumps(dict(place_plan=plan_now),ensure_ascii=False,default=float),flush=True)
                    def release(expected):
                        nonlocal driver
                        result['release']=release_at_slot(driver,geometry,zeros,expected,pump)
                        result['pump_commands'].extend([dict(command='vent_place',pressure_verified=False),dict(command='stop_place',pressure_verified=False)])
                        # 물체를 놓아 하중이 빠지면 관절이 1~2펄스 옮겨 선다. 놓기 전에 붙잡은 목표·보상값을 그대로 쓰면 다음 이동 첫 틱의
                        # 목표 읽기 검사(2펄스)에 걸린다(2026-10-26 Pi: 놓은 뒤 base 목표 2295·위치 2291로 'unexpected hover goal readback',
                        # 책상 정리가 HOME으로 빠짐). 지금 목표를 그대로 두고(쓰기 없음) 위치와의 차이만 다시 받아 붙잡는다. 안 되면 예전 driver로 간다.
                        fresh=ServoDriver(bus,profile)
                        try:
                            fresh.hold_existing_torque(supported=True,keep_goal=True,load_offset=True,settle_s=.3,takeover_attempts=4)
                        except ValueError as exc:
                            result['release_rehold_error']=str(exc)
                        else:
                            driver=fresh
                    def exit_path(transition):
                        # 놓은 뒤 물러나기→빠지기→세우기를 한 번에(사용자 지시 2026-10-26). 칸 안은 place_rate의 절반, 벽 위는 place_rate.
                        # 준비 단계 거부(ExitPathRefused)는 run_place가 받아 예전처럼 단계별로 간다
                        rate=max(13.,min(150.,args.place_rate/2.))
                        return stage('place_exit',transition['target'],lambda record:run_place_exit_trial(
                            driver,geometry,zeros,transition,supported=True,rate=rate,
                            min_contact_z=args.min_contact_z_mm/1000.,tracking_limit=32,record=record,cancelled=is_cancelled))
                    report=run_place(plan_now,hover=hover,fresh_xyz=fresh_xyz,release=release,cancelled=is_cancelled,empty_run=empty,skip_final_untilt=not (args.no_return_home or args.skip_home_after_place),
                                     max_carry_s=args.place_max_s,record=lambda event:result.setdefault('place_steps',[]).append(event),exit_path=exit_path)
                    result['place']=report;save()
                initial=driver._snapshot(torque=1);xyz=fresh_xyz()
                home_delta=max(abs(initial['joints'][n]['position']-home[n]) for n in home)
                at_home=home_delta<=guard_mode.pulses(8)
                if not at_home and home_delta<=guard_mode.pulses(48):       # 복귀 허용(40)보다 넓게: 2026-10-19 wrist가 36펄스 모자라 선 채로 거부됐다
                    # The unpowered arm can settle a few pulses from recorded HOME.
                    # Use it only when every modeled link endpoint still clears the table.
                    joints={j.name:j for j in profile.joints}
                    q=tuple(joints[n].direction*(initial['joints'][n]['position']-zeros[n])*pulse for n in NAMES)
                    if min(point[2] for point in geometry.link_points(q))>=.060:
                        at_home=True
                        result['near_home_takeover_max_delta_pulse']=home_delta
                        save()
                chain_start=False
                if not at_home and args.start_above_tray:
                    # 앞 물체를 놓고(--skip-home-after-place) 트레이 위 운반 높이에 공구를 세운 채 서 있으면 HOME을 거치지 않고 바로 다음 물체로 간다.
                    # 조건(높이·반경 구간, 공구 수직)이 안 맞으면 움직이기 전에 멈춘다(종료 코드 4). 기존 HOME 리셋은 물체 위 같은 가까운 자리용이라
                    # 트레이 위에서는 한 번 이동 한도(1600펄스)를 넘는다(2026-10-21 실물: 거부). 호출자가 return_home.py로 HOME에 간 뒤 다시 한다.
                    q0=tuple(joint_map[n].direction*(initial['joints'][n]['position']-zeros[n])*pulse for n in NAMES)
                    tool_pitch=geometry.forward(q0)[3]
                    chain_start=bool(CHAIN_START_Z[0]<=xyz[2]<=CHAIN_START_Z[1] and CHAIN_START_R[0]<=math.hypot(*xyz[:2])<=CHAIN_START_R[1]
                                     and abs(tool_pitch+math.pi/2)<=CHAIN_PITCH_TOL)
                    result['chain_start']=dict(used=chain_start,start_xyz=list(xyz),tool_pitch_deg=round(math.degrees(tool_pitch),1))
                    save()
                    if not chain_start:
                        result['chain_refused']=True
                        raise ValueError('start above the tray is outside the chain start conditions; return HOME first')
                if not at_home and not chain_start:
                    # Reset from where the previous cycle ended (vertically above/at a box), not from a stored scene.
                    if not .030<=xyz[2]<=.135 or not .14<=math.hypot(*xyz[:2])<=.32:
                        raise ValueError('not at HOME and outside the vertical reset corridor; move to HOME first')
                    for index in range(3):
                        xyz=fresh_xyz()
                        if xyz[2]>=.080:break
                        hover('reset_raise'+str(index),(*xyz[:2],xyz[2]+.030),lift_m=.030)
                    qhome=[-(home[n]-zeros[n])*2*math.pi/4096 for n in driver.names]
                    home_xyz=geometry.forward(qhome)[:3]
                    set_pitch(args.travel_p)
                    set_integral(0 if args.travel_p==64 else 1)
                    stage('home',home_xyz,lambda record:run_recorded_home_trial(driver,geometry,zeros,home,supported=True,record=record,cancelled=is_cancelled,pulses_per_step=approach_pulses if args.fast_space else 1,batch_slow_joints=args.batch_base,fast_travel=args.fast_space,tick_s=approach_period,tracking_limit=approach_limit,native=args.native_moves,smooth_motion=True),home_goal=home)
                if chain_start and manual is None:raise ValueError('--start-above-tray needs planned coordinates: the arm above the tray would block the overhead photo')
                if manual is None:capture('home',wrist=False)   # 좌표를 받은 경우(plan_sort가 방금 찍고 계획 폴더에 저장함)에는 같은 사진을 다시 찍지 않는다(약 3~5초)
                import cv2
                if manual is not None:
                    located=dict(manual);result['selected_home_frame']=None
                else:
                    home_capture=out/'home_astra'
                    frames=json.loads((home_capture/'capture.json').read_text())['captures']
                    depth_quality_errors={'invalid center depth; refusing background substitution',
                                          'insufficient depth coverage',
                                          'depth spread indicates an edge or mixed surfaces'}
                    rejected=[]
                    for frame in frames:
                        try:
                            image,located=locate_image(home_capture/frame['rgb'])
                            result['selected_home_frame']=frame['id']
                            break
                        except ValueError as exc:
                            if not args.depth_measurement or str(exc) not in depth_quality_errors:raise
                            rejected.append(dict(frame=frame['id'],error=str(exc)))
                    else:
                        result['rejected_home_frames']=rejected;save()
                        raise ValueError('no synchronized HOME frame passed depth quality')
                    if rejected:result['rejected_home_frames']=rejected
                    cv2.imwrite(str(out/'target_annotation.png'),annotate(image,located))
                result['located']=located;save()
                print(json.dumps(dict(located=located),ensure_ascii=False,default=float),flush=True)
                if is_cancelled():raise InterruptedError('cancelled after locating the box')
                known=(*located['target_xy'],located['contact_z'])
                radius=math.hypot(*known[:2]);inner=max(.12,radius-.040)
                # Stay on the same ray, 4 cm short of the box, until the low approach.
                set_pitch(args.travel_p)
                set_integral(1)  # extended poses cannot hold at I=0; I=0 was only for the HOME return
                approach_via=(known[0]*inner/radius,known[1]*inner/radius,.120)
                if chain_start:
                    transit_z=max(CHAIN_TRANSIT_Z,fresh_xyz()[2])
                    for index in range(4):
                        cur=fresh_xyz()
                        if transit_z-cur[2]<.006:break
                        lift=min(.030,transit_z-cur[2])
                        try:hover('chain_raise'+str(index),(*cur[:2],cur[2]+lift),lift_m=lift)
                        except ValueError:
                            # 먼 칸에서 낮게 물러나 선 자리는 수직으로 더 올라갈 자세가 없을 수 있다(2026-10-26 Pi: r .262 z .097에서
                            # 'no direct-branch solution'). 움직이기 전에 거부됐으면 경로 거부와 같이 HOME을 거쳐 같은 물체를 다시 하게 한다
                            if math.dist(fresh_xyz(),cur)<.003:result['chain_refused']=True
                            raise
                    cur=fresh_xyz()
                    if cur[2]<transit_z-.008:
                        result['chain_refused']=True      # 멈춰 선 채 운반 높이에 못 미침: HOME을 거쳐 다시 한다
                        raise ValueError('could not rise to the transit height above the tray')
                    path=polar_path(tuple(cur),approach_via,.030)+[(*known[:2],.070)]
                    result['chain_transit']=dict(start_xyz=list(cur),transit_z=transit_z,points=len(path));save()
                    def transit(name,points):
                        # 한 번 이동 한도(1600펄스)를 넘으면 움직이기 전에 거부된다: 경로를 반으로 나눠 이어 간다(place의 move_path와 같다)
                        try:hover(name,points[-1],box_top_m=.040,via=points[:-1],via_pulses=approach_pulses)
                        except ValueError as exc:
                            if str(exc)!='hover change exceeds trial envelope' or len(points)<2:raise
                            mid=len(points)//2
                            transit(name+'a',points[:mid]);transit(name+'b',points[mid:])
                    try:transit('approach',path)
                    except ValueError:
                        # 경로 검사(도달·링크 높이·이동 한도)는 움직이기 전에 거부한다. 그 자리에 그대로 있으면 HOME을 거쳐 다시 하도록 알린다
                        if math.dist(fresh_xyz(),cur)<.003:result['chain_refused']=True
                        raise
                else:
                  try:hover('approach',(*known[:2],.070),box_top_m=.040,via=[approach_via],via_pulses=approach_pulses)
                  except ValueError as exc:
                    # 멀리 있는 물체: 한 번 이동 한도(1600펄스)를 넘으면 움직이기 전에 거부된다. 한도는 그대로 두고 경유점에서 둘로 나눠 이어 간다.
                    if str(exc)!='hover change exceeds trial envelope':raise
                    hover('approach',approach_via,box_top_m=.040)
                    hover('approach_b',(*known[:2],.070),box_top_m=.040)
                if args.hover_photo_only:
                    driver.sleep(1.0)
                    hover_xyz=fresh_xyz();capture('hover70',astra=False)
                    result['hover_photo']=dict(candidate_xyz=hover_xyz,box_target_xy=list(known[:2]),box_top_m=.040,nominal_cup_to_box_top_m=hover_xyz[2]-.040)
                    save()
                    print(json.dumps(dict(hover_photo=result['hover_photo']),ensure_ascii=False,default=float),flush=True)
                    if args.hover_center:
                        def center_capture(label):
                            if is_cancelled():raise InterruptedError('cancelled before hover center photo')
                            stationary_snapshot(driver,settle_attempts=3)
                            capture(label,astra=False)
                            return cv2.imread(str(out/(label+'_wrist.png')))
                        def center_move(delta_mm):
                            if is_cancelled():raise InterruptedError('cancelled before hover center move')
                            before=fresh_xyz();theta=math.atan2(known[1],known[0]);tangent=(-math.sin(theta),math.cos(theta))
                            target=(before[0]+tangent[0]*delta_mm/1000.,before[1]+tangent[1]*delta_mm/1000.,.070)
                            # probe prefix forces the existing slow1pulse path and guards.
                            hover('probe_center',target,box_top_m=.040)
                            result['hover_center_motion']=dict(before_candidate_xyz=before,after_candidate_xyz=fresh_xyz(),requested_tangent_mm=delta_mm)
                            save()
                        def center_save(receipt):
                            result['hover_center']=dict(receipt);save()
                        run_center_once(center_capture,center_move,center_save)
                    if args.hover_probe_tangent_mm:
                        if not math.isfinite(args.hover_probe_tangent_mm) or abs(args.hover_probe_tangent_mm)>6:raise ValueError('hover probe must be finite within +-6 mm')
                        if is_cancelled():raise InterruptedError('cancelled before hover probe')
                        theta=math.atan2(known[1],known[0]);tangent=(-math.sin(theta),math.cos(theta))
                        before=fresh_xyz();moved=(before[0]+tangent[0]*args.hover_probe_tangent_mm/1000.,before[1]+tangent[1]*args.hover_probe_tangent_mm/1000.)
                        hover('probe',(*moved,.070),box_top_m=.040)
                        driver.sleep(1.0)
                        after=fresh_xyz();capture('hover70_probe',astra=False)
                        result['hover_probe']=dict(requested_tangent_mm=args.hover_probe_tangent_mm,tangent_unit=tangent,before_xyz=before,after_xyz=after,
                                                   measured_tangent_move_mm=1000.*((after[0]-before[0])*tangent[0]+(after[1]-before[1])*tangent[1]),
                                                   measured_radial_move_mm=1000.*((after[0]-before[0])*math.cos(theta)+(after[1]-before[1])*math.sin(theta)))
                        save();print(json.dumps(dict(hover_probe=result['hover_probe']),ensure_ascii=False,default=float),flush=True)
                    raise InterruptedError('hover photo captured; stopped before descent by --hover-photo-only')
                set_pitch(args.pitch_p)
                if args.place_empty_run:
                    set_pitch(args.pitch_p)  # same contact gains the real run has when it reaches place (no object is suspended yet)
                    run_place_job(True)
                    result['motion_cycle_complete']=True
                    raise InterruptedError('empty place run complete; no suction was used')
                # Centre the cup on the box at contact height (no parallax) using the
                # wrist camera, against the human-confirmed successful contact image.
                goal_px=red_centroid(cv2.imread(str(BASE/'pickup_twoaxis_p64_trial/before_suction_wrist.png'))) if args.center_cup else None
                signs=[-1.0,1.0];history=[];result['wrist_centering']=history
                result['wrist_centering_enabled']=args.center_cup
                for attempt in range(4 if args.center_cup else 1):
                    tag='' if attempt==0 else str(attempt)
                    xyz=fresh_xyz();gap=xyz[2]-known[2]
                    if not .015<gap<guard_mode.length(.060):raise ValueError('unexpected descent start/reference gap')
                    # 접근(P32)에서 팔이 하중으로 목표보다 약 3~4mm 짧게 멈춘다. 하강은 수직이어야 하므로(가로 3mm 이내)
                    # 남은 가로 오차 중 2.8mm까지는 하강하면서 목표 xy 쪽으로 고친다(2026-10-16: "대체로 짧다").
                    dx,dy=known[0]-xyz[0],known[1]-xyz[1];norm=math.hypot(dx,dy);k=min(1.,.0028/norm) if norm>1e-9 else 0.
                    descent_xy=(xyz[0]+dx*k,xyz[1]+dy*k)
                    result.setdefault('descent_xy_correction_mm',[]).append([round(dx*k*1000,2),round(dy*k*1000,2),round(norm*1000,2)])
                    hover('descent'+tag,(*descent_xy,known[2]),measured_gap_m=gap)  # one move, slow only at the end
                    contact=fresh_xyz()
                    if abs(contact[2]-known[2])>guard_mode.length(.003):raise ValueError('contact candidate differs from confirmed plane')
                    result['contact_candidate_xyz']=contact
                    if not args.center_cup:
                        if args.evidence_photos or (place_job is not None and args.hold_check!='off'):capture('before_suction',astra=False)
                        save();break
                    capture('before_suction'+tag,astra=False)
                    centroid=red_centroid(cv2.imread(str(out/('before_suction'+tag+'_wrist.png'))))
                    err=offset_mm(centroid,goal_px)
                    applied=(signs[0]*err[0] if abs(err[0])>2.5 else 0.,signs[1]*err[1] if abs(err[1])>2.5 else 0.)
                    direction_changed=False
                    if history:
                        # An axis that was corrected but got worse had the wrong sign.
                        for k in (0,1):
                            if history[-1]['applied_mm'][k] and abs(err[k])>abs(history[-1]['error_mm'][k])+1.0:
                                signs[k]=-signs[k];direction_changed=True
                        applied=(signs[0]*err[0] if abs(err[0])>2.5 else 0.,signs[1]*err[1] if abs(err[1])>2.5 else 0.)
                    history.append(dict(attempt=attempt,centroid_px=centroid[:2],error_mm=err,applied_mm=applied,contact_xy=contact[:2],signs=list(signs)));save()
                    print(json.dumps(dict(centering=history[-1]),ensure_ascii=False,default=float),flush=True)
                    if max(abs(err[0]),abs(err[1]))<=2.5:break
                    if not recenter_allowed(attempt,direction_changed):
                        if max(abs(err[0]),abs(err[1]))>5*guard_mode.scale():raise ValueError('cup could not be centred on the box (>5 mm)')
                        break
                    if is_cancelled():raise InterruptedError('cancelled while centring')
                    delta=correction_xy(contact[:2],err,tuple(signs))
                    new=(contact[0]+delta[0],contact[1]+delta[1])
                    hover('rise_a'+str(attempt),(*contact[:2],contact[2]+.030),lift_m=.030)
                    xyz=fresh_xyz();hover('rise_b'+str(attempt),(*xyz[:2],xyz[2]+.030),lift_m=.030)
                    hover('recenter'+str(attempt),(*new,.090),box_top_m=.040)
                    hover('redescend70_'+str(attempt),(*new,.070),box_top_m=.040)
                    known=(*new,known[2])
                if is_cancelled():raise InterruptedError('cancelled before suction')
                if args.contact_photo_only:
                    # 보정용: 접촉 높이(시차 없음)에서 손목 사진 한 장만 찍고, 흡착 없이 20mm 올라가 멈춘다.
                    capture('before_suction',astra=False);save()
                    hover('lift20',(*contact[:2],contact[2]+.020),lift_m=.020)
                    raise InterruptedError('contact photo captured; no suction by --contact-photo-only')
                if not args.no_contact_rgb_log:
                    try:
                        contact_rgb_observation()  # observation verdict never gates suction
                    except InterruptedError:
                        raise
                    except Exception as exc:
                        result['contact_rgb']=unknown_contact_rgb('observation error: '+str(exc))
                if is_cancelled():raise InterruptedError('cancelled before suction')
                command('suction');driver.sleep(.8)
                # 흡착이 붙으며 컵이 눌려 팔이 목표에서 6~7펄스 밀려 서면 lift20 출발 검사("hold is not settled")가 막는다(2026-10-19 실물).
                # 움직임이 끝난 뒤 지금 자리를 새 목표로 다시 인계한다(이동 없음, 5mm·가드는 그대로).
                driver=ServoDriver(bus,profile);driver.hold_existing_torque(supported=True,attempts=4,settle_s=.4,takeover_attempts=4)
                lift_place_plan=None
                # Plan from the measured pickup pose before lifting. Only an
                # unchanged vertical raise/tilt prefix can become one stream.
                if place_job is not None and args.held_rate and not args.pick_tilt_deg and args.hold_check=='off':
                    lift_start=fresh_xyz()
                    if math.hypot(lift_start[0]-contact[0],lift_start[1]-contact[1])>guard_mode.length(.003):
                        raise ValueError('post-pick lift start moved more than 3 mm sideways')
                    try:
                        lift_plan=build_plan(place_job,place_model,place_catalog,(*lift_start[:2],.070),args.place_max_tilt_deg,args.pick_tilt_deg)
                        entry=place_catalog['objects'].get(place_job['label'])
                        # Keep the existing held-object clearance before adding
                        # tilt; distant/unknown geometry uses the legacy path.
                        if entry is None:
                            transition=None
                        else:
                            obstacle_top=max((o['height_mm'] for o in place_job['obstacles']),default=0.)
                            tilt_start_z=max(.070,(obstacle_top+entry['height_mm']+place_catalog['cup_contact_offset_mm']+OBSTACLE_MARGIN_MM)/1000.)
                            transition=build_lift_transition(lift_plan,lift_start,tilt_start_z=tilt_start_z)
                    except ValueError as exc:
                        transition=None
                        result['lift_to_carry_fallback']=str(exc)
                    if transition is not None:
                        try:
                            stage('lift_to_carry',transition['target'],lambda record:run_pick_to_carry_trial(
                                driver,geometry,zeros,transition,supported=True,rate=args.held_rate,
                                min_contact_z=args.min_contact_z_mm/1000.,tracking_limit=32,record=record,cancelled=is_cancelled))
                        except LiftPathRefused as exc:
                            # This exception is emitted only before execution.
                            result['lift_to_carry_fallback']=str(exc)
                        else:
                            lift_place_plan=transition['remaining_plan']
                            result['lift_to_carry']=dict(target=transition['target'],pitch_rad=transition['pitch_rad'],
                                tilt_start_z=transition['tilt_start_z'],waypoints=transition['waypoints'],intermediate_holds=0,source_plan=lift_plan,physically_validated=False)
                    save()
                if lift_place_plan is None:
                    post_pick_rise(hover,fresh_xyz,contact,placing=place_job is not None and bool(args.held_rate))
                result['time_to_lift_s']=time.monotonic()-cycle_start
                lifted=fresh_xyz();result['lifted_candidate_xyz']=lifted;result['candidate_lift_m']=lifted[2]-contact[2]
                if args.evidence_photos:capture('lifted')
                save()
                if place_job is not None and args.hold_check!='off':
                    capture('lifted_check',astra=False)
                    verdict=check_hold(cv2.imread(str(out/'before_suction_wrist.png')),cv2.imread(str(out/'lifted_check_wrist.png')))
                    result['hold_check']=dict(verdict,mode=args.hold_check);save()
                    print(json.dumps(dict(hold_check=dict(held=verdict['held'],reason=verdict['reason'],mode=args.hold_check)),ensure_ascii=False,default=float),flush=True)
                    if hold_decision(verdict['held'],args.hold_check)=='abort':
                        command('vent');driver.sleep(.3);command('stop')   # nothing is held: release the suction, do not carry
                        result['pick_failed']='object not held after lift; place skipped'
                        raise InterruptedError(result['pick_failed'])
                if place_job is not None:
                    run_place_job(False,prepared=lift_place_plan)
                else:
                    current=fresh_xyz();return_gap=current[2]-contact[2]
                    hover('return',contact,measured_gap_m=return_gap)
                    result['release']=release_after_return(driver,geometry,zeros,contact,pump)
                    result['pump_commands'].extend([dict(command='vent_return',pressure_verified=False),dict(command='stop_return',pressure_verified=False)])
                if args.evidence_photos:capture('released')
                result['motion_cycle_complete']=True
        except BaseException as exc:
            result['error']=str(exc);result['error_notes']=getattr(exc,'__notes__',[])
        finally:
            if driver is not None:
                try:driver.hold(attempts=4,settle_s=.4);result['final_hold_confirmed']=driver.hold_confirmed   # 멀리 뻗은 자세는 3~6펄스 밀려 선다: 보상 hold(2026-10-19 칸 2에서 최종 hold 실패로 HOME 복귀가 건너뛰어졌다)
                except BaseException as exc:result['final_hold_error']=str(exc)
            if pump is not None:
                try:pump.close();result['pump_stop_sent_on_close']=True
                except BaseException as exc:result['pump_close_error']=str(exc)
            if recorder[0] is not None:
                try:recorder[0].close()
                except BaseException as exc:result['recorder_error']=str(exc)
            result['total_duration_s']=time.monotonic()-cycle_start
            result['restoration_verified']=bool(gains and gains.restoration_verified)
            try:result['final_snapshot']=bus.read()
            except BaseException as exc:result['final_read_error']=str(exc)
            if not args.no_contact_rgb_log:
                result['contact_rgb_outcome']=outcome_context(result)
                try:
                    (out/'contact_rgb_outcome.json').write_text(json.dumps(result['contact_rgb_outcome'],ensure_ascii=False,indent=2,allow_nan=False))
                except Exception as exc:result['contact_rgb_outcome_record_error']=str(exc)
            save()
    home_return=None
    if place_job is not None and (should_return_home(result,enabled=not (args.no_return_home or args.skip_home_after_place)) or should_return_home_after_failed_pick(result,enabled=not args.no_return_home) or should_return_home_after_release_failure(result,enabled=not args.no_return_home)):
        # The bus, pump and temporary gains are closed/restored by now; the existing guarded return_home.py takes over the held arm.
        home_return=run_home_return(ROOT,args.home_rate,out/'home_return')
        result['home_return']=home_return;save()
    print(json.dumps(dict(output=str(out),motion_cycle_complete=result['motion_cycle_complete'],physical_pick_success=None,pick_failed=result.get('pick_failed'),error=result.get('error'),restoration_verified=result['restoration_verified'],home_return_complete=None if home_return is None else home_return['ok'],home_return_error=None if home_return is None else home_return['error']),ensure_ascii=False))
    if placed_despite_retreat_error(result,home_return):
        # 해제까지 끝난 뒤 후퇴만 실패했고 팔이 HOME에 돌아왔다: 물체는 놓았다. 오류는 그대로 남기고 성공(0)으로 끝낸다.
        result['placed_despite_retreat_error']=result.get('error');save()
        print(json.dumps(dict(placed_despite_retreat_error=result['placed_despite_retreat_error']),ensure_ascii=False),flush=True)
        return 0
    if result.get('chain_refused') and result['restoration_verified'] and not result['motion_cycle_complete']:return CHAIN_REFUSED_EXIT_CODE
    if result.get('pick_failed') and result['restoration_verified'] and (home_return is None or home_return['ok']):return NOT_HELD_EXIT_CODE
    return 0 if result['motion_cycle_complete'] and result['restoration_verified'] and (home_return is None or home_return['ok']) else 1


if __name__=='__main__':raise SystemExit(main())
