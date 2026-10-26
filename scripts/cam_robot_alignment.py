"""Camera recalibration, one point in one run (robot as ruler, 2026-10-20 procedure).

1. pick_auto.py --hover-photo-only: cup hovers over (x, y); the arm FK cup centre is read back.
2. You place the box under the cup and press Enter.
3. return_home.py (150 pulse/s): the arm leaves the camera view.
4. Only after a complete HOME return: photograph the box and append (pixel, arm FK xy)
   to 카메라 보정/재보정_점.jsonl (image 카메라 보정/재보정_<label>.png).
--fit prints the pose-only camera fit afterwards (offline); --fit-only does just that. No pump commands.
"""
import argparse,json,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
OUT=ROOT/'카메라 보정';POINTS=OUT/'재보정_점.jsonl'
PY=sys.executable
HUE_MAX=6      # 상자 색 판정의 위쪽 끝(기본 4). Pi에서 조명이 조금 달라 정육면체 색상이 6이 되면 기본값으로는 못 찾았다(2026-10-26 p2)


def last_json(text,key):
    for line in reversed(text.splitlines()):
        line=line.strip()
        if line.startswith('{') and key in line:
            try:return json.loads(line)
            except ValueError:continue
    return None


def run(args):
    r=subprocess.run([PY,*map(str,args)],cwd=ROOT,text=True,capture_output=True)
    return r.returncode,r.stdout+r.stderr


def hover(x_mm,y_mm,z_mm,label='x'):
    code,text=run(['scripts/pick_auto.py','--manual-target-mm',x_mm,y_mm,z_mm,'--hover-photo-only','--execute','--supported'])
    data=last_json(text,'hover_photo')
    if not data:
        print(text[-1500:])
        # 움직이던 중 실패하면 팔이 HOME이 아니라 물체 위에 뻗은 채 토크가 켜져 서 있다(2026-10-26 p7: 자세 확인 실패). 그대로 두지 않는다
        print('호버 실패(exit %d). 팔이 HOME이 아니라 책상 위에 떠 있을 수 있습니다.'%code,flush=True)
        if input('팔을 받치고 Enter = HOME으로 복귀 / n = 그대로 두기: ').strip().lower()!='n':
            return_home(label);print('HOME 복귀 완료',flush=True)
        raise SystemExit('호버 실패: 기록 안 함')
    return data['hover_photo']['candidate_xyz']


def return_home(label):
    out=ROOT/'output/home_returns'/('calib_%s_%s'%(label,time.strftime('%H%M%S')))
    code,text=run(['scripts/return_home.py','--execute','--supported','--travel-rate',150,'--output',out])
    data=last_json(text,'home_return_complete')
    if code!=0 or not data or not data.get('home_return_complete'):
        print(text[-1500:]);raise SystemExit('HOME 복귀 실패: 팔이 사진을 가릴 수 있어 촬영 안 함')


def record(label,x,y):
    import cv2
    from camera.astra_calibration_camera import AstraCalibrationCamera
    from perception.box_target import detect_box
    OUT.mkdir(exist_ok=True)
    cam=AstraCalibrationCamera(fixed_exposure=False)
    try:
        for _ in range(35):bgr,depth,_s=cam.read()
        bgr=bgr.copy()
    finally:cam.close()
    cv2.imwrite(str(OUT/('재보정_'+label+'.png')),bgr)
    box=detect_box(bgr,hue_max=HUE_MAX)
    row=dict(label=label,pixel=[float(v) for v in box['pixel']],bbox=[int(v) for v in box['bbox']],area_px=int(box['area_px']),
             arm_fk_xy_m=[x,y],time=time.strftime('%Y-%m-%dT%H:%M:%S'))
    with POINTS.open('a') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
    return row


def labels():
    return [json.loads(l)['label'] for l in POINTS.open()] if POINTS.exists() else []


def fit_camera():
    """Pose-only fit (R, t; K fixed) of the camera from the recorded (pixel, arm FK xy) pairs on the box-top plane.
    Offline, prints residuals and leave-one-out error. Does not write the camera transform file."""
    import math
    import numpy as np
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation as Rot
    from robot.robot_frame import BARE_TABLE_PLANE_Z,BASE_TABLE_PLANE_Z
    plane_z=-0.007+(BARE_TABLE_PLANE_Z-BASE_TABLE_PLANE_Z)          # same plane as src/perception/box_target.py
    folder=ROOT/'output/ik_resume_20261012/multipose_review'
    start=folder/'camera_transform_candidate.before_forward38mm_20261020.json'
    start=start if start.exists() else folder/'camera_transform_candidate.json'
    base=json.load(open(start))['model'];K=np.array(base['K']);R0=np.array(base['R']);t0=np.array(base['t'])
    pts=[json.loads(l) for l in open(POINTS)]
    if len(pts)<4:raise SystemExit('점이 %d개뿐: 4개 이상 필요'%len(pts))
    P=np.array([q['pixel'] for q in pts]);W=np.array([q['arm_fk_xy_m'] for q in pts]);names=[q['label'] for q in pts]
    def hit(R,t,px):
        fx,fy,cx,cy=K;d=R.T@np.array([(px[0]-cx)/fx,(px[1]-cy)/fy,1.]);o=-R.T@t
        return (o+(plane_z-o[2])/d[2]*d)[:2]
    def solve(idx,x0):
        f=lambda x:np.concatenate([(hit(Rot.from_rotvec(x[:3]).as_matrix(),x[3:6],P[i])-W[i])*1000 for i in idx])
        return least_squares(f,x0).x
    def err(x,i):return (hit(Rot.from_rotvec(x[:3]).as_matrix(),x[3:6],P[i])-W[i])*1000
    idx=list(range(len(P)));x=solve(idx,np.r_[Rot.from_matrix(R0).as_rotvec(),t0])
    fit=np.array([err(x,i) for i in idx])
    loo=np.array([err(solve([j for j in idx if j!=k],x),k) for k in idx])
    print('시작 모델:',start.name)
    print('점별 맞춤 오차 mm (x,y) / 교차검증 mm (x,y):')
    for n,e,l in zip(names,fit,loo):print(' ',n,e.round(1).tolist(),l.round(1).tolist())
    rms=lambda e:math.sqrt((e**2).sum()/len(e))
    print('RMS 맞춤 %.2f mm (최대 %.2f) | 교차검증 %.2f mm (최대 %.2f)'%(rms(fit),np.abs(fit).max(),rms(loo),np.abs(loo).max()))
    R=Rot.from_rotvec(x[:3]).as_matrix();t=x[3:6]
    print('카메라 위치(m):',(-R.T@t).round(4).tolist(),'광축:',(R.T@np.array([0,0,1.])).round(3).tolist())
    print('R =',R.round(6).tolist());print('t =',t.round(6).tolist())


def main():
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('label',nargs='?',help='point name, e.g. p9')
    p.add_argument('x_mm',nargs='?',type=float);p.add_argument('y_mm',nargs='?',type=float)
    p.add_argument('--contact-z-mm',type=float,default=36.5,help='box-top contact height given to pick_auto (default 36.5)')
    p.add_argument('--replace',action='store_true',help='drop an existing point with the same label first')
    p.add_argument('--fit',action='store_true',help='print the camera fit after recording')
    p.add_argument('--fit-only',action='store_true',help='only print the camera fit (no hardware)')
    a=p.parse_args()
    if not a.fit_only:
        if a.label is None or a.x_mm is None or a.y_mm is None:p.error('label x_mm y_mm required (or --fit-only)')
        if a.label in labels():
            if not a.replace:p.error('label %s already recorded; use --replace or another name'%a.label)
        x,y,z=hover(a.x_mm,a.y_mm,a.contact_z_mm,a.label)
        print('호버 완료: 컵 중심 FK x=%.1f y=%.1f z=%.1f mm'%(x*1000,y*1000,z*1000),flush=True)
        answer=input('컵 바로 아래에 상자를 놓고 Enter (q = 기록 없이 HOME 복귀): ').strip().lower()
        return_home(a.label);print('HOME 복귀 완료',flush=True)
        if answer=='q':raise SystemExit('취소: 기록 안 함')
        if a.replace and a.label in labels():
            rows=[l for l in POINTS.open() if json.loads(l)['label']!=a.label];POINTS.write_text(''.join(rows))
        from perception.box_target import BoxNotFound
        while True:
            try:row=record(a.label,x,y);break
            except BoxNotFound as exc:
                # 팔은 이미 HOME이고 상자는 컵 아래 그 자리에 있다: 같은 자리에서 다시 찍을 수 있다(점을 처음부터 다시 하지 않는다)
                print('상자를 찾지 못했습니다(%s). 사진: 카메라 보정/재보정_%s.png'%(exc,a.label))
                if input('상자를 그 자리에 그대로 두고 Enter = 다시 촬영 / q = 기록 없이 끝내기: ').strip().lower()=='q':
                    raise SystemExit('취소: 기록 안 함')
        print('기록:',json.dumps(row,ensure_ascii=False));print('현재 점 %d개: %s'%(len(labels()),' '.join(labels())))
    if a.fit or a.fit_only:fit_camera()


if __name__=='__main__':main()
