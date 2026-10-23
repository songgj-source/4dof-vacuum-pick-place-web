"""Passive RGB contact-view estimate. It has no control authority or hardware I/O.

Reference appearance/position similarity is not proof of physical contact,
airtight sealing, vacuum pressure or successful pickup.
"""
import cv2
import numpy as np
import math
import subprocess

from camera.wrist_center import red_centroid


def camera_worker(command,timeout_s=3.):
    if not math.isfinite(timeout_s) or not 0<timeout_s<=3:
        raise ValueError('camera observation budget must be at most3seconds')
    return subprocess.run(command,check=True,timeout=timeout_s,capture_output=True,text=True)


def unknown(reason):
    return dict(verdict='unknown',display='판정 불가',reason=reason,
                mode='record_only',can_block_motion=False,
                physical_contact_confirmed=None,vacuum_seal_confirmed=None,
                confidence_calibrated=False,visible_gap_verified=False,
                scope='red box,640x480 wrist view,historical contact reference')


def _features(frame):
    if not isinstance(frame,np.ndarray) or frame.shape!=(480,640,3) or frame.dtype!=np.uint8:
        raise ValueError('640x480 BGR frame required')
    x,y,area=red_centroid(frame)
    # Cup is rigidly mounted relative to the camera; compare its fixed ROI.
    roi=cv2.cvtColor(frame[230:420,260:460],cv2.COLOR_BGR2GRAY)
    sharpness=float(cv2.Laplacian(roi,cv2.CV_64F).var())
    if sharpness<15:raise ValueError('cup ROI blurred or featureless')
    edge=cv2.Canny(roi,50,120).astype(float).ravel()
    norm=float(np.linalg.norm(edge))
    if norm==0:raise ValueError('cup appearance unavailable')
    return np.array([x,y],float),float(area),edge/norm,sharpness


def observe_frames(frames,reference):
    result=unknown('insufficient usable RGB evidence')
    result.update(frame_count=len(frames),heuristic_version='contact_rgb_v1')
    if len(frames)<5 or len(frames)>10:
        return result
    try:
        center,area,edges,_=_features(reference)
        observations=[]
        for frame in frames:
            position,current_area,current_edges,sharpness=_features(frame)
            observations.append(dict(center_px=position.tolist(),
                                     reference_offset_px=(position-center).tolist(),
                                     area_ratio=current_area/area,
                                     cup_edge_similarity=float(np.dot(current_edges,edges)),
                                     sharpness=sharpness))
    except Exception as exc:
        result['reason']=str(exc)
        return result
    positions=np.asarray([v['center_px'] for v in observations])
    result.update(frames=observations,reference_center_px=center.tolist(),
                  median_offset_px=np.median([v['reference_offset_px'] for v in observations],axis=0).tolist(),
                  center_span_px=np.ptp(positions,axis=0).tolist())
    if max(np.ptp(positions,axis=0))>3:
        result['reason']='RGB center varies across frames'
        return result
    offset=np.abs(result['median_offset_px'])
    if max(offset)>15:
        result.update(verdict='alignment_or_gap_suspected',display='어긋남·틈 의심',
                      reason='object differs from contact reference position; height and lateral position are not separable')
        return result
    votes=sum(.8<=v['area_ratio']<=1.2 and v['cup_edge_similarity']>=.65 for v in observations)
    result['reference_consistency_votes']=votes
    if votes<len(frames)-1:
        result['reason']='cup/object appearance does not reliably match contact reference'
        return result
    result.update(verdict='contact_estimate',display='접촉 추정',
                  reason='stable object alignment and cup appearance consistent with contact reference; not proof of contact')
    return result


def record_observation(capture,reference,save):
    """Observation failure never vetoes the caller's existing action.

    Operator cancellation propagates; it is unrelated to RGB verdicts.
    """
    try:
        result=observe_frames(capture(),reference)
    except InterruptedError:
        raise
    except Exception as exc:
        result=unknown(str(exc))
    try:
        save(result)
    except Exception as exc:
        result['record_error']=str(exc)
    return result


def outcome_context(run):
    suction=[p for p in run.get('pump_commands',[]) if p.get('command')=='suction']
    ack=any(bool((p.get('controller_receipt') or {}).get('controller_acknowledged')) for p in suction)
    return dict(suction_command_recorded=bool(suction),suction_controller_acknowledged=ack,
                pressure_verified=False,vacuum_pressure_kpa=None,
                lift_motion_complete=any(p.get('phase') in ('lift20','lift_to_clear') for p in run.get('phases',[])),
                candidate_lift_m=run.get('candidate_lift_m'),
                motion_cycle_complete=run.get('motion_cycle_complete',False),
                actual_contact_observed=None,actual_lift_observed=None,
                physical_pick_success=run.get('physical_pick_success'),
                annotation_source=None,run_error=run.get('error'),
                note='command receipt and candidate FK lift do not prove contact,vacuum pressure or actual pickup')
