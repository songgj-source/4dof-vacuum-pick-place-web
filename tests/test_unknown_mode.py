import sys,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'scripts'))
from perception.unknown_objects import locate_unknowns,StableUnknown,unknown_catalog
from sorting.sort_job import build_job
from sorting.place_job import load_place_job,build_plan
from perception.object_targets import load_catalog
from tray.tray_sorting import choose_slot
LOC=SimpleNamespace(R=np.diag([1.,-1.,-1.]),t=np.array([0.,0.,.5]),fx=500.,fy=500.,cx=50.,cy=235.,offset_xy=np.zeros(2))
class Tests(unittest.TestCase):
 def scene(self):
  rgb=np.full((480,640,3),160,np.uint8);base=np.full((480,640),500,np.uint16);depth=base.copy();depth[210:260,240:300]=465;rgb[210:260,240:300]=(200,70,20);return rgb,depth,base
 def detect(self,rgb,depth,base):
  with patch('perception.unknown_objects.reachable',return_value=True):return locate_unknowns(rgb,depth,base,ROOT,locator=LOC,model={})
 def test_new_object_ignores_colour_name(self):
  rgb,depth,base=self.scene();found=self.detect(rgb,depth,base)
  self.assertEqual(len(found),1);t=found[0];self.assertEqual(t['label'],'unknown');self.assertTrue(t['new_object']);self.assertTrue(t['pickable'],t['problems']);self.assertAlmostEqual(t['object_geometry']['height_mm'],35,delta=1);self.assertFalse(t['suction_verified']);self.assertFalse(t['physical_accuracy_verified'])
 def test_existing_object_not_selected(self):
  rgb,depth,base=self.scene();found=self.detect(rgb,depth,depth.copy());self.assertEqual(len(found),1);self.assertFalse(found[0]['new_object'])
 def test_two_new_objects_refused(self):
  rgb,depth,base=self.scene();depth[280:320,150:190]=470;found=self.detect(rgb,depth,base)
  with self.assertRaisesRegex(ValueError,'more than one unknown'):self.job(found)
 def test_tall_object_is_not_pickable(self):
  rgb,depth,base=self.scene();depth[210:260,240:300]=430;t=self.detect(rgb,depth,base)[0];self.assertFalse(t['pickable'])
 def test_missing_depth_at_contact_refused(self):
  rgb,depth,base=self.scene();depth[220:245,250:280]=0
  found=self.detect(rgb,depth,base);self.assertTrue(not found or all(not t['pickable'] for t in found))
 def test_unflat_top_refused(self):
  rgb,depth,base=self.scene();depth[210:260,240:300]=np.tile(np.arange(60)%15,(50,1))+458
  found=self.detect(rgb,depth,base);self.assertTrue(not found or all(not t['pickable'] for t in found))
 def test_changed_background_refused(self):
  rgb,depth,base=self.scene();depth[:]=490
  with self.assertRaisesRegex(ValueError,'배경|장면'):self.detect(rgb,depth,base)
 def test_stability_resets_on_motion_and_ambiguity(self):
  rgb,depth,base=self.scene();t=self.detect(rgb,depth,base)[0];state=StableUnknown(samples=3)
  self.assertFalse(state.update([t]));self.assertFalse(state.update([t]));self.assertTrue(state.update([t]));self.assertFalse(state.update([t,t]));self.assertFalse(state.update([t]))
 def job(self,targets):
  tray={'found':True,'slots':{3:{'x_mm':180.,'y_mm':30.,'floor_height_mm':10.}},'rim_height_mm':40.}
  return build_job(targets,tray,load_catalog(ROOT/'config/object_catalog.json'),'unknown',3,acknowledge_unverified_suction=True)
 def test_geometry_roundtrip(self):
  rgb,depth,base=self.scene();target=self.detect(rgb,depth,base)[0];result=self.job([target]);self.assertEqual(result['job']['label'],'unknown');self.assertIn('object_geometry',result['job'])
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'place.json';p.write_text(json.dumps(result['job']));job=load_place_job(p)
  catalog=unknown_catalog(load_catalog(ROOT/'config/object_catalog.json'),job['object_geometry']);self.assertEqual(catalog['cup_contact_offset_mm'],0);self.assertEqual(catalog['objects']['unknown']['height_mm'],35.)
  with patch('sorting.place_job.plan_place',return_value={}) as planner:
   build_plan(job,{},catalog,(.2,.03,.055))
   self.assertEqual(planner.call_args.kwargs['held_height_mm'],38.)
 def test_unknown_requires_geometry(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'place.json';p.write_text(json.dumps({'label':'unknown','slot_number':3,'slot':{'x_mm':180,'y_mm':30,'drop_z_mm':60},'rim_height_mm':40}))
   with self.assertRaisesRegex(ValueError,'geometry|크기'):load_place_job(p)
 def test_known_catalog_unchanged(self):
  catalog=load_catalog(ROOT/'config/object_catalog.json');before=json.dumps(catalog,sort_keys=True)
  rgb,depth,base=self.scene();geometry=self.detect(rgb,depth,base)[0]['object_geometry'];unknown_catalog(catalog,geometry)
  self.assertEqual(json.dumps(catalog,sort_keys=True),before);self.assertEqual(choose_slot('red_cube'),2)


 def test_merged_with_existing_object_refused(self):
  rgb,depth,base=self.scene();base[210:260,240:300]=480
  targets=self.detect(rgb,depth,base);self.assertTrue(targets[0]['new_object']);self.assertFalse(targets[0]['pickable'])
 def test_partial_object_refused(self):
  rgb,depth,base=self.scene();depth[:]=500;depth[210:260,410:470]=465
  found=self.detect(rgb,depth,base);self.assertTrue(not found or all(not t['pickable'] for t in found))
 def test_invalid_depth_units_refused(self):
  rgb,depth,base=self.scene()
  with self.assertRaisesRegex(ValueError,'uint16'):self.detect(rgb,depth.astype(float)/1000,base)
 def test_stability_resets_for_moving_object(self):
  import copy
  rgb,depth,base=self.scene();t=self.detect(rgb,depth,base)[0];state=StableUnknown(samples=3)
  self.assertFalse(state.update([t]));self.assertFalse(state.update([t]));moving=copy.deepcopy(t);moving['target_xy'][0]+=.01
  self.assertFalse(state.update([moving]));self.assertFalse(state.update([moving]));self.assertTrue(state.update([moving]))
 def test_invalid_geometry_refused(self):
  from perception.unknown_objects import validate_geometry
  import copy
  rgb,depth,base=self.scene();geometry=self.detect(rgb,depth,base)[0]['object_geometry']
  for key,value in [('height_mm',float('nan')),('height_mm',True),('footprint_mm',[40,-1]),('top_flat',False),('height_uncertainty_mm',10)]:
   with self.subTest(key=key,value=value):
    bad=copy.deepcopy(geometry);bad[key]=value
    with self.assertRaises(ValueError):validate_geometry(bad)
 def test_known_job_output_preserved(self):
  target=dict(label='red_cube',target_xy=[.217,.085],contact_z=.034,radius_m=.23,pickable=True,problems=[],suction_verified=True,pick_tilt_deg=0.)
  tray=dict(found=True,slots={2:dict(x_mm=220.,y_mm=70.,floor_height_mm=10.)},rim_height_mm=40.)
  catalog=load_catalog(ROOT/'config/object_catalog.json');result=build_job([target],tray,catalog,'red_cube',2)
  self.assertEqual(result['manual_target_mm'],[217.,85.,34.]);self.assertEqual(result['job']['slot']['drop_z_mm'],71.5)
  self.assertNotIn('object_geometry',result['job']);self.assertEqual(result['job']['label'],'red_cube')

class WatchTests(unittest.TestCase):
 scene=Tests.scene
 detect=Tests.detect
 def test_waits_for_three_distinct_stable_observations(self):
  from pick_unknown import wait_for_new_object
  rgb,depth,base=self.scene();target=self.detect(rgb,depth,base)[0];tick=[0];messages=[]
  def read():
   tick[0]+=1;return rgb,depth,dict(color_frame_index=tick[0],depth_frame_index=tick[0],color_us=tick[0]*1000000,depth_us=tick[0]*1000000)
  found=wait_for_new_object(read,base,10,detector=lambda *a:[target],now=lambda:tick[0]*.6,notify=lambda **kw:messages.append(kw))
  self.assertEqual(tick[0],3);self.assertEqual(found[2]['label'],'unknown');self.assertEqual(messages[-1]['phase'],'unknown_detected')
 def test_timeout_never_dispatches(self):
  from pick_unknown import wait_for_new_object
  rgb,depth,base=self.scene();tick=[0]
  def read():
   tick[0]+=1;return rgb,depth,dict(color_frame_index=tick[0],depth_frame_index=tick[0],color_us=0,depth_us=0)
  with self.assertRaises(TimeoutError):wait_for_new_object(read,base,2,detector=lambda *a:[],now=lambda:tick[0]*.6,notify=lambda **kw:None)
 def test_unsynchronized_observations_do_not_select(self):
  from pick_unknown import wait_for_new_object
  rgb,depth,base=self.scene();target=self.detect(rgb,depth,base)[0];tick=[0]
  def read():
   tick[0]+=1;return rgb,depth,dict(color_frame_index=tick[0],depth_frame_index=tick[0],color_us=50000,depth_us=0)
  with self.assertRaises(TimeoutError):wait_for_new_object(read,base,2,detector=lambda *a:[target],now=lambda:tick[0]*.6,notify=lambda **kw:None)
 def test_cli_default_plans_and_closes_camera_first(self):
  import pick_unknown
  rgb,depth,base=self.scene();target=self.detect(rgb,depth,base)[0]
  class Camera:
   closed=False
   def __init__(self,**kw):pass
   def read_synchronized(self):return rgb,base,{}
   def close(self):Camera.closed=True
  with tempfile.TemporaryDirectory() as d,patch.object(pick_unknown,'ROOT',Path(d)),patch('camera.astra_calibration_camera.AstraCalibrationCamera',Camera),patch.object(pick_unknown,'wait_for_new_object',return_value=(rgb,depth,target)),patch.object(pick_unknown,'emit'):
   def child(cmd,**kw):
    self.assertTrue(Camera.closed);self.assertNotIn('--execute',cmd);self.assertIn('unknown',cmd);self.assertIn('--unknown-baseline',cmd);return SimpleNamespace(returncode=0)
   with patch.object(pick_unknown,'run_until_clean_exit',side_effect=lambda cmd,cwd:child(cmd,cwd=cwd).returncode):self.assertEqual(pick_unknown.main(['--acknowledge-unverified-suction']),0)
 def test_cli_execute_requires_support_before_camera(self):
  import pick_unknown
  with patch('camera.astra_calibration_camera.AstraCalibrationCamera') as camera,patch('sys.stderr'):
   with self.assertRaises(SystemExit):pick_unknown.main(['--execute'])
   camera.assert_not_called()



class ReviewRegressionTests(unittest.TestCase):
 scene=Tests.scene
 detect=Tests.detect
 job=Tests.job
 def test_offcentre_radius_covers_entire_object(self):
  from perception.object_targets import footprint_radius_mm,_project,DEPTH_MINUS_TOP_CENTER_PX
  from robot.robot_frame import BARE_TABLE_PLANE_Z
  rgb,depth,base=self.scene();depth[:]=500;depth[210:250,210:305]=465
  t=self.detect(rgb,depth,base)[0];radius=footprint_radius_mm(t['object_geometry'])
  plane=BARE_TABLE_PLANE_Z+t['object_geometry']['height_mm']/1000.
  x,y,w,h=t['bbox_px'];corners=np.array([_project(LOC,np.array(q,float)-DEPTH_MINUS_TOP_CENTER_PX,plane)+LOC.offset_xy for q in [(x,y),(x+w,y),(x,y+h),(x+w,y+h)]])*1000
  required=np.linalg.norm(corners-np.array(t['target_xy'])*1000,axis=1).max()
  self.assertGreaterEqual(radius,required)
 def test_moving_baseline_object_is_not_new_addition(self):
  rgb,depth,base=self.scene();base[300:350,100:160]=465
  with self.assertRaisesRegex(ValueError,'이동|제거'):self.detect(rgb,depth,base)
 def test_legacy_unknowns_still_skipped(self):
  from tray.tray_sorting import plan_sorting
  from tray.tray_detect import SLOT_LABEL
  self.assertEqual(SLOT_LABEL[3],'tape_measure')
  result=plan_sorting([{'label':'unknown'}]);self.assertEqual(result['assignments'],[])
 def test_unknown_command_cannot_retry_saved_scene(self):
  import run_sort
  with patch.object(run_sort.subprocess,'run') as child,patch.object(run_sort,'run_until_clean_exit') as pick_child,patch('sys.stderr'):
   with self.assertRaises(SystemExit):run_sort.main(['--command','unknown 3번','--attempts','3'])
   child.assert_not_called();pick_child.assert_not_called()
 def test_unknown_legacy_xy_correction_not_applied(self):
  rgb,depth,base=self.scene();targets=self.detect(rgb,depth,base)
  with patch('perception.pick_correction.apply',side_effect=AssertionError('old correction')):result=self.job(targets)
  self.assertEqual(result['pick_tangential_correction_mm'],0.)

class CapacityTests(unittest.TestCase):
 def test_fits_observed_floor(self):
  from tray.tray_capacity import require_slot_capacity
  depth=np.full((480,640),500,np.uint16);depth[175:275,160:310]=490
  slot=dict(x_mm=180.,y_mm=30.,floor_height_mm=10.)
  clearance=require_slot_capacity(depth,ROOT,slot,25.,locator=LOC);self.assertGreater(clearance,30.)
  with self.assertRaisesRegex(ValueError,'칸보다 큽니다'):require_slot_capacity(depth,ROOT,slot,90.,locator=LOC)
 def test_occupied_slot_refused(self):
  from tray.tray_capacity import require_slot_capacity
  depth=np.full((480,640),500,np.uint16);depth[175:275,160:310]=490;depth[200:230,210:240]=460
  with self.assertRaisesRegex(ValueError,'비어 있지'):require_slot_capacity(depth,ROOT,dict(x_mm=180.,y_mm=30.,floor_height_mm=10.),20.,locator=LOC)
 def test_missing_floor_height_refused(self):
  from tray.tray_capacity import require_slot_capacity
  with self.assertRaises(ValueError):require_slot_capacity(np.full((480,640),500,np.uint16),ROOT,dict(x_mm=180.,y_mm=30.),20.,locator=LOC)

class FreshObservationTests(unittest.TestCase):
 scene=Tests.scene
 detect=Tests.detect
 def test_missing_observation_refused_before_camera(self):
  from pick_unknown import verify_before_motion
  with patch('camera.astra_calibration_camera.AstraCalibrationCamera') as camera:
   with self.assertRaisesRegex(ValueError,'관측 기록'):verify_before_motion({},ROOT)
   camera.assert_not_called()
 def test_stale_observation_refused_before_camera(self):
  from pick_unknown import verify_before_motion
  with patch('camera.astra_calibration_camera.AstraCalibrationCamera') as camera:
   with self.assertRaisesRegex(ValueError,'오래'):verify_before_motion({'unknown_observation':{'planned_at':'2020-01-01T00:00:00+00:00','baseline':'unused','target_xy':[.2,.03]}},ROOT,[.2,.03])
   camera.assert_not_called()
 def test_target_change_refused_and_camera_closed(self):
  from pick_unknown import verify_before_motion
  from datetime import datetime,timezone
  import hashlib,pick_unknown
  rgb,depth,base=self.scene();target=self.detect(rgb,depth,base)[0]
  class Camera:
   closed=False
   def __init__(self,**kw):pass
   def read_synchronized(self):pass
   def close(self):Camera.closed=True
  with tempfile.TemporaryDirectory() as td:
   p=Path(td)/'base.npy';np.save(p,base)
   job={'unknown_observation':{'planned_at':datetime.now(timezone.utc).isoformat(),'baseline':str(p),'baseline_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'target_xy':[.16,0]},'object_geometry':target['object_geometry']}
   with patch('camera.astra_calibration_camera.AstraCalibrationCamera',Camera),patch.object(pick_unknown,'wait_for_new_object',return_value=(rgb,depth,target)),patch.object(pick_unknown,'emit'):
    with self.assertRaisesRegex(ValueError,'바뀌었'):verify_before_motion(job,ROOT,[.16,0])
   self.assertTrue(Camera.closed)



 def test_actual_motion_target_bound_to_observation(self):
  from pick_unknown import verify_before_motion
  from datetime import datetime,timezone
  observation=dict(planned_at=datetime.now(timezone.utc).isoformat(),baseline='unused',target_xy=[.2,.03])
  with patch('camera.astra_calibration_camera.AstraCalibrationCamera') as camera:
   with self.assertRaisesRegex(ValueError,'실행 좌표'):verify_before_motion({'unknown_observation':observation},ROOT,[.23,.03])
   camera.assert_not_called()

class DepthNoiseRegressionTests(unittest.TestCase):
 scene=Tests.scene
 detect=Tests.detect
 def test_scattered_depth_flicker_is_not_object_removal(self):
  rgb,depth,_=self.scene();baseline=depth.copy()
  depth[212:258:3,242:298:3]+=10
  self.assertGreater(int((depth.astype(float)-baseline>8).sum()),100)
  targets=self.detect(rgb,depth,baseline)
  self.assertFalse(any(t['new_object'] for t in targets))
 def test_coherent_depth_change_still_refused(self):
  rgb,depth,_=self.scene();baseline=depth.copy();depth[220:245,250:280]+=10
  with self.assertRaisesRegex(ValueError,'이동|제거'):self.detect(rgb,depth,baseline)

class FlatPatchNoiseTests(unittest.TestCase):
 scene=Tests.scene
 detect=Tests.detect
 def test_quantized_flat_top_remains_pickable(self):
  rgb,depth,base=self.scene();depth[212:258:3,242:298:3]=463
  found=self.detect(rgb,depth,base)
  self.assertTrue(found[0]['pickable'],found[0]['problems'])
  self.assertTrue(found[0]['object_geometry']['top_flat'])

if __name__=='__main__':unittest.main()
