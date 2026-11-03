import importlib.util,unittest,json,tempfile,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
class BridgeTests(unittest.TestCase):
 def bridge(self,events,phase='TRACKING',contact=False):
  p=ROOT/'tools/multicondition_bridge.py';self.assertTrue(p.exists(),'World-side measured bridge missing')
  spec=importlib.util.spec_from_file_location('bridge',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
  return m.measured_scene_state(10.,3.6,-.2,phase,events,contact)
 def test_only_actual_event_counts_and_measurements_cross_boundary(self):
  d=self.bridge([{'event':'PLAN_ACTIVATED'},{'event':'PLANAR_REACQUIRE'},{'event':'PLAN_REQUESTED'},{'event':'PLAN_ACTIVATED'}])
  self.assertEqual(d['reacquisitions'],1);self.assertEqual(d['activated_plan_count'],2)
  self.assertEqual(d['clearance_m'],3.6);self.assertEqual(d['phase'],'TRACKING');self.assertNotIn('future_commands',d)
 def test_actual_contact_has_priority_over_ongoing_tracking(self):
  d=self.bridge([],contact=True);self.assertTrue(d['contact_detected']);self.assertEqual(d['phase'],'CONTACT_HANDOVER')
class SceneInitializationTests(unittest.TestCase):
 def test_continuous_case_creates_empty_event_record_before_any_trigger(self):
  spec=importlib.util.spec_from_file_location('bridge',ROOT/'tools/multicondition_bridge.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
  fn=getattr(m,'initialize_world_scene',None);self.assertTrue(callable(fn),'Persistent empty event evidence initialization missing')
  case=json.loads((ROOT/'config/experiments/multicondition_matrix_v1.json').read_text())['cases'][0]
  with tempfile.TemporaryDirectory() as d:
   folder=Path(d);f=folder/'case.json';f.write_text(json.dumps(case));res=folder/'results';res.mkdir();sys.path.insert(0,str(ROOT))
   program=fn(f,{'case_id':'M01','scenario':'multi_m01'},'multi_m01',res)
   self.assertEqual(json.loads((res/'scene_maneuvers.json').read_text()),[]);self.assertEqual(json.loads((res/'world_case.json').read_text()),case);self.assertTrue(callable(program.step))
if __name__=='__main__':unittest.main()
