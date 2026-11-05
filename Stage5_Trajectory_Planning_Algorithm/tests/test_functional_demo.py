"""Protect existing demonstrations and preserve the accepted controller/profile."""
import importlib.util,json,tempfile,unittest,shutil,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
class DemoPreparationTests(unittest.TestCase):
 def prepare(self,*args):
  path=ROOT/'tools/run_functional_demo.py'
  self.assertTrue(path.exists(),'Functional demo entry is missing')
  spec=importlib.util.spec_from_file_location('demo',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
  return m.prepare_demo(*args)
 def test_existing_output_is_preserved(self):
  with tempfile.TemporaryDirectory() as d:
   out=Path(d)/'existing';out.mkdir();sentinel=out/'evidence.txt';sentinel.write_text('original')
   with self.assertRaises(FileExistsError):self.prepare(ROOT,out,['x'])
   self.assertEqual(sentinel.read_text(),'original');self.assertEqual(len(list(out.iterdir())),1)
 def test_single_axis_keeps_control_configuration_and_source(self):
  with tempfile.TemporaryDirectory() as d:
   out=Path(d)/'fresh';self.prepare(ROOT,out,['y'])
   p=json.loads((out/'declared_protocol.json').read_text());original=json.loads((ROOT/'config/experiments/planar_reacquisition_v1.json').read_text())
   cfg=p['control_configuration'];self.assertEqual(cfg['validation_scenarios'],['planar_reacquire_y'])
   cfg['validation_scenarios']=original['validation_scenarios'];self.assertEqual(cfg,original)
   self.assertEqual((out/'operator/landing/receding_live.py').read_bytes(),(ROOT/'landing/receding_live.py').read_bytes())
 def test_invalid_axis_rejected_before_creating_output(self):
  with tempfile.TemporaryDirectory() as d:
   out=Path(d)/'fresh'
   with self.assertRaises(ValueError):self.prepare(ROOT,out,['z'])
   self.assertFalse(out.exists())
class DemoEnvironmentTests(unittest.TestCase):
 def test_changed_firmware_is_rejected(self):
  path=ROOT/'tools/run_functional_demo.py';spec=importlib.util.spec_from_file_location('demo',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
  verify=getattr(m,'verify_environment',None);self.assertTrue(callable(verify),'Firmware verification missing')
  with tempfile.TemporaryDirectory() as d:
   base=Path(d);(base/'vendor').mkdir();(base/'vendor/arducopter').write_bytes(b'changed firmware')
   with self.assertRaises(ValueError):verify(ROOT,base)
class DemoReviewRegressionTests(unittest.TestCase):
 def module(self):
  spec=importlib.util.spec_from_file_location('demo',ROOT/'tools/run_functional_demo.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
 def test_simultaneous_start_is_rejected_until_owner_releases_lock(self):
  fn=getattr(self.module(),'acquire_lock',None);self.assertTrue(callable(fn),'Shared simulation lock missing')
  with tempfile.TemporaryDirectory() as d:
   first=fn(Path(d))
   try:
    with self.assertRaises(RuntimeError):fn(Path(d))
   finally:first.close()
   second=fn(Path(d));second.close()
 def test_changed_root_profile_is_rejected_before_output_creation(self):
  with tempfile.TemporaryDirectory() as d:
   r=Path(d)/'root';shutil.copytree(ROOT/'simulation/operator_package',r/'simulation/operator_package')
   (r/'simulation/environment_reference').mkdir();shutil.copy2(ROOT/'simulation/environment_reference/demo_gui.config',r/'simulation/environment_reference/demo_gui.config')
   (r/'config/experiments').mkdir(parents=True);cfg=json.loads((ROOT/'config/experiments/planar_reacquisition_v1.json').read_text());cfg['planner_limits']['speed']=99
   (r/'config/experiments/planar_reacquisition_v1.json').write_text(json.dumps(cfg));out=Path(d)/'out'
   with self.assertRaises(ValueError):self.module().prepare_demo(r,out,['x'])
   self.assertFalse(out.exists())
 def test_changed_loaded_plugin_binary_is_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   r=Path(d)/'root';base=Path(d)/'base';ref=r/'simulation/environment_reference';ref.mkdir(parents=True);vendor=base/'vendor';vendor.mkdir(parents=True)
   (vendor/'arducopter').write_bytes(b'fixed firmware');(vendor/'ardupilot_gazebo/build').mkdir(parents=True);(vendor/'ardupilot_gazebo/build/libArduPilotPlugin.so').write_bytes(b'replaced plugin')
   (ref/'source_snapshot.json').write_text(json.dumps(dict(arducopter_sha256=hashlib.sha256(b'fixed firmware').hexdigest(),files={})))
   (ref/'demo_runtime_assets.json').write_text(json.dumps({'vendor/ardupilot_gazebo':{'build/libArduPilotPlugin.so':hashlib.sha256(b'fixed plugin').hexdigest()}}))
   with self.assertRaises(ValueError):self.module().verify_environment(r,base)
 def test_runtime_without_a_pinned_loaded_plugin_is_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   r=Path(d)/'root';base=Path(d)/'base';ref=r/'simulation/environment_reference';ref.mkdir(parents=True);(base/'vendor').mkdir(parents=True);(base/'vendor/arducopter').write_bytes(b'fixed firmware')
   (ref/'source_snapshot.json').write_text(json.dumps(dict(arducopter_sha256=hashlib.sha256(b'fixed firmware').hexdigest(),files={})))
   (ref/'demo_runtime_assets.json').write_text('{}')
   with self.assertRaises(ValueError):self.module().verify_environment(r,base)
 def fixture(self,out):
  (out/'session_001/cycle_001/results').mkdir(parents=True);res=out/'session_001/cycle_001/results';(res/'execution_status.json').write_text(json.dumps(dict(scenario='planar_reacquire_x',completed=True,armed_at_save=False,final_mode='LAND')));(res/'cycle_closed.json').write_text(json.dumps(dict(all_owned_cycle_processes_stopped=True)))
 def test_missing_physical_axis_cannot_be_reported_complete(self):
  fn=getattr(self.module(),'inspect_completion',None);self.assertTrue(callable(fn),'Physical completion reconciliation missing')
  with tempfile.TemporaryDirectory() as d:
   out=Path(d);self.fixture(out);r=fn(out,['x','y'],False);self.assertFalse(r['physics_complete']);self.assertIn('planar_reacquire_y',r['missing_scenarios'])
 def test_no_window_or_recording_exception_cannot_pass_requested_recording(self):
  fn=getattr(self.module(),'inspect_completion',None);self.assertTrue(callable(fn),'Recording completion reconciliation missing')
  with tempfile.TemporaryDirectory() as d:
   out=Path(d);self.fixture(out);r=fn(out,['x'],True);self.assertTrue(r['physics_complete']);self.assertFalse(r['recording_complete'])
   (out/'recording_error.json').write_text(json.dumps(dict(error='capture failed')));self.assertFalse(fn(out,['x'],True)['recording_complete'])
 def test_safe_exit_cannot_count_as_a_demonstrated_landing(self):
  fn=getattr(self.module(),'inspect_completion',None);self.assertTrue(callable(fn),'Physical completion reconciliation missing')
  with tempfile.TemporaryDirectory() as d:
   out=Path(d);self.fixture(out);p=out/'session_001/cycle_001/results/execution_status.json';v=json.loads(p.read_text());v['completed']=False;p.write_text(json.dumps(v));self.assertFalse(fn(out,['x'],False)['physics_complete'])
if __name__=='__main__':unittest.main()
