import importlib.util,unittest,tempfile,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
class ViewerIsolationTests(unittest.TestCase):
 def module(self):
  path=ROOT/'simulation/operator_package/multicondition_viewer.py';self.assertTrue(path.exists(),'Headless viewer isolation missing')
  spec=importlib.util.spec_from_file_location('viewer',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
 def test_headless_mode_does_not_start_display_process(self):
  with tempfile.TemporaryDirectory() as d:
   marker=Path(d)/'viewer_started';p=self.module().launch_viewer([sys.executable,'-c',f'open({str(marker)!r},"w").write("started")'],env={'HS_NO_GUI':'1'})
   self.assertIsNone(p.pid);self.assertIsNone(p.poll());p.terminate();self.assertFalse(marker.exists())
 def test_explicit_view_mode_starts_and_reaps_actual_process(self):
  with tempfile.TemporaryDirectory() as d:
   marker=Path(d)/'viewer_started';p=self.module().launch_viewer([sys.executable,'-c',f'open({str(marker)!r},"w").write("started")'],env={'HS_NO_GUI':'0'})
   self.assertEqual(p.wait(timeout=5),0);self.assertEqual(marker.read_text(),'started')
if __name__=='__main__':unittest.main()
