"""Reject invalid model-scoped joint-axis frames with installed libsdformat."""
from pathlib import Path
import importlib.util,shutil,subprocess,tempfile,unittest
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
@unittest.skipUnless(shutil.which('gz'),'Requires Gazebo SDFormat parser')
class PlanarSdfRuntimeTests(unittest.TestCase):
 def test_generated_joint_frames_resolve_and_chassis_is_unchanged(self):
  spec=importlib.util.spec_from_file_location('builder',ROOT/'tools/build_planar_profile.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
  source=ROOT/'simulation/reference_cycle/models/landing_vehicle/model.sdf'
  with tempfile.TemporaryDirectory() as directory:
   out=Path(directory)/'model.sdf';m.build_planar_model(source,out)
   p=subprocess.run(['gz','sdf','-p',str(out)],capture_output=True,text=True)
   self.assertEqual(p.returncode,0,p.stderr)
   parsed=ET.parse(out);original=ET.parse(source)
   self.assertEqual(ET.tostring(parsed.find('.//link[@name="chassis"]')),ET.tostring(original.find('.//link[@name="chassis"]')))
   self.assertEqual([n.get('expressed_in') for n in parsed.findall('.//joint/axis/xyz')],['__model__','__model__'])
