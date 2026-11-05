import math
import json
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
from landing.planar_motion import PlanarMotionCommand, desired_planar_velocity
from tools.build_planar_profile import build_planar_model

ROOT=Path(__file__).resolve().parents[1]


class PlanarMotionTests(unittest.TestCase):
    def test_new_profile_keeps_original_motion_and_contact_limits(self):
        old=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())
        new=json.loads((ROOT/'config/experiments/planar_landing_v1.json').read_text())
        self.assertEqual(old['planner_limits'],new['planner_limits'])
        self.assertEqual(old['deck_half_extents'],new['deck_half_extents'])
        self.assertEqual(new['platform_motion_model'],'force_driven_planar_guide')
        self.assertEqual(set(new['scenarios']),{'planar_forward_backward','planar_lateral','planar_diagonal','planar_reversal','planar_xy'})

    def test_all_axes_reverse_and_command_vector_limits_hold(self):
        command=PlanarMotionCommand()
        previous_v=(0.,0.);previous_a=(0.,0.);seen=[set(),set()]
        for i in range(2000):
            t=i*.04
            v,a=command.step(t,desired_planar_velocity('planar_xy',t))
            self.assertLessEqual(math.hypot(*v),.8+1e-9)
            self.assertLessEqual(math.hypot(*a),.12+1e-9)
            self.assertLessEqual(math.dist(a,previous_a),.3*.04+1e-9)
            for axis in range(2):
                if abs(v[axis])>.04:seen[axis].add(1 if v[axis]>0 else -1)
            if i:self.assertLessEqual(math.dist(v,previous_v),.12*.04+1e-9)
            previous_v=v;previous_a=a
        self.assertEqual(seen,[{-1,1},{-1,1}])

    def test_reverse_startup_and_nonfinite_unknown_inputs_are_rejected(self):
        command=PlanarMotionCommand()
        command.step(2.,(-.3,0.))
        with self.assertRaises(ValueError):command.step(1.,(0.,0.))
        with self.assertRaises(ValueError):command.step(2.04,(float('nan'),0.))
        with self.assertRaises(ValueError):desired_planar_velocity('invented',0.)

    def test_diagonal_command_does_not_exceed_speed_circle(self):
        command=PlanarMotionCommand()
        for i in range(1000):v,a=command.step(i*.04,(5.,5.))
        self.assertLessEqual(math.hypot(*v),.8+1e-9)
        self.assertGreater(v[0],0.);self.assertGreater(v[1],0.)

    def test_force_driven_fixture_preserves_deck_and_contact_identity(self):
        source=ROOT/'simulation/reference_cycle/models/landing_vehicle/model.sdf'
        before=source.read_bytes()
        with tempfile.TemporaryDirectory() as temp:
            destination=Path(temp)/'model.sdf'
            receipt=build_planar_model(source,destination)
            model=ET.parse(destination).getroot().find('model')
            self.assertEqual(model.get('canonical_link'),'chassis')
            old=ET.fromstring(before).find('model/link[@name="chassis"]')
            new=model.find('link[@name="chassis"]')
            self.assertEqual(ET.tostring(old),ET.tostring(new))
            self.assertEqual(new.find('sensor[@name="deck_contact"]/contact/collision').text,'landing_deck')
            self.assertIsNone(model.find('plugin[@name="gz::sim::systems::DiffDrive"]'))
            controllers=model.findall('plugin[@name="gz::sim::systems::JointController"]')
            self.assertEqual(len(controllers),2)
            self.assertTrue(all(p.findtext('use_force_commands')=='true' for p in controllers))
            self.assertEqual(receipt['physical_validation'],'PENDING')
        self.assertEqual(source.read_bytes(),before)


if __name__=='__main__':unittest.main()
