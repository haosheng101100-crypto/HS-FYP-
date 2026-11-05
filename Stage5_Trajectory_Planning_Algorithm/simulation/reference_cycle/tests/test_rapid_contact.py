import unittest,json,math
from pathlib import Path
from landing.rapid_contact import load_rapid_reference
from landing.contact_geometry import evaluate_airframe
ROOT=Path(__file__).resolve().parents[1]

class RapidContactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.curve,cls.geometry,cls.deck,cls.profile,cls.report=load_rapid_reference(ROOT)
    def test_prescribed_finite_contact_speed_and_no_hover(self):
        s=self.curve.at(self.curve.T);feet=evaluate_airframe(self.geometry,s['motion'],self.deck)
        active=[f for f in feet if abs(f.clearance_m)<1e-8]
        for f in active:
            self.assertAlmostEqual(f.closing_speed_m_s,.45,places=8)
            self.assertLess(f.tangential_speed_upper_bound_m_s,1e-7)
        self.assertAlmostEqual(math.degrees(s['nose_up_rad']),2,places=8)
        self.assertTrue(self.report['reference_checks']['passed']);self.assertTrue(self.report['rapid_motion_checks']['passed'])
        self.assertLess(self.report['rapid_motion_checks']['near_deck_low_vertical_speed_duration_s'],.1)
    def test_same_initial_state_and_no_below_plane_extension(self):
        s=self.curve.at(0)
        self.assertAlmostEqual(s['com_position_world_m'][0],0);self.assertAlmostEqual(s['com_position_world_m'][2],3.65)
        self.assertEqual(s['com_velocity_world_m_s'],(0.,0.,-.4))
        with self.assertRaises(ValueError):self.curve.at(self.curve.T+.001)
        cfg=json.loads((ROOT/'config/experiments/execution_v1.json').read_text())
        self.assertEqual(cfg['handover_limits']['contact_normal_speed_m_s'],.12)
