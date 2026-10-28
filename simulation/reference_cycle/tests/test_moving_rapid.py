from pathlib import Path
import unittest,json,math
from landing.reference_loader import load_verified_reference
from landing.contact_geometry import Pose,RigidMotion,Deck,evaluate_airframe
from landing.moving_platform import translate_deck
from landing.moving_rapid import plan_moving_rapid
ROOT=Path(__file__).resolve().parents[1]

class MovingRapidTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _,cls.g,_,cls.profile=load_verified_reference(ROOT)
        cls.deck=Deck(RigidMotion(Pose((4,0,.8),(0,0,0,1)),(.3,0,0),(0,0,0)),(1,.7))
        cls.curve,cls.ideal,cls.report=plan_moving_rapid(ROOT,cls.g,cls.profile,cls.deck)
    def test_terminal_relative_speed_and_future_deck_boundary(self):
        deck=translate_deck(self.ideal,1.5+self.curve.T)
        feet=evaluate_airframe(self.g,self.curve.at(self.curve.T)['motion'],deck)
        active=[f for f in feet if abs(f.clearance_m)<1e-8];self.assertTrue(active)
        for f in active:
            self.assertAlmostEqual(f.closing_speed_m_s,.45,places=8)
            self.assertLess(f.tangential_speed_upper_bound_m_s,1e-7)
        self.assertAlmostEqual(self.curve.at(self.curve.T)['com_velocity_world_m_s'][0],.3,places=8)
        self.assertTrue(self.report['reference_checks']['passed']);self.assertTrue(self.report['rapid_motion_checks']['passed'])
    def test_initial_state_and_no_polynomial_after_contact(self):
        self.assertEqual(self.curve.at(0)['com_velocity_world_m_s'],(0,0,-.4))
        self.assertAlmostEqual(self.curve.at(0)['com_position_world_m'][2],3.65)
        with self.assertRaises(ValueError):self.curve.at(self.curve.T+.01)
    def test_static_deck_not_accepted_as_moving_scene(self):
        deck=Deck(RigidMotion.stationary(self.deck.motion.pose),(1,.7))
        with self.assertRaises(ValueError):plan_moving_rapid(ROOT,self.g,self.profile,deck)
    def test_rapid_condition_and_no_long_speed_matching_segment(self):
        self.assertLessEqual(self.report['rapid_motion_checks']['time_last_0_5m_s'],1.5)
        self.assertLessEqual(self.report['reference_checks']['relative_low_speed_lead_s'],.7)
        self.assertAlmostEqual(math.degrees(self.curve.at(self.curve.T)['nose_up_rad']),2,places=8)

if __name__=='__main__':unittest.main()
