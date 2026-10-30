import math
import unittest
from landing.contact_geometry import Pose,quaternion_from_rpy
from landing.observations import CausalPoseHistory,PoseSample,ObservationUnavailable


class ObservationChecks(unittest.TestCase):
    def history(self):
        h=CausalPoseHistory((.02,0,-.01))
        for k in range(12):
            t=k*.01
            h.push(PoseSample(t,Pose((1+2*t+.5*.3*t*t,0,3-.4*t),(0,0,0,1)),100+t))
        return h

    def test_known_accelerated_motion_and_future_samples(self):
        h=self.history();o=h.estimate(.09,100.09,True,'GUIDED',100.08)
        self.assertAlmostEqual(o.sim_t_s,.09)
        self.assertAlmostEqual(o.motion.velocity_at_origin_world_m_s[0],2+.3*.09,places=10)
        self.assertAlmostEqual(o.com_acceleration_world_m_s2[0],.3,places=9)
        self.assertLessEqual(h.last_fit['end_sim_t_s'],.09)

    def test_rotation_uses_world_angular_velocity_and_rotating_com_offset(self):
        h=CausalPoseHistory((.05,0,0))
        for k in range(9):
            t=k*.01;h.push(PoseSample(t,Pose((0,0,2),quaternion_from_rpy(0,0,.2*t)),100+t))
        o=h.estimate(.08,100.08,True,'GUIDED',100.08)
        self.assertAlmostEqual(o.motion.angular_velocity_world_rad_s[2],.2,places=5)
        self.assertAlmostEqual(o.com_acceleration_world_m_s2[0],-.002*math.cos(.016),places=5)

    def test_wall_clock_must_be_sampled_after_latest_pose_reception(self):
        h=self.history()
        with self.assertRaisesRegex(ObservationUnavailable,'wall age invalid'):
            h.estimate(.11,100.109,True,'GUIDED',100.10)
        self.assertEqual(h.estimate(.11,100.111,True,'GUIDED',100.10).sim_t_s,.11)

    def test_warmup_pose_gap_and_clock_staleness_rejected(self):
        h=self.history()
        for args in [(.01,100.01,True,'GUIDED',100.01),(.3,100.3,True,'GUIDED',100.3),(.11,102.,True,'GUIDED',102.),(.11,100.11,True,'GUIDED',98.)]:
            with self.subTest(args=args),self.assertRaises(ObservationUnavailable):h.estimate(*args)
        h=CausalPoseHistory((0,0,0))
        for t in [0,.01,.02,.06,.07,.08]:h.push(PoseSample(t,Pose((0,0,1),(0,0,0,1)),100+t))
        with self.assertRaisesRegex(ObservationUnavailable,'Gap'):h.estimate(.08,100.08,True,'GUIDED',100.08)
