import unittest
from landing.contact_geometry import Pose,RigidMotion
from landing.execution import AircraftObservation,ExecutionDecision,BodyReference
from landing.tracking_compensation import compensate_height,terminal_height_feedforward

class HeightCompensationTests(unittest.TestCase):
    def setUp(self):
        self.o=AircraftObservation(1.,RigidMotion.stationary(Pose((0,0,1),(0,0,0,1))),True,'GUIDED')
    def decision(self,z):return ExecutionDecision('TRACK_REFERENCE',False,BodyReference((1,0,z),(.3,0,-.08),(-.34,0,0),0.))
    def test_feedback_sign_and_preservation(self):
        d=self.decision(1.02);out=compensate_height(d,self.o)
        self.assertAlmostEqual(out.reference.position_world_m[2],1.06)
        self.assertEqual(out.reference.position_world_m[:2],d.reference.position_world_m[:2])
        self.assertEqual(out.reference.velocity_world_m_s,d.reference.velocity_world_m_s)
        self.assertEqual(out.reference.acceleration_world_m_s2,d.reference.acceleration_world_m_s2)
        self.assertEqual(d.reference.position_world_m[2],1.02)
    def test_bound_zero_error_and_no_commands_after_contact(self):
        self.assertEqual(compensate_height(self.decision(1.),self.o),self.decision(1.))
        self.assertAlmostEqual(compensate_height(self.decision(2.),self.o).reference.position_world_m[2],2.08)
        stop=ExecutionDecision('LAND_SETTLE',True,request_land=True)
        self.assertIs(compensate_height(stop,self.o),stop)
    def test_terminal_feedforward_smooth_and_inactive_outside_approach(self):
        d=self.decision(1.)
        self.assertEqual(terminal_height_feedforward(d,1.,6.9),d)
        end=terminal_height_feedforward(d,6.9,6.9)
        self.assertAlmostEqual(end.reference.position_world_m[2],1.02)
        self.assertEqual(end.reference.velocity_world_m_s,d.reference.velocity_world_m_s)
        self.assertEqual(end.reference.acceleration_world_m_s2,d.reference.acceleration_world_m_s2)
        from dataclasses import replace
        hold=replace(d,phase='PREPARE_HOLD')
        self.assertIs(terminal_height_feedforward(hold,6.9,6.9),hold)
    def test_terminal_feedforward_derivatives(self):
        d=self.decision(1.);t=4.7;eps=1e-4
        m=terminal_height_feedforward(d,t-eps,6.9).reference
        p=terminal_height_feedforward(d,t+eps,6.9).reference
        c=terminal_height_feedforward(d,t,6.9).reference
        self.assertAlmostEqual((p.position_world_m[2]-m.position_world_m[2])/(2*eps),c.velocity_world_m_s[2]-d.reference.velocity_world_m_s[2],places=7)
        self.assertAlmostEqual((p.velocity_world_m_s[2]-m.velocity_world_m_s[2])/(2*eps),c.acceleration_world_m_s2[2]-d.reference.acceleration_world_m_s2[2],places=7)
