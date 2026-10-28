import unittest,math
from landing.horizontal_feedback import HorizontalFeedback
from landing.execution import AircraftObservation,ExecutionDecision,BodyReference
from landing.contact_geometry import RigidMotion,Pose

class HorizontalFeedbackTests(unittest.TestCase):
    def observation(self,p=(0,0,1),v=(0,0,0),t=1.):return AircraftObservation(t,RigidMotion(Pose(p,(0,0,0,1)),v,(0,0,0)),True,'GUIDED')
    def decision(self,p=(0,0,1),v=(0,0,-.45),a=(0,0,.1)):
        return ExecutionDecision('TRACK_REFERENCE',False,BodyReference(p,v,a,0.))
    def test_feedback_direction_and_vertical_preservation(self):
        c=HorizontalFeedback();d=self.decision(p=(.1,0,1))
        out=c.update(d,self.observation(v=(.2,0,0)),1.)
        self.assertLess(out.reference.acceleration_world_m_s2[0],0.)
        self.assertEqual(out.reference.acceleration_world_m_s2[2],.1)
        self.assertEqual(out.reference.position_world_m,d.reference.position_world_m)
        self.assertEqual(out.reference.velocity_world_m_s,d.reference.velocity_world_m_s)
    def test_vector_bound_no_postcontact_command_and_stale_rejection(self):
        c=HorizontalFeedback();d=c.update(self.decision(p=(10,10,1)),self.observation(),1.)
        self.assertLessEqual(math.hypot(*d.reference.acceleration_world_m_s2[:2]),2.5+1e-12)
        stop=ExecutionDecision('LAND_SETTLE',True,request_land=True)
        self.assertIs(c.update(stop,self.observation(),1.04),stop)
        with self.assertRaises(ValueError):c.update(self.decision(),self.observation(t=.5),1.1)
    def test_reference_jerk_lead(self):
        c=HorizontalFeedback();c.update(self.decision(),self.observation(),1.)
        d=c.update(self.decision(a=(.04,0,.1)),self.observation(t=1.04),1.04)
        self.assertAlmostEqual(d.reference.acceleration_world_m_s2[0],.16)
    def test_damped_feedback_with_actuator_lag_reduces_position_error(self):
        c=HorizontalFeedback();x=.1;v=0.;a=0.;dt=.01
        for i in range(800):
            t=1+i*dt;cmd=c.update(self.decision(),self.observation(p=(x,0,1),v=(v,0,0),t=t),t).reference.acceleration_world_m_s2[0]
            a+=(cmd-a)*dt/.12;v+=a*dt;x+=v*dt
        self.assertLess(abs(x),.002);self.assertLess(abs(v),.002)
