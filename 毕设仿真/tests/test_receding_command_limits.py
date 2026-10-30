from pathlib import Path
import sys,json,math,unittest
from dataclasses import replace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'work/receding_dependencies'))
import numpy as np
from landing.receding_live import FinalCommandLimiter
from landing.execution import BodyReference,ExecutionDecision

class CommandLimitsTests(unittest.TestCase):
    def test_bounded_forecast_composes_across_acceleration_cutoff(self):
        from landing.receding_landing import PlatformState
        p=PlatformState(0.,(0,0,.8),(.3,0,0),(.12,0,0),yaw_rate=.03,motion_horizon_s=2.)
        direct=p.predict(5.);split=p.predict(.4).predict(4.6)
        np.testing.assert_allclose(direct.position,split.position,atol=1e-12)
        self.assertAlmostEqual(direct.yaw,split.yaw)
        self.assertAlmostEqual(direct.velocity[0],.54)
        self.assertEqual(direct.acceleration,(0.,0.,0.))

    def test_adversarial_feedback_remains_bounded_and_slew_limited(self):
        limits=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())['planner_limits']
        rng=np.random.default_rng(103);limiter=FinalCommandLimiter(limits);previous=np.zeros(3)
        for i in range(1000):
            ref=BodyReference((0,0,2),tuple(rng.normal(0,5,3)),tuple(rng.normal(0,8,3)),0.)
            result=limiter.update(ExecutionDecision('TRACKING',False,ref),.04*(i+1)).reference
            a=np.array(result.acceleration_world_m_s2)
            self.assertLessEqual(np.linalg.norm(a),limits['acceleration']+1e-9)
            self.assertLessEqual(abs(a[2]),limits['vertical_acceleration']+1e-9)
            self.assertLessEqual(np.linalg.norm(a[:2]),(9.80665+a[2])*math.tan(limits['tilt_rad'])+1e-9)
            self.assertLessEqual(np.linalg.norm((a-previous)/.04),limits['jerk']+1e-9)
            self.assertLessEqual(np.linalg.norm(result.velocity_world_m_s),limits['speed']+1e-9)
            previous=a

    def test_contact_handover_keeps_reference_cleared(self):
        limits=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())['planner_limits']
        decision=ExecutionDecision('CONTACT_HANDOVER',True,None)
        self.assertEqual(FinalCommandLimiter(limits).update(decision,1.),decision)

    def test_command_reserve_preserves_original_physical_cap_and_jerk(self):
        limits=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())['planner_limits']
        original=dict(limits);limiter=FinalCommandLimiter(limits,acceleration_reserve_m_s2=.1)
        previous=np.zeros(3)
        for i in range(80):
            ref=BodyReference((0,0,2),(0,0,0),(-1.8,-.1,-.04),0.)
            result=limiter.update(ExecutionDecision('TRACKING',False,ref),.04*(i+1))
            a=np.array(result.reference.acceleration_world_m_s2)
            self.assertLessEqual(np.linalg.norm(a),1.4+1e-10)
            self.assertLessEqual(np.linalg.norm(a-previous)/.04,limits['jerk']+1e-10)
            previous=a
        self.assertEqual(limits,original)
        self.assertAlmostEqual(limiter.diagnostics['command_acceleration_cap_m_s2'],1.4)

    def test_invalid_command_reserve_rejected(self):
        limits={'acceleration':1.5}
        for reserve in (-.1,1.5,float('nan')):
            with self.assertRaises(ValueError):FinalCommandLimiter(limits,acceleration_reserve_m_s2=reserve)

if __name__=='__main__':unittest.main()
