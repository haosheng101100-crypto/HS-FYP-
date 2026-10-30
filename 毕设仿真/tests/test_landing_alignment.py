from pathlib import Path
import sys,unittest
from dataclasses import replace
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'work/receding_dependencies'),str(ROOT)]
import numpy as np
from landing.receding_landing import FlightState,PlatformState
from landing.execution import BodyReference
import landing.receding_live as live

class AlignmentTests(unittest.TestCase):
    def test_preparation_depends_on_declared_measurement_quality(self):
        cfg=dict(minimum_leg_gap_m=1.5,dwell_s=.6,only_for_degraded_observations=True)
        self.assertFalse(live.moving_alignment_required(cfg,None))
        self.assertFalse(live.moving_alignment_required(cfg,dict(delay_s=0.,position_sigma_m=0.,yaw_sigma_rad=0.)))
        for key in ('delay_s','position_sigma_m','yaw_sigma_rad'):
            quality=dict(delay_s=0.,position_sigma_m=0.,yaw_sigma_rad=0.);quality[key]=.001
            self.assertTrue(live.moving_alignment_required(cfg,quality))
        self.assertFalse(live.moving_alignment_required(None,dict(delay_s=.04)))
        self.assertTrue(live.moving_alignment_required(dict(cfg,only_for_degraded_observations=False),None))
        for value in (-.01,float('nan'),float('inf')):
            with self.assertRaises(ValueError):live.moving_alignment_required(cfg,dict(delay_s=value))

    def test_initial_plan_times_match_actual_preparation_state(self):
        cfg=dict(initial_candidate_durations_s=[8.,10.,12.],moving_alignment=dict(initial_candidate_durations_s=[4.,5.,6.]))
        self.assertEqual(live.initial_plan_durations(cfg,aligned=False),(8.,10.,12.))
        self.assertEqual(live.initial_plan_durations(cfg,aligned=True),(4.,5.,6.))
        cfg['moving_alignment']={}
        self.assertEqual(live.initial_plan_durations(cfg,aligned=True),(8.,10.,12.))

    def test_moving_reference_converges_before_descent_and_obeys_derivative_limits(self):
        initial=BodyReference((0,0,4),(0,0,0),(0,0,0),0.)
        governor=live.DeckAlignmentReference(initial,(0,0,0),minimum_leg_gap_m=1.5)
        previous_a=np.zeros(3);ready=False
        for i in range(1,701):
            t=i*.04;deck=PlatformState(t,(3+.3*t,0,.8),(.3,0,0),(0,0,0))
            ref=governor.step(t,deck,(0,0,0))
            a=np.array(ref.acceleration_world_m_s2)
            self.assertLessEqual(np.linalg.norm(a),1.1+1e-10)
            self.assertLessEqual(np.linalg.norm(a-previous_a)/.04,2.+1e-9)
            self.assertGreater(ref.position_world_m[2]-.8-.195,1.35)
            self.assertLess(np.linalg.norm(ref.velocity_world_m_s),3.)
            state=FlightState(t,ref.position_world_m,ref.velocity_world_m_s,ref.acceleration_world_m_s2)
            report=governor.readiness(t,state,deck,(0,0,0),attitude_error_rad=0.,angular_speed_rad_s=0.)
            previous_a=a
            if report['ready']:
                ready=True;break
        self.assertTrue(ready,report)
        self.assertLess(t,20.)
        self.assertLess(np.linalg.norm(np.array(ref.velocity_world_m_s)-deck.velocity),.05)

    def test_readiness_requires_continuous_measured_state_dwell(self):
        g=live.DeckAlignmentReference(BodyReference((0,0,2.495),(0,0,0),(0,0,0),0.),(0,0,0))
        p=PlatformState(0,(0,0,.8),(0,0,0),(0,0,0))
        s=FlightState(0,(0,0,2.495),(0,0,0),(0,0,0))
        for t in (0.,.2,.4):self.assertFalse(g.readiness(t,replace(s,stamp=t),replace(p,stamp=t),(0,0,0),0.,0.)['ready'])
        bad=replace(s,stamp=.5,velocity=(.1,0,0))
        self.assertFalse(g.readiness(.5,bad,replace(p,stamp=.5),(0,0,0),0.,0.)['ready'])
        for t in (.6,.8,1.,1.2):r=g.readiness(t,replace(s,stamp=t),replace(p,stamp=t),(0,0,0),0.,0.)
        self.assertTrue(r['ready'])

    def test_moving_initial_plan_hold_is_not_a_stationary_hover(self):
        import tempfile
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as scratch:
            ref=BodyReference((2,0,2.5),(.3,.1,0),(.02,0,0),0.)
            session=live.AsyncLandingReference(live.FilePlannerClient(scratch),(0,0,0),ref,lead_s=.4,moving_hold_start_s=10.)
            state=session._future_state(10.)
            np.testing.assert_allclose(state.position,(2.1216,.04,2.5),atol=1e-12)
            np.testing.assert_allclose(state.velocity,(.308,.1,0),atol=1e-12)
            np.testing.assert_allclose(state.acceleration,(.02,0,0),atol=1e-12)
            current=session.hold_at(10.2)
            np.testing.assert_allclose(current.position_world_m,(2.0604,.02,2.5),atol=1e-12)

if __name__=='__main__':unittest.main()
