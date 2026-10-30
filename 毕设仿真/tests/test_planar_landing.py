"""Causal XY changes and descent authorization, without starting a flight."""
import math
import unittest
from dataclasses import replace
from landing.receding_landing import PlatformState, FlightState
from landing.execution import BodyReference
from landing.receding_live import DeckAlignmentReference,braking_clearance_design
from landing.planar_landing import MotionChangeMonitor, PlanarDesign, PlanarLandingSupervisor, capture_window,inflate_forecast_tube
from landing.prediction_uncertainty import ForecastTube


def platform(t, velocity=(.2, 0., 0.), acceleration=(0., 0., 0.), jerk=(0., 0., 0.)):
    return PlatformState(t, (velocity[0]*t, velocity[1]*t, .8), velocity, acceleration, jerk=jerk)


class PlanarPolicyTests(unittest.TestCase):
    def test_detected_change_inflates_only_current_causal_forecast_reserves(self):
        tube=ForecastTube(1.,(0.,.4,.8),(0.,.01,.02),(0.,.02,.03),(0.,.001,.002),(0,12,12),True)
        motion=dict(stable=False,position_innovation_m=.02,velocity_innovation_m_s=.08,acceleration_change_m_s2=.2)
        enlarged=inflate_forecast_tube(tube,motion)
        self.assertEqual(enlarged.stamp,tube.stamp)
        self.assertGreater(enlarged.position_radius_m[-1],tube.position_radius_m[-1])
        self.assertGreater(enlarged.velocity_radius_m_s[-1],tube.velocity_radius_m_s[-1])
        self.assertEqual(enlarged.sample_counts,tube.sample_counts)
        self.assertIs(inflate_forecast_tube(tube,dict(stable=True)),tube)

    def test_clean_diagonal_motion_settles_without_false_reversal(self):
        monitor=MotionChangeMonitor()
        for i in range(51):
            report=monitor.update(platform(i*.04, (.2, -.15, 0.)))
        self.assertTrue(report['stable'])
        self.assertFalse(report['changed'])

    def test_lateral_reversal_is_detected_across_zero_velocity(self):
        monitor=MotionChangeMonitor()
        for i,v in enumerate((.2,.1,.02,0.,-.02,-.1)):
            report=monitor.update(platform(i*.04,(0.,v,0.)))
        self.assertTrue(report['changed'])
        self.assertIn('DIRECTION_REVERSAL',report['reasons'])
        self.assertFalse(report['stable'])

    def test_stationary_small_noise_does_not_become_direction_reversal(self):
        monitor=MotionChangeMonitor()
        for i in range(80):
            p=platform(i*.04,(.002*math.sin(i),.002*math.cos(i),0.))
            report=monitor.update(replace(p,position=(.0004*math.sin(i),.0004*math.cos(i),.8)))
            self.assertNotIn('DIRECTION_REVERSAL',report['reasons'])
        self.assertTrue(report['stable'])

    def test_stale_or_reversed_observation_cannot_authorize_descent(self):
        monitor=MotionChangeMonitor()
        monitor.update(platform(1.))
        with self.assertRaises(ValueError):monitor.update(platform(.9))
        report=monitor.update(platform(1.3))
        self.assertFalse(report['stable'])
        self.assertIn('OBSERVATION_GAP',report['reasons'])

    def test_capture_window_rejects_sustained_acceleration_and_future_turn(self):
        self.assertFalse(capture_window(platform(0.,acceleration=(0.,.25,0.)),.8)['passed'])
        self.assertFalse(capture_window(platform(0.,jerk=(0.,.3,0.)),.8)['passed'])
        self.assertTrue(capture_window(platform(0.,velocity=(-.2,.15,0.)),.8)['passed'])

    def test_long_prediction_is_conditional_not_a_certified_contact_window(self):
        report=capture_window(platform(0.),8.)
        self.assertTrue(report['conditional'])
        self.assertFalse(report['contact_authorized'])

    def test_supervisor_waits_then_descends_and_reacquires_on_change(self):
        supervisor=PlanarLandingSupervisor(PlanarDesign())
        for i in range(31):
            p=platform(i*.04)
            state=FlightState(p.stamp,(0.,0.,3.),(.2,0.,0.),(0.,0.,0.))
            report=supervisor.update(p.stamp,state,p,2.,aligned=True,descending=False)
        self.assertEqual(report['action'],'DESCEND')
        p=platform(1.24,acceleration=(0.,.4,0.))
        report=supervisor.update(1.24,replace(state,stamp=1.24),p,2.,aligned=True,descending=True)
        self.assertEqual(report['action'],'REACQUIRE')

    def test_low_change_exits_and_contact_always_preempts(self):
        supervisor=PlanarLandingSupervisor(PlanarDesign())
        p=platform(0.,acceleration=(.4,0.,0.))
        state=FlightState(0.,(0.,0.,1.),(0.,0.,-.3),(0.,0.,0.))
        self.assertEqual(supervisor.update(0.,state,p,.2,aligned=False,descending=True)['action'],'EXIT')
        p=replace(p,stamp=.04)
        self.assertEqual(supervisor.update(.04,replace(state,stamp=.04),p,.1,contact=True)['action'],'CONTACT')

    def test_reacquisition_uses_actual_follow_braking_authority(self):
        supervisor=PlanarLandingSupervisor()
        state=FlightState(0.,(0.,0.,2.892),(1.,0.,-.7),(0.,0.,-.98))
        report=supervisor.update(0.,state,platform(0.,acceleration=(0.,.4,0.)),1.897,descending=True)
        self.assertEqual(report['action'],'EXIT')

    def test_reacquisition_also_accounts_for_sent_reference_downward_motion(self):
        supervisor=PlanarLandingSupervisor()
        state=FlightState(0.,(0,0,3),(0,0,0),(0,0,0))
        reference=BodyReference((0,0,3),(1,0,-.7),(0,0,-.98),0.)
        report=supervisor.update(0.,state,platform(0.,acceleration=(0,.4,0)),2.,descending=True,braking_reference=reference)
        self.assertEqual(report['action'],'EXIT')

    def test_planar_follow_reserves_vertical_authority_during_large_xy_error(self):
        initial=BodyReference((20.,20.,3.),(1.,0.,-.7),(0.,0.,-.98),0.)
        reference=DeckAlignmentReference(initial,(0,0,0),start_s=0.,reserve_vertical=True)
        reference.step(0.,platform(0.),(0,0,0))
        for i in range(1,31):
            result=reference.step(i*.04,platform(i*.04),(0,0,0))
        self.assertGreater(result.acceleration_world_m_s2[2],.35)

    def test_admitted_reacquisition_grid_keeps_protected_reference_clearance(self):
        for vz in (-.7,-.4,-.1,0.):
            for az in (-.98,-.4,0.,.5):
                state=FlightState(0.,(20.,20.,3.),(1.,0.,vz),(0.,0.,az))
                loss=braking_clearance_design(state,upward_acceleration=.4,vertical_jerk=1.2)['required_clearance_m']
                gap=1.25+loss+.25+.001
                state=replace(state,position=(20.,20.,.8+.195+gap))
                report=PlanarLandingSupervisor().update(0.,state,platform(0.,velocity=(0,0,0)),gap,descending=True)
                self.assertEqual(report['action'],'REACQUIRE')
                initial=BodyReference(state.position,state.velocity,state.acceleration,0.)
                follow=DeckAlignmentReference(initial,(0,0,0),start_s=0.,reserve_vertical=True)
                previous=initial;minimum=gap
                for i in range(401):
                    ref=follow.step(i*.04,platform(i*.04,velocity=(0,0,0)),(0,0,0))
                    minimum=min(minimum,ref.position_world_m[2]-.8-.195)
                    if i:self.assertLessEqual(math.dist(ref.acceleration_world_m_s2,previous.acceleration_world_m_s2),2.*.04+1e-9)
                    previous=ref
                self.assertGreaterEqual(minimum,1.25)

    def test_wait_budget_cannot_reset_after_reacquisition(self):
        supervisor=PlanarLandingSupervisor(PlanarDesign(maximum_wait_s=.3))
        p=platform(0.,acceleration=(.4,0.,0.));s=FlightState(0.,(0,0,3),(0,0,0),(0,0,0))
        supervisor.update(0.,s,p,2.)
        for i in range(1,10):
            t=i*.04
            report=supervisor.update(t,replace(s,stamp=t),replace(p,stamp=t),2.)
        self.assertEqual(report['action'],'EXIT')
        self.assertEqual(report['reason'],'PLANAR_WAIT_TIMEOUT')

    def test_reacquisition_starts_at_exact_current_command(self):
        initial=BodyReference((1.,2.,3.),(.3,-.1,-.2),(.1,.1,.2),0.)
        reference=DeckAlignmentReference(initial,(0,0,0),start_s=2.)
        first=reference.step(2.,platform(2.),(0,0,0))
        self.assertEqual(first,initial)
        second=reference.step(2.04,platform(2.04),(0,0,0))
        self.assertLessEqual(math.dist(second.acceleration_world_m_s2,first.acceleration_world_m_s2),2.*.04+1e-10)


if __name__=='__main__':unittest.main()
