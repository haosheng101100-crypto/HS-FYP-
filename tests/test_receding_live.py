"""Control-side temporal contracts; no flight is started by these tests."""
from pathlib import Path
import sys,tempfile,unittest,json
from dataclasses import replace
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'work/receding_dependencies'))
from landing.receding_landing import FlightState,PlatformState
from landing.receding_live import FilePlannerClient,AsyncLandingReference,pack_plan,atomic_json,unpack_plan,measured_frame_reference
from landing.execution import BodyReference
from tools.receding_landing_demo import build


class LivePlannerContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner=build();cls.offset=cls.planner.offset
        cls.state=FlightState(0.,(0.,0.,4.),(0,0,0),(0,0,0))
        cls.platform=PlatformState(0.,(2.,0.,.8),(.3,0,0),(0,0,0))
        cls.future=replace(cls.state,stamp=1.2)
        cls.plan,report=cls.planner.solve(cls.future,cls.platform.predict(1.2),1.2,8.)
        assert cls.plan is not None,report

    def setUp(self):
        scratch=ROOT/'tmp';scratch.mkdir(exist_ok=True)
        self.temp=tempfile.TemporaryDirectory(dir=scratch)
        assert Path(self.temp.name).resolve().parent==scratch.resolve()
        self.addCleanup(self.temp.cleanup)
        self.client=FilePlannerClient(self.temp.name)
        hold=BodyReference(tuple(p-o for p,o in zip(self.state.position,self.offset)),(0,0,0),(0,0,0),0.)
        self.session=AsyncLandingReference(self.client,self.offset,hold)

    def step(self,t=0.,requested=True,contact=False,platform=None):
        return self.session.step(t,replace(self.state,stamp=t),platform or self.platform.predict(t),requested,contact)

    def response(self,plan=None):
        atomic_json(Path(self.temp.name)/'response.json',dict(sequence=self.client.outstanding,plan=pack_plan(plan or self.plan),report={'test':True}))

    def test_no_user_landing_request_no_solver_submission(self):
        self.assertIsNone(self.step(requested=False));self.assertIsNone(self.client.outstanding)

    def test_submit_returns_hover_without_waiting_for_solver(self):
        result=self.step();self.assertEqual(result,self.session.hold)
        self.assertEqual(self.client.request_data['now'],1.2)
        self.assertIsNone(self.session.plan)

    def test_initial_capture_times_are_explicit_and_bounded(self):
        session=AsyncLandingReference(self.client,self.offset,self.session.hold,initial_durations=(8.,10.,12.))
        session.step(0.,self.state,self.platform,True)
        self.assertEqual(self.client.request_data['durations'],[8.,10.,12.])
        for values in ((),(0.,),(16.,),(float('nan'),)):
            with self.assertRaises(ValueError):AsyncLandingReference(self.client,self.offset,self.session.hold,initial_durations=values)

    def test_future_result_waits_then_activates(self):
        self.step();self.response();self.step(.3)
        self.assertIsNotNone(self.session.pending);self.assertIsNone(self.session.plan)
        self.step(1.2);self.assertIsNotNone(self.session.plan)
        self.assertEqual(self.session.phase,'TRACKING')

    def test_late_solver_result_cannot_replace_reference(self):
        self.step();self.response();self.step(1.3)
        self.assertIsNone(self.session.plan)
        self.assertIn('SOLVER_RESULT_LATE',str(self.session.events))

    def test_contact_preempts_pending_result_and_is_irreversible(self):
        self.step();self.response();self.step(.4,contact=True)
        self.assertEqual(self.session.phase,'CONTACT_HANDOVER')
        self.assertIsNone(self.client.outstanding)
        self.assertIsNone(self.step(1.2));self.assertIsNone(self.session.plan)

    def test_platform_change_rejects_pending_activation(self):
        self.step();self.response();self.step(.3)
        changed=replace(self.platform.predict(1.2),position=(5.,0.,.8))
        self.step(1.2,platform=changed);self.assertIsNone(self.session.plan)

    def test_expired_plan_is_never_extrapolated(self):
        self.step();self.response();self.step(1.2)
        t=self.plan.arrival
        self.assertIsNone(self.step(t));self.assertEqual(self.session.phase,'TERMINAL_EXPIRED')

    def test_no_replanning_into_terminal_activation_window(self):
        self.session.plan=self.plan;self.session.model=self.platform.predict(self.plan.start)
        self.session.lead=.4;self.session.maximum_extension_s=3.
        self.session.extension_window_s=5.;self.session.early_extension_s=1.
        self.session.committed_arrival=self.plan.arrival;self.session.maximum_arrival=self.plan.arrival+3.
        t=self.plan.arrival-1.1;nominal=self.plan.at(t-self.plan.start)
        state=FlightState(t,tuple(nominal['position']),tuple(nominal['velocity']),tuple(nominal['acceleration']))
        self.assertIsNotNone(self.session.step(t,state,self.platform.predict(t),True,continue_measured_frame=True))
        self.assertIsNone(self.client.outstanding)
        self.assertEqual(self.session.plan.arrival,self.plan.arrival)

    def test_pending_result_cannot_move_committed_terminal_contact(self):
        self.session.plan=self.plan;self.session.model=self.platform.predict(self.plan.start)
        self.session.lead=.4
        t=self.plan.arrival-.5;nominal=self.plan.at(t-self.plan.start)
        # A previously validated pending replacement whose start matches the
        # current executed P/V. It must not reset arrival near physical contact.
        pending=replace(self.plan,start=t,points=self.plan.points.copy())
        dt=pending.duration/len(pending.points)
        pending.points[0,0]=nominal['position']
        pending.points[0,1]=nominal['position']+nominal['velocity']*dt/7
        self.session.pending=(pending,self.platform.predict(t))
        state=FlightState(t,tuple(nominal['position']),tuple(nominal['velocity']),tuple(nominal['acceleration']))
        self.assertIsNotNone(self.session.step(t,state,self.platform.predict(t),True,continue_measured_frame=True))
        self.assertEqual(self.session.plan.arrival,self.plan.arrival)
        self.assertIsNone(self.session.pending)
        self.assertTrue(any(v['event']=='TERMINAL_PLAN_COMMITTED' for v in self.session.events))

    def test_corrupt_result_rejected(self):
        data=pack_plan(self.plan);data['points'][0][0][0]=float('nan')
        with self.assertRaises(ValueError):unpack_plan(data)

    def test_sequence_does_not_allow_abandoned_response(self):
        self.step();sequence=self.client.outstanding;self.client.abandon()
        self.client.submit(self.future,self.platform.predict(1.2),(8.,),None)
        atomic_json(Path(self.temp.name)/'response.json',dict(sequence=sequence,plan=pack_plan(self.plan)))
        self.assertIsNone(self.client.poll())

    def test_replanning_cannot_push_contact_forever(self):
        self.step();self.response();self.step(1.2)
        self.assertAlmostEqual(self.session.maximum_arrival,self.plan.arrival+1.)
        request=self.session.request
        self.assertTrue(all(request['now']+d<=self.session.maximum_arrival+1e-8 for d in request['durations']))
        self.assertEqual(request['previous_arrival'],self.plan.arrival)

    def test_extension_budget_is_reserved_for_late_correction(self):
        self.session.maximum_extension_s=3.;self.session.extension_window_s=4.
        self.step();self.response();self.step(1.2)
        first=self.session.request
        self.assertTrue(all(first['now']+d<=self.plan.arrival+1e-8 for d in first['durations']))
        self.client.abandon();self.session.request=None;self.session.last_submission=-100.
        t=self.plan.arrival-3.;nominal=self.plan.at(t-self.plan.start)
        state=FlightState(t,tuple(nominal['position']),tuple(nominal['velocity']),tuple(nominal['acceleration']))
        self.session.step(t,state,self.platform.predict(t),True)
        late=self.session.request
        self.assertTrue(any(late['now']+d>self.plan.arrival+.9 for d in late['durations']))
        self.assertTrue(all(late['now']+d<=self.session.maximum_arrival+1e-8 for d in late['durations']))

    def test_small_early_adjustment_preserves_late_capture_reserve(self):
        self.session.maximum_extension_s=3.;self.session.extension_window_s=5.;self.session.early_extension_s=1.
        self.step();self.response();self.step(1.2)
        early=self.session.request
        self.assertTrue(any(early['now']+d>self.plan.arrival+.4 for d in early['durations']))
        self.assertTrue(all(early['now']+d<=self.plan.arrival+1.+1e-8 for d in early['durations']))
        self.assertFalse(self.session.extension_window_opened)
        self.client.abandon();self.session.request=None;self.session.last_submission=-100.
        t=self.plan.arrival-3.;nominal=self.plan.at(t-self.plan.start)
        state=FlightState(t,tuple(nominal['position']),tuple(nominal['velocity']),tuple(nominal['acceleration']))
        self.session.step(t,state,self.platform.predict(t),True)
        self.assertTrue(self.session.extension_window_opened)
        self.assertEqual(len([v for v in self.session.events if v['event']=='CAPTURE_TIME_WINDOW_OPENED']),1)

    def test_measured_frame_blend_has_consistent_pva_derivatives(self):
        import numpy as np
        plan=SimpleNamespace(arrival=10.)
        model=PlatformState(0,(0,0,.8),(0,0,0),(0,0,0))
        base=BodyReference((0,0,1.),(0,0,0),(0,0,0),0.)
        def corrected(t):
            platform=PlatformState(t,(.002+.0002*t+.0001*t*t,0,.8),(.0002+.0002*t,0,0),(.0002,0,0))
            return measured_frame_reference(plan,model,platform,t,base)[0]
        t=7.8;h=1e-5;before=corrected(t-h);now=corrected(t);after=corrected(t+h)
        velocity=(np.array(after.position_world_m)-before.position_world_m)/(2*h)
        acceleration=(np.array(after.velocity_world_m_s)-before.velocity_world_m_s)/(2*h)
        np.testing.assert_allclose(velocity,now.velocity_world_m_s,atol=1e-9)
        np.testing.assert_allclose(acceleration,now.acceleration_world_m_s2,atol=1e-8)

    def test_terminal_preview_uses_corrected_tracking_reference(self):
        import numpy as np
        self.session.plan=self.plan;self.session.model=self.platform.predict(self.plan.start)
        t=self.plan.arrival-.5;self.session.started=t-1.;self.session.last_submission=t
        platform=self.platform.predict(t)
        platform=replace(platform,position=tuple(np.array(platform.position)+(.04,0,0)),velocity=tuple(np.array(platform.velocity)+(.06,0,0)))
        nominal=self.plan.at(t-self.plan.start)
        state=FlightState(t,tuple(nominal['position']+(.04,0,0)),tuple(nominal['velocity']+(.06,0,0)),tuple(nominal['acceleration']))
        self.assertIsNotNone(self.session.step(t,state,platform,True,measured_frame_guard=True))
        self.assertEqual(self.session.phase,'TERMINAL_TRACKING')
        # Even a passed preview cannot waive measured tracking bounds.
        state=replace(state,stamp=t+.04,position=tuple(np.array(state.position)+(1.,0,0)))
        self.assertIsNone(self.session.step(t+.04,state,platform.predict(.04),True,measured_frame_guard=True))
        self.assertEqual(self.session.failure,'TRACKING_ENVELOPE')

    def test_replanning_starts_on_executed_corrected_reference(self):
        import numpy as np
        self.session.plan=self.plan;self.session.model=self.platform.predict(self.plan.start)
        t=self.plan.arrival-2.;platform=self.platform.predict(t)
        platform=replace(platform,position=tuple(np.array(platform.position)+(.08,.02,0)),velocity=tuple(np.array(platform.velocity)+(.04,.01,0)))
        future=self.session._future_state(t,platform)
        nominal=self.plan.at(future.stamp-self.plan.start)
        expected,_=measured_frame_reference(self.plan,self.session.model,platform.predict(self.session.lead),future.stamp,BodyReference(tuple(nominal['position']),tuple(nominal['velocity']),tuple(nominal['acceleration']),0.))
        np.testing.assert_allclose(future.position,expected.position_world_m)
        np.testing.assert_allclose(future.velocity,expected.velocity_world_m_s)
        np.testing.assert_allclose(future.acceleration,expected.acceleration_world_m_s2)

    def test_failed_geometry_preview_does_not_revert_executed_frame(self):
        import numpy as np
        self.session.plan=self.plan;self.session.model=self.platform.predict(self.plan.start)
        t=self.plan.arrival-.5;self.session.started=t-1.;self.session.last_submission=t
        platform=self.platform.predict(t)
        platform=replace(platform,position=tuple(np.array(platform.position)+(.4,0,0)),velocity=tuple(np.array(platform.velocity)+(.06,0,0)))
        nominal=self.plan.at(t-self.plan.start)
        state=FlightState(t,tuple(nominal['position']+(.4,0,0)),tuple(nominal['velocity']+(.06,0,0)),tuple(nominal['acceleration']))
        ref=self.session.step(t,state,platform,True,measured_frame_guard=False,continue_measured_frame=True)
        self.assertIsNotNone(ref);self.assertEqual(self.session.phase,'TERMINAL_TRACKING')
        # Geometry remains a separate backend decision; actual reference
        # tracking still rejects a real one-metre miss.
        state=replace(state,stamp=t+.04,position=tuple(np.array(state.position)+(1.,0,0)))
        self.assertIsNone(self.session.step(t+.04,state,platform.predict(.04),True,continue_measured_frame=True))
        self.assertEqual(self.session.failure,'TRACKING_ENVELOPE')

    def test_large_model_velocity_correction_can_have_small_actual_tracking_error(self):
        import numpy as np
        self.session.plan=self.plan;self.session.model=self.platform.predict(self.plan.start)
        t=self.plan.arrival-.5;self.session.started=t-1.;self.session.last_submission=t
        platform=self.platform.predict(t)
        platform=replace(platform,velocity=tuple(np.array(platform.velocity)+(.4,0,0)))
        nominal=self.plan.at(t-self.plan.start)
        state=FlightState(t,tuple(nominal['position']),tuple(nominal['velocity']+(.4,0,0)),tuple(nominal['acceleration']))
        self.assertIsNotNone(self.session.step(t,state,platform,True,continue_measured_frame=True))
        self.assertEqual(self.session.phase,'TERMINAL_TRACKING')

    def test_uncertainty_warmup_holds_then_submits_causal_tube(self):
        from landing.prediction_uncertainty import ForecastTube
        tube=ForecastTube(0.,(0.,2.),(.001,.01),(.003,.03),(.001,.01),(0,0),False)
        result=self.session.step(0.,self.state,self.platform,True,uncertainty=tube)
        self.assertEqual(result,self.session.hold)
        self.assertIsNone(self.client.outstanding)
        tube=replace(tube,ready=True,sample_counts=(0,12))
        self.session.step(0.,self.state,self.platform,True,uncertainty=tube)
        request=self.client.request_data
        self.assertEqual(request['uncertainty']['stamp'],0.)
        self.assertEqual(request['now'],1.2)
        self.assertEqual(request['uncertainty']['sample_counts'],(0,12))

    def test_forecast_uncertainty_can_reject_terminal_geometry(self):
        from landing.prediction_uncertainty import ForecastTube
        from landing.receding_live import terminal_body_prediction
        from landing.swept_envelope import SweptEnvelope
        envelope=SweptEnvelope.load(ROOT/'config/airframes/iris_swept_envelope_v1.json')
        t=self.plan.arrival-.5;nominal=self.plan.at(t-self.plan.start)
        state=FlightState(t,tuple(nominal['position']),tuple(nominal['velocity']),tuple(nominal['acceleration']))
        observation=SimpleNamespace(motion=self.plan.motion(t-self.plan.start))
        tube=ForecastTube(t,(0.,.8),(.001,1.),(.003,.1),(.001,.01),(0,12),True)
        report=terminal_body_prediction(self.plan,state,observation,self.platform.predict(t),envelope,t,.02,uncertainty=tube)
        self.assertFalse(report['passed'])
        self.assertGreater(report['uncertainty_reserve']['reserve_m'],1.)

    def test_contact_gain_transition_is_bounded_and_smooth(self):
        import numpy as np
        from landing.receding_live import terminal_feedback_gains
        base=dict(position_gain=6.25,velocity_gain=5.,acceleration_lead_s=.12,maximum_acceleration_m_s2=2.5)
        end=dict(transition_s=1.,position_gain=1.5,velocity_gain=2.4,acceleration_lead_s=0.)
        for t in np.linspace(0,2,101):
            gains,_=terminal_feedback_gains(base,end,float(t))
            for key in ('position_gain','velocity_gain','acceleration_lead_s'):
                self.assertLessEqual(gains[key],base[key]+1e-12);self.assertGreaterEqual(gains[key],end[key]-1e-12)
        self.assertEqual(terminal_feedback_gains(base,end,None)[0],base)
        h=1e-5
        for boundary in (0.,1.):
            left=terminal_feedback_gains(base,end,boundary-h)[0]['position_gain']
            right=terminal_feedback_gains(base,end,boundary+h)[0]['position_gain']
            self.assertLess(abs((right-left)/(2*h)),1e-5)


if __name__=='__main__':unittest.main()
