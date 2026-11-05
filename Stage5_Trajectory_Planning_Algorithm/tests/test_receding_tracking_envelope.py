from pathlib import Path
import sys,json,unittest
from dataclasses import replace
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'work/receding_dependencies'),str(ROOT)]
import numpy as np
from landing.receding_landing import FlightState,PlatformState,PlannerLimits,BezierLandingPlanner
from landing.contact_geometry import Pose,RigidMotion,AirframeGeometry
from landing.execution import AircraftObservation
from landing.swept_envelope import SweptEnvelope
from landing.prediction_uncertainty import ForecastTube
import landing.receding_live as live

class TrackingEnvelopeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture=ROOT/'outputs/uncertainty_landing_20261024/batch_081/cycle_002'
        cls.rows=[json.loads(l) for l in (cls.fixture/'results/receding_control.jsonl').read_text().splitlines()]
        cls.cfg=json.loads((cls.fixture/'config/experiments/receding_live_v1.json').read_text())
        cls.env=SweptEnvelope.load(cls.fixture/cls.cfg['envelope_profile'])
        cls.profile=json.loads((cls.fixture/'config/airframes/iris_fixed_gimbal_sim.json').read_text())
        cls.geometry=AirframeGeometry.load(cls.fixture/'config/airframes/iris_fixed_gimbal_sim.json')

    def snapshot(self,t,sequence):
        row=next(r for r in self.rows if abs(r['sim_t_s']-t)<1e-8)
        plan=live.unpack_plan(json.loads((self.fixture/f'runtime/receding_planner/response_{sequence:06d}.json').read_text())['plan'])
        data=dict(row['observation']);m=data.pop('motion')
        obs=AircraftObservation(**data,motion=RigidMotion(**dict(m,pose=Pose(**m['pose']))))
        state=FlightState.from_observation(obs,plan.com_offset);platform=PlatformState(**row['platform'])
        return row,plan,state,obs,platform

    def planner(self):
        limits=PlannerLimits(**self.cfg['planner_limits'])
        limits=replace(limits,acceleration=1.2,entry_acceleration_capture_m_s2=1.5,tracking_error_tightening=True)
        return BezierLandingPlanner(self.geometry,self.profile['nominal_com_in_body_m'],limits,self.env)

    def payload(self):
        row,plan,state,obs,platform=self.snapshot(42.28,15)
        return live.tracking_contact_envelope(plan,state,obs,platform,42.28,self.env)

    def test_residual_separates_aircraft_tracking_from_platform_frame_shift(self):
        row,plan,state,obs,platform=self.snapshot(42.8,16)
        payload=live.tracking_contact_envelope(plan,state,obs,platform,42.8,self.env)
        np.testing.assert_allclose(payload['position_error_world_m'],(-.024468712636622387,-.00009840786638935827,0.),atol=1e-10)
        self.assertAlmostEqual(payload['attitude_reserve_m'],.0028855070696215917,places=10)
        self.assertEqual(payload['stamp_s'],42.8)
        self.assertIsNone(live.tracking_contact_envelope(plan,state,obs,platform,plan.arrival,self.env))

    def test_frozen_replan_compensates_residual_without_changing_nominal_bounds(self):
        data=json.loads((self.fixture/'runtime/receding_planner/response_000016.json').read_text());req=data['request']
        planner=self.planner();payload=self.payload()
        plan,report=planner.search(FlightState(**req['state']),PlatformState(**req['platform']),req['now'],req['durations'],req['previous_arrival'],ForecastTube(**req['uncertainty']),tracking_envelope=payload)
        self.assertIsNotNone(plan,report);self.assertTrue(plan.audit['passed'])
        self.assertEqual(plan.audit['tracking_contact_envelope'],payload)
        self.assertTrue(planner._template[0].is_dpp())
        platform=PlatformState(**plan.audit['platform_prediction_input']);error=np.array(payload['position_error_world_m'])
        required=planner.limits.margin+max(planner.limits.position_reserve,plan.audit['uncertainty_reserve']['reserve_m'])
        for t in np.linspace(plan.duration*7/8,plan.duration,41):
            motion=plan.motion(float(t));shifted=replace(motion,pose=replace(motion.pose,position_world_m=tuple(np.array(motion.pose.position_world_m)+error)))
            self.assertGreaterEqual(self.env.clearance(shifted,platform.predict(float(t)).deck())-payload['attitude_reserve_m'],required-2e-5)
        old=np.array(data['plan']['audit']['landing_point_offset_world_m'])
        new=np.array(plan.audit['landing_point_offset_world_m'])
        self.assertLess(np.linalg.norm(new+error),np.linalg.norm(old+error))
        np.testing.assert_allclose(plan.at(0.)['position'],req['state']['position'],atol=2e-5)
        np.testing.assert_allclose(plan.at(0.)['velocity'],req['state']['velocity'],atol=2e-5)
        np.testing.assert_allclose(plan.at(0.)['acceleration'],req['state']['acceleration'],atol=2e-5)

    def test_cached_envelope_is_reset_when_next_request_has_no_feedback(self):
        req=json.loads((self.fixture/'runtime/receding_planner/response_000016.json').read_text())['request']
        args=(FlightState(**req['state']),PlatformState(**req['platform']),req['now'],req['durations'][1],ForecastTube(**req['uncertainty']))
        p=self.planner();p.solve(*args,tracking_envelope=self.payload())
        reset,_=p.solve(*args);fresh,_=self.planner().solve(*args)
        self.assertIsNotNone(reset);self.assertIsNotNone(fresh)
        self.assertIsNone(reset.audit['tracking_contact_envelope'])
        np.testing.assert_allclose(reset.points,fresh.points,atol=1e-5)

    def test_invalid_or_future_tracking_data_fails_closed(self):
        req=json.loads((self.fixture/'runtime/receding_planner/response_000016.json').read_text())['request']
        args=(FlightState(**req['state']),PlatformState(**req['platform']),req['now'],req['durations'][1])
        p=self.planner();payload=self.payload()
        for bad in (dict(payload,position_error_world_m=[float('nan'),0,0]),dict(payload,position_error_world_m=[0,0,.01]),dict(payload,attitude_reserve_m=-.01),dict(payload,stamp_s=req['now']+.01)):
            with self.assertRaises(ValueError):p.solve(*args,tracking_envelope=bad)

    def test_async_request_forwards_exact_current_residual_and_keeps_future_c2_state(self):
        import tempfile
        row,plan,state,obs,platform=self.snapshot(42.28,15);payload=self.payload()
        req=json.loads((self.fixture/'runtime/receding_planner/response_000016.json').read_text())['request']
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as folder:
            client=live.FilePlannerClient(folder)
            session=live.AsyncLandingReference(client,plan.com_offset,plan.body_reference_at(42.28),lead_s=.4)
            session.plan=plan;session.model=PlatformState(**plan.audit['platform_prediction_input'])
            session.committed_arrival=plan.arrival;session.maximum_arrival=plan.arrival+1.
            expected=session._future_state(42.28,platform)
            session.step(42.28,state,platform,landing_requested=True,continue_measured_frame=True,uncertainty=ForecastTube(**req['uncertainty']),tracking_envelope=payload)
            sent=json.loads((Path(folder)/'request.json').read_text())
            self.assertEqual(sent['tracking_envelope'],payload)
            np.testing.assert_allclose(sent['state']['position'],expected.position,atol=1e-12)
            np.testing.assert_allclose(sent['state']['velocity'],expected.velocity,atol=1e-12)
            np.testing.assert_allclose(sent['state']['acceleration'],expected.acceleration,atol=1e-12)

if __name__=='__main__':unittest.main()

