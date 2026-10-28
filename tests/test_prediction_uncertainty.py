from pathlib import Path
import sys, unittest, math
from dataclasses import replace
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'work/receding_dependencies'))
import numpy as np
from landing.prediction_uncertainty import DelayedForecastMonitor, ForecastTube, reaction_reserve
from landing.receding_landing import PlatformState, FlightState
from tools.receding_landing_demo import build


class ForecastUncertaintyTests(unittest.TestCase):
    def state(self, t):
        return PlatformState(t, (.3*t, 0., .8), (.3, 0., 0.), (0., 0., 0.))

    def test_forecast_never_scored_before_target_time(self):
        m = DelayedForecastMonitor(horizons=(.4, .8), minimum_samples=2)
        m.update(self.state(0.))
        for t in np.arange(.04, .4, .04):
            tube = m.update(self.state(float(t)))
            self.assertEqual(tube.sample_counts[1:], (0, 0))
        tube = m.update(self.state(.4))
        self.assertEqual(tube.sample_counts[1:], (1, 0))

    def test_warmup_and_tube_grows_after_unpredicted_acceleration(self):
        m = DelayedForecastMonitor(minimum_samples=4)
        for t in np.arange(0., 3.001, .04):
            baseline = m.update(self.state(float(t)))
        self.assertTrue(baseline.ready)
        for t in np.arange(3.04, 4.001, .04):
            u = float(t-3.)
            s = replace(self.state(float(t)), position=(.3*t+.15*u*u, 0., .8),
                        velocity=(.3+.3*u, 0., 0.), acceleration=(.3, 0., 0.))
            tube = m.update(s)
        self.assertGreater(tube.at(.8)['position_m'], baseline.at(.8)['position_m']*2)
        self.assertTrue(all(b >= a for a,b in zip(tube.position_radius_m,tube.position_radius_m[1:])))

    def test_duplicate_clock_and_unavailable_horizon_rejected(self):
        m = DelayedForecastMonitor(); tube = m.update(self.state(0.))
        with self.assertRaises(ValueError): m.update(self.state(0.))
        with self.assertRaises(ValueError): tube.at(2.01)
        with self.assertRaises(ValueError): reaction_reserve(tube, -.1, 4., .4, .5)

    def test_missing_observations_not_interpolated_across_gap(self):
        m = DelayedForecastMonitor(horizons=(.4,.8), minimum_samples=2)
        m.update(self.state(0.)); tube=m.update(self.state(.9))
        self.assertEqual(tube.sample_counts,(0,0,0))

    def test_heading_wrap_does_not_inflate_tube(self):
        m = DelayedForecastMonitor(minimum_samples=4)
        for t in np.arange(0., 3., .04):
            s=replace(self.state(float(t)),yaw=math.remainder(3.1+.1*t,2*math.pi),yaw_rate=.1)
            tube=m.update(s)
        self.assertLess(tube.at(.8)['heading_rad'],.01)

    def test_transition_errors_do_not_permanently_describe_constant_motion(self):
        m=DelayedForecastMonitor(window_s=4.,minimum_samples=4)
        for t in np.arange(0.,4.001,.04):
            s=self.state(float(t))
            if t<2.:s=replace(s,acceleration=(.12,0.,0.))
            tube=m.update(s)
        # Historical forecasts from the old acceleration regime still reside
        # in the four-second archive, but the current model is constant-speed.
        self.assertTrue(any(r[1]>.03 for r in m.errors[4]))
        self.assertLess(tube.at(.8)['position_m'],.02)

    def test_joint_landing_point_remains_inside_trust_region(self):
        planner=build()
        s=FlightState(0.,(0.,0.,2.),(0.,0.,0.),(0.,0.,0.))
        p=PlatformState(0.,(.5,0.,.8),(.3,0.,0.),(0.,0.,0.))
        plan, report=planner.solve(s,p,0.,4.)
        self.assertIsNotNone(plan,report)
        offset=plan.audit['landing_point_offset_world_m']
        self.assertGreater(np.linalg.norm(offset),1e-7)
        self.assertLessEqual(np.linalg.norm(offset),planner.limits.landing_point_trust_radius_m+1e-7)
        np.testing.assert_allclose(plan.at(4.)['position'],plan.target_position,atol=2e-5)
        self.assertTrue(planner.audit(plan,s,p,plan.audit['usable_radius_m'])['passed'])

    def test_large_uncertainty_cannot_be_silently_accepted(self):
        planner=build()
        tube=ForecastTube(0.,(0.,.8),(0.,1.),(0.,.2),(0.,.1),(0,30),True)
        s=FlightState(0.,(0.,0.,2.),(0.,0.,0.),(0.,0.,0.))
        p=PlatformState(0.,(.5,0.,.8),(.3,0.,0.),(0.,0.,0.))
        plan,report=planner.solve(s,p,0.,4.,tube)
        self.assertIsNone(plan)
        self.assertEqual(report['status'],'UNCERTAINTY_FOOTPRINT_REJECTED')

    def test_feedback_acceleration_reserve_keeps_small_deck_intercept_feasible(self):
        import json
        from landing.receding_landing import PlannerLimits,BezierLandingPlanner
        from landing.swept_envelope import SweptEnvelope
        cfg=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())
        base=build();limits=PlannerLimits(**cfg['planner_limits'])
        nominal=replace(limits,acceleration=limits.acceleration-cfg['nominal_acceleration_reserve_m_s2'])
        planner=BezierLandingPlanner(base.geometry,base.offset,nominal,SweptEnvelope.load(ROOT/cfg['envelope_profile']))
        s=FlightState(0.,(0.,0.,4.),(0,0,0),(0,0,0))
        p=PlatformState(0.,(3.,0.,.8),(.3,0.,0.),(0,0,0),half_extents=tuple(cfg['deck_half_extents']))
        plan,report=planner.solve(s,p,0.,8.)
        self.assertIsNotNone(plan,report)
        self.assertLessEqual(plan.audit['hull_upper_bounds']['acceleration'],nominal.acceleration+2e-5)
        self.assertGreater(limits.acceleration,nominal.acceleration)


if __name__=='__main__': unittest.main()
