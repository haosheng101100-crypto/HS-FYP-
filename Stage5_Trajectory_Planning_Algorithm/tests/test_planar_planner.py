import unittest
from dataclasses import replace
import numpy as np
from landing.receding_landing import FlightState,PlatformState
from landing.planar_landing import PlanarDesign
from landing.platform_filter import CausalPlatformKalman
from landing.contact_geometry import Pose
from landing.observations import PoseSample
from tools.receding_landing_demo import build


class PlanarPlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.planner=build()

    def test_no_short_capture_candidate_under_sustained_lateral_acceleration(self):
        state=FlightState(0.,(0,0,1.2),(0,.2,-.1),(0,0,0))
        platform=PlatformState(0.,(0,0,.8),(0,.2,0),(0,.3,0))
        plan,report=self.planner.search(state,platform,0.,(.8,.9),capture_design=PlanarDesign())
        self.assertIsNone(plan)
        self.assertEqual(report['solved_candidates'],0)
        self.assertEqual(report['score_bound_pruned_candidates'],0)
        self.assertTrue(all(c['status']=='CAPTURE_WINDOW_REJECTED' for c in report['candidates']))

    def test_real_optimized_lateral_and_reverse_approaches_preserve_original_audit(self):
        for velocity in ((0.,.2,0.),(-.2,0.,0.),(-.15,.15,0.)):
            state=FlightState(0.,(0.,0.,3.),(0,0,0),(0,0,0))
            platform=PlatformState(0.,(1.,.3,.8),velocity,(0,0,0))
            plan,report=self.planner.search(state,platform,0.,(6.,8.),capture_design=PlanarDesign())
            self.assertIsNotNone(plan,report)
            self.assertTrue(plan.audit['passed'])
            self.assertTrue(plan.audit['planar_capture']['passed'])
            self.assertTrue(plan.audit['planar_capture']['conditional'])
            self.assertLessEqual(plan.audit['planar_precontact']['maximum_tangential_speed_m_s'],.05+1e-6)

    def test_opt_in_screen_keeps_finite_search_score_and_selected_duration(self):
        s=FlightState(0.,(0,0,3),(0,0,0),(0,0,0));p=PlatformState(0.,(1,0,.8),(.2,0,0),(0,0,0))
        baseline,br=self.planner.search(s,p,0.,(6.,8.))
        new,nr=self.planner.search(s,p,0.,(6.,8.),capture_design=PlanarDesign())
        self.assertEqual(baseline.duration,new.duration)
        self.assertAlmostEqual(br['selected_score'],nr['selected_score'],places=5)

    def test_short_diagonal_contact_candidate_has_original_and_new_checks(self):
        state=FlightState(0.,(0,0,3),(0,0,0),(0,0,0))
        platform=PlatformState(0.,(1.,.3,.8),(-.15,.15,0.),(0,0,0))
        approach,_=self.planner.search(state,platform,0.,(6.,8.),capture_design=PlanarDesign())
        now=approach.arrival-.9;nominal=approach.at(now-approach.start)
        entry=FlightState(now,tuple(nominal['position']),tuple(nominal['velocity']),tuple(nominal['acceleration']))
        plan,report=self.planner.search(entry,platform.predict(now),now,(.8,.9,1.),capture_design=PlanarDesign())
        self.assertIsNotNone(plan,report)
        self.assertTrue(plan.audit['passed'])
        self.assertFalse(plan.audit['planar_capture']['conditional'])
        self.assertTrue(plan.audit['planar_precontact']['passed'])


class AdaptiveFilterTests(unittest.TestCase):
    def test_lateral_maneuver_increases_process_noise_without_future_data(self):
        f=CausalPlatformKalman(innovation_adaptation={'sigma_threshold':4.,'process_noise_multiplier':25.,'hold_s':.6})
        rows=[]
        for i in range(80):
            t=i*.04;y=0. if t<2. else .5*.6*(t-2.)**2
            rows.append(PoseSample(t,Pose((.2*t,y,0.),(0,0,0,1)),0.))
            if i>=8:state=f.update(rows[-20:],t,(.34,.4))
            if i==65:self.assertGreater(f.innovation_report['process_noise_multiplier'],1.)
        snapshot=f.x.copy();count=f.count
        future=PoseSample(100.,Pose((99,99,0),(0,0,0,1)),0.)
        f.update([future],state.stamp,(.34,.4))
        np.testing.assert_array_equal(f.x,snapshot)
        self.assertEqual(f.count,count)


if __name__=='__main__':unittest.main()
