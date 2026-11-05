from pathlib import Path
import sys,unittest
from dataclasses import replace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'work/receding_dependencies'))
import numpy as np
from landing.receding_landing import FlightState,PlatformState,BezierLandingPlanner,derivatives
from tools.receding_landing_demo import build

class EntryCaptureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        original=build()
        cls.limits=replace(original.limits,acceleration=1.2,entry_acceleration_capture_m_s2=1.5)
        cls.planner=BezierLandingPlanner(original.geometry,original.offset,cls.limits)

    def test_exact_bounded_entry_with_tighter_free_acceleration_points(self):
        state=FlightState(0.,(0.,0.,4.),(0.,0.,0.),(1.35,0.,0.))
        deck=PlatformState(0.,(2.,0.,.8),(.3,0.,0.),(0.,0.,0.))
        plan,report=self.planner.solve(state,deck,0.,10.)
        self.assertIsNotNone(plan,report);self.assertTrue(plan.audit['passed'])
        np.testing.assert_allclose(plan.at(0.)['acceleration'],state.acceleration,atol=2e-5)
        for i,points in enumerate(plan.points):
            a=derivatives(points,plan.duration/len(plan.points),2)
            self.assertLessEqual(float(np.linalg.norm(a,axis=1).max()),1.5+2e-5)
            self.assertLessEqual(float(np.linalg.norm(a[1:] if i==0 else a,axis=1).max()),1.2+2e-5)

    def test_capture_never_exceeds_original_physical_bound(self):
        state=FlightState(0.,(0.,0.,4.),(0.,0.,0.),(1.6,0.,0.))
        with self.assertRaises(ValueError):self.planner.solve(state,PlatformState(0.,(2.,0.,.8),(.3,0.,0.),(0.,0.,0.)),0.,10.)

    def test_capture_design_cannot_reduce_nominal_bound(self):
        with self.assertRaises(ValueError):replace(self.limits,entry_acceleration_capture_m_s2=1.)

if __name__=='__main__':unittest.main()
