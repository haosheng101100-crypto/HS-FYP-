"""Regression for long-to-short Clarabel cached numerical-state failures."""
from pathlib import Path
import sys,json,unittest
from dataclasses import replace
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'work/receding_dependencies'),str(ROOT)]
import numpy as np
from landing.receding_landing import FlightState,PlatformState,PlannerLimits,BezierLandingPlanner
from landing.contact_geometry import AirframeGeometry
from landing.swept_envelope import SweptEnvelope
from landing.prediction_uncertainty import ForecastTube

class SolverStabilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.req=json.loads((ROOT/'tests/fixtures/terminal_cache_transition.json').read_text())
        cfg=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())
        limits=PlannerLimits(**cfg['planner_limits']);reserve=cfg['nominal_acceleration_reserve_m_s2']
        cls.limits=replace(limits,acceleration=limits.acceleration-reserve,entry_acceleration_capture_m_s2=limits.acceleration)
        cls.profile=json.loads((ROOT/'config/airframes/iris_fixed_gimbal_sim.json').read_text())
        cls.envelope=SweptEnvelope.load(ROOT/cfg['envelope_profile'])

    def planner(self):
        return BezierLandingPlanner(AirframeGeometry.load(ROOT/'config/airframes/iris_fixed_gimbal_sim.json'),self.profile['nominal_com_in_body_m'],self.limits,self.envelope)

    def warm(self,p):
        plan,report=p.solve(FlightState(0.,(0.,0.,4.),(0.,0.,0.),(0.,0.,0.)),PlatformState(0.,(2.,0.,.8),(.3,0.,0.),(0.,0.,0.)),0.,10.)
        self.assertIsNotNone(plan,report);self.assertTrue(plan.audit['passed'])
        return plan

    def short(self,p,duration,shift=None):
        req=self.req;s=FlightState(**req['state']);d=PlatformState(**req['platform'])
        if shift is not None:
            s=replace(s,position=tuple(np.array(s.position)+shift));d=replace(d,position=tuple(np.array(d.position)+shift))
        return p.solve(s,d,req['now'],duration,ForecastTube(**req['uncertainty']),tracking_envelope=req.get('tracking_envelope'))

    def test_recorded_long_to_short_sequence_returns_certified_infeasibility(self):
        p=self.planner();self.warm(p);template=p._template[0]
        for duration in self.req['durations']:
            plan,report=self.short(p,duration)
            self.assertIsNone(plan)
            self.assertEqual(report['status'],'infeasible',report)
            self.assertIs(p._template[0],template)
        self.assertTrue(template.is_dpp())
        # A rejected short problem must not poison the next feasible approach.
        plan=self.warm(p)
        np.testing.assert_allclose(plan.at(0.)['position'],(0.,0.,4.),atol=2e-5)
        self.assertIs(p._template[0],template)

    def test_cold_and_cached_short_solve_agree(self):
        duration=self.req['durations'][1];cold=self.planner();hot=self.planner()
        a,ar=self.short(cold,duration);self.warm(hot);b,br=self.short(hot,duration)
        self.assertIsNone(a);self.assertIsNone(b)
        self.assertEqual(ar['status'],br['status'])

    def test_world_translation_does_not_reintroduce_cached_failure(self):
        p=self.planner();self.warm(p)
        shift=-np.array(self.req['state']['position'])
        for duration in self.req['durations'][:3]:
            plan,report=self.short(p,duration,shift)
            self.assertIsNone(plan);self.assertEqual(report['status'],'infeasible',report)

    def test_numeric_solver_reinitialized_while_dpp_problem_is_reused(self):
        import cvxpy as cp
        p=self.planner();self.warm(p);template=p._template[0]
        original=cp.Problem.solve;calls=[]
        def observed(problem,*args,**kwargs):
            calls.append(kwargs.copy());return original(problem,*args,**kwargs)
        with patch.object(cp.Problem,'solve',observed):self.short(p,self.req['durations'][0])
        self.assertEqual(len(calls),1)
        self.assertIs(calls[0].get('warm_start'),False)
        self.assertTrue(calls[0]['enforce_dpp']);self.assertEqual(calls[0]['max_iter'],100)
        self.assertIs(p._template[0],template)

    def test_solver_failure_remains_an_error_and_never_becomes_a_plan(self):
        import cvxpy as cp
        p=self.planner();self.warm(p)
        with patch.object(cp.Problem,'solve',side_effect=cp.error.SolverError('injected numerical failure')) as solve:
            plan,report=self.short(p,self.req['durations'][0])
        self.assertIsNone(plan);self.assertEqual(report['status'],'SOLVER_ERROR')
        self.assertIn('injected numerical failure',report['detail'])
        self.assertGreaterEqual(report['elapsed_s'],0.)
        self.assertEqual(report['solver_cache_policy'],'fresh_numeric_solver_reused_dpp_template')
        self.assertEqual(solve.call_count,1)

if __name__=='__main__':unittest.main()
