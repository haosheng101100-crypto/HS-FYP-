import math
from pathlib import Path
import sys
import unittest
from dataclasses import replace

ROOT = Path(__file__).resolve().parents[1]
if (ROOT/'work/receding_dependencies').exists(): sys.path.insert(0, str(ROOT/'work/receding_dependencies'))
import numpy as np
from tools.receding_landing_demo import build
from landing.receding_landing import (FlightState, PlatformState, RecedingLandingSession,
    derivatives, basis, gram)


class RecedingPlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner = build()
        cls.state = FlightState(0., (0.,0.,2.), (0.,0.,0.), (0.,0.,0.))
        cls.deck = PlatformState(0., (.5,0.,.8), (.3,0.,0.), (0.,0.,0.))
        cls.plan, cls.report = cls.planner.solve(cls.state, cls.deck, 0., 4.)

    def test_fixed_time_plan_and_independent_endpoints(self):
        self.assertIsNotNone(self.plan, self.report)
        np.testing.assert_allclose(self.plan.at(0)['position'], self.state.position, atol=2e-5)
        np.testing.assert_allclose(self.plan.at(4)['velocity'], (.3,0.,-.1), atol=2e-5)
        # Physical four foot caps meet the moving deck, not COM at deck height.
        from landing.contact_geometry import evaluate_airframe
        feet = evaluate_airframe(self.planner.geometry, self.plan.motion(4.), self.deck.predict(4.).deck())
        self.assertLess(max(abs(f.clearance_m) for f in feet), 2e-5)
        self.assertLess(max(f.tangential_speed_m_s for f in feet), 2e-5)

    def test_polynomial_derivative_matches_finite_difference(self):
        t = 1.7; h = 1e-5
        numerical = (self.plan.at(t+h)['position']-self.plan.at(t-h)['position'])/(2*h)
        np.testing.assert_allclose(numerical, self.plan.at(t)['velocity'], atol=1e-7)

    def test_gram_matches_numerical_jerk_integral(self):
        c = self.plan.points[0]; dt = self.plan.duration/len(self.plan.points)
        j = derivatives(c, dt, 3)
        exact = dt*sum(j[:, ax]@gram(4)@j[:, ax] for ax in range(3))
        ts = np.linspace(0,1,10001)
        measured = dt*np.trapezoid([np.linalg.norm(basis(4,s)@j)**2 for s in ts], ts)
        self.assertAlmostEqual(exact, measured, places=5)

    def test_too_short_intercept_is_rejected(self):
        plan, report = self.planner.solve(self.state, self.deck, 0., .2)
        self.assertIsNone(plan)
        self.assertNotEqual(report['status'], 'ACCEPTED')

    def test_stale_future_and_nonfinite_rejected(self):
        for now in (-.01, .2, float('nan')):
            with self.assertRaises(ValueError): self.planner.solve(self.state,self.deck,now,4.)
        with self.assertRaises(ValueError): replace(self.state, velocity=(float('nan'),0,0))

    def test_empty_safe_region_rejected(self):
        with self.assertRaises(ValueError):
            self.planner.solve(self.state, replace(self.deck,half_extents=(.2,.2)),0.,4.)

    def test_tampered_solution_fails_audit(self):
        from copy import deepcopy
        plan = deepcopy(self.plan); plan.points[0,3,0] += 3.
        self.assertFalse(self.planner.audit(plan,self.state,self.deck,.12)['passed'])

    def test_manual_gate_and_irreversible_contact(self):
        session = RecedingLandingSession(self.planner, (4.,))
        self.assertEqual(session.update(self.state,self.deck,0.)['action'],'WAIT_OPERATOR')
        self.assertIsNone(session.plan)
        self.assertEqual(session.update(self.state,self.deck,0.,True,True)['action'],'CONTACT_HANDOVER')
        self.assertEqual(session.update(self.state,self.deck,.01,True)['action'],'CONTACT_HANDOVER')

    def test_prediction_uses_no_future_scene_schedule(self):
        d = replace(self.deck,acceleration=(.1,.2,0),yaw_rate=.3)
        np.testing.assert_allclose(d.predict(2.).position, np.array(d.position)+2*np.array(d.velocity)+2*np.array(d.acceleration))
        self.assertAlmostEqual(d.predict(2.).yaw,.6)

    def test_no_reference_after_planned_contact(self):
        with self.assertRaises(ValueError): self.plan.at(4.01)

    def test_body_reference_uses_rotating_com_lever(self):
        t=1.5; h=1e-4
        reference=self.plan.body_reference_at(t)
        numerical=(np.array(self.plan.motion(t+h).velocity_at_origin_world_m_s)-np.array(self.plan.motion(t-h).velocity_at_origin_world_m_s))/(2*h)
        np.testing.assert_allclose(reference.acceleration_world_m_s2,numerical,atol=1e-6)

    def test_parameterized_template_reuses_graph_with_new_target(self):
        template=self.planner._template
        plan,report=self.planner.solve(self.state,replace(self.deck,position=(.7,.1,.8)),0.,5.)
        self.assertIsNotNone(plan,report)
        self.assertIs(template,self.planner._template)
        self.assertGreater(plan.target_position[1],.09)
        np.testing.assert_allclose(self.plan.at(4.)['position'],self.plan.target_position,atol=2e-5)

    def test_fast_rotation_is_rejected_without_losing_all_search(self):
        plan,report=self.planner.search(self.state,replace(self.deck,yaw_rate=.25),0.,(4.,5.))
        self.assertIsNone(plan)
        self.assertTrue(all(r['status']=='CONTACT_BOUNDARY_REJECTED' for r in report['candidates']))

    def test_deadline_discards_late_solution(self):
        session=RecedingLandingSession(self.planner,(4.,),max_compute_s=1e-9)
        decision=session.update(self.state,self.deck,0.,True)
        self.assertEqual(decision['reason'],'COMPUTATION_DEADLINE')
        self.assertIsNone(decision['reference'])

    def test_terminal_reference_is_checked_and_not_extrapolated(self):
        session=RecedingLandingSession(self.planner,(4.,),max_compute_s=5.)
        session.plan=self.plan;session.started=0.
        s=self.plan.at(3.6)
        state=FlightState(3.6,tuple(s['position']),tuple(s['velocity']),tuple(s['acceleration']))
        result=session.update(state,self.deck.predict(3.6),3.6,True)
        self.assertEqual(result['action'],'TRACK_TERMINAL_CANDIDATE')
        state=replace(state,stamp=4.)
        result=session.update(state,self.deck.predict(4.),4.,True)
        self.assertIsNone(result['reference'])
        self.assertEqual(result['reason'],'REFERENCE_EXPIRED_WITHOUT_CONTACT')


if __name__ == '__main__': unittest.main()
