"""Slow terminal approach is an extra planning constraint, not relaxed contact."""
import unittest,json,math
from pathlib import Path
from dataclasses import replace
from copy import deepcopy
import numpy as np
from landing.receding_landing import PlannerLimits,FlightState,PlatformState,BezierLandingPlanner,derivatives
from landing.contact_geometry import AirframeGeometry
from landing.swept_envelope import SweptEnvelope
ROOT=Path(__file__).resolve().parents[1]

class TerminalDescentSpeedTests(unittest.TestCase):
 @classmethod
 def inputs(cls,cap):
  cfg=json.loads((ROOT/'config/experiments/planar_reacquisition_v1.json').read_text());limits=PlannerLimits(**cfg['planner_limits']);limits=replace(limits,terminal_downward_speed_m_s=cap)
  profile=json.loads((ROOT/'config/airframes/iris_fixed_gimbal_sim.json').read_text());p=BezierLandingPlanner(AirframeGeometry.load(ROOT/'config/airframes/iris_fixed_gimbal_sim.json'),profile['nominal_com_in_body_m'],limits,SweptEnvelope.load(ROOT/cfg['envelope_profile']));state=FlightState(0,(0,0,5.0),(.25,0,0),(0,0,0));deck=PlatformState(0,(0,0,.802),(.25,0,0),(0,0,0),half_extents=tuple(cfg['deck_half_extents']));return p,state,deck
 def test_tightened_terminal_hull_has_feasible_existing_twelve_second_candidate(self):
  p,s,d=self.inputs(.08);short,short_report=p.solve(s,d,0.,10.);self.assertIsNone(short,short_report);plan,report=p.solve(s,d,0.,12.);self.assertIsNotNone(plan,report)
  dt=plan.duration/len(plan.points);count=min(len(plan.points),math.ceil(p.limits.terminal_coverage_s/dt));peak=max(float(-np.min(derivatives(c,dt,1)[:,2])) for c in plan.points[-count:]);self.assertLessEqual(peak,.08002)
  for t in np.linspace(plan.duration-count*dt,plan.duration,101):self.assertGreaterEqual(plan.at(t)['velocity'][2],-.08002)
  self.assertTrue(plan.audit['passed']);self.assertLessEqual(plan.audit['terminal_descent_speed']['hull_peak_m_s'],.08002)
 def test_auditor_rejects_prior_fast_terminal_curve_even_with_valid_contact_endpoint(self):
  plain,s,d=self.inputs(None);plan,report=plain.solve(s,d,0.,10.);self.assertIsNotNone(plan,report);tight,_,_=self.inputs(.08)
  report=tight.audit(plan,s,d,plan.audit['usable_radius_m']);self.assertIn('terminal_downward_speed',report['failures'])
 def test_new_cap_is_finite_and_cannot_exceed_global_speed_or_conflict_with_endpoint(self):
  for cap in [0.,-.1,float('nan'),float('inf'),.75,.04]:
   with self.subTest(cap=cap):
    with self.assertRaises(ValueError):replace(PlannerLimits(downward_speed=.7,contact_speed=.08),terminal_downward_speed_m_s=cap)
