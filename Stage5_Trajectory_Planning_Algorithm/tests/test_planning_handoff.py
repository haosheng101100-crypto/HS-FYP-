import unittest,json
from pathlib import Path
from landing.contact_geometry import Pose,RigidMotion,Deck
from landing.execution import AircraftObservation,EntryLimits,HandoverLimits
from landing.reference_loader import load_verified_reference
from landing.interrupted_descent import nominal_uniform_descent,nominal_descent_executor
from landing.planning_handoff import seed_reference_at_planning_time

class PlanningHandoffTests(unittest.TestCase):
 def setup_case(self):
  root=Path(__file__).resolve().parents[1];_,g,_,profile=load_verified_reference(root)
  o=AircraftObservation(0.,RigidMotion(Pose((20,0,4),(0,0,0,1)),(5,0,0),(0,0,0)),True,'GUIDED',(0,0,0))
  d=Deck(RigidMotion(Pose((20.03,0,.8),(0,0,0,1)),(5,0,0),(0,0,0)),(1,.7))
  curve,terminal,_=nominal_uniform_descent(g,profile,o,d);cfg=json.loads((root/'config/experiments/execution_v1.json').read_text())
  executor=nominal_descent_executor(curve,g,terminal,EntryLimits(**cfg['entry_limits']),HandoverLimits(**cfg['handover_limits']),lambda t:d)
  return curve,o,executor
 def test_delayed_first_dispatch_keeps_input_epoch_and_advances_reference(self):
  curve,o,executor=self.setup_case();self.assertTrue(seed_reference_at_planning_time(executor,0.,o,None,None,.02))
  current=AircraftObservation(.02,curve.at(.02)['motion'],True,'GUIDED',(0,0,0));decision=executor.step(.02,current)
  self.assertEqual(executor.start_s,0.)
  self.assertAlmostEqual(decision.reference.position_world_m[0],curve.at(.02)['motion'].pose.position_world_m[0])
  self.assertGreater(decision.reference.position_world_m[0],curve.at(0.)['motion'].pose.position_world_m[0])
 def test_zero_delay_does_not_double_step_the_entry(self):
  _,o,executor=self.setup_case();self.assertFalse(seed_reference_at_planning_time(executor,0.,o,None,None,0.))
  self.assertEqual(executor.step(0.,o).phase,'TRACK_REFERENCE')
 def test_excessive_planning_delay_rejected_before_executor_started(self):
  _,o,executor=self.setup_case()
  with self.assertRaises(ValueError):seed_reference_at_planning_time(executor,0.,o,None,None,.081)
  self.assertIsNone(executor.start_s)
