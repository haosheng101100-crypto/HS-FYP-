import unittest,json,tempfile
from pathlib import Path
from tools import run_multicondition as m
ROOT=Path(__file__).resolve().parents[1]
class TerminalProfileTests(unittest.TestCase):
 def test_only_the_new_tighter_terminal_planning_cap_can_differ_from_baseline(self):
  base=json.loads((ROOT/'config/experiments/planar_reacquisition_v1.json').read_text());new=json.loads(json.dumps(base));new['planner_limits']['terminal_downward_speed_m_s']=base['planner_limits']['contact_speed'];new['moving_alignment']['initial_candidate_durations_s']=list(base['initial_candidate_durations_s'])
  self.assertTrue(m.verify_terminal_profile(base,new))
  for key in ['speed','acceleration','jerk','downward_speed']:
   bad=json.loads(json.dumps(new));bad['planner_limits'][key]+=.01
   with self.assertRaises(ValueError):m.verify_terminal_profile(base,bad)
  bad=json.loads(json.dumps(new));bad['planner_limits']['terminal_downward_speed_m_s']=.7
  with self.assertRaises(ValueError):m.verify_terminal_profile(base,bad)
 def test_prepare_freezes_selected_profile_without_mutating_matrix_case_or_baseline(self):
  p=ROOT/'config/experiments/planar_terminal_recovery_v2.json';baseline=ROOT/'config/experiments/planar_reacquisition_v1.json';before=baseline.read_bytes();matrix=json.loads((ROOT/'config/experiments/multicondition_matrix_v1.json').read_text())
  with tempfile.TemporaryDirectory() as d:
   out=Path(d)/'batch';frozen=m.prepare(ROOT,out,matrix,profile=p);cfg=json.loads((out/'profiles/M18.json').read_text());self.assertEqual(cfg['planner_limits']['terminal_downward_speed_m_s'],.08);self.assertEqual(frozen['cases'],matrix['cases']);self.assertEqual(baseline.read_bytes(),before);self.assertEqual(frozen['active_profile_sha256'],m.sha(p));m.verify_freeze(out)
