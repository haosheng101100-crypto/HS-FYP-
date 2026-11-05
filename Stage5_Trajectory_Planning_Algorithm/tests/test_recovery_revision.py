import tempfile,unittest,json
from pathlib import Path
from tools import run_multicondition as m
ROOT=Path(__file__).resolve().parents[1]

class ControlRevisionReceiptTests(unittest.TestCase):
 def revised(self):
  return {'run_commit':'a'*40,'control_commit':'a'*40,'control_baseline_commit':m.CONTROL_COMMIT,'control_changes':{'landing/receding_live.py':'b'*64},'source_sha256':{'landing/receding_live.py':'b'*64}}
 def test_changed_control_requires_its_published_revision_and_explicit_source_hashes(self):
  r=self.revised();self.assertEqual(m.verify_control_revision(r),'a'*40)
  for k in ['control_baseline_commit','control_changes']:
   bad=dict(r);bad.pop(k)
   with self.assertRaises(ValueError):m.verify_control_revision(bad)
  bad=self.revised();bad['control_changes']['landing/receding_live.py']='c'*64
  with self.assertRaises(ValueError):m.verify_control_revision(bad)
  bad=self.revised();bad['control_commit']='d'*40
  with self.assertRaises(ValueError):m.verify_control_revision(bad)
 def test_historical_baseline_receipt_remains_readable(self):
  self.assertEqual(m.verify_control_revision({'run_commit':'a'*40,'control_commit':m.CONTROL_COMMIT}),m.CONTROL_COMMIT)
 def test_actual_attempt_records_new_control_revision(self):
  with tempfile.TemporaryDirectory() as d:
   out=Path(d)/'batch';m.prepare(ROOT,out,json.loads((ROOT/'config/experiments/multicondition_matrix_v1.json').read_text()));row=m.begin_attempt(out,'M18','a'*40,control_commit='a'*40);self.assertEqual(row['control_commit'],'a'*40)
 def test_only_declared_targeted_cases_are_selected_in_requested_order(self):
  cases=json.loads((ROOT/'config/experiments/multicondition_matrix_v1.json').read_text())['cases']
  self.assertEqual([c['case_id'] for c in m.select_unattempted_cases(cases,set(),case_ids=['M18','M17','M01'])],['M18','M17','M01'])
  self.assertEqual([c['case_id'] for c in m.select_unattempted_cases(cases,{'M18'},case_ids=['M18','M17','M01'])],['M17','M01'])
  for bad in [['M99'],['M18','M18']]:
   with self.assertRaises(ValueError):m.select_unattempted_cases(cases,set(),case_ids=bad)
