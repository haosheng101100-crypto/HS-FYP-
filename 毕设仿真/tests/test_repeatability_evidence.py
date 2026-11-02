"""Prevent success counts from hiding missing physical or request evidence."""
import json,unittest
from pathlib import Path
from copy import deepcopy
from tools import run_repeatability_group_a as batch
ROOT=Path(__file__).resolve().parents[1]
class EvidenceClassificationTests(unittest.TestCase):
 def setUp(self):
  case=ROOT/'experiments/ubuntu_reacquisition_20261102_session001/cases/planar_reacquire_x'
  self.status=json.loads((case/'execution_status.json').read_text())
  self.audit=json.loads((case/'independent_planar_reacquisition_audit.json').read_text())
 def classify(self,status=None,audit=None,history=True):
  fn=getattr(batch,'classify',None);self.assertTrue(callable(fn),'Evidence classification missing')
  return fn(status or self.status,audit or self.audit,0,True,history)
 def test_real_audited_landing_counts_only_with_request_history(self):
  self.assertEqual(self.classify(),'complete_function_success')
  self.assertEqual(self.classify(history=False),'environment_or_insufficient_evidence')
 def test_armed_state_cannot_be_counted_as_success(self):
  s=deepcopy(self.status);s['armed_at_save']=True
  self.assertEqual(self.classify(status=s),'algorithm_control_or_physical_failure')
 def test_missing_trigger_is_coverage_failure_even_if_landed(self):
  a=deepcopy(self.audit);a['passed']=False;a['checks']['one_measured_trigger']=False
  self.assertEqual(self.classify(audit=a),'coverage_failure')
 def test_audited_safe_exit_is_separate_from_landing(self):
  s=deepcopy(self.status);s['completed']=False
  a=deepcopy(self.audit);a['passed']=False;a['checks']['true_landing']=False
  self.assertEqual(self.classify(status=s,audit=a),'correct_safe_exit')

class RequestHistoryAuditTests(unittest.TestCase):
 def test_cancelled_middle_request_must_not_disappear(self):
  import tempfile
  with tempfile.TemporaryDirectory() as tmp:
   run=Path(tmp);folder=run/'runtime/receding_planner';folder.mkdir(parents=True)
   for n in [1,3]:
    (folder/f'request_{n:06d}.json').write_text(json.dumps({'sequence':n,'state':{'stamp':n}}))
    (folder/f'response_{n:06d}.json').write_text(json.dumps({'sequence':n}))
   (folder/'request.json').write_text((folder/'request_000003.json').read_text())
   self.assertFalse(batch.history_check(run)['passed'],'Cancelled middle request missing from archive')
 def test_final_mailbox_must_equal_its_archived_request(self):
  import tempfile
  with tempfile.TemporaryDirectory() as tmp:
   run=Path(tmp);folder=run/'runtime/receding_planner';folder.mkdir(parents=True)
   (folder/'request_000001.json').write_text('{"sequence":1,"state":{"stamp":1}}')
   (folder/'response_000001.json').write_text('{"sequence":1}')
   (folder/'request.json').write_text('{"sequence":1,"state":{"stamp":999}}')
   self.assertFalse(batch.history_check(run)['passed'],'Mailbox was changed after original archive')

class AttemptLedgerTests(unittest.TestCase):
 def fixture(self,tmp):
  import hashlib
  out=Path(tmp)/'out';out.mkdir();pkg=Path(tmp)/'pkg';pkg.mkdir();root=Path(tmp)/'repo';(root/'tools').mkdir(parents=True)
  # Temporary process fixture writes corrupt evidence; never starts Gazebo or an FCU.
  (pkg/'source_manifest.json').write_text('{"source_sha256":{}}')
  (pkg/'operator_supervisor.py').write_text('''import os,json
from pathlib import Path
s=Path(os.environ['HS_OPERATOR_SESSION']);r=s/'cycle_001/results';r.mkdir(parents=True)
(r/'execution_status.json').write_text(json.dumps(dict(completed=True,armed_at_save=False,final_mode='LAND')))
(r/'cycle_closed.json').write_text('{}')
c=s/'cycle_001/config/experiments';c.mkdir(parents=True);(c/'receding_live_v1.json').write_text('{}')
p=s/'cycle_001/runtime/receding_planner';p.mkdir(parents=True)
(p/'request_000001.json').write_text('{')
(p/'response_000001.json').write_text('{"sequence":1}')
(p/'request.json').write_text('{"sequence":1}')
''')
  (root/'tools/audit_planar_reacquisition.py').write_text('import sys;sys.exit(1)')
  profile=out/'x.json';profile.write_text('{}')
  case=dict(case_id='A_X_01',scenario='planar_reacquire_x',repeat=1,profile=str(profile),profile_sha256=hashlib.sha256(b'{}').hexdigest(),session=str(out/'cases/A_X_01/session_001'))
  (out/'matrix.json').write_text(json.dumps({'cases':[case]}));(out/'matrix_hash.json').write_text(json.dumps({'sha256':hashlib.sha256((out/'matrix.json').read_bytes()).hexdigest()}))
  (out/'code_receipt.json').write_text(json.dumps(dict(run_commit='1'*40,source_sha256={},runtime_asset_sha256={})))
  return root,out,pkg,case
 def test_existing_results_are_rejected_without_overwriting_aggregate(self):
  import tempfile
  with tempfile.TemporaryDirectory() as tmp:
   root,out,pkg,case=self.fixture(tmp);original='{"executed_A":17,"preserve":true}'
   (out/'result_index.json').write_text(original);Path(case['session']).parent.mkdir(parents=True)
   try:batch.run(root,out,root,pkg,root,root)
   except FileExistsError:pass
   except Exception as exc:self.fail('Must reject existing experiment first: '+repr(exc))
   else:self.fail('Existing experiment was accepted')
   self.assertEqual((out/'result_index.json').read_text(),original)
 def test_corrupt_evidence_records_launched_attempt_before_stopping(self):
  import tempfile
  with tempfile.TemporaryDirectory() as tmp:
   root,out,pkg,case=self.fixture(tmp)
   try:code=batch.run(root,out,root,pkg,root,root)
   except Exception as exc:self.fail('Evidence error escaped without finalizing attempt: '+repr(exc))
   self.assertEqual(code,2)
   index=json.loads((out/'result_index.json').read_text())
   self.assertEqual(index['executed_A'],1);self.assertEqual(index['unexecuted_A'],29)
   self.assertEqual(index['stage'],'STOPPED_FOR_INVESTIGATION')
   self.assertEqual(index['results'][0]['category'],'environment_or_insufficient_evidence')
   self.assertTrue((Path(case['session']).parent/'result.json').exists())

if __name__=='__main__':unittest.main()
