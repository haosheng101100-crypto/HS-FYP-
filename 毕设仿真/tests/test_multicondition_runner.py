"""Batch accounting and fail-closed evidence checks, without starting physics."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

def module():
    spec = importlib.util.spec_from_file_location('multicondition_runner', ROOT/'tools/run_multicondition.py')
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def matrix():
    return {'cases': [dict(case_id=f'M{i:02d}', scenario=f'multi_m{i:02d}', kind='constant', parameters={}, trigger={}, coverage={}, expected='landing', seed=i, initial={}, budget={}, auditor_version='1') for i in range(1,21)]}


class BatchTests(unittest.TestCase):
    def test_prepare_preserves_control_and_freezes_twenty_unique_cases(self):
        m=module()
        with tempfile.TemporaryDirectory() as d:
            output=Path(d)/'batch'
            m.prepare(ROOT, output, matrix())
            frozen=m.read_json(output/'matrix.json')
            self.assertEqual(len(frozen['cases']),20)
            cfg=m.read_json(output/'profiles/M01.json')
            original=m.read_json(ROOT/'config/experiments/planar_reacquisition_v1.json')
            self.assertEqual(cfg['multicondition_case'],{'case_id':'M01','scenario':'multi_m01'})
            self.assertNotIn('reacquisition_trial',cfg)
            allowed={'validation_scenarios','default_scenario','scenarios','validation_expectations','multicondition_case','reacquisition_trial'}
            self.assertEqual({k:v for k,v in cfg.items() if k not in allowed},{k:v for k,v in original.items() if k not in allowed})
            self.assertEqual(cfg['validation_scenarios'],['multi_m01'])
            m.verify_freeze(output)
            with self.assertRaises(FileExistsError):m.prepare(ROOT,output,matrix())
            (output/'cases/M01/case.json').write_text('{}')
            with self.assertRaises(ValueError):m.verify_freeze(output)

    def test_duplicate_case_rejected_before_creating_output(self):
        m=module();bad=matrix();bad['cases'][-1]=copy.deepcopy(bad['cases'][0])
        with tempfile.TemporaryDirectory() as d:
            output=Path(d)/'batch'
            with self.assertRaises(ValueError):m.prepare(ROOT,output,bad)
            self.assertFalse(output.exists())

    def test_attempt_is_recorded_before_launch_and_cannot_be_overwritten(self):
        m=module()
        with tempfile.TemporaryDirectory() as d:
            output=Path(d)/'batch';m.prepare(ROOT,output,matrix())
            row=m.begin_attempt(output,'M01','a'*40)
            index=m.read_json(output/'result_index.json')
            self.assertEqual(index['planned_cases'],20)
            self.assertEqual(index['attempted_cases'],1)
            self.assertEqual(index['unattempted_cases'],19)
            self.assertEqual(index['attempts'][0]['category'],'in_progress')
            self.assertTrue(Path(row['attempt_dir']).is_dir())
            with self.assertRaises(FileExistsError):m.begin_attempt(output,'M01','a'*40)
            row['category']='algorithm_control_or_physical_failure';m.finish_attempt(output,row)
            m.begin_attempt(output,'M02','a'*40)
            index=m.read_json(output/'result_index.json')
            self.assertEqual(index['attempted_cases'],2)
            self.assertEqual(index['planned_cases'],20)
            self.assertEqual(index['counts']['algorithm_control_or_physical_failure'],1)

    def test_success_claim_requires_closed_disarmed_native_evidence(self):
        m=module();audit={'classification':'complete_function_success','passed':True,'physics':{'passed':True},'safety':{'passed':True},'coverage':{'passed':True},'landing_objective':{'achieved':True}}
        status=dict(completed=True,armed_at_save=False,final_mode='LAND')
        good=dict(all_owned_cycle_processes_stopped=True)
        self.assertEqual(m.classify(audit,status,good,True,True,0),'complete_function_success')
        for changed in ({},dict(all_owned_cycle_processes_stopped=False)):
            self.assertEqual(m.classify(audit,status,changed,True,True,0),'environment_or_insufficient_evidence')
        self.assertEqual(m.classify(audit,status,good,False,True,0),'environment_or_insufficient_evidence')
        self.assertEqual(m.classify(audit,status,good,True,False,0),'environment_or_insufficient_evidence')
        broken={'classification':'landing'}
        self.assertEqual(m.classify(broken,status,good,True,True,0),'environment_or_insufficient_evidence')
        bad=dict(status,armed_at_save=True)
        self.assertNotEqual(m.classify(audit,bad,good,True,True,0),'complete_function_success')

    def test_malformed_history_is_retained_as_failure(self):
        m=module()
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d)/'runtime/receding_planner';folder.mkdir(parents=True)
            (folder/'request_000001.json').write_text('{broken')
            result=m.safe_history_check(Path(d))
            self.assertFalse(result['passed']);self.assertIn('error',result)

    def test_proven_zero_request_native_exit_uses_auditor_history_rules(self):
        m=module()
        with tempfile.TemporaryDirectory() as d:
            run=Path(d);results=run/'results';results.mkdir()
            (run/'runtime/receding_planner').mkdir(parents=True)
            status={'completed':False,'armed_at_save':False,'planner_request_sequence_max':0}
            rows=[{'sim_t_s':1.0,'planner_phase':'PLANAR_FOLLOW','planner_request_sequence':0},
                  {'sim_t_s':26.04,'planner_phase':'SAFE_EXIT','planner_request_sequence':0,
                   'planar_supervision':{'action':'EXIT'}}]
            events=[{'time':1.0,'event':'PLANAR_INITIAL_FOLLOW'},
                    {'time':26.04,'event':'PLANAR_EXIT'}]
            m.save(results/'execution_status.json',status)
            m.save(results/'receding_session.json',{'events':events})
            (results/'receding_control.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
            proof=m.safe_history_check(run)
            self.assertTrue(proof['passed'],proof)
            self.assertFalse(proof['planner_exercised'])
            self.assertEqual(proof['requests'],0)
            del status['planner_request_sequence_max'];m.save(results/'execution_status.json',status)
            self.assertFalse(m.safe_history_check(run)['passed'])
            status['planner_request_sequence_max']=1;m.save(results/'execution_status.json',status)
            self.assertFalse(m.safe_history_check(run)['passed'])
            status['planner_request_sequence_max']=0;m.save(results/'execution_status.json',status)
            m.save(run/'runtime/receding_planner/request.json',{'sequence':1})
            self.assertFalse(m.safe_history_check(run)['passed'])

    def test_only_dedicated_multicondition_manifest_is_accepted(self):
        m=module()
        with tempfile.TemporaryDirectory() as d:
            package=Path(d);legacy=package/'source_manifest.json'
            m.save(legacy,{'source_sha256':{'operator_backend.py':'legacy'}})
            legacy_bytes=legacy.read_bytes()
            with self.assertRaises(FileNotFoundError):m.load_package_manifest(package,m.sha(legacy))
            dedicated=package/'multicondition_source_manifest.json'
            m.save(dedicated,{'source_sha256':{'operator_multicondition_backend.py':'new'}})
            self.assertEqual(m.load_package_manifest(package,m.sha(dedicated)),{'operator_multicondition_backend.py':'new'})
            self.assertEqual(legacy.read_bytes(),legacy_bytes)
            with self.assertRaises(ValueError):m.load_package_manifest(package,'0'*64)

    def test_isolated_supervisor_and_backend_are_the_actual_launch_and_deployment(self):
        m=module()
        manifest={'operator_backend.py':'legacy',
                  'operator_multicondition_backend.py':'new_backend',
                  'operator_multicondition_supervisor.py':'new_supervisor',
                  'tools/multicondition_scene.py':'scene',
                  'tools/multicondition_bridge.py':'bridge',
                  'landing/receding_live.py':'control'}
        checks=m.deployment_map(manifest)
        self.assertEqual(checks['tools/operator_backend.py'],'new_backend')
        self.assertEqual(checks['landing/receding_live.py'],'control')
        self.assertEqual(checks['tools/multicondition_bridge.py'],'bridge')
        command=m.supervisor_command(Path('/package'),'/python')
        self.assertEqual(command[-2:],['/python','/package/operator_multicondition_supervisor.py'])
        self.assertIn('-screen 0 1280x800x24 -nolisten tcp',command)
        del manifest['operator_multicondition_backend.py']
        with self.assertRaises(ValueError):m.deployment_map(manifest)

    def test_corrupt_denominator_is_rejected_before_attempt(self):
        m=module()
        with tempfile.TemporaryDirectory() as d:
            output=Path(d)/'batch';m.prepare(ROOT,output,matrix())
            index=m.read_json(output/'result_index.json');index['planned_cases']=19
            m.save(output/'result_index.json',index)
            with self.assertRaises(ValueError):m.begin_attempt(output,'M01','a'*40)
            self.assertFalse((output/'cases/M01/attempt_001').exists())

    def test_missing_or_corrupt_persisted_audit_cannot_pass_reconciliation(self):
        m=module()
        with tempfile.TemporaryDirectory() as d:
            output=Path(d)/'batch';m.prepare(ROOT,output,matrix())
            row=m.begin_attempt(output,'M01','a'*40)
            row['category']='complete_function_success';m.finish_attempt(output,row)
            result=Path(row['attempt_dir'])/'result.json';result.write_text('{broken')
            report=m.audit_batch(output)
            self.assertFalse(report['complete'])
            self.assertEqual(report['cases'][0]['category'],'environment_or_insufficient_evidence')

    def test_compression_losslessly_preserves_original_and_no_overwrite(self):
        m=module()
        with tempfile.TemporaryDirectory() as d:
            raw=Path(d)/'model_states.jsonl';raw.write_bytes(b'{"raw":1}\n'*100)
            digest=m.sha(raw);r=m.compress_raw(Path(d))
            self.assertTrue(r['verified_lossless']);self.assertEqual(r['sha256_uncompressed'],digest)
            self.assertFalse(raw.exists());self.assertTrue(raw.with_suffix('.jsonl.gz').exists())
            raw.write_bytes(b'new attempt')
            with self.assertRaises(FileExistsError):m.compress_raw(Path(d))
            self.assertEqual(raw.read_bytes(),b'new attempt')

if __name__=='__main__':unittest.main()
