import copy
import json
import tempfile
import unittest
from pathlib import Path
from tools.audit_planar_reacquisition import validate_evidence,raw_reversal,run_auditor


class ReacquisitionEvidenceTests(unittest.TestCase):
    def test_existing_exclusive_audit_reports_are_recomputed_without_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            run=Path(directory);results=run/'results';results.mkdir()
            report=results/'native_landing_audit.json';original=b'{"passed":true}\n';report.write_bytes(original)
            script=run/'audit_native_landing.py'
            script.write_text("from pathlib import Path\nimport sys\np=Path(sys.argv[1])/'results/native_landing_audit.json'\nassert not p.exists()\np.write_text('{\\\"passed\\\":true}\\n')\n")
            for _ in range(2):
                self.assertEqual(run_auditor(script,run,'native_landing_audit.json'),0)
                self.assertEqual(report.read_bytes(),original)
            self.assertEqual(len(list((results/'reacquisition_audit_replays').glob('*.json'))),2)

    def test_failed_recomputation_restores_original_report(self):
        with tempfile.TemporaryDirectory() as directory:
            run=Path(directory);results=run/'results';results.mkdir()
            report=results/'independent_contact_audit.json';report.write_text('{"passed":true}')
            script=run/'audit_receding_physics.py';script.write_text('raise RuntimeError("intentional audit failure")')
            self.assertNotEqual(run_auditor(script,run,'independent_contact_audit.json'),0)
            self.assertEqual(json.loads(report.read_text()),{'passed':True})

    def test_raw_reversal_rejects_isolated_negative_velocity_noise(self):
        with tempfile.TemporaryDirectory() as directory:
            run=Path(directory);(run/'results').mkdir();position=0.;records=[]
            for i in range(50):
                t=9.4+i*.033
                velocity=.2 if t<10. else (-.2 if i%2 else .2)
                position+=velocity*.033
                records.append(json.dumps({'kind':'car_pose','message':{'pose':[{'name':'landing_vehicle','header':{'stamp':{'sec':int(t),'nsec':round((t-int(t))*1e9)}},'position':{'x':position,'y':0.,'z':0.}}]}}))
            (run/'results/model_states.jsonl').write_text('\n'.join(records))
            self.assertFalse(raw_reversal(run,10.,11.1,'planar_reacquire_x'))

    def evidence(self):
        events=[dict(time=1.,event='PLANAR_INITIAL_FOLLOW'),dict(time=3.,event='ALIGNMENT_COMPLETE'),
            dict(time=4.,event='PLAN_ACTIVATED'),dict(time=5.2,event='PLANAR_REACQUIRE'),
            dict(time=8.,event='ALIGNMENT_COMPLETE'),dict(time=8.4,event='PLAN_ACTIVATED'),dict(time=11.,event='NATIVE_CAPTURE_STARTED')]
        scene=[dict(time=5.,event='PLANAR_MANEUVER_TRIGGERED',measured_leg_gap_m=2.3,measured_vertical_speed_m_s=-.1,planner_phase='TRACKING')]
        rows=[dict(sim_t_s=5.,planner_phase='TRACKING',feet=[dict(clearance_m=2.3)],observation={'motion':{'velocity_at_origin_world_m_s':[0,0,-.1]}}),
            dict(sim_t_s=5.2,planar_supervision={'action':'REACQUIRE','reacquisitions':1,'wait_elapsed_s':2.}),dict(sim_t_s=8.,planar_supervision={'action':'FOLLOW','wait_elapsed_s':5.})]
        reports={name:{'passed':True} for name in ('policy','native','contact','motion','platform')}
        reports['contact']['raw_contact_s']=11.05
        cfg={'planar_landing':{'maximum_reacquisitions':3,'maximum_wait_s':25.,'maximum_task_s':40.},'reacquisition_trial':{'trigger_gap_m':2.35,'minimum_trigger_gap_m':2.,'minimum_descent_speed_m_s':.02}}
        status={'completed':True,'armed_at_save':False,'final_mode':'LAND'}
        return events,scene,rows,reports,cfg,status

    def test_complete_branch_has_strict_ordered_landing_evidence(self):
        result=validate_evidence(*self.evidence(),raw_reversal_passed=True)
        self.assertTrue(result['passed'],result)

    def test_old_five_scene_success_or_exit_never_counts_as_reacquisition(self):
        for key in ('missing_reacquire','exit','missing_plan','contact_before_resume','budget','forged_trigger','no_raw_reversal','missing_original_audit'):
            e,s,rows,r,c,st=self.evidence();raw=True
            if key=='missing_reacquire':e=[v for v in e if v['event']!='PLANAR_REACQUIRE']
            elif key=='exit':st['completed']=False
            elif key=='missing_plan':e=e[:-2]+e[-1:]
            elif key=='contact_before_resume':r['contact']['raw_contact_s']=7.
            elif key=='budget':rows[-1]['planar_supervision']['wait_elapsed_s']=26.
            elif key=='forged_trigger':s[0]['measured_leg_gap_m']=1.
            elif key=='no_raw_reversal':raw=False
            else:r.pop('motion')
            self.assertFalse(validate_evidence(e,s,rows,r,c,st,raw_reversal_passed=raw)['passed'],key)


if __name__=='__main__':unittest.main()
