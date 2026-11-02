"""Strict branch-coverage acceptance; a safe exit is NOT a passing landing trial."""
from pathlib import Path
import argparse,gzip,json,math,subprocess,sys,uuid
import numpy as np


def validate_evidence(events,scene_events,rows,reports,cfg,status,*,raw_reversal_passed=False):
    triggers=[v for v in scene_events if v.get('event')=='PLANAR_MANEUVER_TRIGGERED']
    changes=[v for v in events if v.get('event')=='PLANAR_REACQUIRE']
    checks=dict(one_measured_trigger=len(triggers)==1,reacquisition_present=bool(changes),
                original_audits_passed=all(reports.get(k,{}).get('passed') is True for k in ('policy','native','contact','motion','platform')),
                true_landing=status.get('completed') is True and status.get('armed_at_save') is False and status.get('final_mode')=='LAND',
                actual_platform_reversal=bool(raw_reversal_passed))
    design=cfg['planar_landing'];trial=cfg['reacquisition_trial']
    contact=reports.get('contact',{}).get('raw_contact_s')
    checks['finite_contact_time']=isinstance(contact,(int,float)) and math.isfinite(contact)
    for name in ('trigger_matches_raw_control','reacquisition_after_trigger','real_reacquisition_control','resume_and_new_plan_before_contact','budgets_respected'):
        checks[name]=False
    if checks['one_measured_trigger'] and checks['reacquisition_present']:
        trigger=triggers[0];t=trigger['time'];first=changes[0]['time']
        original=[v for v in events if v['event']=='PLANAR_INITIAL_FOLLOW']
        measured=[v for v in rows if abs(v['sim_t_s']-t)<1e-8]
        if len(measured)==1:
            row=measured[0];gap=min(v['clearance_m'] for v in row.get('feet',[])) if row.get('feet') else math.nan
            vz=row.get('observation',{}).get('motion',{}).get('velocity_at_origin_world_m_s',[math.nan]*3)[2]
            checks['trigger_matches_raw_control']=bool(trial['minimum_trigger_gap_m']<=gap<=trial['trigger_gap_m'] and vz<-trial['minimum_descent_speed_m_s']
                and row.get('planner_phase')=='TRACKING' and math.isclose(gap,trigger['measured_leg_gap_m'],abs_tol=1e-8)
                and math.isclose(vz,trigger['measured_vertical_speed_m_s'],abs_tol=1e-8))
        checks['reacquisition_after_trigger']=bool(first>t and all(v['time']>t for v in changes))
        checks['real_reacquisition_control']=all(any(abs(row['sim_t_s']-v['time'])<1e-8 and (row.get('planar_supervision') or {}).get('action')=='REACQUIRE' for row in rows) for v in changes)
        last=changes[-1]['time']
        resumed=[v['time'] for v in events if v['event']=='ALIGNMENT_COMPLETE' and v['time']>last]
        if resumed and checks['finite_contact_time']:
            plans=[v['time'] for v in events if v['event']=='PLAN_ACTIVATED' and v['time']>resumed[0]]
            checks['resume_and_new_plan_before_contact']=bool(plans and last<resumed[0]<plans[0]<contact)
        waits=[(row.get('planar_supervision') or {}).get('wait_elapsed_s',0.) for row in rows]
        checks['budgets_respected']=bool(len(original)==1 and len(changes)<=design['maximum_reacquisitions'] and all(math.isfinite(w) and 0<=w<=design['maximum_wait_s']+1e-8 for w in waits)
            and checks['finite_contact_time'] and 0<contact-original[0]['time']<=design['maximum_task_s']+1e-8)
    return dict(passed=all(checks.values()),checks=checks,scope='Reacquisition then landing branch only; requires original physical/native/raw-motion audits',
                reacquisitions=len(changes),trigger_count=len(triggers))


def raw_reversal(run,trigger_s,end_s,scenario):
    raw=run/'results/model_states.jsonl'
    source=raw.open('rb') if raw.exists() else gzip.open(raw.with_suffix('.jsonl.gz'),'rb')
    poses={}
    with source:
        for line in source:
            data=json.loads(line)
            if data['kind']!='car_pose':continue
            for p in data['message'].get('pose',[]):
                if p['name']!='landing_vehicle':continue
                stamp=p['header']['stamp'];t=float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9
                if trigger_s-.6<=t<=end_s:
                    point=tuple(p.get('position',{}).get(k,0.) for k in 'xyz')
                    if t in poses and poses[t]!=point:raise ValueError('Conflicting platform raw poses')
                    poses[t]=point
    times=np.asarray(sorted(poses));points=np.asarray([poses[t] for t in times])
    if len(times)<8 or np.diff(times).max()>.06000001:return False
    velocity=np.diff(points,axis=0)/np.diff(times)[:,None];mid=(times[:-1]+times[1:])/2
    axes=(0,) if scenario=='planar_reacquire_x' else (1,) if scenario=='planar_reacquire_y' else (0,1)
    # Require sustained actual sign change, rather than a single noisy derivative.
    before=velocity[(mid>=trigger_s-.4)&(mid<trigger_s)]
    after=velocity[(mid>trigger_s)&(mid<=end_s)]
    return bool(len(before)>=4 and len(after)>=4 and all(np.median(before[:,axis])>.04 and
        np.convolve((after[:,axis]<-.04).astype(int),np.ones(4,dtype=int),mode='valid').max()>=4 for axis in axes))


def run_auditor(script,run,exclusive_report=None):
    """Independently recompute exclusive reports, retaining original bytes.

    The run must be finished. Originals are temporarily moved within results,
    fresh reports retained as replay evidence, then originals restored even if
    recomputation fails. Differences fail closed; no evidence is overwritten.
    """
    script=Path(script);run=Path(run);results=run/'results'
    status=results/'execution_status.json'
    if status.exists() and json.loads(status.read_text()).get('armed_at_save',True):
        raise ValueError('Audit only completed disarmed simulation runs')
    token=uuid.uuid4().hex
    replay=results/'reacquisition_audit_replays';replay.mkdir(exist_ok=True)
    output=results/exclusive_report if exclusive_report else None
    backup=replay/(token+'_original.json') if output is not None and output.exists() else None
    if backup:output.rename(backup)
    code=1
    try:
        with (replay/(script.stem+'_'+token+'.log')).open('w') as log:
            code=subprocess.run([sys.executable,str(script.resolve()),str(run.resolve())],stdout=log,stderr=subprocess.STDOUT).returncode
        if backup:
            if not output.exists() or json.loads(output.read_text())!=json.loads(backup.read_text()):code=1
        return code
    finally:
        if backup:
            if output.exists():output.rename(replay/(token+'_recomputed.json'))
            backup.rename(output)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args();run=args.run.resolve();root=Path(__file__).resolve().parents[1]
    cfg=json.loads((run/'config/experiments/receding_live_v1.json').read_text());res=run/'results'
    if not cfg.get('reacquisition_trial'):parser.error('Explicit reacquisition profile required')
    status=json.loads((res/'execution_status.json').read_text());exit_codes={}
    scripts=['audit_planar_policy.py','audit_native_landing.py']
    if status.get('completed'):scripts.append('audit_receding_physics.py')
    scripts+=['audit_planar_guided_motion.py','audit_planar_physics.py']
    for name in scripts:
        exclusive={'audit_native_landing.py':'native_landing_audit.json','audit_receding_physics.py':'independent_contact_audit.json'}.get(name)
        exit_codes[name]=run_auditor(root/'tools'/name,run,exclusive)
    session=res/'receding_session.json'
    record=json.loads(session.read_text()) if session.exists() else json.loads((res/'receding_attempts.json').read_text())[-1]
    names=dict(policy='independent_planar_policy_audit.json',native='native_landing_audit.json',contact='independent_contact_audit.json',motion='independent_guided_motion_audit.json',platform='independent_planar_physics_audit.json')
    reports={k:json.loads((res/v).read_text()) for k,v in names.items() if (res/v).exists()}
    scene_path=res/'scene_maneuvers.json';scene=json.loads(scene_path.read_text()) if scene_path.exists() else []
    rows=[json.loads(line) for line in (res/'receding_control.jsonl').read_text().splitlines()]
    reversal=False
    if scene and reports.get('contact',{}).get('raw_contact_s'):
        reversal=raw_reversal(run,scene[0]['time'],reports['contact']['raw_contact_s'],status['scenario'])
    report=validate_evidence(record['events'],scene,rows,reports,cfg,status,raw_reversal_passed=reversal)
    report['audit_exit_codes']=exit_codes;report['checks']['auditors_reran_successfully']=bool(exit_codes) and all(code==0 for code in exit_codes.values())
    report['passed']=all(report['checks'].values())
    (res/'independent_planar_reacquisition_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2));return 0 if report['passed'] else 1


if __name__=='__main__':sys.exit(main())
