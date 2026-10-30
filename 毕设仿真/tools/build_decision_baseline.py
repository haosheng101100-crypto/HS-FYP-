"""Build the step15 offline decision baseline and traceable per-flight dataset."""
from pathlib import Path
import argparse,json,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from landing.decision_baseline import import_flight,reference_candidate,propose_from_evidence,file_sha


def build(root,out):
    cfg_path=root/'config/experiments/decision_baseline_v1.json';cfg=json.loads(cfg_path.read_text())
    if out.exists():raise ValueError('Preserve existing output; choose a new directory')
    records=[import_flight(root/n,root) for n in cfg['source_runs']]
    canonical=next(r for r in records if r['record_id']==cfg['canonical_controller_run'])
    candidates=[]
    for T in cfg['candidate_durations_s']:
        for speed in cfg['candidate_target_closing_speeds_m_s']:
            c=reference_candidate(root,speed,T,cfg['reference_check_samples'])
            c['action']['controller_context_id']=canonical['features']['controller_context_id']
            matching=[r for r in records if r['action']==c['action']]
            c['matching_physics_records']=[r['record_id'] for r in matching]
            c['physics_validated']=any(r['labels']['full_experiment_passed'] for r in matching)
            if c['physics_validated']:
                fine=reference_candidate(root,speed,T,cfg['final_candidate_check_samples'])
                assert fine['reference_feasible']
                c['reference_check']=fine['reference_check']
                rp=json.loads((root/canonical['record_id']/'results/execution_status.json').read_text())['rapid_plan']
                error=max(abs(a-b) for ax,cs in zip(c['normalized_polynomial_coefficients'],rp['normalized_polynomial_coefficients']) for a,b in zip(ax,cs))
                assert error<1e-10,'Reference reconstruction differs from archived passing action'
                c['archived_coefficient_max_abs_error']=error
            candidates.append(c)
    common=[]
    for T in cfg['candidate_durations_s']:
        cs=[c for c in candidates if c['action']['duration_s']==T]
        if all(c['reference_feasible'] for c in cs):common.append(T)
    tasks=[]
    if common:
        T=min(common)
        for c in candidates:
            if c['action']['duration_s']!=T:continue
            tasks.append({'candidate_id':c['candidate_id'],'action':c['action'],'new_physics_run_needed':not c['physics_validated'],
                'reason':'Same nominal initial condition, time, controller, model and reference envelope; vary target closing speed only. Actual entry residuals must be recorded.'})
    if not common:
        # A rejected slow reference must not become a flight task merely to
        # fill a three-way comparison. Scope the feasible medium/fast pair.
        T=min(cfg['candidate_durations_s'])
        for c in candidates:
            if c['action']['duration_s']!=T:continue
            tasks.append({'candidate_id':c['candidate_id'],'action':c['action'],
                'status':'REFERENCE_REDESIGN_NEEDED' if not c['reference_feasible'] else ('REUSE_ARCHIVED_RUN' if c['physics_validated'] else 'NEEDS_NEW_PHYSICS_RUN'),
                'new_physics_run_needed':c['reference_feasible'] and not c['physics_validated'],
                'reference_failure_reasons':c['reference_check'].get('failures',[]),
                'reason':'Fixed time and controller. Do not fly a reference-rejected action; a three-speed matched comparison has not yet been established.'})
    demonstrations={
        'nominal_entry_speed_preference_0_5':propose_from_evidence(canonical['features'],records,cfg['context_match'],.5),
        'same_entry_speed_preference_0_3':propose_from_evidence(canonical['features'],records,cfg['context_match'],.3)}
    groups={r['scenario_group_id'] for r in records};contexts={r['features']['controller_context_id'] for r in records}
    comparable=[r for r in records if r['features']['controller_context_id']==canonical['features']['controller_context_id']]
    readiness={'ready_for_learned_decision_training':False,'physical_flight_records':len(records),'nominal_scenario_groups':len(groups),
        'controller_contexts':len(contexts),'canonical_controller_physical_records':len(comparable),
        'canonical_controller_distinct_actions':len({json.dumps(r['action'],sort_keys=True) for r in comparable}),
        'reasons':['Only one nominal scenario group: no independent scenario holdout.','Only one observed action with the canonical controller: no matched speed-choice comparison.',
            'No structural safety labels; solver load is an outcome, not a safe/unsafe label.'],
        'splitting_rule':'Keep all actions, repeats and frames from a nominal scenario together; never randomly split trajectory frames as independent flights.',
        'generalisation_or_classification_accuracy_reported':False}
    out.mkdir(parents=True)
    with (out/'flight_records.jsonl').open('w',encoding='utf-8') as f:
        for r in records:f.write(json.dumps(r,ensure_ascii=False,allow_nan=False)+'\n')
    for name,value in [('reference_candidates.json',candidates),('matched_collection_plan.json',{'common_reference_feasible_durations_s':common,'tasks':tasks,'new_physics_flights_started':0}),
                       ('decision_demonstrations.json',demonstrations),('training_readiness.json',readiness),('nominal_query.json',{'features':canonical['features'],'maximum_target_closing_speed_m_s':.5})]:
        (out/name).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    summary={'implementation_completed':True,'trained_model':False,'simulation_only':True,'actuation_allowed':False,'hardware_deployment_allowed':False,
        'records':len(records),'reference_candidates':len(candidates),'reference_feasible_candidates':sum(c['reference_feasible'] for c in candidates),
        'canonical_controller_observed_passing_actions':sum(c['physics_validated'] for c in candidates),
        'nominal_lookup_proposal':demonstrations['nominal_entry_speed_preference_0_5'],'training_readiness':readiness,
        'common_reference_feasible_durations_s':common,'new_physics_flights_started':0,
        'source_sha256':{p.relative_to(root).as_posix():file_sha(p) for p in [cfg_path,root/'landing/decision_baseline.py',Path(__file__).resolve()]}}
    (out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ['source_sha256','nominal_lookup_proposal']},ensure_ascii=False))
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=ROOT/'outputs/model_audit/step15/decision_baseline_v2');a=p.parse_args();build(ROOT,a.out.resolve())
