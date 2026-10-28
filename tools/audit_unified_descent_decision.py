"""Independent common-interface recomputation and immutable executor checks."""
from pathlib import Path
import json,sys,hashlib,math
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(Path(sys.argv[2])),str(R),str(R/'tools')]
from verify_unified_decision_interface import observation,deck
from landing.reference_loader import load_verified_reference
from landing.descent_decision import assess_interrupted_descent,state_metrics

run=Path(sys.argv[1]);pkg=Path(sys.argv[2]);res=run/'results';out=run/'unified_analysis';out.mkdir(exist_ok=True)
target=out/(sys.argv[3] if len(sys.argv)>3 else 'verification.json');assert target.parent==out and not target.exists()
s=json.loads((res/'execution_status.json').read_text());rows=[json.loads(l) for l in (res/'execution_feedback.jsonl').read_text().splitlines()]
decisions=json.loads((res/'unified_decisions.json').read_text());decision=s.get('unified_event_decision')
checks={'one_causal_event_decision':len(decisions)==1 and decisions[0]==decision,
        'same_aircraft_package_frozen':True,
        'actual_flight_records_present':len(rows)>100,
        'online_completed_without_recovery':s['completed'] and not s['external_failures'],
        'original_actual_flight_and_touchdown_limits_kept':s['contact_limits']==json.loads((R/'outputs/model_audit/step26/run_005/results/execution_status.json').read_text())['contact_limits']}
manifest=json.loads((pkg/'package_manifest.json').read_text())['source_sha256']
attempt=json.loads((res/'attempt_manifest.json').read_text())['source_sha256']
supplement=out/'supplemental_package_reference_hashes.json'
proof=json.loads(supplement.read_text()) if supplement.exists() else {}
identity=json.loads((res/'attempt_manifest.json').read_text())
guest=json.loads((res/'guest_execution_provenance.json').read_text())
proof_valid=proof.get('run')==identity['run'] and proof.get('boot_id')==guest['boot_id']
checks['same_aircraft_package_frozen']=all(
    hashlib.sha256((pkg/n).read_bytes()).hexdigest()==h and
    (attempt.get(n)==h if n in attempt else
     n.startswith('outputs/') and proof_valid and proof.get('source_sha256',{}).get(n)==h)
    for n,h in manifest.items() if n!='package_manifest.json')
_,g,_,_=load_verified_reference(R)
case='acceleration' if s.get('accelerating_decision_trial') else 'braking'
candidate=s.get('direct_braking_plan') if case=='braking' else None
if decision is not None:
 o=observation(s['decision_observation']);d=deck(s['decision_deck']);at=decision['decision_sim_t_s'];a=decision['metrics']['platform_acceleration_m_s2']
 if s.get('state_selected_braking_trial') and case=='braking':
  from landing.braking_candidates import assess_state_selected_braking
  replay=assess_state_selected_braking(g,o,d,a,at,candidate)
 else:replay=assess_interrupted_descent(g,o,d,a,at,True,candidate)
 checks['decision_recomputed_from_saved_current_state']=json.loads(json.dumps(replay))==decision
 event=json.loads((res/('scenario_acceleration_event.json' if case=='acceleration' else 'scenario_brake_event.json')).read_text())
 checks['decision_follows_external_event_without_future_input']=0<at-event['command_sim_t_s']<=.2 and o.sim_t_s<=at
 expected='ABORT_DESCENT_VALIDATE_RETURN_CORRIDOR' if case=='acceleration' else 'EXECUTE_BRAKING_DESCENT_CANDIDATE'
 checks['expected_action_selected']=decision['action']==expected
 checks['active_action_is_consistent']=all(r['unified_action']==expected for r in rows if r['sim_t_s']>=at)
 checks['correct_outcome_not_confused_with_opposite_branch']=(s.get('abort_return_completed') and not s.get('platform_landing_completed') and not s.get('platform_landing_authorised')) if case=='acceleration' else (s.get('platform_landing_completed') and s.get('platform_landing_authorised') and not s.get('abort_return_completed'))
else:checks['decision_recomputed_from_saved_current_state']=False
errors=[]
for row in rows:
 recomputed=state_metrics(g,observation(row['observation']),deck(row['measured_deck']),row['shared_acceleration_m_s2'],row['sim_t_s'])
 recorded=row['unified_state_metrics']
 errors.append(max(abs(recomputed[k]-recorded[k]) for k in ['lowest_foot_gap_m','nominal_arrest_clearance_reserve_m','whole_footprint_margin_m','tilt_deg','angular_speed_deg_s']))
checks['all_live_state_and_reserve_metrics_recomputed']=bool(errors) and max(errors)<1e-10
checks['arrest_formula_not_mislabelled_as_identified']=all(not r['unified_state_metrics']['abort_arrest_model_identified'] for r in rows)
for name in ['attitude_braking.py','abort_return.py','platform_imu.py','interrupted_descent.py']:
 checks['unchanged_executor_'+name]=(pkg/'landing'/name).read_bytes()==(R/'work/acceleration_decision_v3/landing'/name).read_bytes()
src=(pkg/'tools/contact_approach_demo.py').read_text()
checks['aircraft_does_not_read_private_scenario']=all(t not in src for t in ['trigger_foot_gap_m','scenario_brake_event.json','scenario_acceleration_event.json','independent_braking_driver.json','independent_acceleration_driver.json'])
result={'passed':all(checks.values()),'case':case,'checks':checks,'rows_recomputed':len(rows),'maximum_numeric_recomputation_error':max(errors) if errors else None,
        'scope':'Common bounded regression decision interface; no generic safe-set or identified arrest guarantee',
        'source_sha256':{n:hashlib.sha256((res/n).read_bytes()).hexdigest() for n in ['execution_status.json','execution_feedback.jsonl','unified_decisions.json','attempt_manifest.json']}}
target.write_text(json.dumps(result,indent=2));print(json.dumps(result))
