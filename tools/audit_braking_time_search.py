"""Recompute adaptive candidates and verify unchanged closed-loop acceptance."""
from pathlib import Path
import json,sys,math,hashlib
R=Path(__file__).resolve().parents[1]
run=Path(sys.argv[1])
if len(sys.argv)>2:pkg=Path(sys.argv[2])
else:
 actual=json.loads((run/'results/attempt_manifest.json').read_text())['source_sha256']['landing/braking_candidates.py']
 pkg=next(p for p in (run.parent/'frozen_source').iterdir() if (p/'landing/braking_candidates.py').exists() and hashlib.sha256((p/'landing/braking_candidates.py').read_bytes()).hexdigest()==actual)
sys.path[:0]=[str(pkg),str(R),str(R/'tools')]
from verify_unified_decision_interface import observation,deck
from landing.reference_loader import load_verified_reference
from landing.braking_candidates import state_selected_braking_plan
res=run/'results';out=run/'search_analysis';out.mkdir(exist_ok=True)
s=json.loads((res/'execution_status.json').read_text());_,g,_,profile=load_verified_reference(R)
o=observation(s['decision_observation']);d=deck(s['decision_deck']);a=s['measured_braking_detection']['measured_acceleration_m_s2']
curve,terminal,plan=state_selected_braking_plan(g,profile,o,d,a)
original=s['direct_braking_plan'];q=original['candidate_search']
bench=json.loads((res/'benchmark_search.log').read_text())
choices=plan['candidate_search']['candidates'];oldchoices=q['candidates']
def equivalent(x,y):
 if isinstance(x,dict):return isinstance(y,dict) and x.keys()==y.keys() and all(equivalent(x[k],y[k]) for k in x)
 if isinstance(x,(list,tuple)):return isinstance(y,(list,tuple)) and len(x)==len(y) and all(equivalent(a,b) for a,b in zip(x,y))
 if type(x) is bool or isinstance(x,str) or x is None:return x==y
 return isinstance(y,(int,float)) and abs(x-y)<=1e-8
module_path=Path(sys.modules[state_selected_braking_plan.__module__].__file__).resolve()
physics_verification=run/('verification_v2.json' if (run/'verification_v2.json').exists() else 'verification.json')
checks={'recomputed_with_frozen_candidate_module':module_path== (pkg/'landing/braking_candidates.py').resolve(),
        'actual_success_and_centre_contract_passed':json.loads(physics_verification.read_text())['passed'],
        'candidate_duration_recomputed_from_current_state':curve.T==s['landing_start']['duration_s']==q['selected_duration_s'],
        'candidate_grid_preserved':list(q['durations_s'])==[2.5,2.7,3.,3.3,3.6,4.],
        'all_candidate_checks_independently_recomputed':equivalent(choices,oldchoices),
        'selected_full_path_checks_passed':all(q['selected_path_checks'].values()) and q['selected_path_checks']==plan['candidate_search']['selected_path_checks'],
        'unarmed_search_computation_budget_passed':bench['maximum_wall_s']<=.04,
        'actual_event_search_within_declared_budget':q['computation_wall_s']<=.04,
        'duration_was_not_fixed_three_seconds':curve.T!=3.,
        'terminal_coordination_uses_chosen_duration':original['terminal_coordination']['continuous_descent_duration_s']==curve.T,
        'future_scenario_input_not_used':not q['future_scenario_input_used'],
        'no_closed_loop_or_contact_guarantee_claim':not q['closed_loop_or_contact_guaranteed']}
if original.get('vertical_velocity_feedback'):
 rows=[json.loads(l) for l in (res/'execution_feedback.jsonl').read_text().splitlines()]
 active=[r for r in rows if r.get('attitude_command')]
 errors=[];blend=0.;previous=None;gate_checks=[]
 for r in active:
  c=r['attitude_command'];v=c.get('vertical_velocity_tracking');ref=r['body_reference'];obs=r['observation']['motion']
  if v is None:errors.append(math.inf);continue
  raw=max(-1.2,min(.5,ref['velocity_world_m_s'][2]+1.5*(ref['position_world_m'][2]-obs['pose']['position_world_m'][2])+.18*ref['acceleration_world_m_s2'][2]))
  error=ref['velocity_world_m_s'][2]-obs['velocity_at_origin_world_m_s'][2]
  if 'horizontal_braking_released_and_measured_settled' in v:
   dt=.04 if previous is None else r['sim_t_s']-previous
   ready=r['sim_t_s']-s['main_approach_start_sim_t_s']>=original['horizontal_braking_duration_s'] and abs(obs['velocity_at_origin_world_m_s'][0])<=.1 and r['tilt_deg']<=3.
   blend=min(1.,blend+dt/.2) if ready else 0.
   gate_checks.append(ready==v['horizontal_braking_released_and_measured_settled'] and abs(blend-v['feedback_blend'])<1e-10)
  else:blend=1.
  expected=max(-1.2,min(.5,raw+blend*.8*error));previous=r['sim_t_s']
  errors.append(max(abs(v['unmodified_climb_command_m_s']-raw),abs(v['error_m_s']-error),abs(c['climb_rate_command_m_s']-expected)))
 checks['measured_vertical_feedback_recomputed']=bool(errors) and max(errors)<1e-10
 checks['feedback_gain_is_declared_and_no_fixed_height_offset']=original['vertical_velocity_feedback']['gain']==.8 and not original['vertical_velocity_feedback']['fixed_height_offset_used']
 checks['feedback_uses_current_reference_and_observation']=all(r['attitude_command']['vertical_velocity_tracking']['reference_vz_m_s']==r['body_reference']['velocity_world_m_s'][2] and r['attitude_command']['vertical_velocity_tracking']['measured_vz_m_s']==r['observation']['motion']['velocity_at_origin_world_m_s'][2] for r in active)
 if gate_checks:checks['phase_gate_and_blend_recomputed_from_actual_motion']=all(gate_checks)
result={'passed':all(checks.values()),'checks':checks,'selected_duration_s':curve.T,'measured_event_search_wall_s':q['computation_wall_s'],
        'unarmed_maximum_search_wall_s':bench['maximum_wall_s'],'candidates':q['candidates'],
        'physics_verification_file':str(physics_verification),'frozen_candidate_module':str(module_path),'source_status_sha256':hashlib.sha256((res/'execution_status.json').read_bytes()).hexdigest()}
target=out/(sys.argv[3] if len(sys.argv)>3 else 'verification.json');assert target.parent==out and not target.exists();target.write_text(json.dumps(result,indent=2));print(json.dumps(result))
