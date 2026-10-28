"""Audit planning epoch and first-command freshness without relabelling lag."""
from pathlib import Path
import sys,json,hashlib,math
R=Path(__file__).resolve().parents[1];run=Path(sys.argv[1]).resolve();pkg=Path(sys.argv[2]).resolve();sys.path[:0]=[str(pkg),str(R/'tools'),str(R)]
from verify_unified_decision_interface import observation,deck
from landing.braking_candidates import state_selected_braking_plan,assess_state_selected_braking
from landing.reference_loader import load_verified_reference
from landing.execution import BodyReference
s=json.loads((run/'results/execution_status.json').read_text());h=s['planning_handoff'];_,g,_,profile=load_verified_reference(R);a=s['measured_braking_detection']['measured_acceleration_m_s2']
curve,_,plan=state_selected_braking_plan(g,profile,observation(s['decision_observation']),deck(s['decision_deck']),a)
rows=[json.loads(l) for l in (run/'results/execution_feedback.jsonl').read_text().splitlines()];first=next(r for r in rows if r.get('attitude_command'));start=h['planning_input_sim_t_s'];t=h['first_command_observation_sim_t_s'];delta=t-start;expected=BodyReference.from_state(curve.at(delta));actual=BodyReference(**first['body_reference'])
error=max(abs(x-y) for key in ['position_world_m','velocity_world_m_s','acceleration_world_m_s2'] for x,y in zip(getattr(expected,key),getattr(actual,key)))
command_acceleration=h['current_admission']['metrics']['platform_acceleration_m_s2']
queries=json.loads((run/'results/shared_platform_states.json').read_text())
matching=[q for q in queries if q['acceleration_m_s2']==command_acceleration and 0<=t-q['query_sim_t_s']<=.05 and 0<=t-q['sample_sim_t_s']<=.05]
replay=assess_state_selected_braking(g,observation(h['command_observation']),deck(h['command_deck']),command_acceleration,t,s['direct_braking_plan'])
checks={'reference_epoch_kept_at_measured_input':s['main_approach_start_sim_t_s']==start and h['reference_epoch_retained'],'same_first_command_timestamp_and_captured_state':first['sim_t_s']==t and first['observation']==h['command_observation'],'planning_advance_within_original_update_gap':0<=delta<=s['contact_limits']['update_gap_s'],'measured_physics_advance_not_hidden':abs(delta-h['observed_physics_advance_during_planning_s'])<1e-10,'first_reference_uses_elapsed_physical_phase':error<1e-8,'command_observation_is_causal_and_fresh':0<=t-h['command_observation']['sim_t_s']<=.06,'command_acceleration_matches_recorded_fresh_shared_query':bool(matching),'shared_query_chain_independently_matches_physics':json.loads((run/'landing_analysis/verification.json').read_text())['physical_platform_imu']['passed'],'current_admission_recomputed':json.loads(json.dumps(replay))==h['current_admission'] and replay['execution_permitted_for_simulation'],'no_physical_state_reset':not h['physical_state_reset_used'],'seeding_only_when_nonzero_elapsed':h['seeded_without_actuator_dispatch']==(delta>0)}
result={'passed':all(checks.values()),'checks':checks,'observed_input_to_command_advance_s':delta,'maximum_first_reference_replay_error':error,'wall_planning_and_refresh_s':h['planning_and_refresh_wall_s'],'scope':'Observed simulation input-to-command phase audit; not real radio latency or identified hardware response.','source_status_sha256':hashlib.sha256((run/'results/execution_status.json').read_bytes()).hexdigest()};target=run/'landing_analysis'/(sys.argv[3] if len(sys.argv)>3 else 'planning_handoff_verification.json');assert target.parent==run/'landing_analysis' and not target.exists();target.write_text(json.dumps(result,indent=2));print(json.dumps(result))
