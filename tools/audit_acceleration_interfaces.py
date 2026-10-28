"""Supplemental native-command and long-duration IMU-velocity checks."""
from pathlib import Path
import sys,json,math,hashlib
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R),str(R/'tools'),str(R/'work/plot_dependencies')]
from analyze_moving_rapid import raw_data
from analyze_moving_contact import fit
run=Path(sys.argv[1]);res=run/'results';out=run/'decision_analysis'
q=json.loads((out/'verification.json').read_text());assert q['passed']
s=json.loads((res/'execution_status.json').read_text());rows=[json.loads(l) for l in (res/'execution_feedback.jsonl').read_text().splitlines()]
raw,_=raw_data(run);assert hashlib.sha256((res/'model_states.jsonl').read_bytes()).hexdigest()==q['source_sha256']['model_states.jsonl']
epoch=q['event_sim_t_s'];shared=json.loads((res/'shared_platform_states.json').read_text())
errors=[];late=[]
for row in shared[::20]:
 t=row['query_sim_t_s']
 if not epoch+.2<t<q['vehicle_physical_support_audit_end_s']-.1:continue
 o,f=fit(raw['car_pose'],t,True);vx=o.motion.velocity_at_origin_world_m_s[0]+f['body_acceleration_world_m_s2'][0]*(t-o.sim_t_s)
 error=abs(row['velocity_m_s'][0]-vx);errors.append(error)
 if t-epoch>42:late.append(error)
commands=[r for r in rows if r.get('attitude_command')];g=s['verified_native_climb_limits_m_s']
checks={'full_duration_shared_velocity_matches_independent_raw':bool(errors) and max(errors)<.03,
 'velocity_after_history_rollover_matches_independent_raw':len(late)>5 and max(late)<.03,
 'native_attitude_target_received':json.loads((res/'fcu_diagnostic_capture.json').read_text())['counts'].get('ATTITUDE_TARGET',0)>0,
 'abort_quaternions_are_unit_and_native_mask_correct':bool(commands) and all(r['attitude_command']['type_mask']==7 and abs(sum(x*x for x in r['attitude_command']['quaternion_ned_wxyz'])-1)<1e-9 for r in commands),
 'abort_commands_match_declared_x_1_5_y_0_5_axis_limits':all(abs(r['attitude_command']['horizontal_acceleration_command_m_s2'][0])<=1.5+1e-8 and abs(r['attitude_command']['horizontal_acceleration_command_m_s2'][1])<=.5+1e-8 and math.hypot(*r['attitude_command']['horizontal_acceleration_command_m_s2'])<=math.sqrt(2.5)+1e-8 for r in commands),
 'abort_climb_encoding_matches_verified_native_limits':all(abs((r['attitude_command']['normalized_climb_field']-.5)*2*(g['WP_SPD_UP'] if r['attitude_command']['climb_rate_command_m_s']>=0 else g['WP_SPD_DN'])-r['attitude_command']['climb_rate_command_m_s'])<1e-9 for r in commands),
 'abort_command_slew_within_70_deg_s':all(math.hypot(b['attitude_command']['roll_command_rad']-a['attitude_command']['roll_command_rad'],b['attitude_command']['pitch_command_rad']-a['attitude_command']['pitch_command_rad'])/(b['sim_t_s']-a['sim_t_s'])<=math.radians(70)+1e-8 for a,b in zip(commands,commands[1:])),
 'guid_options_and_climb_scaling_verified':s['guid_options_confirmed']==32 and g['WP_SPD_UP']==2.5 and g['WP_SPD_DN']==1.5}
result={'passed':all(checks.values()),'checks':checks,'full_duration_velocity_samples':len(errors),'after_history_rollover_velocity_samples':len(late),'maximum_full_duration_velocity_error_m_s':max(errors),'maximum_after_rollover_velocity_error_m_s':max(late),'source_log_sha256':q['source_sha256']['model_states.jsonl']}
p=out/(sys.argv[2] if len(sys.argv)>2 else 'interface_verification.json');assert p.parent==out and not p.exists();p.write_text(json.dumps(result,indent=2));print(json.dumps(result))
