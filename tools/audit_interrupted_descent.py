"""Independent audit of the nominal descent / external event / causal replan chain."""
from pathlib import Path
import sys,json,math,hashlib
R=Path(__file__).resolve().parents[1]
run=Path(sys.argv[1]);pkg=Path(sys.argv[2]);res=run/'results';out=run/'landing_analysis'
s=json.loads((res/'execution_status.json').read_text());event=json.loads((res/'scenario_brake_event.json').read_text())
q=json.loads((out/'verification.json').read_text());motion=json.loads((out/'raw_motion_verification.json').read_text())
rows=[json.loads(l) for l in (res/'execution_feedback.jsonl').read_text().splitlines()]
commands=[json.loads(l) for l in (res/'scenario_driver_commands.jsonl').read_text().splitlines()]
epoch=event['command_sim_t_s'];det=s['measured_braking_detection']['sim_t_s'];normal=[r for r in rows if r['stage']=='NORMAL_DESCENT' and r['sim_t_s']<epoch]
start=normal[0]['sim_t_s'];plan=s['nominal_descent_plan'];T=plan['duration_s'];b=plan['terminal_boundary']['aircraft']['pose']['position_world_m']
d=plan['initial_measured_deck']['motion'];expected_x=d['pose']['position_world_m'][0]+d['velocity_at_origin_world_m_s'][0]*T
prefix=[r for r in normal if r['sim_t_s']>start+.2*T];tail=[r for r in commands if epoch<=r['publish_sim_t_s']<epoch+.5]
src=(pkg/'tools/contact_approach_demo.py').read_text();io=(pkg/'tools/scene_live_io.py').read_text()
join=s['replanning_join'];delay=det-epoch
baseline=(R/'work/centred_braking_v4/landing/attitude_braking.py').read_text()
controller=(pkg/'landing/attitude_braking.py').read_text()
expected=baseline.replace('measured_thrust_mapping=False):','measured_thrust_mapping=False,own_reference_preview_s=None):').replace('        self.measured_thrust_mapping=measured_thrust_mapping','        self.measured_thrust_mapping=measured_thrust_mapping\n        if own_reference_preview_s is not None and not 0<=own_reference_preview_s<=.2:\n            raise ValueError(\'Own-reference preview must be explicitly bounded to 0..0.2 s\')\n        self.own_reference_preview_s=own_reference_preview_s').replace('        preview=.10 if self.native_rate_mode else .35 if self.braking_tail_guard else .18','        preview=.10 if self.native_rate_mode else .35 if self.braking_tail_guard else .18\n        if self.own_reference_preview_s is not None:preview=self.own_reference_preview_s')
expected_velocity=expected.replace('own_reference_preview_s=None):','own_reference_preview_s=None,horizontal_velocity_gain=2.4):').replace('        self.own_reference_preview_s=own_reference_preview_s','        self.own_reference_preview_s=own_reference_preview_s\n        if not math.isfinite(horizontal_velocity_gain) or not 0<horizontal_velocity_gain<=4.:\n            raise ValueError(\'Horizontal velocity feedback must stay within 0..4 per second\')\n        self.horizontal_velocity_gain=horizontal_velocity_gain').replace('+2.4*(reference.velocity_world_m_s[k]-v[k])','+self.horizontal_velocity_gain*(reference.velocity_world_m_s[k]-v[k])')
preview=s['direct_braking_plan']['terminal_coordination']['own_horizontal_preview_s']
import ast
preview_values=[k.value.value for node in ast.walk(ast.parse(src)) if isinstance(node,ast.Call) for k in node.keywords if k.arg=='own_reference_preview_s' and isinstance(k.value,ast.Constant)]
preview_declared=preview==.12
if s.get('state_selected_braking_trial') and preview!=.12:
 protocols=[]
 for path in run.parent.glob('experiment_protocol*.json'):
  record=json.loads(path.read_text())
  if any(c.get('index')==run.name.rsplit('_',1)[1] and c.get('package')==pkg.name for c in record.get('cases',[])):protocols.append(record)
 import datetime
 attempt=json.loads((res/'attempt_manifest.json').read_text())
 preview_declared=len(protocols)==1 and protocols[0].get('own_horizontal_preview_s')==preview and 0<=preview<=.2 and datetime.datetime.fromisoformat(protocols[0]['frozen_utc']).timestamp()<=attempt['created_unix']
checks={
 'independent_landing_and_motion_audits_passed':q['passed'] and motion['passed'],
 'full_nominal_landing_plan_precedes_braking':T==8 and all(plan['checks'].values()) and not plan['contains_future_braking_input'] and abs(plan['terminal_boundary']['contact_centroid_world_m'][0]-expected_x)<1e-7,
 'actual_normal_descent_before_external_event':epoch-start>=1.5 and len(prefix)>25 and max(r['observation']['motion']['velocity_at_origin_world_m_s'][2] for r in prefix)<-.01,
 'actual_car_uniform_before_external_event':max(abs(r['measured_deck']['motion']['velocity_at_origin_world_m_s'][0]-5) for r in normal)<.03,
 'brake_driver_triggered_during_actual_descent':all(event['checks'].values()) and event['aircraft_vertical_speed_m_s']<-.2 and abs(event['car_speed_m_s']-5)<.03,
 'aircraft_has_no_brake_preview_inputs':all(token not in src+io for token in ['trigger_foot_gap_m','scenario_brake_event','independent_braking_driver.json',"advertise('/vehicle/cmd_vel'"]),
 'car_braking_overrides_aircraft_cruise_request':len(tail)>10 and all(r['brake_latched'] and r['actual_vehicle_command_m_s']==0 and r['aircraft_cruise_request_m_s']==5 for r in tail),
 'replanning_occurs_after_shared_measured_braking':0<delay<=.2 and s['measured_braking_detection']['measured_acceleration_m_s2'][0]<-2.2 and all(r.get('attitude_command') is None for r in rows if r['sim_t_s']<det),
 'replanning_initialised_from_actual_position_velocity':join['new_to_measured_position_error_m']<.005 and join['new_to_measured_velocity_error_m_s']<.01,
 'reference_switch_bounded_without_C2_claim':join['old_to_new_position_jump_m']<.1 and join['old_to_new_velocity_jump_m_s']<.15 and not join['claim_of_exact_reference_C2_continuity'],
 'new_reference_keeps_descending':join['new_still_descends'],
 'raw_controller_status_never_received_external_event_time':s['braking_start_sim_t_s'] is None and s['measured_braking_detection']['delay_since_driver_command_s'] is None and 'scenario_trigger' not in s,
 'braking_controller_changes_limited_to_declared_preview_and_velocity_feedback':controller==baseline or controller==expected and s['direct_braking_plan']['terminal_coordination']['own_horizontal_preview_s']==.12 and 'own_reference_preview_s=.12' in src or controller==expected_velocity and preview_declared and preview in preview_values and s['direct_braking_plan']['terminal_coordination']['horizontal_velocity_gain_per_s']==4. and 'horizontal_velocity_gain=4.' in src,
}
result={'passed':all(checks.values()),'checks':checks,'normal_descent_before_brake_s':epoch-start,'actual_detection_delay_s':delay,'reference_join':join,'nominal_terminal_x_m':b[0],'braking_epoch_postflight_only':True,'source_sha256':{n:hashlib.sha256((res/n).read_bytes()).hexdigest() for n in ['execution_status.json','scenario_brake_event.json','scenario_driver_commands.jsonl','execution_feedback.jsonl']}}
target=out/(sys.argv[3] if len(sys.argv)>3 else 'interruption_verification.json');assert target.parent==out and not target.exists();target.write_text(json.dumps(result,indent=2));print(json.dumps(result))
