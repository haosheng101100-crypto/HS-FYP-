"""Raw-physics audit of a policy rejection and return; never label this a platform landing."""
from pathlib import Path
import sys,json,math,gzip,shutil,hashlib,bisect,datetime
from dataclasses import asdict
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R),str(R/'tools'),str(R/'work/plot_dependencies')]
from analyze_moving_rapid import raw_data
from analyze_moving_contact import fit
from landing.contact_geometry import Pose,RigidMotion,Deck,evaluate_airframe,norm
from landing.reference_loader import load_verified_reference
from landing.outcome_audit import classify_disarm

run=Path(sys.argv[1]);pkg=Path(sys.argv[2]);res=run/'results';out=run/'decision_analysis';out.mkdir(exist_ok=True)
target=out/'verification.json';assert not target.exists()
path=res/'model_states.jsonl'
if not path.exists():
 with gzip.open(res/'model_states.jsonl.gz','rb') as a,path.open('xb') as b:shutil.copyfileobj(a,b)
post=json.loads((res/'postflight_checks.json').read_text());assert hashlib.sha256(path.read_bytes()).hexdigest()==post['raw_sha256']
s=json.loads((res/'execution_status.json').read_text());rows=[json.loads(l) for l in (res/'execution_feedback.jsonl').read_text().splitlines()]
event=json.loads((res/'scenario_acceleration_event.json').read_text());epoch=event['command_sim_t_s'];decision=s.get('policy_decision',{});dt=decision.get('decision_sim_t_s')
raw,contacts=raw_data(run);car=raw['car_pose'];cts=[t for t,p in car];_,g,_,profile=load_verified_reference(R)
def car_pose(t):
 i=bisect.bisect_right(cts,t)-1;at,p=car[i]
 if t==at:return p
 bt,q=car[i+1];assert 0<t-at<.06 and bt-at<.06;f=(t-at)/(bt-at)
 return Pose(tuple(a+(b-a)*f for a,b in zip(p.position_world_m,q.position_world_m)),p.orientation_xyzw)
def deck(t):
 p=car_pose(t);return Deck(RigidMotion.stationary(Pose(p.transform((0,0,.8)),p.orientation_xyzw)),(1,.7))
df=[json.loads(l) for l in (res/'dataflash_final/control_messages.jsonl').read_text().splitlines()];arms=[v for v in df if v.get('mavpackettype')=='ARM']
stop=classify_disarm(s['armed_at_save'],s.get('events',[])+[v.get('Message','') for v in df if v.get('mavpackettype')=='MSG'],s['final_mode'],arms)
checks={'online_return_completed':s['completed'] and s.get('abort_return_completed',False),
 'not_mislabelled_as_platform_landing':not s.get('platform_landing_completed',True) and not s.get('platform_landing_authorised',True) and not s.get('landing_on_braking_platform_verified',True),
 'no_execution_failure_recovery':not s['external_failures'] and not s.get('recovery_rows'),
 'normal_native_disarm':stop['normal_disarm_observed'],'no_crash_or_forced_disarm':not stop['crash_detected'] and not stop['forced_disarm_detected'],
 'no_physical_deck_contact':not contacts,'all_owned_physics_processes_cleaned':not post['remaining_owned_processes'],
 'actual_event_during_normal_descent':all(event['checks'].values()) and event['aircraft_vertical_speed_m_s']<-.2 and abs(event['car_speed_m_s']-5)<.03,
 'positive_acceleration_decision_after_event':dt is not None and 0<dt-epoch<=.2 and decision['measured_acceleration_m_s2'][0]>2.2,
 'controller_has_no_external_event_timestamp':s.get('event_start_sim_t_s') is None and 'scenario_trigger' not in s}
result={'passed':False,'checks':checks,'outcome':'POLICY_ABORT_AND_RETURN_HOME; PLATFORM_LANDING_NOT_PERFORMED','normal_disarm':stop,'event_sim_t_s':epoch,'source_sha256':{n:hashlib.sha256((res/n).read_bytes()).hexdigest() for n in ['model_states.jsonl','execution_status.json','execution_feedback.jsonl','scenario_acceleration_event.json','attempt_manifest.json']}}
if not s.get('return_home_hover_verified'):
 result['passed']=False;target.write_text(json.dumps(result,indent=2));print(json.dumps(result));sys.exit(0)
end=s['return_home_hover_verified']['sim_t_s'];ret=s['return_reference'];rs=ret['start_sim_t_s']
normal=[r for r in rows if r['stage']=='NORMAL_DESCENT' and r['sim_t_s']<epoch]
checks['actual_car_uniform_during_nominal_descent']=len(normal)>25 and max(abs(r['measured_deck']['motion']['velocity_at_origin_world_m_s'][0]-5) for r in normal)<.03
checks['nominal_descent_executed_before_event']=len(normal)>25 and epoch-normal[0]['sim_t_s']>1.5 and all(s['nominal_descent_plan']['checks'].values())
acc=[]
for i in range(2,18):
 _,f=fit(car,epoch+i*.1,True);acc.append(f['body_acceleration_world_m_s2'][0])
checks['actual_positive_2_5_m_s2_acceleration']=all(abs(a-2.5)<.08 for a in acc)
vehicle_audit_end=min(car[-2][0],rows[-1]['sim_t_s'])
full_acc=[fit(car,epoch+i,True)[1]['body_acceleration_world_m_s2'][0] for i in range(1,int(vehicle_audit_end-epoch))]
checks['car_acceleration_continues_through_return']=bool(full_acc) and all(abs(a-2.5)<.08 for a in full_acc)
u0,_=fit(raw['uav_pose'],epoch)
gap_range=(1.45,1.52)
if len(sys.argv)>3:
 protocol_path=Path(sys.argv[3]);protocol=json.loads(protocol_path.read_text())
 case=next(c for c in protocol['cases'] if c['index']==run.name.rsplit('_',1)[1])
 assert case['scenario']=='acceleration' and pkg.name==case['package']
 assert datetime.datetime.fromisoformat(protocol['frozen_utc']).timestamp()<=json.loads((res/'attempt_manifest.json').read_text())['created_unix']
 assert hashlib.sha256((pkg/'tools/contact_approach_demo.py').read_bytes()).hexdigest()==case['aircraft_controller_sha256']
 gap_range=case['entry_gap_audit_range_m']
 result['predeclared_protocol_sha256']=hashlib.sha256(protocol_path.read_bytes()).hexdigest()
 result['declared_event_gap_range_m']=gap_range
checks['same_initial_height_and_aircraft_speed']=gap_range[0]<=event['lowest_foot_gap_m']<=gap_range[1] and abs(u0.motion.velocity_at_origin_world_m_s[0]-5)<.08
required=math.degrees(math.atan2(abs(decision['measured_acceleration_m_s2'][0]),9.80665));lim=decision['limits']
checks['quantified_policy_rejection_is_correct']=abs(required-decision['required_acceleration_matching_tilt_deg'])<1e-10 and required>lim['contact_tilt_deg'] and not decision['checks']['sustained_precontact_matching'] and not decision['platform_landing_authorised']
checks['no_global_impossibility_or_unverified_contact_claim']=not decision['global_infeasibility_claim'] and not decision['landing_physics_validated']
flight=[(t,p) for t,p in raw['uav_pose'] if dt<=t<=end]
gap=[min(f.clearance_m for f in evaluate_airframe(g,RigidMotion.stationary(p),deck(t))) for t,p in flight if t<=rs]
car_tail=[p for t,p in car if epoch<=t<=vehicle_audit_end]
checks['vehicle_remains_physically_supported_on_ground']=bool(car_tail) and max(abs(p.position_world_m[2]) for p in car_tail)<.01 and max(math.degrees(math.acos(max(-1.,min(1.,p.rotate((0,0,1))[2])))) for p in car_tail)<2.
checks['actual_abort_clearance_above_0_3_m']=min(gap)>=.3
checks['abort_then_measured_clear_hover_before_return']=s['return_corridor_checks']['car_ahead_of_return_path'] and s['return_corridor_checks']['clear_hover_altitude'] and norm(tuple(ret['initial_observation']['motion']['velocity_at_origin_world_m_s']))<.08
return_entry=ret['initial_observation']['motion']['pose']['position_world_m']
checks['raw_vehicle_clear_of_return_corridor']=car_pose(rs).position_world_m[0]-return_entry[0]>=3.
turning=[r for r in rows if dt<=r['sim_t_s']<s['return_home_hover_verified']['sim_t_s'] and r['observation']['motion']['velocity_at_origin_world_m_s'][0]<-.1]
checks['return_reversal_only_after_actual_clearance']=bool(turning) and min(r['sim_t_s'] for r in turning)>=rs and all(car_pose(r['sim_t_s']).position_world_m[0]-r['observation']['motion']['pose']['position_world_m'][0]>=3. for r in turning)
checks['no_nominal_reference_after_cancellation']=all(r['body_reference'] is None for r in rows if r['stage']=='ABORT_CLEARANCE')
tilts=[math.degrees(math.acos(max(-1.,min(1.,p.rotate((0,0,1))[2])))) for t,p in flight];omega=[]
for (at,p),(bt,q) in zip(flight,flight[1:]):
 if bt<=at:continue
 cosine=abs(sum(a*b for a,b in zip(p.orientation_xyzw,q.orientation_xyzw)));omega.append(2*math.acos(max(-1.,min(1.,cosine)))/(bt-at))
checks['raw_actual_tilt_below_20_deg']=max(tilts)<=20.
checks['raw_actual_angular_rate_below_80_deg_s']=max(omega)<=math.radians(80.)
checks['actual_abort_return_altitude_below_5_5_m']=max(p.position_world_m[2] for t,p in flight)<=5.5
reconstruction=[]
for r in rows:
 if r['stage']!='RETURN_HOME':continue
 t=r['sim_t_s']-rs;ref=r['body_reference']
 for k,c in enumerate(ret['axes_coefficients']):
  for order,name in enumerate(['position_world_m','velocity_world_m_s','acceleration_world_m_s2']):
   expected=sum(c[j]*math.factorial(j)/math.factorial(j-order)*(t/ret['duration_s'])**(j-order) for j in range(order,len(c)))/ret['duration_s']**order
   reconstruction.append(abs(expected-ref[name][k]))
checks['executed_return_reference_matches_saved_coefficients']=bool(reconstruction) and max(reconstruction)<1e-7
home=[r for r in rows if r['stage']=='HOME_HOLD'];hover=s['return_home_hover_verified'];hm=hover['observation']['motion']
home_error=math.dist(hm['pose']['position_world_m'],[0,0,3.6])
checks['actual_home_hover_position_speed_and_dwell']=bool(home) and home_error<.15 and norm(hm['velocity_at_origin_world_m_s'])<.08 and hover['stable_dwell_s']>=1.5
landevents=[e for e in s['events'] if 'Requested normal LAND' in e.get('message','')]
checks['ground_land_only_after_completed_home_return']=bool(landevents) and all(e['stage']=='HOME_CLEANUP_LAND' for e in landevents) and s['ground_land_scope'].startswith('Normal cleanup')
after=[r for r in rows if not r['observation']['armed'] and r['stage']=='HOME_CLEANUP_LAND']
checks['three_seconds_ground_support_after_disarm']=bool(after) and after[-1]['sim_t_s']-after[0]['sim_t_s']>=3.-.04 and max(norm(r['observation']['motion']['velocity_at_origin_world_m_s']) for r in after)<.05
checks['home_ground_landing_within_0_2_m']=bool(after) and max(math.hypot(*r['observation']['motion']['pose']['position_world_m'][:2]) for r in after)<.2
commands=[json.loads(l) for l in (res/'scenario_driver_commands.jsonl').read_text().splitlines()];tail=[r for r in commands if epoch<=r['publish_sim_t_s']<epoch+.5]
checks['independent_driver_overrides_unchanged_cruise_request']=len(tail)>10 and all(r['acceleration_latched'] and r['actual_vehicle_command_m_s']==200 and r['aircraft_cruise_request_m_s']==5 for r in tail)
src=(pkg/'tools/contact_approach_demo.py').read_text();io=(pkg/'tools/scene_live_io.py').read_text()
checks['aircraft_source_has_no_event_preview_or_physical_car_publisher']=all(t not in src+io for t in ['scenario_acceleration_event','independent_acceleration_driver.json','trigger_foot_gap_m',"advertise('/vehicle/cmd_vel'"])
manifest=json.loads((res/'attempt_manifest.json').read_text())['source_sha256'];names=['landing/acceleration_decision.py','landing/abort_return.py','tools/contact_approach_demo.py','tools/independent_acceleration_driver.py','landing/platform_imu.py']
checks['frozen_executed_controller_sources_match']=all(manifest[n]==hashlib.sha256((pkg/n).read_bytes()).hexdigest() for n in names)
checks['original_airframe_hashes_match']=all(manifest[n.replace('\\','/')]==d for n,d in profile['sources']['fixed_model_sha256'].items())
old=json.loads((R/'outputs/model_audit/step26/run_005/results/dataflash_final/parameters.json').read_text());new=json.loads((res/'dataflash_final/parameters.json').read_text())
params={k:abs(v-new[k])<1e-8 for k,v in old.items() if k.startswith('ATC_')};checks['native_attitude_parameters_unchanged']=all(params.values())
from audit_shared_imu import imu_evidence
imu=imu_evidence(run,car);checks.update(imu['checks'])
result.update(passed=all(checks.values()),normal_descent_before_event_s=epoch-normal[0]['sim_t_s'],decision_delay_s=dt-epoch,
 actual_acceleration_range_m_s2=[min(full_acc),max(full_acc)],minimum_abort_foot_gap_m=min(gap),maximum_actual_tilt_deg=max(tilts),maximum_raw_angular_rate_deg_s=math.degrees(max(omega)),
 event_to_clear_return_entry_s=rs-epoch,event_to_home_hover_s=end-epoch,home_hover_position_error_m=home_error,
 maximum_return_reference_reconstruction_error=max(reconstruction),physical_platform_imu=imu,native_attitude_parameter_count=len(params),
 required_contact_matching_tilt_deg=required,vehicle_physical_support_audit_end_s=vehicle_audit_end,landing_not_performed=True,real_battery_mapping_or_hardware_validated=False)
target.write_text(json.dumps(result,indent=2));print(json.dumps(result))
