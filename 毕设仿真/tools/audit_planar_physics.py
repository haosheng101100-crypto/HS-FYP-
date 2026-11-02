"""Supplementary raw-pose audit for the declared force-driven planar fixture.

Original contact, aircraft COM and DataFlash auditors remain separate. The
historical event-specific DiffDrive exit auditor cannot describe this mechanism.
"""
from pathlib import Path
import sys,json,gzip,bisect,hashlib,xml.etree.ElementTree as ET
import numpy as np
repo=next((p for p in Path(__file__).resolve().parents if (p/'landing/contact_geometry.py').is_file()),Path(__file__).resolve().parent/'repository/毕设仿真');sys.path.insert(0,str(repo))
from landing.contact_geometry import Pose,RigidMotion,Deck,AirframeGeometry,evaluate_airframe,norm
from landing.execution import parse_deck_contacts
from tools.audit_guided_motion import motion_peaks

def axis_evidence(times,points,scenario,speed_limit):
 t=np.asarray(times,dtype=float);points=np.asarray(points,dtype=float)
 if t.ndim!=1 or len(t)<4 or points.shape!=(len(t),3) or not np.isfinite(t).all() or not np.isfinite(points).all():raise ValueError('Finite raw platform sequence required')
 dt=np.diff(t)
 if (dt<=0).any() or dt.max()>.06000001:raise ValueError('Raw platform pose gap or non-increasing timestamp')
 velocity=np.diff(points,axis=0)/dt[:,None];vt=(t[:-1]+t[1:])/2;acceleration=np.diff(velocity,axis=0)/np.diff(vt)[:,None]
 speeds=np.linalg.norm(velocity,axis=1);accelerations=np.linalg.norm(acceleration,axis=1)
 peaks=dict(checks={'actual_speed':bool(speeds.max()<=speed_limit)},samples=len(t),start_s=float(t[0]),end_s=float(t[-1]),max_raw_gap_s=float(dt.max()),platform_pose_freshness_s=.06,peak_speed_m_s=float(speeds.max()),peak_acceleration_m_s2=float(accelerations.max()),speed_limit_m_s=speed_limit)
 ranges=np.ptp(np.asarray(points),axis=0)
 axes=(0,) if scenario in ('planar_forward_backward','planar_reacquire_x') else (1,) if scenario in ('planar_lateral','planar_reacquire_y') else (0,1)
 return peaks,ranges,bool(all(ranges[i]>.01 for i in axes))

def audit(run):
 run=Path(run);res=run/'results';status=json.loads((res/'execution_status.json').read_text());cfg=json.loads((run/'config/experiments/receding_live_v1.json').read_text());ex=json.loads((run/'config/experiments/execution_v1.json').read_text());rows=[json.loads(s) for s in (res/'receding_control.jsonl').read_text().splitlines()]
 geometry=AirframeGeometry.load(run/ex['airframe_profile']);mapping={ex['leg_collision_prefix']+leg.name:leg.name for leg in geometry.legs};poses={'uav_pose':{},'car_pose':{}};contacts=[]
 raw=res/'model_states.jsonl';src=raw.open('rb') if raw.exists() else gzip.open(raw.with_suffix('.jsonl.gz'),'rb');h=hashlib.sha256()
 with src:
  for line in src:
   h.update(line);v=json.loads(line);k=v['kind'];m=v['message']
   if k in poses:
    for p in m.get('pose',[]):
     if p['name']!=('uav' if k=='uav_pose' else 'landing_vehicle'):continue
     stamp=p['header']['stamp'];t=float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9
     pose=Pose(tuple(p.get('position',{}).get(n,0.) for n in 'xyz'),tuple(p.get('orientation',{}).get(n,0.) for n in 'xyzw'))
     if t in poses[k]:assert poses[k][t]==pose,'Conflicting raw platform/body poses'
     poses[k][t]=pose
   elif k=='deck_contact' and parse_deck_contacts(m,ex['deck_collision'],mapping,ex['uav_model_prefix']).active:contacts.append(float(m.get('header',{}).get('stamp',{}).get('sec',0))+float(m.get('header',{}).get('stamp',{}).get('nsec',0))*1e-9)
 car=sorted(poses['car_pose'].items());ct=[t for t,p in car];start=next(v['sim_t_s'] for v in rows if v['stage']=='LANDING');end=rows[-1]['sim_t_s'];seq=[(t,p.position_world_m) for t,p in car if start<=t<=end]
 # Acceleration is measured, not compared to an invented platform hard limit.
 peaks,ranges,axes_exercised=axis_evidence([t for t,p in seq],[p for t,p in seq],status['scenario'],cfg['planar_landing']['maximum_platform_speed_m_s'])
 scenario=status['scenario']
 raw_velocity=np.diff(np.asarray([p for t,p in seq]),axis=0)/np.diff([t for t,p in seq])[:,None]
 model=ET.parse(run/'models/landing_vehicle/model.sdf');original=ET.parse(run/'models/landing_vehicle/original_differential_drive.sdf')
 canonical=lambda n:ET.tostring(n,encoding='unicode')
 plugins=[p for p in model.findall('.//plugin') if p.get('name')=='gz::sim::systems::JointController']
 checks={'raw_hash_matches_lossless_record':h.hexdigest()==json.loads((res/'raw_compression.json').read_text())['sha256_uncompressed'],'actual_platform_speed':peaks['checks']['actual_speed'],'declared_axes_exercised':axes_exercised,'original_chassis_and_deck_preserved':canonical(model.find('.//link[@name="chassis"]'))==canonical(original.find('.//link[@name="chassis"]')),'two_force_controllers':len(plugins)==2 and all(p.findtext('use_force_commands')=='true' for p in plugins),'two_prismatic_joints':len(model.findall('.//joint[@type="prismatic"]'))==2}
 report={'passed':False,'checks':checks,'scenario':scenario,'actual_platform_motion':peaks,'actual_axis_displacement_ranges_m':ranges.tolist(),'raw_velocity_axis_min_m_s':raw_velocity.min(axis=0).tolist(),'raw_velocity_axis_max_m_s':raw_velocity.max(axis=0).tolist(),'axis_exercise_threshold_m':.01,'note':'1 cm is an evidence/excitation threshold, not a flight/contact safety tolerance. No platform physical acceleration hard limit is inferred from command bounds.','raw_sha256':h.hexdigest(),'physical_model_sha256':hashlib.sha256((run/'models/landing_vehicle/model.sdf').read_bytes()).hexdigest()}
 if not status['completed']:
  recovery=next(v for v in rows if v['stage']=='RECOVERING');abort=recovery['sim_t_s'];returned=rows[-1];home=rows[0]['decision']['reference']['position_world_m'];clearances=[]
  for t,p in sorted(poses['uav_pose'].items()):
   if not abort<=t<=returned['sim_t_s']:continue
   i=bisect.bisect_right(ct,t)-1;tt,cp=car[i];assert 0<=t-tt<=.06
   deck=Deck(RigidMotion.stationary(Pose(cp.transform((0,0,.8)),cp.orientation_xyzw)),tuple(cfg['deck_half_extents']))
   clearances.append(min(f.clearance_m for f in evaluate_airframe(geometry,RigidMotion.stationary(p),deck)))
  attempts=json.loads((res/'receding_attempts.json').read_text());last=max(poses['uav_pose'].items())[1]
  checks.update(no_physical_deck_contact=not contacts,positive_raw_clearance_through_return=bool(clearances) and min(clearances)>.05,return_confirmed=bool(attempts) and returned['stage']=='HOVER',home_position_reached=norm(tuple(x-y for x,y in zip(returned['observation']['motion']['pose']['position_world_m'],home)))<.08,home_speed_settled=norm(returned['observation']['motion']['velocity_at_origin_world_m_s'])<.08,original_rejection_reported=bool(attempts) and not attempts[-1]['success'] and bool(attempts[-1]['reason']),normal_ground_land_finished=not status['armed_at_save'] and status['final_mode']=='LAND',no_unexpected_execution_error=status['external_failures']==['Automatic trial returned home after failed attempt'],raw_final_pose_on_ground=abs(last.position_world_m[2]-.195)<.03)
  report.update(minimum_exit_leg_clearance_m=min(clearances) if clearances else None,scope='Same 5 cm clearance, 8 cm return position/speed and 3 cm ground-pose gates as original exit auditor; no DiffDrive-specific event claim.')
 report['passed']=all(checks.values());return report

if __name__=='__main__':
 report=audit(sys.argv[1]);Path(sys.argv[1],'results/independent_planar_physics_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));sys.exit(0 if report['passed'] else 1)

