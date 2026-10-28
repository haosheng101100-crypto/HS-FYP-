"""Supplement finite-interval audit without the controller's pose-fit smoothing."""
from pathlib import Path
import sys,json,math,hashlib
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R),str(R/'tools')]
from analyze_moving_rapid import raw_data

run=Path(sys.argv[1]);res=run/'results';s=json.loads((res/'execution_status.json').read_text())
v=json.loads((run/'landing_analysis/verification.json').read_text())
assert hashlib.sha256((res/'model_states.jsonl').read_bytes()).hexdigest()==v['source_sha256']['model_states.jsonl']
raw,_=raw_data(run);epoch=s['braking_start_sim_t_s'];touch=v['raw_first_contact_s']
if epoch is None:epoch=json.loads((res/'scenario_brake_event.json').read_text())['command_sim_t_s']
seq=[(t,p) for t,p in raw['uav_pose'] if epoch<=t<touch]
tilts=[math.degrees(math.acos(max(-1.,min(1.,p.rotate((0,0,1))[2])))) for t,p in seq]
speeds=[];rates=[]
for (at,p),(bt,q) in zip(seq,seq[1:]):
 dt=bt-at
 if dt<=0:continue
 a=p.orientation_xyzw;b=q.orientation_xyzw
 cosine=abs(sum(x*y for x,y in zip(a,b)));rates.append(2*math.acos(max(-1.,min(1.,cosine)))/dt)
 speeds.append(tuple((y-x)/dt for x,y in zip(p.position_world_m,q.position_world_m)))
minimum=math.inf;rebound=0
for speed in speeds:minimum=min(minimum,speed[0]);rebound=max(rebound,speed[0]-minimum)
checks={'raw_all_sample_tilt_below_20_deg':bool(tilts) and max(tilts)<=20.,'raw_pose_monotone_forward':min(x[0] for x in speeds)>=-1e-6,
 'raw_continuous_descent':max(x[2] for x in speeds)<=-.15,
 'raw_no_forward_speed_burst':rebound<=.15,
 'raw_angular_speed_below_80_deg_s':max(rates)<=math.radians(80)}
result={'passed':all(checks.values()),'checks':checks,'raw_maximum_tilt_deg':max(tilts),'raw_forward_speed_range_m_s':[min(x[0] for x in speeds),max(x[0] for x in speeds)],
 'raw_maximum_speed_rebound_m_s':rebound,'raw_minimum_downward_speed_m_s':min(-x[2] for x in speeds),
 'raw_maximum_angular_speed_deg_s':math.degrees(max(rates)),'finite_difference_intervals':len(speeds),
 'source_log_sha256':v['source_sha256']['model_states.jsonl']}
out=run/'landing_analysis'/(sys.argv[2] if len(sys.argv)>2 else 'raw_motion_verification.json');assert out.parent==run/'landing_analysis' and not out.exists();out.write_text(json.dumps(result,indent=2));print(json.dumps(result))
