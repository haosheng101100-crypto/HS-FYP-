"""Postflight metadata join. Never supplies event time to the aircraft controller."""
from pathlib import Path
import json,sys,gzip,shutil,hashlib,bisect
from dataclasses import asdict
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R),str(R/'tools'),str(R/'work/plot_dependencies')]
from analyze_moving_rapid import raw_data
from analyze_moving_contact import fit
from landing.contact_geometry import Pose,RigidMotion,Deck,evaluate_airframe
from landing.reference_loader import load_verified_reference

run=Path(sys.argv[1]);res=run/'results';out=run/'landing_analysis';out.mkdir(exist_ok=True)
path=res/'model_states.jsonl'
if not path.exists():
 with gzip.open(res/'model_states.jsonl.gz','rb') as src,path.open('xb') as dst:shutil.copyfileobj(src,dst)
s=json.loads((res/'execution_status.json').read_text());assert s['external_scenario_driver'] and s['braking_start_sim_t_s'] is None
event=json.loads((res/'scenario_brake_event.json').read_text());epoch=event['command_sim_t_s'];raw,_=raw_data(run)
u,_=fit(raw['uav_pose'],epoch);c,_=fit(raw['car_pose'],epoch,True)
cp=raw['car_pose'][bisect.bisect_right([t for t,p in raw['car_pose']],u.sim_t_s)-1][1]
d=Deck(RigidMotion(Pose(cp.transform((0,0,.8)),cp.orientation_xyzw),c.motion.velocity_at_origin_world_m_s,c.motion.angular_velocity_world_rad_s),(1,.7))
_,g,_,_=load_verified_reference(R)
s['scenario_trigger']={'sim_t_s':epoch,'aircraft':asdict(u),'deck':asdict(d),'feet':[asdict(f) for f in evaluate_airframe(g,u.motion,d)],'postflight_only':True}
s['braking_start_sim_t_s']=epoch
s['measured_braking_detection']['delay_since_driver_command_s']=s['measured_braking_detection']['sim_t_s']-epoch
s['postflight_metadata_join']={'not_controller_input':True,'original_execution_status_preserved':True,'sources_sha256':{n:hashlib.sha256((res/n).read_bytes()).hexdigest() for n in ['execution_status.json','scenario_brake_event.json','model_states.jsonl']}}
target=out/'assembled_execution_status.json';assert not target.exists();target.write_text(json.dumps(s,indent=2))
print(json.dumps({'assembled_status':str(target),'brake_sim_t_s':epoch,'detection_delay_s':s['measured_braking_detection']['delay_since_driver_command_s'],'initial_gap_m':s['scenario_trigger']['feet'][0]['clearance_m']}))
