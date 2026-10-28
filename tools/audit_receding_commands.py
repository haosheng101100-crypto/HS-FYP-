"""Reconstruct bounds from commands actually passed to the dispatcher."""
from pathlib import Path
import argparse,json,math
import numpy as np
p=argparse.ArgumentParser();p.add_argument('run',type=Path);run=p.parse_args().run
cfg=json.loads((run/'config/experiments/receding_live_v1.json').read_text());L=cfg['planner_limits']
checks=dict(enabled=cfg.get('final_command_limits') is True,speed=True,acceleration=True,vertical_acceleration=True,tilt=True,jerk=True)
previous=np.zeros(3);last=None;count=0
for line in (run/'results/receding_control.jsonl').open():
 row=json.loads(line);ref=row['sent_decision']['reference'];t=row['sim_t_s']
 if ref is None:previous=np.zeros(3);last=None;continue
 a=np.array(ref['acceleration_world_m_s2']);v=np.array(ref['velocity_world_m_s']);dt=cfg['control_interval_s'] if last is None else t-last
 assert 0<dt<=.080001
 checks['speed'] &= np.linalg.norm(v)<=L['speed']+1e-8
 checks['acceleration'] &= np.linalg.norm(a)<=L['acceleration']+1e-8
 checks['vertical_acceleration'] &= abs(a[2])<=L['vertical_acceleration']+1e-8
 checks['tilt'] &= np.linalg.norm(a[:2])<=(9.80665+a[2])*math.tan(L['tilt_rad'])+1e-8
 checks['jerk'] &= np.linalg.norm(a-previous)/dt<=L['jerk']+1e-8
 previous=a;last=t;count+=1
checks={k:bool(v) for k,v in checks.items()};checks['commands_present']=count>0
report=dict(passed=all(checks.values()),checks=checks,scope='Final sent PVA command bounds; not a guarantee of native physical acceleration or a continuous Bezier certificate for feedback corrections.')
out=run/'results/independent_command_audit.json';assert not out.exists();out.write_text(json.dumps(report,indent=2));print(json.dumps(report))
