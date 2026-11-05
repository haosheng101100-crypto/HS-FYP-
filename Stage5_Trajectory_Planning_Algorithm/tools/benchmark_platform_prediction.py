"""Use completed raw motion as offline labels; no future enters any fit."""
from pathlib import Path
import sys,json,bisect,math,hashlib
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'work/receding_dependencies')]
import numpy as np
from landing.observations import PoseSample
from landing.contact_geometry import Pose
from landing.platform_prediction import estimate_platform

run=Path(sys.argv[1]);samples={}
for line in (run/'results/model_states.jsonl').open():
 r=json.loads(line)
 if r['kind']!='car_pose':continue
 for p in r['message'].get('pose',[]):
  if p['name']!='landing_vehicle':continue
  s=p['header']['stamp'];t=float(s.get('sec',0))+float(s.get('nsec',0))*1e-9
  samples[t]=PoseSample(t,Pose(tuple(p['position'].get(k,0.) for k in 'xyz'),tuple(p['orientation'].get(k,0.) for k in 'xyzw')),0.)
times=sorted(samples);poses=[samples[t] for t in times]
rows=[json.loads(x) for x in (run/'results/receding_control.jsonl').read_text().splitlines()]
exit_t=next(r['sim_t_s'] for r in rows if r['stage']=='RECOVERING')
data=[]
for span in (.35,.65,1.):
 errors=[]
 for row in rows:
  t=row['sim_t_s'];h=.8
  if not exit_t-4<=t<exit_t-h-.05:continue
  index=bisect.bisect_right(times,t)
  model=estimate_platform(poses[max(0,index-130):index],t,(.34,.4082),span=span)
  end=t+h;pred=model.predict(h);i=bisect.bisect_right(times,end)-1
  before=bisect.bisect_right(times,end-.03)-1;after=bisect.bisect_left(times,end+.03)
  velocity=(np.array(poses[after].pose.position_world_m)-np.array(poses[before].pose.position_world_m))/(times[after]-times[before])
  pe=np.linalg.norm(np.array(pred.position)[:2]-np.array(poses[i].pose.position_world_m)[:2]);ve=np.linalg.norm(np.array(pred.velocity)[:2]-velocity[:2])
  errors.append((float(pe),float(ve)))
 data.append(dict(span_s=span,probes=len(errors),rms_position_error_m=float(np.sqrt(np.mean([a*a for a,b in errors]))),rms_velocity_error_m_s=float(np.sqrt(np.mean([b*b for a,b in errors]))),max_velocity_error_m_s=max(b for a,b in errors)))
report=dict(scope='Offline 0.8s prediction before recovery; future poses used only as evaluation labels, never by estimator',metrics=data,source_sha256=hashlib.sha256((run/'results/model_states.jsonl').read_bytes()).hexdigest())
(run/'platform_prediction_benchmark.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
