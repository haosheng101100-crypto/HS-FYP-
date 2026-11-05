"""Post-flight plots from full raw physical poses; target reference labelled apart."""
from pathlib import Path
import json,gzip,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif']=['Noto Sans CJK SC','DejaVu Sans'];plt.rcParams['axes.unicode_minus']=False
out=Path(sys.argv[1]);index=json.loads((out/'result_index.json').read_text());summary=[]
for attempt in index['attempts']:
 if attempt['category']=='in_progress':continue
 run=Path(attempt.get('run',Path(attempt['attempt_dir'])/'session_001/cycle_001'));res=run/'results';case=json.loads((out/'cases'/attempt['case_id']/'case.json').read_text());a=attempt.get('multicondition_audit',{})
 row=dict(case_id=attempt['case_id'],kind=case['kind'],category=attempt['category'],run_commit=attempt['run_commit'],landing_objective=a.get('landing_objective'),coverage=a.get('coverage',{}).get('passed'),safety=a.get('safety',{}).get('passed'),failure=a.get('error'))
 summary.append(row)
 if not (res/'receding_control.jsonl').is_file():continue
 control=[json.loads(s) for s in (res/'receding_control.jsonl').read_text().splitlines()];poses={'uav':{},'landing_vehicle':{}}
 raw=res/'model_states.jsonl';source=raw.open('rb') if raw.exists() else gzip.open(raw.with_suffix('.jsonl.gz'),'rb')
 with source:
  for line in source:
   item=json.loads(line)
   if item['kind'] not in ('car_pose','uav_pose'):continue
   for p in item['message'].get('pose',[]):
    if p['name'] not in poses:continue
    stamp=p['header']['stamp'];t=float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9;poses[p['name']][t]=[p.get('position',{}).get(k,0.) for k in 'xyz']
 T=np.array([r['sim_t_s'] for r in control]);start=T[0];target=np.array([(r.get('planar_command') or {}).get('world_scene_target_velocity_m_s',(0,0)) for r in control]);ref=np.zeros_like(target)
 for i in range(1,len(T)):ref[i]=ref[i-1]+.5*(target[i-1]+target[i])*(T[i]-T[i-1])
 native={}
 for name,rows in poses.items():
  times=np.array(sorted(rows));points=np.array([rows[t] for t in times]);mask=(times>=start)&(times<=T[-1]);times=times[mask];points=points[mask];native[name]=(times,points)
 if not len(native['landing_vehicle'][0]):continue
 ref+=native['landing_vehicle'][1][0,:2]
 fig,axs=plt.subplots(2,3,figsize=(16,9));xy=axs[0,0]
 for name,(times,points) in native.items():
  step=max(1,len(times)//2500);xy.plot(points[::step,0],points[::step,1],label='平台原始物理抽样' if name=='landing_vehicle' else '无人机原始物理抽样')
  if name=='uav':axs[0,1].plot(times[::step]-start,points[::step,2],label='无人机机体原点')
  velocity=np.gradient(points,times,axis=0);axs[0,2].plot(times[::step]-start,np.linalg.norm(velocity[::step],axis=1),label=name)
 axs[0,1].plot(T-start,[r['measured_deck']['motion']['pose']['position_world_m'][2] for r in control],label='实测甲板');axs[0,1].legend(fontsize=8)
 xy.plot(ref[:,0],ref[:,1],'--',alpha=.6,label='世界端目标速度积分参考（非物理位置）');xy.axis('equal');xy.set_title('实际XY与数值参考');xy.legend(fontsize=8)
 gap=np.array([min((v['clearance_m'] for v in r.get('feet',[])),default=np.nan) for r in control]);axs[1,0].plot(T-start,gap);axs[1,0].set_title('实测最低脚底净空/m')
 for axis in range(2):
  sent=[(r.get('planar_command') or {}).get('velocity_world_xy_m_s',(0,0))[axis] for r in control];axs[1,1].plot(T-start,target[:,axis],'--',label='目标'+str(axis));axs[1,1].plot(T-start,sent,label='整形命令'+str(axis))
 rel=[]
 for r in control:
  u=np.array(r['observation']['motion']['velocity_at_origin_world_m_s']);p=np.array(r['measured_deck']['motion']['velocity_at_origin_world_m_s']);rel.append(np.linalg.norm((u-p)[:2]))
 axs[1,2].plot(T-start,rel);axs[1,2].set_title('实测水平机体/甲板相对速度/m/s（精确脚底接触另见审计）')
 axs[0,1].set_title('实际机体原点与甲板高度/m');axs[0,2].set_title('原始位姿差分原点速度/m/s（COM另见审计）');axs[1,1].set_title('世界端目标与整形后的命令/m/s');axs[1,1].legend(fontsize=8)
 events=json.loads((res/'scene_maneuvers.json').read_text()) if (res/'scene_maneuvers.json').exists() else []
 for ax in axs.flat:
  ax.grid(alpha=.25)
  if ax is not xy:
   ax.set_xlabel('仿真时间/s（首控制样本为零）')
   for e in events:ax.axvline(e['time']-start,color='red',alpha=.35)
 fig.suptitle(f"{case['case_id']} {case['kind']} | {attempt['category']}");fig.tight_layout();plots=out/'plots';plots.mkdir(exist_ok=True);fig.savefig(plots/(case['case_id']+'.png'),dpi=120);plt.close(fig)
 # Label decimation; full-frequency original remains separately archived.
 stride=max(1,len(control)//2500);sample=[dict(sim_t_s=r['sim_t_s'],uav_position_m=r['observation']['motion']['pose']['position_world_m'],measured_deck_position_m=r['measured_deck']['motion']['pose']['position_world_m'],minimum_foot_gap_m=float(gap[i]),scene_target_velocity_m_s=target[i].tolist(),planner_phase=r.get('planner_phase')) for i,r in enumerate(control) if i%stride==0]
 (plots/(case['case_id']+'_plot_samples.json')).write_text(json.dumps(dict(scope='Decimated plotting data, not full-frequency audit evidence',stride=stride,rows=sample),ensure_ascii=False))
(out/'twenty_case_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));lines=['|工况|运动|结局|落台目标|覆盖|安全|代码|','|---|---|---|---|---|---|---|']
for r in summary:lines.append('|'+ '|'.join(str(r.get(k)) for k in ['case_id','kind','category','landing_objective','coverage','safety','run_commit'])+'|')
(out/'二十工况结果表.md').write_text('\n'.join(lines)+'\n')
