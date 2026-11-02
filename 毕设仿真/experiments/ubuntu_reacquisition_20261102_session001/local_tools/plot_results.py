from pathlib import Path
import json,gzip,csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
w=Path(__file__).resolve().parents[2];o=w/'outputs/重新跟随专项_20261102';summary=json.loads((o/'new_run_summary.json').read_text())
f=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
if f.exists():font_manager.fontManager.addfont(str(f));plt.rcParams['font.family']=font_manager.FontProperties(fname=str(f)).get_name()
plt.rcParams['axes.unicode_minus']=False
names={'planar_reacquire_x':'X换向','planar_reacquire_y':'Y换向','planar_reacquire_xy':'XY换向'}
labels={True:'专项通过',False:'专项未通过'}
fig,axes=plt.subplots(1,3,figsize=(14,4.8),layout='constrained');xy,axy=plt.subplots(figsize=(10,7),layout='constrained');colors=plt.get_cmap('tab10')
for i,(row,ax) in enumerate(zip(summary,axes.flat)):
 r=o/'session_001'/row['cycle']/'results';samples={'uav_pose':[],'car_pose':[]};last={k:-1. for k in samples};dest=o/'plot_data'/row['scenario'];dest.mkdir(parents=True,exist_ok=True)
 with gzip.open(r/'model_states.jsonl.gz','rt') as stream:
  for line in stream:
   d=json.loads(line);k=d['kind']
   if k not in samples:continue
   for p in d['message'].get('pose',[]):
    if p['name']!=('uav' if k=='uav_pose' else 'landing_vehicle'):continue
    s=p['header']['stamp'];t=float(s.get('sec',0))+float(s.get('nsec',0))*1e-9
    if t-last[k]<.039999:continue
    last[k]=t;v=p.get('position',{});q=p.get('orientation',{});x,y,z=[v.get(n,0) for n in 'xyz']
    if k=='car_pose':
     qx,qy,qz,qw=[q.get(n,0) for n in 'xyzw'];x+=1.6*(qx*qz+qw*qy);y+=1.6*(qy*qz-qw*qx);z+=.8*(1-2*(qx*qx+qy*qy))
    samples[k].append((t,x,y,z))
 with (dest/'plot_samples.csv').open('w',newline='') as f:
  writer=csv.writer(f);writer.writerow(['kind','sim_t_s','x_world_m','y_world_m','z_world_m'])
  for k,values in samples.items():
   for v in values:writer.writerow(['uav_origin' if k=='uav_pose' else 'deck_top',*v])
 u=samples['uav_pose'];c=samples['car_pose'];ax.plot([v[0] for v in u],[v[3] for v in u],label='机体原点');ax.plot([v[0] for v in c],[v[3] for v in c],linestyle='--',label='甲板顶面');ax.set_title(names[row['scenario']]+' · '+labels[row['passed']]);ax.set(xlabel='仿真时间 / 秒',ylabel='世界高度 / 米');ax.grid(alpha=.2)
 if i==0:ax.legend()
 axy.plot([v[1] for v in u],[v[2] for v in u],color=colors(i),label=names[row['scenario']]+'：飞机');axy.plot([v[1] for v in c],[v[2] for v in c],color=colors(i),linestyle='--',label=names[row['scenario']]+'：平台')
for ax in list(axes.flat)[len(summary):]:ax.remove()
fig.suptitle('高净空重新跟随三专项：Gazebo 原始位姿高度\n机体原点不等同脚底净空；通过与否以独立审计为准');fig.savefig(o/'专项实际高度曲线.png',dpi=160)
axy.set(xlabel='世界 X / 米',ylabel='世界 Y / 米',title='重新跟随专项实际飞机与平台轨迹');axy.axis('equal');axy.grid(alpha=.2);axy.legend(fontsize=8);xy.savefig(o/'专项实际XY轨迹.png',dpi=160);plt.close('all');print('Plots and sampled data saved')
