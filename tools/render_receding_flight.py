"""Draw recorded physical observations; never run a controller or invent poses."""
from pathlib import Path
import sys,json,argparse,hashlib,math
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'work/plot_dependencies'),str(ROOT/'work/receding_dependencies'),str(ROOT)]
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
from scipy.spatial import ConvexHull
from landing.contact_geometry import Pose

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    source=a.run/'results/receding_control.jsonl';rows=[json.loads(l) for l in source.open()]
    first=next(i for i,r in enumerate(rows) if r['planner_phase'] is not None);rows=rows[first:]
    cfg=json.loads((a.run/'config/experiments/receding_live_v1.json').read_text());envelope=json.loads((a.run/cfg['envelope_profile']).read_text());hx,hy=cfg['deck_half_extents']
    times=np.array([r['sim_t_s'] for r in rows]);positions=np.array([r['observation']['motion']['pose']['position_world_m'] for r in rows])
    plt.rcParams['font.sans-serif']=['Microsoft YaHei','SimHei','DejaVu Sans'];plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(1,2,figsize=(11,4.8),gridspec_kw={'width_ratios':[1.5,1]});fig.subplots_adjust(bottom=.19,top=.82,wspace=.25)
    title=fig.suptitle('真实仿真观测回放：移动小平台降落',fontsize=15)
    fig.text(.5,.045,'由已记录的飞机/平台位姿重绘（加速播放），不是 Gazebo 录屏；蓝色包含桨叶扫掠与机身包络。',ha='center',fontsize=9)
    indices=np.unique(np.searchsorted(times,np.linspace(times[0],times[-1],100)).clip(0,len(rows)-1))
    def draw(frame):
        i=indices[frame];r=rows[i];data=r['observation']['motion']['pose'];body=Pose(tuple(data['position_world_m']),tuple(data['orientation_xyzw']))
        d=r['measured_deck']['motion']['pose'];deck=Pose(tuple(d['position_world_m']),tuple(d['orientation_xyzw']))
        points=list(envelope['fixed_points_body_m']);rotor_paths=[]
        for rotor in envelope['rotors']:
            center=np.array(rotor['origin']);circle=np.array([center+rotor['swept_radius']*np.array([math.cos(t),math.sin(t),0.]) for t in np.linspace(0,2*math.pi,32)])
            points.extend(circle.tolist());rotor_paths.append(np.array([body.transform(tuple(v)) for v in circle]))
        world=np.array([body.transform(tuple(v)) for v in points]);relative=np.array([deck.inverse_rotate(tuple(v-np.array(deck.position_world_m))) for v in world])
        for ax in axes:ax.clear();ax.grid(alpha=.2)
        ax=axes[0];ax.plot(positions[:i+1,0],positions[:i+1,2],color='#2878bf',lw=1.7,label='实际飞行轨迹')
        for circle in rotor_paths:ax.plot(circle[:,0],circle[:,2],color='#185a90',lw=1.5)
        center=np.array(body.position_world_m)
        for rotor in envelope['rotors']:
            end=np.array(body.transform(tuple(rotor['origin'])));ax.plot([center[0],end[0]],[center[2],end[2]],color='#185a90',lw=2)
        for number,foot in enumerate(r['feet']):
            point=np.array(foot['point_world_m'])
            ax.plot([center[0],point[0]],[center[2],point[2]],color='#185a90',lw=.8)
            ax.scatter(point[0],point[2],s=14,color='#123c60',zorder=6,label='脚架最低点' if number==0 else None)
        corners=np.array([deck.transform((x,y,0)) for x,y in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy),(-hx,-hy)]])
        ax.plot(corners[:,0],corners[:,2],color='#d28c20',lw=5,label='实际移动甲板')
        ax.axhline(0,color='#888',lw=1);ax.set(xlim=(min(-.8,positions[:,0].min()-.5),positions[:,0].max()+1),ylim=(-.1,4.6),xlabel='世界 X / m',ylabel='世界高度 / m',title='侧视投影');ax.legend(loc='upper right',fontsize=8)
        ax=axes[1];ax.fill([-hx,hx,hx,-hx],[-hy,-hy,hy,hy],color='#f5e6ca',ec='#d28c20')
        hull=ConvexHull(relative[:,:2]);outline=relative[hull.vertices,:2];ax.fill(outline[:,0],outline[:,1],color='#2878bf',alpha=.35,ec='#185a90')
        ax.set(xlim=(-.65,.65),ylim=(-.65,.65),xlabel='甲板相对 X / m',ylabel='甲板相对 Y / m',title='俯视：整机投影与台面');ax.set_aspect('equal')
        stage={'ALIGNING':'平台上方接近与稳定跟随','LANDING':'跟随与下降','RECOVERING':'退出、抬升返航','COMPLETE':'落台、停转与支撑完成','HOVER':'返航悬停','PREPARING':'稳定悬停'}.get(r['stage'],r['stage'])
        title.set_text(f'真实仿真观测回放 · {stage} · t={times[i]-times[0]:.1f} s')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    animation=FuncAnimation(fig,draw,frames=len(indices),interval=80);animation.save(a.output,writer=PillowWriter(fps=12))
    draw(len(indices)-1);fig.savefig(a.output.with_suffix('.png'),dpi=160)
    a.output.with_suffix('.json').write_text(json.dumps(dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),renderer_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),recorded_observations=True,recorded_foot_points_drawn=True,gazebo_screen_recording=False),indent=2))

if __name__=='__main__':main()
