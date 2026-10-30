"""Plot measured simulation traces; never synthesize target tracking results."""
import csv
from pathlib import Path
import math
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parents[1]
source=Path(sys.argv[1]) if len(sys.argv)>1 else root/'results/attitude_demo.csv'
with source.open() as f:rows=list(csv.DictReader(f))
rows=[r for r in rows if r.get('target_roll_deg')]
if not rows:raise RuntimeError('No actual PID flight samples to plot')
zero=float(rows[0]['sim_t']);times=[float(r['sim_t'])-zero for r in rows]
def series(key):return [float(r[key]) if r.get(key) else math.nan for r in rows]

plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True,constrained_layout=True)
for axis,name in zip(axes[:,0],('roll','pitch','yaw')):
    axis.plot(times,series('target_'+name+'_deg'),color='#d2701f',label='Target',linewidth=1.7)
    axis.plot(times,series('actual_'+name+'_deg'),color='#176c9a',label='Measured',linewidth=1.3)
    axis.set_ylabel(name.capitalize()+' (deg)');axis.grid(alpha=.2);axis.legend(loc='best')
axes[0,1].plot(times,series('uav_z'),color='#176c9a',label='UAV body height')
axes[0,1].plot(times,[float(r['landing_vehicle_z'])+.8 for r in rows],color='#d2701f',linestyle='--',label='Vehicle roof')
axes[0,1].set_ylabel('World height (m)');axes[0,1].legend();axes[0,1].grid(alpha=.2)
axes[1,1].plot(times,series('thrust'),color='#176c9a',label='Own thrust command')
axes[1,1].set_ylabel('Normalized thrust');axes[1,1].grid(alpha=.2);axes[1,1].legend()
for name,color in [('roll','#1b9e77'),('pitch','#d95f02'),('yaw','#7570b3')]:
    axes[2,1].plot(times,[math.degrees(v) for v in series('rate_'+name)],label=name,color=color,linewidth=1.2)
axes[2,1].set_ylabel('Own body-rate target (deg/s)');axes[2,1].grid(alpha=.2);axes[2,1].legend()
for phase,color in [('YAW_DEMO','#d6e8f4'),('LAND_DECK','#e7e7e7')]:
    span=[t for t,r in zip(times,rows) if r['phase']==phase]
    if span:
        for axis in axes.flat:axis.axvspan(min(span),max(span),color=color,alpha=.35)
for axis in axes[-1,:]:axis.set_xlabel('Simulation seconds since own PID takeoff')
fig.suptitle('Actual Gazebo flight: own attitude-angle PID + ArduCopter rate inner loop\nBlue shading: 20 deg yaw command; gray shading: roof descent',fontsize=13)
destination=root/'results/attitude_pid_response.png'
fig.savefig(destination,dpi=160);fig.savefig(destination.with_suffix('.svg'));plt.close(fig)
print(destination)
