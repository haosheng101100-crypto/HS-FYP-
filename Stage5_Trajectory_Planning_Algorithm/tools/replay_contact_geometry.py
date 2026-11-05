"""Reconstruct a saved Gazebo approach and audit its original planner reference.

Offline only: no VM, sockets, actuator commands, model edits or flight start.
The deck is the stationary nominally level deck from the saved scene and car
position/yaw records. Full car roll/pitch were not recorded in this dataset.
"""
from dataclasses import asdict
from pathlib import Path
import csv
import hashlib
import importlib.util
import json
import math
import sys
import xml.etree.ElementTree as E

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'work/plot_dependencies'))
import numpy as np
from landing.contact_geometry import AirframeGeometry,Deck,Pose,RigidMotion,evaluate_airframe,quaternion_from_rpy
from landing.trajectory_audit import audit_reference

RUN=ROOT/'outputs/model_audit/step04/run_004'
OUT=ROOT/'outputs/model_audit/step05';OUT.mkdir(parents=True,exist_ok=True)
PROFILE=ROOT/'config/airframes/iris_fixed_gimbal_sim.json'
geometry=AirframeGeometry.load(PROFILE)
status=json.loads((RUN/'results/cooperation_status.json').read_text())
manifest=json.loads((RUN/'attempt_manifest.json').read_text())
for path,digest in manifest['source_sha256'].items():
    assert hashlib.sha256((RUN/path).read_bytes()).hexdigest()==digest,path
start=status['approach_start_sim_t']

def timestamp(message):
    stamp=message.get('header',{}).get('stamp',{})
    return float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9

def rotation(pose):
    return np.array([pose.rotate(axis) for axis in [(1,0,0),(0,1,0),(0,0,1)]]).T

poses={};contacts=[]
with (RUN/'results/model_states.jsonl').open() as stream:
    for line in stream:
        row=json.loads(line);msg=row['message']
        if row['kind']=='uav_pose':
            for p in msg.get('pose',[]):
                if p['name']=='uav':
                    q=tuple(p['orientation'].get(k,0) for k in ['x','y','z','w'])
                    pose=Pose(tuple(p['position'].get(k,0) for k in ['x','y','z']),q)
                    poses[timestamp(p)]=pose
        if row['kind']=='deck_contact' and timestamp(msg)>=start:
            hits=[c for c in msg.get('contact',[]) if any('uav' in c.get(k,{}).get('name','') for k in ['collision1','collision2'])]
            if hits:contacts.append((timestamp(msg),hits))
touch,first_contacts=min(contacts,key=lambda item:item[0])
raw_leg_names={c[k]['name'].split('::')[-1] for c in first_contacts for k in ['collision1','collision2'] if '_leg_collision' in c[k]['name']}
csv_rows=list(csv.DictReader((RUN/'results/cooperation.csv').open()))
car_times=np.array([float(row['sim_t']) for row in csv_rows])
car_p=np.array([[float(row['landing_vehicle_'+axis]) for axis in 'xyz'] for row in csv_rows])
car_yaw=np.unwrap(np.array([float(row['landing_vehicle_yaw']) for row in csv_rows]))

# Resolve the actual deck top from the preserved collision box, not a magic height.
vehicle=E.parse(RUN/'models/landing_vehicle/model.sdf').find('model')
chassis=vehicle.find("link[@name='chassis']")
collision=chassis.find("collision[@name='landing_deck']")
link_pose=list(map(float,chassis.findtext('pose').split()))
box_pose=list(map(float,collision.findtext('pose').split()))
assert link_pose[3:]==box_pose[3:]==[0,0,0]
size=list(map(float,collision.findtext('geometry/box/size').split()))
deck_offset=np.array(link_pose[:3])+np.array(box_pose[:3])+np.array([0.,0.,size[2]/2])

def deck_at(t):
    if not car_times[0]<=t<=car_times[-1]:raise ValueError('No platform history at this time')
    position=tuple(float(np.interp(t,car_times,car_p[:,k])) for k in range(3))
    q=quaternion_from_rpy(0,0,float(np.interp(t,car_times,car_yaw)))
    model_pose=Pose(position,q)
    top_pose=Pose(model_pose.transform(tuple(deck_offset)),q)
    # The platform is stationary in this experiment; this is not a moving-car estimator.
    return Deck(RigidMotion.stationary(top_pose),(size[0]/2,size[1]/2))

car_select=(car_times>=start)&(car_times<touch)
car_displacement=float(np.max(np.linalg.norm(car_p[car_select]-car_p[car_select][0],axis=1)))
assert car_displacement<.002,'This replay adapter assumes a stationary platform'
times=sorted(poses)
actual_rows=[]
for index,t in enumerate(times):
    if not start<=t<=touch-.002:continue
    history=[stamp for stamp in times[max(0,index-12):index+1] if stamp>=t-.081]
    if len(history)<5:continue
    dt=np.array(history)-t
    positions=np.array([poses[stamp].position_world_m for stamp in history])
    matrices=np.array([rotation(poses[stamp]) for stamp in history])
    velocity=np.polyfit(dt,positions,2)[1]
    R=rotation(poses[t]);Rdot=np.polyfit(dt,matrices.reshape(-1,9),2)[1].reshape(3,3)
    skew=(Rdot@R.T-R@Rdot.T)/2
    omega=(float(skew[2,1]),float(skew[0,2]),float(skew[1,0]))
    motion=RigidMotion(poses[t],tuple(velocity),omega)
    feet=evaluate_airframe(geometry,motion,deck_at(t))
    actual_rows.append({'sim_t':t,'elapsed_s':t-start,'fit_span_s':float(dt[-1]-dt[0]),
        'fit_samples':len(history),'body_origin_velocity_world_m_s':motion.velocity_at_origin_world_m_s,
        'angular_velocity_world_rad_s':omega,'feet':[asdict(f) for f in feet]})
pre=actual_rows[-1]
foot=pre['feet'][0]
lead=touch-pre['sim_t']
eta=foot['clearance_m']/foot['closing_speed_m_s']
reconstructed_touch=pre['sim_t']+eta
first_height_error=abs(foot['clearance_m'])
contact_period=float(np.median(np.diff(sorted({t for t,_ in contacts}))))
observed_plane_z=float(np.mean([p['z'] for c in first_contacts for p in c['position']]))
nominal_plane_z=deck_at(pre['sim_t']).motion.pose.position_world_m[2]
normal_deviation=max(math.hypot(n.get('x',0),n.get('y',0)) for c in first_contacts for n in c['normal'])

# Replay the untouched old quintic. Its position reference is neither the
# COM nor a physical individual foot: reproduce its explicit adapter only here.
legacy_path=RUN/'first_stage/terminal_trajectory.py'
spec=importlib.util.spec_from_file_location('preserved_terminal_trajectory',legacy_path)
legacy=importlib.util.module_from_spec(spec);sys.modules[spec.name]=legacy;spec.loader.exec_module(legacy)
plan=status['plan'];T=plan['duration_s']
quintic=legacy.Quintic(plan['initial_foot_position'],plan['initial_velocity'],(0,0,0),
    plan['target_foot_position'],plan['terminal_velocity'],plan['terminal_acceleration'],T)
old_audit=json.loads((ROOT/'outputs/model_audit/step01/model_parameters.json').read_text())
legacy_height_offset=-old_audit['previous_control_reference']['body_offset_m'][2]
reference_deck=deck_at(start)
gravity_proxy=9.80665  # exactly the old planner's proxy, not identified thrust

def nominal_pose(t):
    p,v,a,j=quintic.at(t)
    force=np.array([a[0],a[1],gravity_proxy+a[2]])
    pitch=math.atan2(force[0],force[2])
    roll=-math.asin(float(force[1]/np.linalg.norm(force)))
    q=quaternion_from_rpy(roll,pitch,0.)
    return Pose((p[0],p[1],p[2]+legacy_height_offset),q)

def nominal_motion(t):
    pose=nominal_pose(t);p,v,a,j=quintic.at(t)
    lo=max(0.,t-1e-4);hi=min(T,t+1e-4)
    R=rotation(pose);Rdot=(rotation(nominal_pose(hi))-rotation(nominal_pose(lo)))/(hi-lo)
    skew=(Rdot@R.T-R@Rdot.T)/2
    return RigidMotion(pose,v,(float(skew[2,1]),float(skew[0,2]),float(skew[1,0])))

reference_audit=audit_reference(geometry,nominal_motion,lambda t:reference_deck,T,1001)
reference_audit['reference_attitude_assumption']='Quasi-static thrust direction from old acceleration and gravity proxy, yaw=0; not a dynamically validated attitude trajectory.'
reference_audit['legacy_world_vertical_body_offset_m']=legacy_height_offset
reference_audit['original_plan_sha256']=hashlib.sha256((RUN/'results/trajectory_plan.json').read_bytes()).hexdigest()
(OUT/'baseline_reference_audit.json').write_text(json.dumps(reference_audit,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
terminal_gap=min(f['clearance_m'] for f in reference_audit['terminal_feet'])
terminal_rate=float(np.linalg.norm(reference_audit['terminal_angular_velocity_world_rad_s']))
checks={
 'preserved_experiment_source_hashes_match':True,
 'platform_translation_within_2mm_before_touch':car_displacement<.002,
 'lowest_leg_matches_first_recorded_contact':foot['name'] in raw_leg_names,
 'precontact_clearance_within_2mm':0<=foot['clearance_m']<=.002,
 'local_constant_velocity_touch_extrapolation_within_25ms':abs(reconstructed_touch-touch)<=.025,
 'precontact_pose_is_before_first_1khz_contact':.002-1e-9<=lead<=.02 and contact_period<=.002,
 'all_leg_cylinder_projections_fit_on_platform_at_touch':all(min(f['whole_cylinder_footprint_margins_m'])>0 for f in pre['feet']),
 'nominal_level_plane_within_2mm_of_contact_positions':abs(nominal_plane_z-observed_plane_z)<.002,
 'reported_contact_normal_is_level':normal_deviation<1e-6,
 'legacy_terminal_gap_matches_declared_15mm_penetration':abs(terminal_gap+plan['limits']['penetration'])<1e-10,
}
report={'passed':all(checks.values()),'simulation_only':True,'new_flight_performed':False,
 'scope':'Retrospective kinematic reconstruction of run_004 and geometry-only audit of its original reference',
 'checks':checks,'profile_id':geometry.profile_id,'raw_first_contact_sim_t':touch,
 'raw_contact_legs':sorted(raw_leg_names),'precontact_reconstruction':pre,
 'precontact_lead_s':lead,'observed_contact_period_s':contact_period,
 'local_touch_extrapolation_sim_t':reconstructed_touch,'local_touch_extrapolation_error_s':reconstructed_touch-touch,
 'nominal_deck_height_m':nominal_plane_z,'contact_position_mean_height_m':observed_plane_z,
 'reference_findings':{'declared_duration_s':T,'quasi_static_first_plane_touch_s':reference_audit['first_plane_touch']['time_s'],
     'terminal_plane_clearance_m':terminal_gap,'quasi_static_terminal_angular_speed_rad_s':terminal_rate,
     'quasi_static_terminal_angular_speed_deg_s':math.degrees(terminal_rate)},
 'limits_of_evidence':['The precontact reconstruction window is selected retrospectively using the known first contact, not a prospective online prediction evaluation.',
     'Speeds use a trailing 80ms pose fit, not exact instantaneous impact velocities.',
     'Deck roll/pitch were not recorded; the adapter assumes the stationary level platform and checks the resulting plane against the first contact afterward.',
     'Geometry does not model body/gimbal collision, platform edge impacts, compliance or post-contact forces.',
     'The reference attitude is inferred from acceleration; its angle rate is not a measured vehicle response.',
     'No controller or original planner has been modified by this replay.'],
 'input_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [PROFILE,RUN/'results/model_states.jsonl',RUN/'results/cooperation.csv',legacy_path,
     ROOT/'landing/contact_geometry.py',ROOT/'landing/trajectory_audit.py',Path(__file__).resolve()]},
}
(OUT/'replay_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
with (OUT/'actual_leg_clearances_and_velocities.csv').open('w',newline='',encoding='utf-8-sig') as stream:
    writer=csv.DictWriter(stream,fieldnames=['sim_t','elapsed_s','leg','clearance_m','closing_speed_m_s','tangential_speed_m_s','support_set_kind'])
    writer.writeheader()
    for row in actual_rows:
        for f in row['feet']:
            writer.writerow({'sim_t':row['sim_t'],'elapsed_s':row['elapsed_s'],'leg':f['name'],
                **{key:f[key] for key in ['clearance_m','closing_speed_m_s','tangential_speed_m_s','support_set_kind']}})

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
plt.rcParams['font.family']=font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name()
plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(1,2,figsize=(12,4.7),layout='constrained')
recent=[row for row in actual_rows if row['sim_t']>=touch-.4]
for leg in geometry.legs:
    values=[next(f for f in row['feet'] if f['name']==leg.name)['clearance_m']*1000 for row in recent]
    label={'front_left_leg_collision':'前左','front_right_leg_collision':'前右','rear_left_leg_collision':'后左','rear_right_leg_collision':'后右'}[leg.name]
    axes[0].plot([row['sim_t']-touch for row in recent],values,label=label)
axes[0].axhline(0,c='#555',lw=.8);axes[0].axvline(0,ls='--',c='#b44',label='首次实际碰撞')
axes[0].set(xlabel='距首次碰撞的时间 / s',ylabel='脚底距平台平面 / mm',title='实际位姿重建：后左腿最先接近平台')
samples=reference_audit['samples']
near=[row for row in samples if row['t_s']>=T-.5]
axes[1].plot([row['t_s'] for row in near],[row['minimum_plane_clearance_m']*1000 for row in near],label='原轨迹＋准静态姿态的最低脚底')
axes[1].axhline(0,c='#555',lw=.8)
axes[1].axvline(T,ls=':',c='#777',label='原设定终点')
axes[1].axvline(reference_audit['first_plane_touch']['time_s'],ls='--',c='#b44',label='几何首次接触')
axes[1].set(xlabel='进场参考时间 / s',ylabel='最低脚底距平台 / mm',title='原终点在平台下方 15 mm：接触早于终点')
for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('真实脚底模块验证与原终端参考审计（离线，无新飞行）')
fig.savefig(OUT/'contact_geometry_validation.png',dpi=170);plt.close(fig)

note=f'''# 第五步：真实脚底几何模块与原轨迹接触审计

本轮新增 landing/contact_geometry.py 与 landing/trajectory_audit.py。前者计算四只圆柱起落架的最低支撑点、平台净空、材料点速度、相对平台速度及有限甲板覆盖裕度；后者接受规划器提供的位姿/速度轨迹，检查首次几何接触与终点脚底状态。模块从独立机型配置读取几何，不写死 Iris 脚长，不发控制指令。

## 验证结果

数学/边界测试 18 项通过，包含后倾先触后腿、质心与机体原点换算、同一材料点速度的差分验证、旋转平台相对速度、全局坐标旋转平移不变性、平台外假接触、圆柱底面/侧面退化和未知真机参数拒绝。平放时最低支撑集是一整片圆面，模块不将圆面中心的速度冒充整个接触面的最大速度，而额外给出保守上界。

对第四步 run_004 的原始 Gazebo 位姿及碰撞记录进行离线重建，核对结果：**{'通过' if report['passed'] else '有未通过项，详见 replay_verification.json'}**。

- 重建的最低脚为 {foot['name']}，与首次实际碰撞一致。
- 最后一个接触前样本领先碰撞 {lead*1000:.1f} ms，脚底距平台约 {foot['clearance_m']*1000:.3f} mm。
- 只用接触前 {pre['fit_span_s']*1000:.0f} ms 的 {pre['fit_samples']} 个位姿点拟合，脚底相对平台水平速度约 {foot['tangential_speed_m_s']:.5f} m/s，向下速度约 {foot['closing_speed_m_s']:.5f} m/s。
- 将这个样本的净空除以法向接近速度，局部匀速外推触地时刻相对实际记录误差约 {(reconstructed_touch-touch)*1000:+.2f} ms；这是已知接触附近的事后重建检查，不能称为在线触地预测准确率。
- 平台平面从保存的碰撞盒和车辆位置记录计算；其高度与首次碰撞点平均高度相差约 {abs(nominal_plane_z-observed_plane_z)*1000:.4f} mm。该数据集未记录车辆滚转/俯仰，本次使用静止水平平台假设，并在事后以碰撞法向与位置核对。

## 本轮发现的终端边界问题

保持原有五次轨迹不变，将其原控制接口中的“中心参考＋世界竖直偏移”还原为机体原点；仅为审计，用加速度与重力代理构建准静态姿态，并求其角速度。这不是新增的真实动力学轨迹或飞行结果。

| 原参考的审计量 | 结果 |
|---|---:|
| 原设定结束时间 | {T:.3f} s |
| 准静态参考的首次几何接触 | {reference_audit['first_plane_touch']['time_s']:.3f} s |
| 原终点最低脚底相对平台 | {terminal_gap*1000:.3f} mm |
| 由准静态参考推导的终端角速度大小 | {math.degrees(terminal_rate):.3f} °/s |

旧规划器为触发实际接触，明确把终点参考放在平台下方 15 mm。因此它的“最后一个数学时刻”不等于首次触地时刻，指定终端速度也不能直接当作触地速度。另外，五次轨迹的终端加速度为零，并不保证 jerk 为零，所以由推力方向推导的终端角速度未必为零。上表的角速度只是该准静态参考的推导量，不是实测飞机角速度。

这两点说明，下一步应把边界建立在**真实脚底首次触地**上，同时约束接触位置、脚底相对速度、姿态和角速度，而不是继续依赖中心参考的终点值。当前新接口已能输出这些几何量，尚未替换原飞行规划器或控制器，也未完成动力学/电机可行性优化。

![脚底几何与终端参考审计](contact_geometry_validation.png)

## 适用边界与复现

- 所有数值只对应当前仿真机型与该次已保存实验；真机配置仍为空值待定。
- 只计算接触前刚性几何和运动学；不包含机体/云台网格碰撞、甲板侧边碰撞、摩擦、支撑冲击或缓冲变形。
- 原始位姿约 100 Hz，实测持续碰撞消息约 1 kHz，速度是有限时间窗估计。
- 采样轨迹检查可能漏掉采样之间的越界，区间细化只提高已捕获接触的时刻精度。
- 本轮未启动新 Gazebo 飞行，未修改历史模型、原实验与控制器。

复现：Python 运行 `-m unittest discover -s tests -v`，再运行 `tools/replay_contact_geometry.py`。数学测试只依赖标准库；离线重建和科学绘图使用现有 work/plot_dependencies。

明细：replay_verification.json、baseline_reference_audit.json、actual_leg_clearances_and_velocities.csv。代码和机型配置已分别记录哈希，便于后续复核。
'''
(OUT/'第五步_真实脚底与接触边界.md').write_text(note,encoding='utf-8')
print(json.dumps({'passed':report['passed'],'checks':checks,'lowest_leg':foot['name'],
    'precontact_gap_mm':foot['clearance_m']*1000,'touch_extrapolation_error_ms':(reconstructed_touch-touch)*1000,
    'foot_tangential_speed_m_s':foot['tangential_speed_m_s'],'foot_closing_speed_m_s':foot['closing_speed_m_s'],
    'reference_findings':report['reference_findings']},ensure_ascii=False,indent=2))
assert report['passed']
