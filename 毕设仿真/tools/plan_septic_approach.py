"""Offline duration-grid search for a planar, physical-contact reference."""
from pathlib import Path
from dataclasses import asdict
import csv
import hashlib
import io
import json
import math
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from landing.contact_geometry import AirframeGeometry,Deck,Pose,RigidMotion
from landing.touchdown_boundary import ContactDesignLimits,TouchdownRequest,solve_touchdown_boundary
from landing.septic_approach import ApproachDesignLimits,PlanarSepticApproach,check_approach_reference

OUT=ROOT/'outputs/model_audit/step07/septic_recheck';OUT.mkdir(parents=True,exist_ok=True)
config_path=ROOT/'config/experiments/septic_approach_v1.json'
cfg=json.loads(config_path.read_text())
boundary_path=ROOT/cfg['boundary_config'];bcfg=json.loads(boundary_path.read_text())
profile_path=ROOT/bcfg['airframe_profile'];profile=json.loads(profile_path.read_text())
geometry=AirframeGeometry.load(profile_path)
assert cfg['simulation_only'] and geometry.simulation_only and not cfg['hardware_deployment_allowed']
deck=Deck(RigidMotion.stationary(Pose(bcfg['deck_top_center_world_m'],bcfg['deck_orientation_xyzw'])),bcfg['deck_half_extents_m'])
b=bcfg['design_limits']
boundary_limits=ContactDesignLimits(math.radians(b['max_relative_tilt_deg']),math.radians(b['max_relative_angular_speed_deg_s']),
    b['max_tangential_speed_m_s'],b['min_closing_speed_m_s'],b['max_closing_speed_m_s'],b['footprint_margin_m'])
limit_values=dict(cfg['design_limits'])
for degree_key,rad_key in [('max_tilt_deg','max_tilt_rad'),('max_pitch_rate_deg_s','max_pitch_rate_rad_s'),
                           ('max_pitch_acceleration_deg_s2','max_pitch_acceleration_rad_s2'),('near_contact_tilt_deg','near_contact_tilt_rad')]:
    limit_values[rad_key]=math.radians(limit_values.pop(degree_key))
limits=ApproachDesignLimits(**limit_values)
offset=profile['nominal_com_in_body_m']
initial=(cfg['initial_com_x_m'],offset[1],cfg['initial_com_z_m'])

test_stream=io.StringIO()
test_result=unittest.TextTestRunner(stream=test_stream,verbosity=2).run(unittest.defaultTestLoader.discover(str(ROOT/'tests')))
(OUT/'test_output.txt').write_text(test_stream.getvalue(),encoding='utf-8')
assert test_result.wasSuccessful(),test_stream.getvalue()

all_attempts=[];solutions=[];primary_approach=None;primary_report=None;primary_boundary=None
search=cfg['duration_search']
ticks=round((search['maximum_s']-search['minimum_s'])/search['step_s'])
for angle in cfg['contact_angles_deg']:
    request=TouchdownRequest.planar(math.radians(angle),math.radians(cfg['contact_pitch_rate_deg_s']),
        bcfg['target_support_centroid_deck_xy_m'],bcfg['target_support_centroid_relative_velocity_deck_m_s'])
    boundary=solve_touchdown_boundary(geometry,deck,request,boundary_limits)
    solution=None
    for tick in range(ticks+1):
        T=round(search['minimum_s']+search['step_s']*tick,8)
        approach=PlanarSepticApproach(initial,cfg['initial_com_velocity_world_m_s'],boundary,offset,
            cfg['reference_gravity_m_s2'],profile['mass_kg'],T)
        coarse=check_approach_reference(approach,geometry,deck,limits,cfg['candidate_samples'])
        attempt={'contact_nose_up_deg':angle,'duration_s':T,'coarse_passed':coarse['passed'],
                 'coarse_rejection':None if coarse['passed'] else {k:v for k,v in coarse.items() if k!='samples'}}
        if coarse['passed']:
            dense=check_approach_reference(approach,geometry,deck,limits,cfg['final_samples'])
            attempt['dense_passed']=dense['passed']
            if not dense['passed']:attempt['dense_rejection']={k:v for k,v in dense.items() if k!='samples'}
            if dense['passed']:
                solution={k:v for k,v in dense.items() if k!='samples'}
                solution['contact_nose_up_deg']=angle
                solution['interpretation']='First passing duration on this grid, not a proof of global minimum time or actual flight feasibility.'
                solutions.append(solution)
                all_attempts.append(attempt)
                if angle==cfg['primary_contact_angle_deg']:
                    primary_approach,primary_report,primary_boundary=approach,dense,boundary
                break
        all_attempts.append(attempt)
    print(json.dumps({'angle_deg':angle,'found':solution is not None,'duration_s':None if solution is None else solution['duration_s']}),flush=True)
    if solution is None:solutions.append({'passed':False,'contact_nose_up_deg':angle,'reason':'No passing sampled duration in configured grid'})

(OUT/'duration_search.json').write_text(json.dumps(all_attempts,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
(OUT/'angle_comparison.json').write_text(json.dumps(solutions,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
assert primary_report is not None,'Primary 2-degree boundary needs further design; inspect rejection records'

rows=primary_report['samples']
peak=max(rows,key=lambda r:r['body_velocity_world_m_s'][0])
braking_start=peak['t_s']
braking_state=primary_approach.at(braking_start)
braking_distance=primary_boundary.contact_centroid_world_m[0]-braking_state['motion'].pose.position_world_m[0]
primary_summary={k:v for k,v in primary_report.items() if k!='samples'}
primary_summary.update(contact_nose_up_deg=cfg['primary_contact_angle_deg'],
    initial_com_position_world_m=initial,initial_com_velocity_world_m_s=cfg['initial_com_velocity_world_m_s'],
    initial_state_interpretation=cfg['initial_state_interpretation'],
    com_offset_body_m=offset,gravity_m_s2=cfg['reference_gravity_m_s2'],simulation_mass_kg=profile['mass_kg'],
    boundary=primary_boundary.report(),planned_braking_start_s=braking_start,
    planned_peak_body_forward_speed_m_s=peak['body_velocity_world_m_s'][0],
    body_origin_x_at_braking_start_m=braking_state['motion'].pose.position_world_m[0],
    horizontal_distance_from_body_origin_to_contact_anchor_at_braking_start_m=braking_distance,
    body_origin_height_above_deck_at_braking_start_m=braking_state['motion'].pose.position_world_m[2]-.8,
    initial_com_jet=[list(axis.initial) for axis in primary_approach.axes],
    terminal_com_jet=[list(axis.terminal) for axis in primary_approach.axes],
    normalized_septic_coefficients=[list(axis.coefficients) for axis in primary_approach.axes],
    statement='Offline candidate reference only; no new Gazebo run, rotor torque calibration, FCU tracking or post-contact validation.')
(OUT/'approach_plan.json').write_text(json.dumps(primary_summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
with (OUT/'reference.csv').open('w',newline='',encoding='utf-8-sig') as stream:
    fields=['t_s','body_x_m','body_z_m','body_vx_m_s','body_vz_m_s','theta_deg','theta_rate_deg_s',
            'minimum_leg_gap_m','required_thrust_N']
    writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
    for row in rows:
        writer.writerow({'t_s':row['t_s'],'body_x_m':row['body_position_world_m'][0],'body_z_m':row['body_position_world_m'][2],
            'body_vx_m_s':row['body_velocity_world_m_s'][0],'body_vz_m_s':row['body_velocity_world_m_s'][2],
            'theta_deg':math.degrees(row['nose_up_rad']),'theta_rate_deg_s':math.degrees(row['nose_up_rate_rad_s']),
            'minimum_leg_gap_m':row['minimum_leg_clearance_m'],'required_thrust_N':row['required_thrust_N']})

verification={'passed':primary_report['passed'] and test_result.wasSuccessful(),'simulation_only':True,'new_flight_performed':False,
    'tests_run':test_result.testsRun,'primary_contact_angle_deg':cfg['primary_contact_angle_deg'],
    'primary_duration_s':primary_approach.T,'primary_terminal_checks':primary_report['checks'],
    'samples_per_final_candidate':cfg['final_samples'],'duration_step_s':search['step_s'],
    'all_limits_are_reference_design_conditions':True,'actual_thrust_or_torque_capability_checked':False,
    'flight_controller_tracking_checked':False,'post_contact_stability_checked':False,
    'input_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [config_path,boundary_path,profile_path,
        ROOT/'landing/septic_approach.py',ROOT/'landing/touchdown_boundary.py',ROOT/'landing/contact_geometry.py',Path(__file__).resolve()]}}
(OUT/'verification.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

sys.path.insert(0,str(ROOT/'work/plot_dependencies'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
plt.rcParams['font.family']=font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name()
plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(3,1,figsize=(10,10),sharex=True,layout='constrained')
t=[r['t_s'] for r in rows]
axes[0].plot(t,[math.degrees(r['nose_up_rad']) for r in rows],label='推力方向确定的后倾角 θ')
axes[0].plot(t,[math.degrees(r['nose_up_rate_rad_s']) for r in rows],label='后倾角速度 / (°/s)',ls='--')
axes[0].set_ylabel('角度 / °；角速度 / (°/s)')
axes[1].plot(t,[r['body_velocity_world_m_s'][0] for r in rows],label='机体向前速度')
axes[1].plot(t,[-r['body_velocity_world_m_s'][2] for r in rows],label='机体下降速度')
axes[1].set_ylabel('速度 / (m/s)')
axes[2].plot(t,[r['minimum_leg_clearance_m'] for r in rows],label='最低真实脚底距甲板平面')
axes[2].set(ylabel='脚底净空 / m',xlabel='进场参考时间 / s')
for ax in axes:
    ax.axvline(braking_start,ls=':',c='#777',label='候选制动开始')
    ax.axvline(primary_approach.T,ls='--',c='#b44',label='规划首次触地')
    ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('七次完整进场候选：后倾 2°、角速度 0 的脚底触地边界\n离线平面点质量参考，未验证飞控跟踪与落地冲击')
fig.savefig(OUT/'septic_approach_reference.png',dpi=170);plt.close(fig)
comparison='\n'.join(f"| {s['contact_nose_up_deg']:g}° | {s['duration_s']:.2f} s | {s['low_horizontal_speed_lead_s']:.3f} s |" if s['passed'] else f"| {s['contact_nose_up_deg']:g}° | 本网格未找到 | — |" for s in solutions)
extrema=primary_report['sampled_extrema']
note=f'''# 第七步：脚底触地边界下的完整进场候选轨迹

## 本轮完成

新增 landing/septic_approach.py。使用七次质心位置多项式，同时匹配起终点的位置、速度、加速度和 jerk；通过质心加速度与重力得到推力方向、姿态，再由 jerk 与 snap 推导角速度和角加速度。按机型质心偏移，将质心位置、速度、加速度转换为机体原点参考，没有混用两个原点。

触地边界来自第六步物理脚底模块：候选后倾 2°，角速度 0，脚底支撑中心在甲板中心，水平相对速度 0、下降速度 0.08 m/s。终端水平加速度为 `−g tan(2°)`，终端 jerk 为零，使后倾角和角速度与位置轨迹相容；没有把终点放进平台下方。

本轮 {test_result.testsRun} 项数学/回归测试通过。时间网格按 {search['step_s']:.2f} s 搜索，每个候选先检查 {cfg['candidate_samples']} 点，选中前再检查 {cfg['final_samples']} 点。**这些是采样参考设计条件，不是实际推力/力矩或飞控能力证明，也不是全局时间最优解。**

## 主候选结果（离线）

| 项目 | 候选参考结果 |
|---|---:|
| 进场至规划首次触地 | {primary_approach.T:.3f} s |
| 候选制动开始 | {braking_start:.3f} s |
| 制动开始时机体距接触支撑中心的水平距离 | {braking_distance:.3f} m |
| 制动开始时机体高出甲板 | {primary_summary['body_origin_height_above_deck_at_braking_start_m']:.3f} m |
| 最大机体向前速度 | {peak['body_velocity_world_m_s'][0]:.3f} m/s |
| 最大参考倾角绝对值 | {math.degrees(extrema['tilt_rad']):.3f}° |
| 最大参考角速度绝对值 | {math.degrees(extrema['pitch_rate_rad_s']):.3f} °/s |
| 进入最终低水平速度区至触地的间隔 | {primary_report['low_horizontal_speed_lead_s']:.3f} s |
| 点质量模型合推力需求范围 | {primary_report['point_mass_thrust_demand_range_N'][0]:.3f}～{primary_report['point_mass_thrust_demand_range_N'][1]:.3f} N |

前倾加速、后倾制动与下降共享一条轨迹。触地时机体水平速度和脚底水平相对速度为零，后倾角为 2°、角速度为零；最低脚底位于甲板平面，前腿仍有净空。选中候选在采样点未出现提前脚底穿透、临边低空进入、水平反向或下降中爬升，并符合配置的近接触速度和姿态条件。

时间来自设定的仿真参考初态：质心 x={initial[0]:.3f} m、z={initial[2]:.3f} m，速度 {cfg['initial_com_velocity_world_m_s']} m/s，初始水平姿态/角速度为零。该初态取在既有实验附近，**不是当前真实测量，也不是与第四步严格同初态的重放**，不能据此直接宣称比旧方法节省多少时间。

接触目标是活动脚底支撑中心而非机体原点；后倾后腿触地时，机体原点向前偏移约 {primary_boundary.aircraft.pose.position_world_m[0]-5:.3f} m。与旧方法公平比较时必须统一初态、目标点和接口。

![完整候选参考](septic_approach_reference.png)

## 角度网格对比

| 候选触地后倾角 | 首个通过的网格时长 | 最终低水平速度区间 |
|---|---:|---:|
{comparison}

“首个通过”只针对指定网格、初态和参考约束。未找到也只表示本次参数化与网格未找到候选，不能推断实际任务无解。低水平速度阈值为 0.12 m/s，最大提前区间为 0.7 s，均是设计条件；没有要求在零时间内完成减速。

## 仍需验证

这一步完整轨迹是平面点质量模型下的离线候选。机型推力/力矩可用范围、旋翼自转耦合、电机响应和飞控跟踪仍未辨识验证，图中姿态不是新 Gazebo 实测数据。触地后前腿接触和稳定停转也未验证。本轮未运行新飞行，未替换历史飞控或规划器。

参考只定义到计划首次触地，不隐式外推一个平台下方的末段。接入 Gazebo 前还要明确实测初态初始化、跟踪误差下的实际接触判断和接触后的控制交接。否则“离线端点正确”仍不能保证“实际首次接触姿态和速度正确”。

复现：运行 tools/plan_septic_approach.py。配置在 config/experiments/septic_approach_v1.json。证据包括 verification.json、approach_plan.json、reference.csv、duration_search.json、angle_comparison.json、test_output.txt。旧实验文件与真机待定配置保持独立。
'''
(OUT/'第七步_完整进场候选轨迹.md').write_text(note,encoding='utf-8')
print(json.dumps({'passed':verification['passed'],'tests_run':test_result.testsRun,'duration_s':primary_approach.T,
    'braking_start_s':braking_start,'braking_distance_m':braking_distance,'low_speed_lead_s':primary_report['low_horizontal_speed_lead_s'],
    'extrema':extrema,'terminal_residuals':primary_report['terminal_residuals']},ensure_ascii=False,indent=2))
