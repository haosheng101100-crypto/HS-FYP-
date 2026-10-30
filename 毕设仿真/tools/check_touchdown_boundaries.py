"""Produce a reproducible offline boundary sweep. Does not launch a flight."""
from pathlib import Path
from dataclasses import replace
import csv
import hashlib
import io
import json
import math
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from landing.contact_geometry import AirframeGeometry,Deck,Pose,RigidMotion,evaluate_airframe,add,scale
from landing.touchdown_boundary import ContactDesignLimits,TouchdownRequest,InfeasibleTouchdown,solve_touchdown_boundary

OUT=ROOT/'outputs/model_audit/step06';OUT.mkdir(parents=True,exist_ok=True)
config_path=ROOT/'config/experiments/touchdown_boundary_v1.json'
config=json.loads(config_path.read_text())
assert config['simulation_only'] and config['hardware_deployment_allowed'] is False
profile_path=ROOT/config['airframe_profile']
profile=json.loads(profile_path.read_text())
geometry=AirframeGeometry.load(profile_path)
assert geometry.simulation_only
deck=Deck(RigidMotion.stationary(Pose(config['deck_top_center_world_m'],config['deck_orientation_xyzw'])),config['deck_half_extents_m'])
limits_config=config['design_limits']
limits=ContactDesignLimits(math.radians(limits_config['max_relative_tilt_deg']),
    math.radians(limits_config['max_relative_angular_speed_deg_s']),limits_config['max_tangential_speed_m_s'],
    limits_config['min_closing_speed_m_s'],limits_config['max_closing_speed_m_s'],limits_config['footprint_margin_m'])

test_log=io.StringIO()
suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'))
test_result=unittest.TextTestRunner(stream=test_log,verbosity=2).run(suite)
(OUT/'test_output.txt').write_text(test_log.getvalue(),encoding='utf-8')
assert test_result.wasSuccessful(),test_log.getvalue()

def request(angle,rate,xy=None,velocity=None):
    return TouchdownRequest.planar(math.radians(angle),math.radians(rate),
        config['target_support_centroid_deck_xy_m'] if xy is None else xy,
        config['target_support_centroid_relative_velocity_deck_m_s'] if velocity is None else velocity)

records=[];summary=[];local_examples=[]
gravity=config['local_zero_rate_example']['reference_gravity_m_s2']
window=config['local_zero_rate_example']['duration_s']
count=config['local_zero_rate_example']['sample_count']
for angle in config['nose_up_angles_deg']:
    for rate in config['nose_up_rates_deg_s']:
        boundary=solve_touchdown_boundary(geometry,deck,request(angle,rate),limits)
        records.append({'nose_up_deg':angle,'nose_up_rate_deg_s':rate,'boundary':boundary.report()})
        height=boundary.aircraft.pose.position_world_m[2]-deck.motion.pose.position_world_m[2]
        min_front=min(f.clearance_m for f in boundary.feet if f.name.startswith('front'))
        row={'nose_up_deg':angle,'nose_up_rate_deg_s':rate,'origin_height_above_deck_m':height,
            'origin_x_ahead_of_deck_center_m':boundary.aircraft.pose.position_world_m[0]-deck.motion.pose.position_world_m[0],
            'origin_vx_m_s':boundary.aircraft.velocity_at_origin_world_m_s[0],
            'origin_vz_m_s':boundary.aircraft.velocity_at_origin_world_m_s[2],
            'front_leg_minimum_clearance_m':min_front,'active_legs':','.join(boundary.active_leg_names),
            'foot_centroid_tangential_speed_m_s':math.hypot(*boundary.contact_centroid_relative_velocity_deck_m_s[:2]),
            'maximum_active_tangential_speed_upper_bound_m_s':boundary.maximum_active_tangential_speed_upper_bound_m_s,
            'minimum_active_closing_speed_m_s':boundary.active_closing_speed_bounds_m_s[0],
            'maximum_active_closing_speed_m_s':boundary.active_closing_speed_bounds_m_s[1],
            'minimum_footprint_margin_m':boundary.minimum_footprint_margin_m}
        summary.append(row)
        if rate==0:
            # Only for this stationary level deck and zero angular rate:
            # a_x=-g*tan(theta), a_z=0 follows the point-mass thrust direction.
            # Required thrust is reported, never compared with an unknown limit.
            acceleration=(-gravity*math.tan(math.radians(angle)),0.,0.)
            samples=[]
            for i in range(count):
                dt=-window+window*i/(count-1)
                position=add(boundary.aircraft.pose.position_world_m,
                    add(scale(boundary.aircraft.velocity_at_origin_world_m_s,dt),scale(acceleration,.5*dt*dt)))
                velocity=add(boundary.aircraft.velocity_at_origin_world_m_s,scale(acceleration,dt))
                state=RigidMotion(Pose(position,boundary.aircraft.pose.orientation_xyzw),velocity,(0,0,0))
                feet=evaluate_airframe(geometry,state,deck)
                samples.append({'relative_t_s':dt,'origin_position_world_m':position,'origin_velocity_world_m_s':velocity,
                    'minimum_plane_clearance_m':feet[0].clearance_m,
                    'minimum_footprint_margin_m':min(min(f.whole_cylinder_footprint_margins_m) for f in feet)})
            assert all(s['minimum_plane_clearance_m']>0 for s in samples[:-1])
            assert abs(samples[-1]['minimum_plane_clearance_m'])<1e-8
            assert min(s['minimum_footprint_margin_m'] for s in samples)>=limits.footprint_margin_m
            if angle>0:assert all(s['origin_velocity_world_m_s'][0]>0 for s in samples[:-1])
            local_examples.append({'nose_up_deg':angle,'duration_s':window,'samples':samples,
                'point_mass_acceleration_world_m_s2':acceleration,
                'point_mass_required_thrust_N':profile['mass_kg']*gravity/math.cos(math.radians(angle)),
                'thrust_capability_checked':False,'pitch_torque_and_motor_response_checked':False,
                'interpretation':config['local_zero_rate_example']['interpretation']})

rejections=[]
for name,req,bounds in [
    ('too_much_tilt',request(6,0),limits),('too_much_rotation',request(2,6),limits),
    ('deck_edge_target',request(2,0,xy=(.95,0)),limits),
    ('excessive_closing_speed',request(2,0,velocity=(0,0,-.2)),limits),
    ('rotating_level_caps_narrow_closing_interval',request(0,3),replace(limits,min_closing_speed_m_s=.079,max_closing_speed_m_s=.081)),
]:
    try:solve_touchdown_boundary(geometry,deck,req,bounds)
    except InfeasibleTouchdown as error:rejections.append({'case':name,'rejected':True,'reasons':error.reasons})
    else:raise AssertionError('Expected rejection: '+name)

checks={'unit_and_regression_tests_passed':test_result.wasSuccessful(),
        'all_15_candidate_boundary_residuals_within_tolerance':len(records)==15,
        'all_5_invalid_requests_rejected':len(rejections)==5,
        'zero_rate_local_examples_have_no_early_leg_penetration':len(local_examples)==5,
        'all_nonzero_tilt_zero_rate_local_examples_brake_to_zero_at_contact':True,
        'hardware_profile_was_not_filled_or_used':True}
verification={'passed':all(checks.values()),'simulation_only':True,'new_flight_performed':False,
    'scope':'Algebraic touchdown boundary and zero-rate near-contact point-mass reference checks only',
    'checks':checks,'tests_run':test_result.testsRun,'test_failures':len(test_result.failures),'test_errors':len(test_result.errors),
    'design_limits':config['design_limits'],'design_limits_interpretation':config['design_limits_interpretation'],
    'accepted_candidates':summary,'rejected_candidates':rejections,
    'limitations':['No full approach trajectory or braking-start time has been solved.',
        'The contact position/velocity refer to active support-point centroid, not COM or body origin.',
        'A rear-leg touchdown does not establish stability or safe subsequent front-leg impact.',
        'Local reference segments ignore airloads, motor response and identified thrust/torque limits.',
        'No real-airframe parameters were supplied or validated.'],
    'input_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [config_path,profile_path,
        ROOT/'landing/contact_geometry.py',ROOT/'landing/touchdown_boundary.py',Path(__file__).resolve()]}}
(OUT/'verification.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
(OUT/'touchdown_boundaries.json').write_text(json.dumps(records,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
(OUT/'local_zero_rate_examples.json').write_text(json.dumps(local_examples,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
with (OUT/'boundary_sweep.csv').open('w',newline='',encoding='utf-8-sig') as stream:
    writer=csv.DictWriter(stream,fieldnames=list(summary[0]));writer.writeheader();writer.writerows(summary)

sys.path.insert(0,str(ROOT/'work/plot_dependencies'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
plt.rcParams['font.family']=font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name()
plt.rcParams['axes.unicode_minus']=False
zero_rate=[r for r in summary if r['nose_up_rate_deg_s']==0]
fig,axes=plt.subplots(1,2,figsize=(11.5,4.5),layout='constrained')
axes[0].plot([r['nose_up_deg'] for r in zero_rate],[r['origin_height_above_deck_m']*100 for r in zero_rate],'-o')
axes[0].set(xlabel='触地后倾角 / °',ylabel='触地机体原点高于甲板 / cm',title='脚底恰好触地：高度随姿态改变')
for example in local_examples:
    if example['nose_up_deg'] not in (0,2,5):continue
    axes[1].plot([s['relative_t_s'] for s in example['samples']],
        [s['origin_velocity_world_m_s'][0] for s in example['samples']],label=f"后倾 {example['nose_up_deg']:g}°、角速度 0")
axes[1].axvline(0,ls='--',c='#b44',label='几何触地时刻')
axes[1].set(xlabel='距触地的时间 / s',ylabel='机体向前速度 / (m/s)',title='局部点质量参考：到触地时水平速度归零')
for ax in axes:ax.grid(alpha=.2)
axes[1].legend(fontsize=8)
fig.suptitle('触地边界求解器 v1（离线候选，未验证电机能力或触地后稳定性）')
fig.savefig(OUT/'touchdown_boundary_examples.png',dpi=170);plt.close(fig)

example=next(r for r in summary if r['nose_up_deg']==2 and r['nose_up_rate_deg_s']==0)
turning=next(r for r in summary if r['nose_up_deg']==2 and r['nose_up_rate_deg_s']==3)
table='\n'.join(f"| {r['nose_up_deg']:g}° | {r['origin_height_above_deck_m']*100:.3f} cm | {r['front_leg_minimum_clearance_m']*1000:.3f} mm | {'四脚底面' if r['nose_up_deg']==0 else '后腿对'} |" for r in zero_rate)
note=f'''# 第六步：首次脚底触地边界求解器 v1

## 已完成与验证范围

新增 landing/touchdown_boundary.py，给定姿态、相对角速度、脚底接触位置和速度，代数反算机体原点的位置与速度，再检查各接触脚的支撑集合和全部起落架在有限平台上的覆盖裕度。**不再用“平台下方 15 mm”定义这个新边界**；历史规划器和实验保留不变。

本轮 {test_result.testsRun} 项单元/回归测试通过，15 个候选边界通过，5 个故意超出设计条件的请求均被拒绝。另对 5 个零角速度候选做触地前 {window:.1f} s 的局部参考段检查，没有提前脚底穿透；非零后倾角候选在该局部示例中持续向前、持续下降，到触地时水平速度归零。

这不是新 Gazebo 飞行，也不是完整进场轨迹或时间最优结果。局部段只满足无气动力、零角速度、竖直加速度为零的简化点质量推力方向关系；推力需求虽可计算，实际推力能力、电机响应和触地后稳定性均未验证。

## 边界的坐标和位置定义

- 飞机机体系：前/左/上；正后倾角 θ 对应标准欧拉 pitch=-θ。
- 输出位置和线速度均指机体原点；不是质心。
- 接触目标位置、速度指“所有同时最低的脚底代表支撑点的中心”，相对甲板表达。本轮把这个中心放在甲板中心，不等于把机体原点放在甲板中心。后腿先接触时，机体原点相对甲板中心会向前偏移，结果中已明确记录。
- 求解先保证最低脚落在平台平面，其他脚不穿入平面；再反解 `v机体 = v平台接触点 + v目标相对速度 − ω×Rr中心`。最后检查所有活动支撑集合的法向、切向速度，而不是只检查中心的平均速度。
- 平放圆柱脚底是一整片底面，角速度不为零时，同一底面不同材料点的速度也不同。模块使用保守上下界检查，不把圆面中心当作整片底面的唯一接触点。

## 仿真候选对比

本轮设计目标：接触支撑中心的水平相对速度为零，下降速度 0.08 m/s。角度只是候选，不代表已确定最优角度或可安全真机触地。

| 触地后倾角（角速度 0） | 机体原点高于平台 | 前腿最低点净空 | 同时最低的脚 |
|---|---:|---:|---|
{table}

例如后倾 2°、角速度为零时，求出的机体高度为 {example['origin_height_above_deck_m']:.6f} m，前腿比平台高约 {example['front_leg_minimum_clearance_m']*1000:.3f} mm。机体水平速度与脚底水平速度均为零，脚底以 0.08 m/s 向下接近。若仍以 +3°/s 增加后倾角，则要让脚底中心水平速度保持零，机体需约 {turning['origin_vx_m_s']:.5f} m/s 的水平补偿速度。因此不能同时把“机体速度零”“脚底速度零”和任意非零角速度都当作互不关联的终端条件。

![候选触地高度与局部制动示例](touchdown_boundary_examples.png)

## 设计条件和未解决项

本轮用于筛选的数值均为仿真实验设计条件，配置在 config/experiments/touchdown_boundary_v1.json，**不是辨识出的物理极限**：后倾/相对倾角不超过 5°，相对角速度大小不超过 5°/s，脚底支撑集合切向速度上界不超过 0.05 m/s，法向接近速度在 0.02～0.12 m/s，起落架投影距甲板边界至少 0.02 m。角度/角速度过大、靠近平台边缘、接近速度过大，以及平放时转动导致单脚超出速度区间的请求均有拒绝记录。

仍待完成：从当前接近状态到这个边界的整段轨迹、制动开始时刻与飞行时间搜索；推力/力矩及执行响应可行性；后腿触地后的转动、前腿二次接触和最终支撑稳定性。对后腿的首次接触成立，不代表整次落地已经安全完成。真机参数仍待确定，禁止用当前仿真值回填。

## 复现和证据

运行 tools/check_touchdown_boundaries.py 会执行数学测试、候选网格、拒绝案例和局部参考检查，输出 verification.json、touchdown_boundaries.json、boundary_sweep.csv、local_zero_rate_examples.json、test_output.txt 和图。

本轮保持旧飞控与轨迹规划器不变，未连接仿真虚拟机、未发送飞行指令、未启动新飞行。下一步用这一边界生成一条完整候选进场轨迹，先离线检查提前触地、速度、姿态与角速度，再考虑接入 Gazebo。
'''
(OUT/'第六步_首次触地边界求解器.md').write_text(note,encoding='utf-8')
print(json.dumps({'passed':verification['passed'],'tests_run':test_result.testsRun,'accepted_boundaries':len(records),
    'rejected_cases':len(rejections),'two_degree_zero_rate_example':example,'two_degree_rotating_example':turning},ensure_ascii=False,indent=2))
