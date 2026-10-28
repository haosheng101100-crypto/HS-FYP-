"""Publish step11 evidence while retaining the original failed touchdown goal."""
from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'outputs/model_audit/step11';old=ROOT/'outputs/model_audit/step09/run_004';run=out/'run_001'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text(encoding='utf-8'))
new_flight=read(run/'analysis/flight_verification.json');old_flight=read(old/'analysis/flight_verification.json')
chain=read(run/'control_chain/verification.json');old_chain=read(out/'previous_control_chain_v2/verification.json')
post=read(run/'results/postflight_checks.json');capture=read(run/'results/fcu_diagnostic_capture.json')
files=[]
for directory in ['models','worlds','config']:
    for path in (old/directory).rglob('*'):
        if not path.is_file() or '__pycache__' in path.parts:continue
        relative=path.relative_to(old);assert sha(path)==sha(run/relative),str(relative)
        files.append(relative.as_posix())
for path in (old/'landing').glob('*.py'):assert sha(path)==sha(run/'landing'/path.name)
original=(old/'tools/contact_approach_demo.py').read_text()
instrumented=(run/'tools/contact_approach_demo.py').read_text()
stripped=instrumented.replace('\nfrom landing.fcu_recording import FcuDiagnosticCapture','').replace('\n        self.fcu_capture=FcuDiagnosticCapture()','').replace('\n            self.fcu_capture.record(msg,time.monotonic(),self.clock[0] if self.clock else None)','').replace('for mid in (30,32,33,245,83,85,62):','for mid in (30,32,33,245):').replace("self.fcu_capture.save(ROOT/'results')\n        ",'')
assert stripped==original,'Unexpected controller logic change'
pa=read(out/'previous_dataflash/parameters.json');pb=read(run/'results/dataflash_final/parameters.json')
prefixes=('PSC_','ATC_','GUID_','WPNAV_','MOT_','SIM_','INS_','AHRS_')
keys=sorted(k for k in pa if k.startswith(prefixes));changed={k:[pa[k],pb.get(k)] for k in keys if pa[k]!=pb.get(k)}
assert not changed,changed
assert post['remaining_owned_live_processes']==[] and post['normal_disarm_observed'] and post['linux_tests_passed_before_flight']==71
assert chain['diagnostic_passed'] and old_chain['diagnostic_passed'] and not new_flight['passed']
real=read(ROOT/'config/airframes/real_airframe_pending.json');assert real['mass_kg'] is None and not real['hardware_deployment_allowed']
last=chain['last_common_precontact_sample'];prior=old_chain['last_common_precontact_sample'];pre=new_flight['precontact']
report={'passed':False,'diagnostic_and_flow_checks_passed':True,'precision_touchdown_goal_passed':False,'new_physics_flight_performed':True,
        'simulation_only':True,'normal_disarm_observed':True,'linux_unit_tests_passed':71,'hardware_deployment_allowed':False,
        'physics_config_model_files_identical_to_step09':files,'control_module_files_identical_to_step09':True,
        'bridge_core_after_removing_only_diagnostic_capture_is_identical':True,'controller_gain_and_simulation_parameters_compared':len(keys),'changed_control_or_simulation_parameters':changed,
        'telemetry_capture_counts':capture['counts'],'native_final_dataflash_hash':read(run/'results/dataflash_final/extraction_manifest.json')['input_sha256'],
        'actual_approach_to_contact_s':new_flight['actual_main_approach_to_first_contact_s'],'actual_precontact_nose_up_deg':pre['nose_up_deg'],
        'actual_precontact_foot_tangential_speed_upper_bound_m_s':max(v['tangential_speed_upper_bound_m_s'] for v in pre['feet'].values()),
        'last_common_precontact_stage_sample':last,'previous_flight_stage_sample':prior,
        'remaining_open_questions':['Vertical tracking bias root cause not isolated by this experiment.','No new controller or compensation has been validated.','Real-time camera-plus-control observation stall remains unresolved.','No repeatability, time-optimality, statistical success-rate, or hardware validation claim.'],
        'source_sha256':{'flight_report':sha(run/'analysis/flight_verification.json'),'new_control_chain':sha(run/'control_chain/verification.json'),'previous_control_chain':sha(out/'previous_control_chain_v2/verification.json'),'postflight':sha(run/'results/postflight_checks.json'),'telemetry':sha(run/'results/fcu_diagnostic_messages.jsonl'),'finalizer':sha(Path(__file__))}}
(out/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
note=f'''# 第十一步：真实飞控控制链路复核

本步新增了一轮完整 Gazebo / ArduCopter 物理飞行，规范目录 `run_001`，不带相机订阅。轨迹、模型、质量/几何、控制逻辑和验收门限保持原值；只增加诊断遥测。没有设置飞行位姿、强制停转或部署真机。

**流程和诊断检查通过，精确触地目标仍失败，顶层 passed=false。** 本轮是定位控制阶段的复跑，没有宣称算法已修复。

## 新一轮实际结果

| 项目 | 新一轮实际结果 |
|---|---:|
| 主进场到首次甲板碰撞 | {new_flight['actual_main_approach_to_first_contact_s']:.3f} s（计划 6.400 s） |
| 接触前后倾角 | {pre['nose_up_deg']:.4f}° |
| 接触前首次活动脚切向速度上界 | {report['actual_precontact_foot_tangential_speed_upper_bound_m_s']:.5f} m/s |
| 原始首次活动碰撞 | {', '.join(pre['actual_first_collisions'])} |
| 速度估计窗口 | 约 80 ms，9 点，止于首次碰撞前 {pre['sample_lead_s']*1000:.1f} ms |
| LAND / 正常停转 / 停转后支撑检查 | 完成 |
| Linux 测试 | 71 项通过，起飞前运行 |
| 结束后自建进程 | 0 |

0.05 m/s 水平速度和 2° 目标姿态（允许误差 1°）仍未满足。速度是有限窗口因果拟合，不是瞬时冲击速度。

## 找到的控制阶段抵消

原始机载 DataFlash 日志已经包含 GUIP、PSCE/PSCD、PIDE 和 ATT。本步还新增了原生 MAVLink 期望姿态、位置目标、实际姿态等记录，共 {capture['rows']} 条。缺失消息保留为缺失，没有伪造；固件版本由 DataFlash VER 取得：{chain['version']['FWS']}。

下面是新一轮距离首次碰撞约 {last['time_before_first_contact_s']*1000:.1f} ms 的同一内部日志采样点，不是碰撞瞬间：

| 控制阶段 | 记录值 |
|---|---:|
| 外部轨迹推导的后倾参考 | {last['nominal_pitch_deg']:.4f}° |
| 飞控真正给姿态环的后倾目标 | {last['fcu_desired_pitch_deg']:.4f}° |
| 飞控测得实际后倾 | {last['fcu_actual_pitch_deg']:.4f}° |
| 水平位置目标减估计位置 | {last['horizontal_position_error_m']*100:.3f} cm |
| 整形后的前向速度前馈 | {last['desired_velocity_e_m_s']:.5f} m/s |
| 加位置纠偏后的速度目标 | {last['corrected_target_velocity_e_m_s']:.5f} m/s |
| 整形后的制动加速度前馈 | {last['desired_acceleration_e_m_s2']:.6f} m/s² |
| 速度外环给出的向前修正 | +{last['velocity_feedback_correction_m_s2']:.6f} m/s² |
| 最终净加速度目标 | {last['corrected_target_acceleration_e_m_s2']:.6f} m/s² |

飞机落后于位置目标约 8.4 cm。该时刻位置外环增加追赶速度，速度外环输出正向加速度修正，抵消了部分负向制动前馈；因此飞控交给姿态环的目标已小于轨迹推导角度。期望与实际姿态相差约 {abs(last['fcu_desired_pitch_deg']-last['fcu_actual_pitch_deg']):.4f}°。最后约 1.6 s 的内环姿态平均绝对跟踪误差为 {chain['tail_statistics']['pitch_tracking_mean_absolute_error_deg']:.4f}°。

这条阶段关系由对应版本[位置控制源码](https://github.com/ArduPilot/ardupilot/blob/dbe79216/libraries/AC_AttitudeControl/AC_PosControl.cpp)中“位置误差形成速度修正、速度 PID 修正叠加加速度前馈、净推力方向进入姿态环”的结构解释，并由实际日志逐项闭合。末段加速度恒等式的最大残差约 {chain['tail_statistics']['maximum_acceleration_stage_closure_error_m_s2']:.3g} m/s²。这里说明本次记录中的具体控制机制，不代表所有条件下的唯一误差原因。

上一轮 run_004 也出现相同结构：约 {prior['time_before_first_contact_s']*1000:.1f} ms 接触前，轨迹后倾 {prior['nominal_pitch_deg']:.4f}°，飞控姿态目标 {prior['fcu_desired_pitch_deg']:.4f}°，实际 {prior['fcu_actual_pitch_deg']:.4f}°；制动前馈 {prior['desired_acceleration_e_m_s2']:.6f} 与向前修正 +{prior['velocity_feedback_correction_m_s2']:.6f} 叠加为 {prior['corrected_target_acceleration_e_m_s2']:.6f} m/s²。两轮不等于统计成功率研究。

## 时间、字段与输入核对

- DataFlash 的 TimeUS 与 Gazebo 时刻没有直接混用。以 SIM2 的仿真真值位置拟合恒定时移 {chain['clock_alignment']['dataflash_to_gazebo_offset_s']*1000:.1f} ms，并用交错留出与前后半程检验。留出位置 RMS 残差约 {chain['clock_alignment']['holdout_position_rmse_m']*1000:.3f} mm，分析另留 {chain['clock_alignment']['precontact_exclusion_guard_s']*1000:.1f} ms 的接触前排除区间。这不是硬件时钟同步或通信延迟测量。
- 152 条分析区间内 GUIP 接收目标与保存的外部 P/V/A 决策逐条匹配，最大字段误差低于 1e-4；这比只重放发送函数多了一层实际接收端记录证据。
- PSCE.AE 是由实际倾角反算的水平加速度量，**不是实测平动加速度**，未拿它验证平动物理响应。PSCE.DAE 是内部整形后的前馈，不直接等于当前外部发来的 GUIP.aY；两个阶段分别保留。
- 最后采样点的 PSCD 估计位置低于其内部高度目标约 {last['vertical_estimate_below_internal_target_m']*100:.3f} cm。竖直偏低的根因仍未被单因素隔离，不能宣称本步已解决提前接触。
- 已逐项比较 {len(files)} 个模型/世界/配置文件与 {len(keys)} 个控制/仿真参数，保持一致；移除诊断记录与额外遥测请求后，桥接控制源码与原版完全相同。
- 新日志在所有自建仿真进程停止后重新提取，使用 `results/dataflash_final`。运行中首次提取的 `results/dataflash` 只作调试历史，避免使用仍在写入的二进制作为最终证据。旧控制链分析最终目录为 `previous_control_chain_v2`；初版报告一个时移元数据字段被临时变量覆盖，初版不作为最终证据，数值比较已在 v2 重算。

## 下一步

修改重点转到外部轨迹与飞控位置/速度外环的协调，避免末段“追赶位置”抵消所需制动；同时以单因素对照继续处理竖直跟踪。不能因为末端角度小就直接提高姿态内环增益。新控制方案还没有实现或验证；接触速度/姿态门限不放宽，禁止平台下方参考或触地后继续空中轨迹。

并行相机录制的观测超时、视觉/GPS 链路和真机参数事项保持原状态。没有录制本轮在飞相机录像，不把状态重渲染当新录像。真机参数仍待定，没有借用 Iris 数值填充。

![飞控控制阶段](run_001/control_chain/fcu_control_chain.png)
'''
(out/'第十一步_飞控外环与后倾目标复核.md').write_text(note,encoding='utf-8')
(out/'previous_control_chain/NOT_CANONICAL.md').write_text('初版存在 first_half_offset_s 元数据变量覆盖问题；最终重算结果在 ../previous_control_chain_v2。不要使用本目录为最终证据。',encoding='utf-8')
print(json.dumps({'diagnostic_and_flow_checks_passed':True,'precision_goal_passed':False,'physics_files_compared':len(files),'control_parameters_compared':len(keys),'new_precontact_pitch_deg':pre['nose_up_deg'],'last_control_stage_sample':last},ensure_ascii=False,indent=2))
