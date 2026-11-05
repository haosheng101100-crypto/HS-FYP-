"""Offline, same-time residual audit; never sends commands or changes a flight."""
from pathlib import Path
import argparse,hashlib,json,math,sys
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'work/plot_dependencies'))
from landing.contact_geometry import Pose,RigidMotion,evaluate_airframe,sub
from landing.reference_loader import load_verified_reference
from landing.mavlink_reference import dispatch,LocalNedAlignment

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def pitch(pose):return math.degrees(math.asin(max(-1.,min(1.,pose.rotate((1,0,0))[2]))))
def feet(geometry,motion,deck):return {f.name:f for f in evaluate_airframe(geometry,motion,deck)}

def diagnose(run,out):
    run=run.resolve();out=out.resolve()
    assert out!=run and run not in out.parents,'Write derived diagnosis outside preserved run'
    out.mkdir(parents=True,exist_ok=True)
    prior_path=run/'analysis/flight_verification.json'
    prior=json.loads(prior_path.read_text(encoding='utf-8'))
    for name,digest in prior['source_sha256'].items():
        assert sha(run/name.replace('\\','/'))==digest,('Changed evidence',name)
    assert prior['checks']['preserved_source_hashes_match']
    attempt=json.loads((run/'attempt_manifest.json').read_text())
    assert all(sha(run/name.replace('\\','/'))==digest for name,digest in attempt['source_sha256'].items())
    assert sha(ROOT/'landing/mavlink_reference.py')==sha(run/'landing/mavlink_reference.py'),'Dry-run dispatcher must match frozen flight source'
    curve,geometry,deck,profile=load_verified_reference(run)
    assert deck.motion.velocity_at_origin_world_m_s==(0.,0.,0.)
    assert deck.motion.angular_velocity_world_rad_s==(0.,0.,0.)
    assert deck.motion.pose.rotate((0,0,1))==(0.,0.,1.)
    status=prior['controller_status'];start=status['main_approach_start_sim_t_s'];pre=prior['precontact']
    sample=pre['sample_sim_t_s'];touch=pre['raw_first_contact_sim_t_s'];elapsed=sample-start
    assert sample<touch and 0<elapsed<curve.T
    raw_pose=None
    with (run/'results/model_states.jsonl').open() as stream:
        for line in stream:
            entry=json.loads(line)
            if entry['kind']!='uav_pose':continue
            p=next(p for p in entry['message']['pose'] if p['name']=='uav')
            stamp=p['header']['stamp'];t=float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9
            if abs(t-sample)<1e-9:
                raw_pose=Pose(tuple(p.get('position',{}).get(k,0.) for k in 'xyz'),tuple(p['orientation'].get(k,0.) for k in ['x','y','z','w']))
                break
    assert raw_pose is not None
    actual=RigidMotion(raw_pose,pre['body_velocity_world_m_s'],pre['angular_velocity_world_rad_s'])
    reference=curve.at(elapsed);terminal=curve.at(curve.T)
    af,rf,ef=[feet(geometry,m,deck) for m in (actual,reference['motion'],terminal['motion'])]
    # Swap position while retaining actual attitude: exact level-deck gap decomposition.
    swapped=RigidMotion(Pose(reference['motion'].pose.position_world_m,actual.pose.orientation_xyzw),actual.velocity_at_origin_world_m_s,actual.angular_velocity_world_rad_s)
    sf=feet(geometry,swapped,deck)
    comparisons={}
    for name in pre['actual_first_collisions']:
        a,r,e,s=[f[name] for f in [af,rf,ef,sf]]
        phase=sub(r.relative_velocity_deck_m_s,e.relative_velocity_deck_m_s)
        tracking=sub(a.relative_velocity_deck_m_s,r.relative_velocity_deck_m_s)
        residual=sub(a.relative_velocity_deck_m_s,e.relative_velocity_deck_m_s)
        gap_translation=a.clearance_m-s.clearance_m
        gap_attitude=s.clearance_m-r.clearance_m
        assert abs(a.clearance_m-r.clearance_m-gap_translation-gap_attitude)<1e-12
        assert max(abs(residual[k]-phase[k]-tracking[k]) for k in range(3))<1e-12
        assert abs(a.tangential_speed_upper_bound_m_s-pre['feet'][name]['tangential_speed_upper_bound_m_s'])<1e-10
        comparisons[name]={
            'actual_precontact':{'gap_m':a.clearance_m,'velocity_deck_m_s':a.relative_velocity_deck_m_s,'tangential_speed_upper_bound_m_s':a.tangential_speed_upper_bound_m_s},
            'reference_same_time':{'gap_m':r.clearance_m,'velocity_deck_m_s':r.relative_velocity_deck_m_s,'tangential_speed_upper_bound_m_s':r.tangential_speed_upper_bound_m_s},
            'reference_terminal':{'gap_m':e.clearance_m,'velocity_deck_m_s':e.relative_velocity_deck_m_s,'tangential_speed_upper_bound_m_s':e.tangential_speed_upper_bound_m_s},
            'exact_signed_velocity_identity':{'actual_minus_terminal_m_s':residual,'reference_phase_minus_terminal_m_s':phase,'actual_minus_same_time_reference_m_s':tracking,'interpretation':'Signed vectors at the named support representatives. Algebraic comparison, not causal attribution, norm decomposition, or a prediction if flight continued.'},
            'exact_gap_identity':{'actual_minus_same_time_reference_m':a.clearance_m-r.clearance_m,'translation_contribution_m':gap_translation,'attitude_support_geometry_contribution_m':gap_attitude,'interpretation':'Instantaneous position/attitude swap in the same static level deck geometry. Not a feasible new flight or a contact-delay prediction.'}}
    feedback=[json.loads(line) for line in (run/'results/execution_feedback.jsonl').read_text().splitlines()]
    rows=[];max_command_error=0.
    for row in feedback:
        t=row['sim_t_s']
        if not start<=t<touch or row['decision']['reference'] is None:continue
        ref=curve.at(t-start);motion=row['observation']['motion'];cmd=row['decision']['reference']
        actual_motion=RigidMotion(Pose(**motion['pose']),motion['velocity_at_origin_world_m_s'],motion['angular_velocity_world_rad_s'])
        for key,expected in [('position_world_m',ref['motion'].pose.position_world_m),('velocity_world_m_s',ref['motion'].velocity_at_origin_world_m_s),('acceleration_world_m_s2',ref['body_acceleration_world_m_s2'])]:
            max_command_error=max(max_command_error,max(abs(a-b) for a,b in zip(cmd[key],expected)))
        # Execute the hash-matched dispatcher into an in-memory sink only.
        packets=[]
        sink=SimpleNamespace(mav=SimpleNamespace(set_position_target_local_ned_send=lambda *args:packets.append(args)))
        decision=SimpleNamespace(**dict(row['decision'],reference=SimpleNamespace(**cmd)))
        def unexpected_land():raise AssertionError('Precontact dry-run must not request LAND')
        dispatch(decision,sink,unexpected_land,LocalNedAlignment(tuple(status['alignment_offset_ned_m'])),t)
        assert len(packets)==1
        packet=packets[0];assert packet[3]==1 and packet[4]==2048
        reconstructed_ned={'position_m':packet[5:8],'velocity_m_s':packet[8:11],'acceleration_m_s2':packet[11:14],'yaw_rad':packet[14],'type_mask':packet[4]}
        f=feet(geometry,actual_motion,deck);r=feet(geometry,ref['motion'],deck)
        rows.append({'elapsed_s':t-start,'sim_t_s':t,'pitch_actual_deg':pitch(actual_motion.pose),'pitch_reference_deg':math.degrees(ref['nose_up_rad']),
                     'body_position_error_m':sub(actual_motion.pose.position_world_m,ref['motion'].pose.position_world_m),
                     'body_velocity_actual_m_s':actual_motion.velocity_at_origin_world_m_s,'body_velocity_reference_m_s':ref['motion'].velocity_at_origin_world_m_s,
                     'body_acceleration_actual_m_s2':row['fit']['body_acceleration_world_m_s2'],'body_acceleration_reference_m_s2':ref['body_acceleration_world_m_s2'],
                     'foot_horizontal_speed_actual_m_s':max(f[n].tangential_speed_upper_bound_m_s for n in comparisons),
                     'foot_horizontal_speed_reference_m_s':max(r[n].tangential_speed_upper_bound_m_s for n in comparisons),
                     'minimum_gap_actual_m':min(x.clearance_m for x in f.values()),'minimum_gap_reference_m':min(x.clearance_m for x in r.values()),
                     'reconstructed_requested_mavlink_ned':reconstructed_ned})
    assert rows and max_command_error<1e-10
    prior_live_pitch=pre['nose_up_deg'];ref_pitch=math.degrees(reference['nose_up_rad']);end_pitch=math.degrees(terminal['nose_up_rad'])
    checks={'source_evidence_hashes_match':True,'pinned_reference_and_model_hashes_match':True,'only_precontact_samples_used':all(r['sim_t_s']<touch for r in rows),
            'requested_pva_matches_pinned_body_origin_reference':max_command_error<1e-10,'frozen_dispatcher_dry_run_sends_one_pva_packet_per_precontact_decision':True,'foot_speeds_reproduce_prior_audit':True,
            'signed_velocity_and_gap_identities_close':True,'actual_endpoint_still_fails_original_precision_goal':not prior['passed']}
    result={'diagnostic_checks_passed':all(checks.values()),'checks':checks,'simulation_only':True,'new_physics_flight_performed':False,'flight_goal_passed':prior['passed'],
            'reference_or_controller_modified':False,'causal_identification_completed':False,'sample_lead_before_first_collision_s':touch-sample,
            'contact_early_by_s':start+curve.T-touch,'remaining_reference_time_at_sample_s':curve.T-elapsed,
            'pitch_comparison_deg':{'actual':prior_live_pitch,'same_time_reference':ref_pitch,'terminal_reference':end_pitch,
                'actual_minus_same_time_reference':prior_live_pitch-ref_pitch,'same_time_reference_minus_terminal':ref_pitch-end_pitch},
            'body_position_actual_minus_same_time_reference_m':sub(actual.pose.position_world_m,reference['motion'].pose.position_world_m),
            'body_acceleration_actual_m_s2':pre['body_acceleration_world_m_s2'],'body_acceleration_same_time_reference_m_s2':reference['body_acceleration_world_m_s2'],
            'foot_comparisons':comparisons,'max_reconstructed_requested_pva_error':max_command_error,'precontact_control_sample_count':len(rows),
            'interface_audit':{'position_velocity_acceleration_active':True,'force_flag_active':False,'yaw_active':True,'yaw_rate_ignored':True,'independent_pitch_command_sent':False,
                'scope':'Hash-matched frozen dispatcher replayed into an in-memory sink; no transport connection, wire capture, FCU internal target telemetry, packet-acceptance proof, or audit of binary internal implementation.'},
            'limitations':['One closed-loop flight cannot identify response delay, gains, actuator capabilities, or a unique cause.','No actual state after first contact is extrapolated as free flight.','Near-contact velocity is an 80 ms causal pose fit ending 3 ms before collision, not exact impact velocity.','The counterfactual geometry and signed residual identity do not prove that correcting height alone yields a successful landing.'],
            'source_sha256':{'run_004/'+name:digest for name,digest in prior['source_sha256'].items()}}
    result['source_sha256']['prior_analysis']=sha(prior_path);result['source_sha256']['diagnostic_script']=sha(Path(__file__))
    (out/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    (out/'tracking_samples.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    plot(rows,curve,touch-start,out)
    write_note(result,out)
    print(json.dumps({k:result[k] for k in ['diagnostic_checks_passed','new_physics_flight_performed','flight_goal_passed','contact_early_by_s','pitch_comparison_deg','foot_comparisons']},ensure_ascii=False,indent=2))

def plot(rows,curve,touch,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
    plt.rcParams['font.family']=font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name();plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(3,2,figsize=(12,10),layout='constrained',sharex=True)
    t=[r['elapsed_s'] for r in rows]
    panels=[('pitch_actual_deg','pitch_reference_deg','后倾角 / °'),('foot_horizontal_speed_actual_m_s','foot_horizontal_speed_reference_m_s','首次触地后腿水平速度 / (m/s)'),('minimum_gap_actual_m','minimum_gap_reference_m','最低脚底净空 / m')]
    for ax,(actual,ref,label) in zip(axes.flat,panels):
        ax.plot(t,[r[actual] for r in rows],label='实际：接触前因果观测');ax.plot(t,[r[ref] for r in rows],'--',label='同一仿真时刻的参考');ax.set_ylabel(label)
    axes[0,1].axhline(.05,c='#b44',ls=':',label='触地验收上限')
    axes[0,0].set_ylim(-.5,7.)
    axes[0,1].set_ylim(-.02,1.35)
    axes[1,0].axhline(0,c='#777',ls=':')
    axes[1,0].set_ylim(-.005,.12);axes[1,0].set_ylabel('末段脚底净空放大 / m')
    for ax,k,label in [(axes[1,1],0,'前向位置实际减参考 / m'),(axes[2,0],2,'高度实际减参考 / m')]:
        ax.plot(t,[r['body_position_error_m'][k] for r in rows],label='同一时刻的位置残差');ax.axhline(0,c='#777',ls=':');ax.set_ylabel(label)
    axes[2,1].plot(t,[r['body_acceleration_actual_m_s2'][0] for r in rows],label='实际机体加速度：因果拟合')
    axes[2,1].plot(t,[r['body_acceleration_reference_m_s2'][0] for r in rows],'--',label='实际发送决策中的加速度参考');axes[2,1].set_ylabel('前向机体加速度 / (m/s²)')
    axes[2,1].set_ylim(-1.2,.05)
    for ax in axes.flat:
        ax.axvline(touch,c='#b44',ls='--',label='实际首次碰撞');ax.axvline(curve.T,c='#777',ls=':',label='参考计划终点');ax.set_xlim(4.4,curve.T+.05);ax.grid(alpha=.2);ax.legend(fontsize=7)
    for ax in axes[-1]:ax.set_xlabel('进场后的仿真时间 / s')
    fig.suptitle('第十步：原始 run_004 同时刻误差诊断（未进行新飞行）\n实际线止于首次碰撞前；没有外推碰撞后的空中响应')
    fig.savefig(out/'touchdown_tracking_diagnostic.png',dpi=150);plt.close(fig)

def write_note(r,out):
    foot=next(iter(r['foot_comparisons'].values()));a=foot['actual_precontact'];s=foot['reference_same_time'];gap=foot['exact_gap_identity'];p=r['pitch_comparison_deg']
    note=f'''# 第十步：把提前触地与制动态度误差分开

本步完成离线诊断，没有新物理飞行，没有修改轨迹、控制器或验收门限。数据来自 step09/run_004；原始精确触地验收仍未通过。诊断完整性检查通过，不代表着陆目标通过。

## 关键结果

在首次碰撞前约 3 ms 的同一时刻，参考还剩 {r['remaining_reference_time_at_sample_s']:.3f} s 才到计划终点。首次物理接触比计划早 {r['contact_early_by_s']:.3f} s。

| 项目 | 实际 | 同时刻参考 | 计划终点 |
|---|---:|---:|---:|
| 后倾角 | {p['actual']:.4f}° | {p['same_time_reference']:.4f}° | {p['terminal_reference']:.4f}° |
| 左后腿水平速度上界 | {a['tangential_speed_upper_bound_m_s']:.5f} m/s | {s['tangential_speed_upper_bound_m_s']:.5f} m/s | 0 m/s |
| 左后腿脚底净空 | {a['gap_m']*1000:.3f} mm | {s['gap_m']*1000:.3f} mm | 0 mm |

实际左后腿水平速度略低于同时刻参考，不能把触地速度超标直接解释为“实际刹得比参考慢”。该时刻参考本身仍在运动，实际提前碰到平台。两个后腿的完整有符号速度分解见 verification.json；这是代数残差分解，不是因果贡献率。

同一时刻，实际机体高度比参考低约 {-r['body_position_actual_minus_same_time_reference_m'][2]*1000:.2f} mm。左后腿净空误差可精确分成：平移项 {gap['translation_contribution_m']*1000:.3f} mm，加姿态/支撑几何项 {gap['attitude_support_geometry_contribution_m']*1000:.3f} mm，总计 {gap['actual_minus_same_time_reference_m']*1000:.3f} mm。此处采用静止水平甲板上的位姿交换计算，不是一次可行飞行，也不能据此预测修正后接触时刻。

姿态误差则没有被提前触地解释掉：同时刻参考 {p['same_time_reference']:.4f}° 已接近终端 2°，实际仍只有 {p['actual']:.4f}°，差约 {p['actual_minus_same_time_reference']:.4f}°。

## 指令链路核对

逐条核对 {r['precontact_control_sample_count']} 个接触前控制决策，P/V/A 与固定参考的机体原点状态一致，最大数值差 {r['max_reconstructed_requested_pva_error']:.3g}。ENU→NED 使用 x/y 交换、z 反号及独立初始平移；掩码 2048 启用位置、速度、加速度和偏航，忽略偏航角速度，未设置力标志。没有直接发送独立俯仰角命令；2° 是由参考质心加速度推导的期望姿态，不是已经被飞控强制实现的角度。

另用哈希与原始飞行一致的发送函数，把全部接触前决策发送到内存接收器，确认每次恰好生成一条 PVA 指令；没有建立飞控连接或发送新命令。此检查覆盖保存的决策与桥接源码，不是线上报文抓包或飞控内部目标记录。当前工程保存的是 ArduCopter 二进制，在此次检查的 vendor/ardupilot 与用户主目录候选路径中未找到对应飞控内部源码；不把指令字段重建称为已确认内部控制增益、延迟或执行行为。

## 下一次物理试验应解决什么

先核实竖直跟踪为什么在末段偏低，恢复真实脚底接触时刻；同时独立检查水平净加速度与后倾角的执行偏差。先增加飞控内部目标/实测响应记录，做保持模型、初态与门限一致的单因素对照，再选择反馈或响应补偿，不直接按这一条轨迹拟合出的偏置宣称普遍有效。

不能把实际接触后的自由飞行状态外推出来，也不能宣称只修高度就必然同时满足姿态与速度条件。并行相机录制时的观测超时仍未修复；视觉/GPS 仍按约定暂缓，真机未知参数保持空值。

![同时刻残差](touchdown_tracking_diagnostic.png)
'''
    (out/'第十步_提前触地与制动态度诊断.md').write_text(note,encoding='utf-8')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run-dir',type=Path,required=True);parser.add_argument('--out-dir',type=Path,required=True)
    args=parser.parse_args();diagnose(args.run_dir,args.out_dir)
