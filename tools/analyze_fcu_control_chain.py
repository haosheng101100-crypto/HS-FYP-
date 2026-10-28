"""Audit frozen DataFlash controller stages against archived Gazebo physics."""
from pathlib import Path
from collections import defaultdict
import argparse,hashlib,json,math,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'work/plot_dependencies'))
import numpy as np
from landing.reference_loader import load_verified_reference

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def run_analysis(run,dataflash,out):
    out.mkdir(parents=True,exist_ok=False)
    flight=json.loads((run/'analysis/flight_verification.json').read_text())
    for name,digest in flight['source_sha256'].items():assert sha(run/name.replace('\\','/'))==digest,name
    extraction=json.loads((dataflash/'extraction_manifest.json').read_text())
    assert sha(run/'runtime/logs/00000001.BIN')==extraction['input_sha256'],'Require closed immutable DataFlash binary'
    native=defaultdict(list)
    for line in (dataflash/'control_messages.jsonl').open():
        row=json.loads(line);native[row['mavpackettype']].append(row)
    params=json.loads((dataflash/'parameters.json').read_text())
    source_manifest=json.loads((ROOT/'outputs/model_audit/step11/official_source/manifest.json').read_text())
    for item in source_manifest['files']:assert sha(ROOT/'outputs/model_audit/step11/official_source'/item['path'])==item['sha256']
    assert 'dbe79216' in native['VER'][0]['FWS']
    curve,_,_,_=load_verified_reference(run)
    start=flight['controller_status']['main_approach_start_sim_t_s'];touch=flight['precontact']['raw_first_contact_sim_t_s']
    raw=[]
    with (run/'results/model_states.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            if row['kind']!='uav_pose':continue
            p=next(p for p in row['message']['pose'] if p['name']=='uav')
            h=p['header']['stamp'];t=float(h.get('sec',0))+float(h.get('nsec',0))*1e-9
            xyz=[p.get('position',{}).get(k,0.) for k in 'xyz'];q=[p['orientation'].get(k,0.) for k in 'xyzw']
            # Forward body x component along world z, in the actual FLU model.
            pitch=math.degrees(math.asin(max(-1.,min(1.,2*(q[0]*q[2]-q[3]*q[1])))))
            raw.append([t,*xyz,pitch])
    raw=np.array(raw);rt=raw[:,0]
    sim2=np.array([[r['TimeUS']*1e-6,r['PE'],r['PN'],-r['PD']] for r in native['SIM2']])
    selected=sim2[(sim2[:,0]>start+.6)&(sim2[:,0]<touch-.4)][::10]
    assert len(selected)>50
    grid=np.linspace(-.1,.1,1001)
    def fit_clock(points):
        errors=[]
        for delta in grid:
            pred=np.column_stack([np.interp(points[:,0]+delta,rt,raw[:,i]) for i in range(1,4)])
            errors.append(float(np.mean(np.sum((pred-points[:,1:])**2,axis=1))))
        index=int(np.argmin(errors));assert 0<index<len(grid)-1
        return float(grid[index]),float(math.sqrt(errors[index]))
    delta,rmse=fit_clock(selected[::2])
    # Evaluate hold-out at the training offset, instead of refitting to it.
    hold=selected[1::2];hold_pred=np.column_stack([np.interp(hold[:,0]+delta,rt,raw[:,i]) for i in range(1,4)])
    hold_rmse=float(np.sqrt(np.mean(np.sum((hold_pred-hold[:,1:])**2,axis=1))))
    offset_first,_=fit_clock(selected[:len(selected)//2]);offset_second,_=fit_clock(selected[len(selected)//2:])
    guard=.01+abs(offset_first-offset_second)+.0025
    assert rmse<.01 and hold_rmse<.01 and abs(offset_first-offset_second)<.02,'Clock matching insufficient; refuse precise stage comparison'
    groups={typ:{r['TimeUS']:r for r in native[typ]} for typ in ['PSCE','PSCD','PIDE','ATT']}
    common=sorted(set.intersection(*(set(v) for v in groups.values())))
    rows=[]
    for time_us in common:
        t=time_us*1e-6+delta
        if not start+.05<t<touch-guard:continue
        e,d,pid,att=[groups[k][time_us] for k in ['PSCE','PSCD','PIDE','ATT']]
        ref=curve.at(t-start)
        pid_sum=sum(pid.get(k,0.) for k in ['P','I','D','FF','DFF'])
        rows.append({'dataflash_time_us':time_us,'aligned_gazebo_sim_t_s':t,'elapsed_s':t-start,'PSCE':e,'PSCD':d,'PIDE':pid,'ATT':att,
            'reference_pitch_deg':math.degrees(ref['nose_up_rad']),'raw_gazebo_pitch_interpolated_deg':float(np.interp(t,rt,raw[:,4])),
            'reference_body_position_m':ref['motion'].pose.position_world_m,
            'horizontal_position_error_m':e['TPE']-e['PE'],'horizontal_velocity_position_correction_m_s':e['TVE']-e['DVE'],
            'horizontal_acceleration_feedback_correction_m_s2':e['TAE']-e['DAE'],'horizontal_velocity_pid_terms_sum_m_s2':pid_sum,
            'horizontal_acceleration_stage_closure_m_s2':e['TAE']-e['DAE']-pid_sum,
            'position_to_velocity_linear_residual_m_s':e['TVE']-e['DVE']-params['PSC_NE_POS_P']*(e['TPE']-e['PE']),
            'vertical_estimate_below_internal_position_target_m':d['PD']-d['TPD']})
    assert len(rows)>40
    tail=[r for r in rows if r['elapsed_s']>4.5];last=tail[-1]
    assert max(abs(r['horizontal_acceleration_stage_closure_m_s2']) for r in tail)<.01
    assert max(abs(r['position_to_velocity_linear_residual_m_s']) for r in tail)<.005
    assert max(abs(r['ATT']['Pitch']-r['raw_gazebo_pitch_interpolated_deg']) for r in tail)<.5
    feedback=[json.loads(line) for line in (run/'results/execution_feedback.jsonl').read_text().splitlines()]
    commands=[];command_times=[];offset=flight['controller_status']['alignment_offset_ned_m']
    for r in feedback:
        cmd=r['decision']['reference']
        if cmd is None or not start<=r['sim_t_s']<touch:continue
        p,v,a=[cmd[k] for k in ['position_world_m','velocity_world_m_s','acceleration_world_m_s2']]
        commands.append([p[1]+offset[0],p[0]+offset[1],-p[2]+offset[2],v[1],v[0],-v[2],a[1],a[0],-a[2]])
        command_times.append(r['sim_t_s'])
    commands=np.array(commands);matches=[]
    for r in native['GUIP']:
        t=r['TimeUS']*1e-6+delta
        if not start+.05<t<touch-guard:continue
        values=np.array([r[k] for k in ['pX','pY','pZ','vX','vY','vZ','aX','aY','aZ']])
        errors=np.max(abs(commands-values),axis=1);i=int(np.argmin(errors))
        matches.append({'native_time_us':r['TimeUS'],'max_abs_pva_field_error':float(errors[i]),'matching_decision_sim_t_s':command_times[i],'aligned_receipt_minus_send_sim_s':float(t-command_times[i])})
    assert len(matches)>100 and max(r['max_abs_pva_field_error'] for r in matches)<1e-4
    def brief(row):
        return {'aligned_sim_t_s':row['aligned_gazebo_sim_t_s'],'time_before_first_contact_s':touch-row['aligned_gazebo_sim_t_s'],
                'nominal_pitch_deg':row['reference_pitch_deg'],'fcu_desired_pitch_deg':row['ATT']['DesPitch'],'fcu_actual_pitch_deg':row['ATT']['Pitch'],
                'horizontal_position_error_m':row['horizontal_position_error_m'],'desired_velocity_e_m_s':row['PSCE']['DVE'],'corrected_target_velocity_e_m_s':row['PSCE']['TVE'],
                'desired_acceleration_e_m_s2':row['PSCE']['DAE'],'velocity_feedback_correction_m_s2':row['horizontal_acceleration_feedback_correction_m_s2'],
                'corrected_target_acceleration_e_m_s2':row['PSCE']['TAE'],'pid_terms_sum_m_s2':row['horizontal_velocity_pid_terms_sum_m_s2'],
                'acceleration_closure_error_m_s2':row['horizontal_acceleration_stage_closure_m_s2'],
                'vertical_estimate_below_internal_target_m':row['vertical_estimate_below_internal_position_target_m']}
    report={'diagnostic_passed':True,'new_physics_flight_performed':run.name=='run_001','flight_goal_passed':flight['passed'],'version':native['VER'][0],
            'clock_alignment':{'method':'Fit a constant timestamp shift by SIM2 simulator-position agreement with raw Gazebo body positions, without fitting spatial offsets. Hold-out interleaved samples plus split-window stability check. Not hardware clock synchronization or a command latency measurement.',
                'dataflash_to_gazebo_offset_s':delta,'training_position_rmse_m':rmse,'holdout_position_rmse_m':hold_rmse,'first_half_offset_s':offset_first,'second_half_offset_s':offset_second,'precontact_exclusion_guard_s':guard},
            'checks':{'closed_binary_hash_matches_extraction':True,'official_source_hashes_match':True,'received_guip_targets_match_saved_sent_pva':True,'native_acceleration_feedback_stage_closes':True,'position_velocity_correction_closes_in_tail':True,'native_actual_pitch_agrees_with_gazebo':True},
            'last_common_precontact_sample':brief(last),'tail_statistics':{'sample_count':len(tail),'pitch_tracking_mean_absolute_error_deg':float(np.mean([abs(r['ATT']['DesPitch']-r['ATT']['Pitch']) for r in tail])),
                'maximum_acceleration_stage_closure_error_m_s2':max(abs(r['horizontal_acceleration_stage_closure_m_s2']) for r in tail),
                'maximum_native_gazebo_pitch_difference_deg':max(abs(r['ATT']['Pitch']-r['raw_gazebo_pitch_interpolated_deg']) for r in tail)},
            'received_guip_match_count':len(matches),'received_guip_max_pva_error':max(r['max_abs_pva_field_error'] for r in matches),
            'field_semantics':{'PSCE_AE':'Computed from actual lean angles by lean_angles_to_accel_NE_mss(), NOT measured translational acceleration. Never used as a physical acceleration measurement in this diagnosis.',
                'PSCE_DAE':'Shaped desired acceleration before velocity-feedback correction. Distinct from the raw GUIP command.',
                'PSCE_TAE':'Net target acceleration after velocity controller, feedforward, offsets and limits.',
                'ATT_DesPitch':'FCU attitude-controller target, not the external polynomial inferred angle.',
                'PSCD_PD_minus_TPD':'Actual controller-estimated Down position minus its internal Down target. Positive means below target; this is not an identified root cause of vertical error.'},
            'scope':'Explains a recorded control-stage cancellation and reproduces one unchanged-reference flight. Does not prove a replacement controller, cause of all vertical error, or robust success.',
            'source_sha256':{'dataflash_binary':extraction['input_sha256'],'dataflash_messages':sha(dataflash/'control_messages.jsonl'),'flight_verification':sha(run/'analysis/flight_verification.json'),'analysis_script':sha(Path(__file__))}}
    assert all(isinstance(report['clock_alignment'][k],float) for k in ['dataflash_to_gazebo_offset_s','first_half_offset_s','second_half_offset_s'])
    (out/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    (out/'aligned_control_stages.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    (out/'received_pva_matches.json').write_text(json.dumps(matches,indent=2),encoding='utf-8')
    plot(rows,curve,touch-start,out)
    print(json.dumps({'clock_alignment':report['clock_alignment'],'last_common_precontact_sample':report['last_common_precontact_sample'],'tail_statistics':report['tail_statistics'],'received_pva_matches':len(matches)},ensure_ascii=False,indent=2))
    return report

def plot(rows,curve,touch,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc');plt.rcParams['font.family']=font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name();plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(2,2,figsize=(12,8),sharex=True,layout='constrained');t=[r['elapsed_s'] for r in rows]
    axes[0,0].plot(t,[r['reference_pitch_deg'] for r in rows],'--',label='外部轨迹推导的期望后倾');axes[0,0].plot(t,[r['ATT']['DesPitch'] for r in rows],label='飞控实际给姿态内环的目标');axes[0,0].plot(t,[r['ATT']['Pitch'] for r in rows],label='飞控实测姿态');axes[0,0].set_ylabel('后倾角 / °')
    axes[0,1].plot(t,[r['PSCE']['DAE'] for r in rows],'--',label='飞控整形后的加速度前馈');axes[0,1].plot(t,[r['horizontal_acceleration_feedback_correction_m_s2'] for r in rows],label='速度外环反馈修正');axes[0,1].plot(t,[r['PSCE']['TAE'] for r in rows],label='修正后净加速度目标');axes[0,1].set_ylabel('前向加速度目标 / (m/s²)')
    axes[1,0].plot(t,[r['PSCE']['DVE'] for r in rows],'--',label='整形后的前向速度前馈');axes[1,0].plot(t,[r['PSCE']['TVE'] for r in rows],label='加入位置纠偏后的速度目标');axes[1,0].plot(t,[r['PSCE']['VE'] for r in rows],label='飞控估计前向速度');axes[1,0].set_ylabel('前向速度 / (m/s)')
    axes[1,1].plot(t,[r['vertical_estimate_below_internal_position_target_m']*100 for r in rows],label='飞控估计位置低于其高度目标');axes[1,1].axhline(0,c='#777',ls=':');axes[1,1].set_ylabel('偏低量 / cm')
    for ax in axes.flat:ax.set_xlim(4.4,curve.T+.05);ax.axvline(touch,c='#b44',ls='--',label='原始首次碰撞');ax.axvline(curve.T,c='#777',ls=':',label='计划终点');ax.grid(alpha=.2);ax.legend(fontsize=7)
    axes[0,0].set_ylim(-.5,7.);axes[0,1].set_ylim(-1.25,.6);axes[1,0].set_ylim(-.05,1.55)
    for ax in axes[-1]:ax.set_xlabel('进场后的仿真时间 / s')
    fig.suptitle('第十一步：实际机载日志的控制链路\nAE 字段不作为实测平动加速度；所有曲线在首次碰撞前截止')
    fig.savefig(out/'fcu_control_chain.png',dpi=150);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,required=True);p.add_argument('--dataflash-dir',type=Path,required=True);p.add_argument('--out-dir',type=Path,required=True);a=p.parse_args();run_analysis(a.run_dir.resolve(),a.dataflash_dir.resolve(),a.out_dir.resolve())
