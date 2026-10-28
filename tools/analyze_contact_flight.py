"""Audit raw physics/contact logs of the live reference/hand-over experiment."""
from pathlib import Path
import argparse,hashlib,json,math,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'work/plot_dependencies'))
import numpy as np
from landing.contact_geometry import Pose,RigidMotion,evaluate_airframe
from landing.reference_loader import load_verified_reference

p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,required=True);args=p.parse_args()
run=args.run_dir.resolve();rs=run/'results';out=run/'analysis';out.mkdir(exist_ok=True)
status=json.loads((rs/'execution_status.json').read_text());manifest=json.loads((run/'attempt_manifest.json').read_text())
hashes_ok=all(hashlib.sha256((run/Path(name.replace('\\','/'))).read_bytes()).hexdigest()==digest for name,digest in manifest['source_sha256'].items())
curve,geometry,deck,profile=load_verified_reference(run)
feedback=[json.loads(line) for line in (rs/'execution_feedback.jsonl').read_text().splitlines()]
poses={};contacts=[];rotors=[];gimbal_relative=[]
def timestamp(msg):
    s=msg.get('header',{}).get('stamp',{});return float(s.get('sec',0))+float(s.get('nsec',0))*1e-9
def pose(data):return Pose(tuple(data.get('position',{}).get(k,0.) for k in 'xyz'),tuple(data['orientation'].get(k,0.) for k in ['x','y','z','w']))
def matrix(p):return np.array([p.rotate(axis) for axis in [(1,0,0),(0,1,0),(0,0,1)]]).T
with (rs/'model_states.jsonl').open() as stream:
    for line in stream:
        row=json.loads(line);msg=row['message']
        if row['kind']=='uav_pose':
            for item in msg.get('pose',[]):
                if item['name']=='uav':poses[timestamp(item)]=pose(item)
                if '::gimbal' in item['name']:gimbal_relative.append((timestamp(item),item))
        if row['kind']=='deck_contact':
            hits=[c for c in msg.get('contact',[]) if any(c[k]['name'].startswith('uav::') for k in ['collision1','collision2'])]
            if hits:contacts.append((timestamp(msg),hits))
        if row['kind']=='rotors':
            rotors.extend((timestamp(msg),item['name'],item.get('axis1',{}).get('velocity',0.)) for item in msg.get('joint',[]) if item['name'].startswith('rotor_'))
times=sorted(poses);start=status.get('main_approach_start_sim_t_s')
first=min(contacts,key=lambda row:row[0]) if contacts else None
precontact=None
if first:
    touch,hits=first
    pre=[t for t in times if t<=touch-.002]
    t=pre[-1];window=[x for x in pre if x>=t-.081];dt=np.array(window)-t
    pc=np.polyfit(dt,np.array([poses[x].position_world_m for x in window]),2)
    rc=np.polyfit(dt,np.array([matrix(poses[x]).reshape(-1) for x in window]),2)
    R=matrix(poses[t]);W=rc[1].reshape(3,3)@R.T;W=(W-W.T)/2
    omega=tuple(float(x) for x in [W[2,1],W[0,2],W[1,0]])
    motion=RigidMotion(poses[t],tuple(float(x) for x in pc[1]),omega)
    feet={f.name:f for f in evaluate_airframe(geometry,motion,deck)}
    names=sorted({c[k]['name'].split('::')[-1] for c in hits for k in ['collision1','collision2'] if c[k]['name'].startswith('uav::')})
    precontact={'sample_sim_t_s':t,'raw_first_contact_sim_t_s':touch,'sample_lead_s':touch-t,
        'fit_span_s':window[-1]-window[0],'fit_samples':len(window),'actual_first_collisions':names,
        'nose_up_deg':math.degrees(math.asin(max(-1.,min(1.,poses[t].rotate((1,0,0))[2])))),
        'angular_velocity_world_rad_s':omega,'body_velocity_world_m_s':motion.velocity_at_origin_world_m_s,
        'body_acceleration_world_m_s2':tuple(float(x) for x in 2*pc[0]),
        'body_position_world_m':motion.pose.position_world_m,
        'feet':{name:{'clearance_m':feet[name].clearance_m,'tangential_speed_upper_bound_m_s':feet[name].tangential_speed_upper_bound_m_s,
            'closing_speed_m_s':feet[name].closing_speed_m_s} for name in names if name in feet},
        'scope':'Raw-pose finite-window fit strictly before first raw collision; not instantaneous impact speed.'}
phases=[]
for row in feedback:
    key=row['decision']['phase']
    if not phases or phases[-1]['phase']!=key:phases.append({'sim_t_s':row['sim_t_s'],'phase':key,'mode':row['observation']['flight_mode']})
main_rows=[r for r in feedback if start is not None and r['sim_t_s']>=start]
tracking_diagnostic=None
if first is not None and start is not None:
    candidates=[r for r in main_rows if r['sim_t_s']<first[0] and 'fcu_local_position_ned' in r]
    if candidates:
        row=candidates[-1];fcu=row['fcu_local_position_ned'];offset=status['alignment_offset_ned_m']
        mapped=(fcu['y']-offset[1],fcu['x']-offset[0],-(fcu['z']-offset[2]))
        body=row['observation']['motion']['pose']['position_world_m']
        ref=curve.at(min(curve.T,max(0.,row['sim_t_s']-start)))
        tracking_diagnostic={'sample_sim_t_s':row['sim_t_s'],'gazebo_body_position_world_m':body,
            'fcu_position_mapped_with_initial_alignment_world_m':mapped,
            'fcu_minus_gazebo_position_m':tuple(a-b for a,b in zip(mapped,body)),
            'reference_body_position_world_m':ref['motion'].pose.position_world_m,
            'actual_minus_reference_body_position_m':tuple(a-b for a,b in zip(body,ref['motion'].pose.position_world_m)),
            'reference_nose_up_deg':math.degrees(ref['nose_up_rad']),
            'fcu_position_wall_age_s':row['fcu_position_wall_age_s'],
            'interpretation':'Receipt-time diagnostic, not a synchronized estimator-bias identification or causal proof.'}
spinning={name:max([abs(v) for t,n,v in rotors if n==name and start is not None and t>=start] or [0.]) for name in [f'rotor_{k}_joint' for k in range(4)]}
first_online=status.get('first_contact')
actual_duration=None if first is None or start is None else first[0]-start
angles=[math.degrees(math.asin(max(-1.,min(1.,poses[t].rotate((1,0,0))[2])))) for t in times if start is not None and start<=t<=(first[0] if first else start+curve.T)]
checks={
    'preserved_source_hashes_match':hashes_ok,
    'physical_takeoff_observed':max(p.position_world_m[2] for p in poses.values())>3.,
    'measured_main_entry_passed':bool(status.get('main_entry_report',{}).get('ready')) if status.get('main_entry_report') else False,
    'all_four_rotors_spun_during_approach':all(v>100 for v in spinning.values()),
    'raw_deck_collision_observed':first is not None,
    'online_first_event_matches_raw':first is not None and first_online is not None and abs(first_online['contact_sim_t_s']-first[0])<1e-8,
    'online_contact_envelope_passed':first_online is not None and all(first_online['checks'].values()),
    'normal_disarm_observed':bool(status.get('normal_disarm_observed')) and not status.get('armed_at_save',True),
    'executor_completed_support_and_post_disarm_checks':status.get('final_phase')=='COMPLETE' and any(item['phase']=='POST_DISARM' for item in phases),
    'no_saved_controller_failures':not status.get('failures') and not status.get('external_failures'),
    'land_mode_observed_after_contact':bool(first and any(r['sim_t_s']>=first[0] and r['observation']['flight_mode']=='LAND' for r in feedback)),
    'no_pva_after_online_handover':bool(first_online) and all(r['decision']['reference'] is None for r in feedback if r['sim_t_s']>=first_online['decision_sim_t_s'])}
report={'passed':all(checks.values()),'simulation_only':True,'new_physics_flight_performed':True,'run':str(run),'checks':checks,
    'actual_main_approach_to_first_contact_s':actual_duration,'planned_approach_duration_s':curve.T,
    'actual_lead_in_elapsed_s':None if start is None else start-status['preparation_reference_start_sim_t_s'],
    'actual_approach_nose_up_range_deg':None if not angles else [min(angles),max(angles)],
    'precontact':precontact,'phase_transitions':phases,'maximum_rotor_speeds_rad_s':spinning,
    'precontact_tracking_diagnostic':tracking_diagnostic,
    'controller_status':status,'external_forces_or_motor_capabilities_identified':False,
    'scope':'One Gazebo/DART and ArduCopter physics experiment with truth observations; not repeatability, optimality or hardware proof.',
    'source_sha256':{str(path.relative_to(run)).replace('\\','/'):hashlib.sha256(path.read_bytes()).hexdigest() for path in [run/'attempt_manifest.json',rs/'execution_status.json',rs/'execution_feedback.jsonl',rs/'model_states.jsonl']}}
(out/'flight_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
if start is not None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
    plt.rcParams['font.family']=font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name();plt.rcParams['axes.unicode_minus']=False
    selected=[(t,poses[t]) for t in times if start<=t<=(first[0]+1 if first else start+curve.T+1)]
    fig,axes=plt.subplots(3,1,figsize=(10,9),sharex=True,layout='constrained')
    ts=[t-start for t,_ in selected]
    axes[0].plot(ts,[math.degrees(math.asin(max(-1.,min(1.,p.rotate((1,0,0))[2])))) for _,p in selected],label='Gazebo 实测后倾角')
    rt=np.linspace(0,curve.T,1001)
    axes[0].plot(rt,[math.degrees(curve.at(float(t))['nose_up_rad']) for t in rt],ls='--',label='离线参考')
    axes[0].set_ylabel('后倾角 / °')
    axes[1].plot([r['sim_t_s']-start for r in main_rows],[r['observation']['motion']['velocity_at_origin_world_m_s'][0] for r in main_rows],label='实测拟合向前速度')
    axes[1].plot([r['sim_t_s']-start for r in main_rows],[-r['observation']['motion']['velocity_at_origin_world_m_s'][2] for r in main_rows],label='实测拟合下降速度')
    axes[1].set_ylabel('速度 / (m/s)')
    axes[2].plot([r['sim_t_s']-start for r in main_rows],[r['minimum_leg_gap_m'] for r in main_rows],label='实际姿态下最低脚底净空')
    axes[2].set(ylabel='脚底净空 / m',xlabel='从进场开始的仿真时间 / s')
    for ax in axes:
        ax.axvline(curve.T,c='#777',ls=':',label='原计划参考终点')
        if actual_duration is not None:ax.axvline(actual_duration,c='#b44',ls='--',label='原始首次物理碰撞')
        ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('新方案实际 Gazebo / ArduCopter 飞行\n验收'+('通过' if report['passed'] else '未通过；保留实测失败原因'))
    fig.savefig(out/'actual_flight_response.png',dpi=160);plt.close(fig)
print(json.dumps({'passed':report['passed'],'checks':checks,'actual_approach_duration_s':actual_duration,
    'precontact':None if precontact is None else {k:v for k,v in precontact.items() if k not in ['scope']},
    'failures':status.get('failures'),'external_failures':status.get('external_failures')},ensure_ascii=False,indent=2))
