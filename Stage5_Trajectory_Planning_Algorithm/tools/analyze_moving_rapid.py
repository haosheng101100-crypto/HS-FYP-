"""Raw physics audit for finite-speed contact, including rebound and load."""
from pathlib import Path
import argparse,json,sys,math,bisect,hashlib
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'work/plot_dependencies')]
from dataclasses import asdict
from landing.contact_geometry import Pose,RigidMotion,Deck,add,sub,scale,norm,evaluate_airframe
from landing.reference_loader import load_verified_reference
from landing.rapid_contact import load_rapid_reference,rapid_motion_metrics
from analyze_moving_contact import fit,stamp,pose
from landing.moving_rapid import plan_moving_rapid
from landing.moving_platform import check_moving_reference,translate_deck

def raw_data(run):
    streams={'uav_pose':[],'car_pose':[]};contacts={};first_legs=None
    for line in (run/'results/model_states.jsonl').open():
        row=json.loads(line);kind=row['kind'];msg=row['message']
        if kind in streams:
            name='uav' if kind=='uav_pose' else 'landing_vehicle'
            for p in msg.get('pose',[]):
                if p['name']==name:streams[kind].append((stamp(p),pose(p)))
        elif kind=='deck_contact':
            legs=[];forces=[]
            for c in msg.get('contact',[]):
                names=[c[k]['name'] for k in ('collision1','collision2')]
                if not any('landing_deck' in n for n in names):continue
                u=[i for i,n in enumerate(names) if n.startswith('uav::')]
                if not u:continue
                i=u[0];legs.append(names[i].rsplit('::',1)[-1])
                for w in c.get('wrench',[]):
                    f=w.get(f'body_{i+1}_wrench',{}).get('force',{})
                    forces.append(tuple(float(f.get(k,0.)) for k in 'xyz'))
            if legs:
                net=tuple(sum(f[k] for f in forces) for k in range(3));contacts[stamp(msg)]={'legs':sorted(set(legs)),'net_force_magnitude_N':norm(net)}
    for key in streams:streams[key].sort(key=lambda r:r[0])
    return streams,contacts

def descent_metrics(rows,geometry,deck_at,start,touch):
    sequence=[]
    for t,p in rows:
        if start<=t<touch:
            feet=evaluate_airframe(geometry,RigidMotion.stationary(p),deck_at(t))
            sequence.append((t,feet[0].clearance_m,p.position_world_m[2]))
    crossed=next((t for t,g,z in sequence if g<=.5),None)
    slow_time=0.
    for a,b in zip(sequence,sequence[1:]):
        if max(a[1],b[1])<=.5 and -(b[2]-a[2])/(b[0]-a[0])<.15:slow_time+=b[0]-a[0]
    return {'time_last_0_5m_s':None if crossed is None else touch-crossed,
        'near_deck_downward_speed_below_0_15m_s_duration_s':slow_time}

def analyze(run):
    out=run/'rapid_analysis';out.mkdir(exist_ok=False)
    status=json.loads((run/'results/execution_status.json').read_text());rp=status['rapid_plan']
    _,g,nominal,profile=load_verified_reference(ROOT)
    md=rp['deck_at_planning'];m=md['motion']
    initial_deck=Deck(RigidMotion(Pose(**m['pose']),m['velocity_at_origin_world_m_s'],m['angular_velocity_world_rad_s']),md['half_extents_m'])
    curve,ideal,plan=plan_moving_rapid(ROOT,g,profile,initial_deck,rp['lead_s'],durations=[rp['duration_s']],validate=False)
    plan['reference_checks']=check_moving_reference(curve,g,translate_deck(ideal,rp['lead_s']),rp['effective_reference_limits'],4001)
    plan['rapid_motion_checks']=rapid_motion_metrics(curve,g,translate_deck(ideal,rp['lead_s']),rp['experiment_configuration']['rapid_motion_checks'],4001)
    assert max(abs(a-b) for ax,c in zip(curve.axes,rp['normalized_polynomial_coefficients']) for a,b in zip(ax.coefficients,c))<1e-10
    raw,contacts=raw_data(run);touch=min(contacts);start=status['main_approach_start_sim_t_s']
    car=raw['car_pose'];cts=[t for t,p in car]
    def actual_deck(t):
        ct,p=car[bisect.bisect_right(cts,t)-1];assert 0<=t-ct<.06
        return Deck(RigidMotion.stationary(Pose(p.transform((0,0,.8)),p.orientation_xyzw)),nominal.half_extents_m)
    u,uf=fit(raw['uav_pose'],touch-.0125);c,cf=fit(car,u.sim_t_s,True)
    top=add(c.motion.pose.transform((0,0,.8)),scale(c.motion.velocity_at_origin_world_m_s,u.sim_t_s-c.sim_t_s))
    d=Deck(RigidMotion(Pose(top,c.motion.pose.orientation_xyzw),c.motion.velocity_at_origin_world_m_s,c.motion.angular_velocity_world_rad_s),nominal.half_extents_m)
    feet=evaluate_airframe(g,u.motion,d);active=[f for f in feet if f.name in contacts[touch]['legs']]
    nose=math.degrees(math.atan2(u.motion.pose.rotate((1,0,0))[2],u.motion.pose.rotate((1,0,0))[0]));tangent=max(f.tangential_speed_upper_bound_m_s for f in active)
    closing=max(f.closing_speed_upper_bound_m_s for f in active);closing_min=min(2*f.closing_speed_m_s-f.closing_speed_upper_bound_m_s for f in active)
    pre={'observation_sim_t_s':u.sim_t_s,'guard_to_contact_s':touch-u.sim_t_s,'nose_up_deg':nose,
        'tangential_speed_upper_bound_m_s':tangent,'closing_speed_upper_bound_m_s':closing,'closing_speed_lower_bound_m_s':closing_min,
        'aircraft':asdict(u),'actual_deck':asdict(d),'feet':[asdict(f) for f in active],'uav_fit':uf,'car_fit':cf}
    limits=status['contact_limits'];motion=descent_metrics(raw['uav_pose'],g,actual_deck,start,touch)
    post=[]
    for t,p in raw['uav_pose']:
        if t<touch:continue
        deck=actual_deck(t);f=evaluate_airframe(g,RigidMotion.stationary(p),deck)
        post.append({'sim_t_s':t,'lowest_gap_m':f[0].clearance_m,'minimum_footprint_margin_m':min(min(x.whole_cylinder_footprint_margins_m) for x in f),
            'tilt_deg':math.degrees(math.acos(max(-1.,min(1.,p.rotate((0,0,1))[2]))))})
    peak=max(contacts,key=lambda t:contacts[t]['net_force_magnitude_N']);contact_times=sorted(contacts)
    gross=rp['experiment_configuration']['postcontact_gross_motion_checks']
    checks={'live_execution_contract':status['completed'],'reference_dense_design_bounds':plan['reference_checks']['passed'],
        'tangential_speed':tangent<=limits['contact_tangential_speed_m_s'],
        'finite_closing_speed':limits['contact_minimum_closing_speed_m_s']<=closing_min and closing<=limits['contact_normal_speed_m_s'],
        'nose_up_target':abs(nose-2)<=math.degrees(limits['contact_attitude_error_rad']),
        'contact_time':abs(touch-start-curve.T)<=limits['contact_time_error_s'],
        'rapid_last_half_metre':motion['time_last_0_5m_s']<=1.5,'no_near_deck_slow_descent':motion['near_deck_downward_speed_below_0_15m_s_duration_s']<=.10,
        'postcontact_tilt':max(x['tilt_deg'] for x in post)<=gross['maximum_sampled_tilt_deg'],
        'rebound_clearance':max(x['lowest_gap_m'] for x in post)<=gross['maximum_lowest_foot_rebound_clearance_m'],
        'postcontact_footprint':min(x['minimum_footprint_margin_m'] for x in post)>=gross['minimum_footprint_margin_m'],
        'normal_disarm':status['normal_disarm_observed'] and not status['armed_at_save']}
    feedback=[json.loads(line) for line in (run/'results/execution_feedback.jsonl').read_text().splitlines()]
    execution=[row for row in feedback if row['stage']=='EXECUTION' and row.get('measured_deck')]
    vx=[row['measured_deck']['motion']['velocity_at_origin_world_m_s'][0] for row in execution]
    vehicle={'command_m_s':.3,'measured_x_speed_min_m_s':min(vx),'measured_x_speed_max_m_s':max(vx),
        'forward_displacement_m':execution[-1]['measured_deck']['motion']['pose']['position_world_m'][0]-execution[0]['measured_deck']['motion']['pose']['position_world_m'][0]}
    checks['vehicle_constant_velocity']=all(abs(v-.3)<.01 for v in vx)
    checks['planned_rapid_motion']=plan['rapid_motion_checks']['passed']
    baseline=ROOT/'outputs/model_audit/step12/run_001'
    bstatus=json.loads((baseline/'results/execution_status.json').read_text());br,bc=raw_data(baseline);bt=min(bc);bs=bstatus['main_approach_start_sim_t_s']
    bm=descent_metrics(br['uav_pose'],g,lambda t:nominal,bs,bt)
    baseline_duration=bt-bs
    report={'passed':all(checks.values()),'vehicle_motion':vehicle,'executed_reference_dense_check':plan['reference_checks'],'simulation_only':True,'hardware_deployment_allowed':False,'checks':checks,'controller_status':status,
        'planned_approach_s':curve.T,'contact_sim_t_s':touch,'actual_approach_s':touch-start,'precontact':pre,'actual_terminal_descent':motion,
        'postcontact':{'maximum_sampled_tilt_deg':max(x['tilt_deg'] for x in post),'maximum_lowest_foot_clearance_m':max(x['lowest_gap_m'] for x in post),
            'minimum_footprint_margin_m':min(x['minimum_footprint_margin_m'] for x in post),'sampled_pose_duration_s':post[-1]['sim_t_s']-touch},
        'contact_load':{'peak_solver_net_force_magnitude_N':contacts[peak]['net_force_magnitude_N'],'peak_after_contact_s':peak-touch,
            'maximum_contact_message_spacing_s':max(b-a for a,b in zip(contact_times,contact_times[1:])),
            'interpretation':'Recorded Gazebo/DART solver force magnitude, not structural stress or hardware safety certification.'},
        'comparison':{'baseline':'step12/run_001','baseline_approach_s':baseline_duration,'main_approach_reduction_percent':100*(1-(touch-start)/baseline_duration),
            'baseline_terminal_descent':bm,'same_nominal_initial_com_and_deck_target':False,'caveat':'Moving versus static scene; descriptive only, not a matched speed-effect comparison. Bounded measured entry residuals, not exact identical initial state. New rapid design envelope includes higher allowed speed and acceleration; not an isolated terminal-speed causal experiment. Startup hover, takeoff and post-contact motor-disarm time excluded.'},
        'source_sha256':{'model_states':hashlib.sha256((run/'results/model_states.jsonl').read_bytes()).hexdigest()},
        'limitations':['Truth observations, one moving-deck case, no wind/noise robustness validation.','100 Hz pose samples may miss faster rebound/tilt transients.','Rigid landing legs and unvalidated contact properties: cannot establish a safe hardware impact speed.']}
    (out/'verification.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(3,1,figsize=(10,9),sharex=True)
    for name,seq,st,tt,deck in [('Moving rapid actual',raw['uav_pose'],start,touch,actual_deck)]:
        a=[(t-st,p) for t,p in seq if st<=t<=tt];ts=[x[0] for x in a]
        gaps=[evaluate_airframe(g,RigidMotion.stationary(p),deck(t+st))[0].clearance_m for t,p in a]
        vz=[-(a[i][1].position_world_m[2]-a[i-1][1].position_world_m[2])/(ts[i]-ts[i-1]) for i in range(1,len(a))]
        pitch=[math.degrees(math.atan2(p.rotate((1,0,0))[2],p.rotate((1,0,0))[0])) for t,p in a]
        axs[0].plot(ts,gaps,label=name);axs[1].plot(ts[1:],vz,label=name);axs[2].plot(ts,pitch,label=name)
    axs[0].set_ylabel('Lowest foot gap (m)');axs[1].set_ylabel('Downward speed (m/s)');axs[2].set_ylabel('Nose-up (deg)');axs[2].set_xlabel('Main approach elapsed time (s)')
    for ax in axs:ax.grid(alpha=.3);ax.legend()
    fig.suptitle('Actual Gazebo flight: moving deck rapid contact');fig.tight_layout();fig.savefig(out/'rapid_comparison.png',dpi=160)
    print(json.dumps({k:v for k,v in report.items() if k not in ['controller_status','precontact']},ensure_ascii=False))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('run',type=Path);arg=a.parse_args();analyze(arg.run)
