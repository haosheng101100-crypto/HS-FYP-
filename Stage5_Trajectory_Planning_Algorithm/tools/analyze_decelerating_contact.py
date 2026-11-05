"""Independent raw-state audit of the decelerating-platform experiment."""
from pathlib import Path
import sys,json,math,bisect,hashlib,argparse
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'tools'),str(ROOT/'work/plot_dependencies')]
from dataclasses import asdict
from landing.reference_loader import load_verified_reference
from landing.contact_geometry import Pose,RigidMotion,Deck,evaluate_airframe,add,scale,sub,norm
from landing.accelerating_platform import check_accelerating_reference
from landing.decelerating_platform import restore_decelerating_plan as restore_accelerating_plan
from landing.septic_approach import NinthScalar
from analyze_moving_rapid import raw_data,descent_metrics
from analyze_moving_contact import fit
from physical_validation_metrics import touchdown_target_errors,raw_forward_progress


def analyze(run):
    out=run/'deceleration_analysis_v2';out.mkdir(exist_ok=True)
    assert not (out/'verification.json').exists(),'Preserve completed independent audits'
    status=json.loads((run/'results/execution_status.json').read_text())
    raw,contacts=raw_data(run);_,g,_,profile=load_verified_reference(ROOT)
    rows=[json.loads(l) for l in (run/'results/execution_feedback.jsonl').read_text().splitlines()]
    result={'simulation_only':True,'hardware_deployment_allowed':False,'controller_status':status,
            'raw_first_contact_s':min(contacts) if contacts else None,'checks':{},'passed':False}
    checks=result['checks'];checks['live_execution_contract']=status['completed']
    checks['normal_disarm']=status.get('normal_disarm_observed',False) and not status['armed_at_save']
    if not contacts or status.get('main_approach_start_sim_t_s') is None:
        (out/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='controller_status'}));return
    touch=min(contacts);start=status['main_approach_start_sim_t_s'];rp=status['rapid_plan']
    c=restore_accelerating_plan(g,profile,rp)
    for update in status['rolling_updates']:
        T=update['remaining_s']
        for axis,coeff in zip(c.axes,update['segments']):
            end=tuple(sum(coeff[k]*math.factorial(k)/math.factorial(k-o) for k in range(o,len(coeff)))/T**o for o in range(5))
            a=NinthScalar((0.,)*5,end,T);a.coefficients=tuple(coeff)
            axis.pieces.append((update['t_s'],a))
    rawcar=raw['car_pose'];cts=[t for t,p in rawcar]
    def actual_pose_deck(t):
        index=bisect.bisect_right(cts,t)-1
        ct,p=rawcar[index];assert 0<=t-ct<.06
        # Offline evaluation only: align both archived bodies to the SAME time.
        # A last-sample position hold at 10 m/s creates centimetre-scale false
        # displacement. Causal estimator inputs above/below remain unchanged.
        if t>ct and index+1<len(rawcar):
            nt,np=rawcar[index+1];assert nt-ct<=.05
            fraction=(t-ct)/(nt-ct)
            position=tuple(a+(b-a)*fraction for a,b in zip(p.position_world_m,np.position_world_m))
            q0=p.orientation_xyzw;q1=np.orientation_xyzw
            if sum(a*b for a,b in zip(q0,q1))<0:q1=tuple(-x for x in q1)
            q=tuple(a+(b-a)*fraction for a,b in zip(q0,q1));length=math.sqrt(sum(x*x for x in q))
            p=Pose(position,tuple(x/length for x in q))
        return Deck(RigidMotion.stationary(Pose(p.transform((0,0,.8)),p.orientation_xyzw)),(1,.7))
    def fit_deck(t):
        co,cf=fit(rawcar,t,True);dt=t-co.sim_t_s
        acc=cf['body_acceleration_world_m_s2']
        p=add(co.motion.pose.transform((0,0,.8)),add(scale(co.motion.velocity_at_origin_world_m_s,dt),scale(acc,.5*dt*dt)))
        return Deck(RigidMotion(Pose(p,co.motion.pose.orientation_xyzw),add(co.motion.velocity_at_origin_world_m_s,scale(acc,dt)),co.motion.angular_velocity_world_rad_s),(1.,.7)),cf
    main,_=fit_deck(start)
    dense=check_accelerating_reference(c,g,main,(-.5,0,0),rp['configuration']['design_limits'],4001)
    result['reconstructed_reference_checks']=dense;checks['reconstructed_reference_dense']=dense['passed']
    # Compare reconstructed commands to each saved decision (before feedback).
    errors=[]
    for row in rows:
        ref=row['decision'].get('reference');elapsed=row['sim_t_s']-start
        if row['decision']['phase']=='TRACK_REFERENCE' and ref and 0<=elapsed<=c.T:
            state=c.at(elapsed)
            errors.append(max(norm(sub(ref['position_world_m'],state['motion'].pose.position_world_m)),norm(sub(ref['velocity_world_m_s'],state['motion'].velocity_at_origin_world_m_s)),norm(sub(ref['acceleration_world_m_s2'],state['body_acceleration_world_m_s2']))))
    result['reconstruction_maximum_pva_error']=max(errors) if errors else None
    checks['executed_command_reconstruction']=bool(errors) and max(errors)<1e-6
    u,uf=fit(raw['uav_pose'],touch-.0125);deck,cf=fit_deck(u.sim_t_s)
    feet=evaluate_airframe(g,u.motion,deck);active=[f for f in feet if f.name in contacts[touch]['legs']]
    tangent=max(f.tangential_speed_upper_bound_m_s for f in active)
    closing=max(f.closing_speed_upper_bound_m_s for f in active);lower=min(2*f.closing_speed_m_s-f.closing_speed_upper_bound_m_s for f in active)
    nose=math.degrees(math.atan2(u.motion.pose.rotate((1,0,0))[2],u.motion.pose.rotate((1,0,0))[0]))
    # Target is acceleration-coherent forward tilt, unlike the old 2 deg nose-up.
    target=-math.degrees(math.atan2(status['rolling_updates'][-1]['acceleration_m_s2'][0] if status['rolling_updates'] else -.5,9.80665))
    result['precontact']={'causal_guard_s':touch-u.sim_t_s,'nose_up_deg':nose,'target_nose_up_deg':target,
        'tangential_speed_upper_bound_m_s':tangent,'closing_speed_upper_bound_m_s':closing,'closing_speed_lower_bound_m_s':lower,
        'aircraft':asdict(u),'deck':asdict(deck),'uav_fit':uf,'car_fit':cf}
    limits=status['contact_limits']
    checks.update(relative_foot_speed=tangent<=limits['contact_tangential_speed_m_s'],
        closing_speed=limits['contact_minimum_closing_speed_m_s']<=lower and closing<=limits['contact_normal_speed_m_s'],
        acceleration_coherent_attitude=abs(nose-target)<=math.degrees(limits['contact_attitude_error_rad']),
        contact_time=abs(touch-start-c.T)<=limits['contact_time_error_s'])
    centroid=tuple(sum(f.point_world_m[k] for f in active)/len(active) for k in range(3))
    offset=deck.motion.pose.inverse_rotate(sub(centroid,deck.motion.pose.position_world_m))
    result['precontact']['first_reported_contact_centroid_distance_from_deck_center_m']=math.hypot(*offset[:2])
    boundary=status['rolling_updates'][-1]['terminal_boundary'] if status['rolling_updates'] else rp['terminal_boundary']
    terminal_deck_position=sub(boundary['contact_centroid_world_m'],boundary['contact_centroid_deck_m'])
    target_errors=touchdown_target_errors(g,u.motion,deck,boundary,terminal_deck_position)
    result['precontact'].update(target_errors)
    checks['contact_target_xy']=max(target_errors['body_target_xy_error_m'],target_errors['planned_support_centroid_xy_error_m'])<=limits['contact_xy_error_m']
    checks['first_contact_in_planned_support']=all(f.name in boundary['active_leg_names'] for f in active)
    checks['known_leg_contacts']=len(active)==len(contacts[touch]['legs'])
    checks['precontact_footprint']=all(min(f.whole_cylinder_footprint_margins_m)>=limits['footprint_margin_m'] for f in feet)
    result['actual_approach_s']=touch-start
    motion=descent_metrics(raw['uav_pose'],g,actual_pose_deck,start,touch);result['actual_terminal_descent']=motion
    checks['rapid_last_half_metre']=motion['time_last_0_5m_s']<=1.5
    checks['no_slow_near_deck_descent']=motion['near_deck_downward_speed_below_0_15m_s_duration_s']<=.1
    post=[];first_rel=None
    for t,p in raw['uav_pose']:
        if t<touch:continue
        d=actual_pose_deck(t);f=evaluate_airframe(g,RigidMotion.stationary(p),d)
        relative=d.motion.pose.inverse_rotate(sub(p.position_world_m,d.motion.pose.position_world_m))
        if first_rel is None:first_rel=relative
        post.append({'t':t,'gap':f[0].clearance_m,'margin':min(min(q.whole_cylinder_footprint_margins_m) for q in f),
            'tilt':math.degrees(math.acos(max(-1.,min(1.,p.rotate((0,0,1))[2])))),
            'relative_body_displacement_m':math.hypot(relative[0]-first_rel[0],relative[1]-first_rel[1])})
    result['postcontact']={'maximum_tilt_deg':max(r['tilt'] for r in post),'maximum_lowest_gap_m':max(r['gap'] for r in post),
        'minimum_footprint_margin_m':min(r['margin'] for r in post),'maximum_relative_body_displacement_m':max(r['relative_body_displacement_m'] for r in post),
        'time_alignment':'Offline bracketing interpolation of raw car poses to each UAV pose time; not used by online estimator/planner.'}
    checks.update(postcontact_tilt=max(r['tilt'] for r in post)<=10,rebound=max(r['gap'] for r in post)<=.1,footprint=min(r['margin'] for r in post)>=.02)
    disarmed=[r for r in rows if not r['observation']['armed'] and r['sim_t_s']>touch]
    if disarmed:
        since=disarmed[0]['sim_t_s'];tail=rows[-1]['sim_t_s'];relative_speeds=[]
        for i in range(int((tail-since)/.1)+1):
            t=since+.1*i;uo,_=fit(raw['uav_pose'],t);dd,_=fit_deck(uo.sim_t_s)
            relative_speeds.append(norm(sub(uo.motion.velocity_at_origin_world_m_s,dd.motion.velocity_at_origin_world_m_s)))
        result['independent_post_disarm']={'observed_duration_s':tail-since,'maximum_sampled_relative_body_speed_m_s':max(relative_speeds),'sample_interval_s':.1}
        checks['three_seconds_after_disarm']=tail-since>=3.-.04
        checks['post_disarm_relative_speed']=max(relative_speeds)<=limits['support_speed_m_s']
    else:checks['three_seconds_after_disarm']=False
    authority=status.get('supported_land_authority_transition',{})
    checks['support_guarded_land_authority']=bool(authority) and authority.get('parameter_acknowledged',False) and all(authority['support_checks'].values()) and authority['support_dwell_s']>=.4-1e-8
    checks['no_failure_recovery_used']=not status.get('recovery_platform_braking_used',False)
    epoch=status['braking_start']['sim_t_s'];end=rows[-1]['sim_t_s']
    accel=[];speed=[];stopped=[];sampled=[]
    for i in range(int((end-epoch-.3)/.1)):
        t=epoch+.3+i*.1;d,f=fit_deck(t)
        a=f['body_acceleration_world_m_s2'][0];v=d.motion.velocity_at_origin_world_m_s[0]
        sampled.append((t,v,a,d.motion.pose.position_world_m[0]))
        if t<epoch+5.7:accel.append(a);speed.append(v)
        if t>=epoch+6.4:stopped.append(abs(v))
    v0=status['braking_start']['measured_initial_speed_m_s']
    x0=status['braking_start']['deck']['motion']['pose']['position_world_m'][0]
    zero=next((r for r in sampled if r[0]>epoch+5.7 and abs(r[1])<.01),None)
    contact_deck,contact_fit=fit_deck(touch-.0125)
    contact_v=contact_deck.motion.velocity_at_origin_world_m_s[0]
    contact_a=contact_fit['body_acceleration_world_m_s2'][0]
    raw_progress=raw_forward_progress(rawcar,epoch,x0)
    result['vehicle_motion']={'measured_initial_speed_m_s':v0,'acceleration_range_m_s2':[min(accel),max(accel)],
        'touchdown_speed_m_s':contact_v,'touchdown_acceleration_m_s2':contact_a,
        **raw_progress,
        'maximum_speed_after_stop_m_s':max(stopped) if stopped else None,
        'minimum_quadratic_fit_speed_m_s':min(r[1] for r in sampled),'samples':len(sampled)}
    checks['initial_3_m_s']=abs(v0-3)<.02
    checks['physical_half_m_s2_braking']=bool(accel) and all(abs(a+.5)<.04 for a in accel)
    checks['contact_while_still_braking']=contact_v>.05 and abs(contact_a+.5)<.04 and touch<epoch+6.
    checks['six_second_stop']=raw_progress['stop_elapsed_s'] is not None and abs(raw_progress['stop_elapsed_s']-6.)<=.25
    checks['nine_metre_braking_distance']=raw_progress['travel_to_stop_m'] is not None and abs(raw_progress['travel_to_stop_m']-9.)<=.15
    checks['no_reverse']=raw_progress['minimum_raw_secant_speed_m_s']>=-.01
    checks['remains_stopped']=bool(stopped) and max(stopped)<=.01
    peak=max(contacts,key=lambda t:contacts[t]['net_force_magnitude_N'])
    result['contact_load']={'peak_solver_net_force_N':contacts[peak]['net_force_magnitude_N'],'peak_after_contact_s':peak-touch,'structural_safety':'unknown'}
    result['rolling_replans']=len(status['rolling_updates']);result['passed']=all(checks.values())
    result['source_sha256']={name:hashlib.sha256((run/'results'/name).read_bytes()).hexdigest() for name in ['model_states.jsonl','execution_feedback.jsonl','shared_platform_states.json','execution_status.json']}
    result['limitations']=['Single nominal case; shared state derived from Gazebo truth, JSON interface is not a real radio or visual acceleration estimator.',
        'Sampled reference checks and rigid-contact solver loads do not prove continuous feasibility or structural safety.',
        'Relative body displacement includes contact settling and is not identical to material-point slip.']
    result['audit_revision']={'previous_audit':'deceleration_analysis_v1/verification.json (retained)',
        'target_geometry':'Use the original body target and planned rear-pair centroid. A first single rear-left contact is planned at y=+0.21 m; its absolute distance from deck center is not the target error.',
        'stopping_derivative':'Check reversal and stop using raw pose increments, independently of quadratic derivative overshoot across a braking-to-zero transition. Minimum fitted velocity retained as diagnostic.',
        'thresholds_changed':False,'flight_or_controller_changed':False}
    (out/'verification.json').write_text(json.dumps(result,indent=2))
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,1,figsize=(10,10),sharex=True)
    seq=[r for r in rows if r['sim_t_s']>=start]
    ts=[r['sim_t_s']-start for r in seq]
    axes[0].plot(ts,[r['minimum_leg_gap_m'] for r in seq]);axes[0].set_ylabel('Lowest foot gap (m)')
    axes[1].plot(ts,[r['observation']['motion']['velocity_at_origin_world_m_s'][0] for r in seq],label='UAV')
    axes[1].plot(ts,[r['measured_deck']['motion']['velocity_at_origin_world_m_s'][0] for r in seq],label='Deck');axes[1].legend();axes[1].set_ylabel('World speed (m/s)')
    axes[2].plot(ts,[r['shared_acceleration_m_s2'][0] for r in seq]);axes[2].set_ylabel('Shared deck a (m/s2)')
    axes[3].plot(ts,[r['feet'][0]['tangential_speed_upper_bound_m_s'] for r in seq]);axes[3].set_ylabel('Foot relative speed (m/s)');axes[3].set_xlabel('Main approach elapsed (s)')
    for ax in axes:ax.axvline(touch-start,color='r',linestyle='--');ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(out/'decelerating_contact.png',dpi=150)
    print(json.dumps({k:v for k,v in result.items() if k not in ['controller_status','precontact']},ensure_ascii=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);analyze(p.parse_args().run)
