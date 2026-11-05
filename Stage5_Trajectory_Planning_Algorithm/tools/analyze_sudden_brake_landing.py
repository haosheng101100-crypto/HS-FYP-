"""Independent raw physics audit: sudden braking followed by actual deck landing."""
from pathlib import Path
import sys,json,math,bisect,argparse,hashlib,gzip,shutil
from types import SimpleNamespace
from dataclasses import asdict
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R),str(R/'tools'),str(R/'work/plot_dependencies')]
from analyze_moving_rapid import raw_data,descent_metrics
from analyze_moving_contact import fit
from landing.contact_geometry import Pose,RigidMotion,Deck,add,scale,sub,norm,evaluate_airframe
from landing.reference_loader import load_verified_reference
from landing.execution import attitude_error
from landing.septic_approach import PlanarSepticApproach,NinthScalar
from physical_validation_metrics import touchdown_target_errors


def analyze(run):
    res=run/'results';path=res/'model_states.jsonl'
    if not path.exists():
        with gzip.open(res/'model_states.jsonl.gz','rb') as src,path.open('xb') as dst:shutil.copyfileobj(src,dst)
    q=json.loads((res/'postflight_checks.json').read_text());assert hashlib.sha256(path.read_bytes()).hexdigest()==q['raw_sha256']
    out=run/'landing_analysis';out.mkdir(exist_ok=True);assert not (out/'verification.json').exists(),'Preserve completed audit'
    status=json.loads((res/'execution_status.json').read_text());raw,contacts=raw_data(run)
    _,g,_,profile=load_verified_reference(R)
    rows=[json.loads(l) for l in (res/'execution_feedback.jsonl').read_text().splitlines()]
    common_end=min(raw['uav_pose'][-1][0],raw['car_pose'][-1][0])
    rows=[r for r in rows if r['sim_t_s']<=common_end]
    checks={'online_landing_contract':status['completed'],'normal_disarm':status['normal_disarm_observed'] and not status['armed_at_save'],
        'no_failure_recovery':not status.get('recovery_platform_braking_used',False),'actual_deck_contact':bool(contacts),
        'no_planned_ground_landing':not status.get('planned_ground_land_used',True)}
    result={'simulation_only':True,'hardware_deployment_allowed':False,'passed':False,'checks':checks,
        'outcome_class':'Sudden braking, alignment recovery, then landing AFTER vehicle stop; not a before-stop landing'}
    if not contacts or 'landing_start' not in status:
        (out/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));return
    car=raw['car_pose'];cts=[t for t,p in car]
    def actual_car_pose(t):
        i=bisect.bisect_right(cts,t)-1;at,p=car[i]
        if abs(t-at)<1e-10:return p
        bt,q=car[i+1];assert 0<t-at<.06 and bt-at<.06
        f=(t-at)/(bt-at);pos=tuple(a+(b-a)*f for a,b in zip(p.position_world_m,q.position_world_m))
        a=p.orientation_xyzw;b=q.orientation_xyzw
        if sum(x*y for x,y in zip(a,b))<0:b=tuple(-x for x in b)
        quat=tuple(x+(y-x)*f for x,y in zip(a,b));length=math.sqrt(sum(x*x for x in quat))
        return Pose(pos,tuple(x/length for x in quat))
    def actual_deck(t):
        p=actual_car_pose(t);return Deck(RigidMotion.stationary(Pose(p.transform((0,0,.8)),p.orientation_xyzw)),(1,.7))
    def fitted_deck(t):
        c,cf=fit(car,t,True);dt=t-c.sim_t_s;a=cf['body_acceleration_world_m_s2'];p=actual_car_pose(t)
        return Deck(RigidMotion(Pose(p.transform((0,0,.8)),p.orientation_xyzw),add(c.motion.velocity_at_origin_world_m_s,scale(a,dt)),c.motion.angular_velocity_world_rad_s),(1,.7)),cf
    epoch=status['braking_start_sim_t_s'];touch=min(contacts);start=status['main_approach_start_sim_t_s'];T=status['landing_start']['duration_s']
    increments=[(bt,(q.position_world_m[0]-p.position_world_m[0])/(bt-at)) for (at,p),(bt,q) in zip(car,car[1:]) if bt>at and at>=epoch]
    stop=next(t for t,v in increments if t>epoch+1.8 and abs(v)<.01)
    acceleration=[]
    for i in range(3,18):
        _,f=fitted_deck(epoch+i*.1);acceleration.append(f['body_acceleration_world_m_s2'][0])
    trigger=status['scenario_trigger'];initial_car=trigger['deck']['motion']['velocity_at_origin_world_m_s'][0]
    checks.update(initial_car_5_m_s=abs(initial_car-5)<.02,initial_aircraft_5_m_s=abs(trigger['aircraft']['motion']['velocity_at_origin_world_m_s'][0]-5)<.08,
        initial_height=1.45<=min(f['clearance_m'] for f in trigger['feet'])<=1.52,
        actual_half_two_m_s2_braking=all(abs(a+2.5)<.08 for a in acceleration),
        normal_vehicle_stop=abs(stop-epoch-2.)<.2 and min(v for t,v in increments)>=-.01,
        clearly_post_stop_touchdown=touch>stop)
    boundary=status['landing_start']['terminal_boundary'];terminal_deck_p=status['landing_start']['deck']['motion']['pose']['position_world_m']
    u,uf=fit(raw['uav_pose'],touch-.0125);d,cf=fitted_deck(u.sim_t_s);feet=evaluate_airframe(g,u.motion,d)
    active=[f for f in feet if f.name in contacts[touch]['legs']]
    tangent=max(f.tangential_speed_upper_bound_m_s for f in active)
    upper=max(f.closing_speed_upper_bound_m_s for f in active);lower=min(2*f.closing_speed_m_s-f.closing_speed_upper_bound_m_s for f in active)
    target=Pose((0,0,0),tuple(boundary['aircraft']['pose']['orientation_xyzw']))
    angle=attitude_error(u.motion.pose,target);limits=status['contact_limits']
    errors=touchdown_target_errors(g,u.motion,d,boundary,terminal_deck_p)
    checks.update(relative_foot_speed=tangent<=limits['contact_tangential_speed_m_s'],
        finite_downward_contact=lower>=limits['contact_minimum_closing_speed_m_s'] and upper<=limits['contact_normal_speed_m_s'],
        target_attitude=angle<=limits['contact_attitude_error_rad'],angular_speed=norm(u.motion.angular_velocity_world_rad_s)<=limits['contact_angular_speed_rad_s'],
        contact_target_xy=errors['body_target_xy_error_m']<=limits['contact_xy_error_m'],
        contact_time=abs(touch-start-T)<=limits['contact_time_error_s'],
        known_legs=len(active)==len(contacts[touch]['legs']),
        precontact_footprint=all(min(f.whole_cylinder_footprint_margins_m)>=.02 for f in feet))
    result['precontact']={'guard_s':touch-u.sim_t_s,'relative_foot_tangent_upper_m_s':tangent,
        'closing_bounds_m_s':[lower,upper],'target_attitude_error_deg':math.degrees(angle),
        **errors,'aircraft':asdict(u),'deck':asdict(d),'uav_fit':uf,'car_fit':cf}
    # Restore exactly the saved landing polynomial; no replayed simulator controls.
    curve=PlanarSepticApproach.__new__(PlanarSepticApproach);curve.T=T;curve.gravity=9.80665;curve.mass=profile['mass_kg'];curve.com_offset_body_m=profile['nominal_com_in_body_m']
    axes=[]
    for coeff in status['landing_start']['reference_coefficients']:
        endpoint=lambda s,o:sum(coeff[k]*math.factorial(k)/math.factorial(k-o)*s**(k-o) for k in range(o,len(coeff)))/T**o
        axis=NinthScalar(tuple(endpoint(0.,o) for o in range(5)),tuple(endpoint(1.,o) for o in range(5)),T)
        axis.coefficients=tuple(coeff);axes.append(axis)
    curve.axes=tuple(axes);reconstruction=[]
    for row in rows:
        ref=row.get('body_reference');dt=row['sim_t_s']-start
        if row.get('decision',{}).get('phase')=='TRACK_REFERENCE' and ref and 0<=dt<=T:
            state=curve.at(dt)
            reconstruction.append(max(norm(sub(ref['position_world_m'],state['motion'].pose.position_world_m)),norm(sub(ref['velocity_world_m_s'],state['motion'].velocity_at_origin_world_m_s)),norm(sub(ref['acceleration_world_m_s2'],state['body_acceleration_world_m_s2']))))
    checks['reference_reconstruction']=bool(reconstruction) and max(reconstruction)<1e-7
    checks['no_airborne_reference_after_contact']=all(row['body_reference'] is None for row in rows if row['sim_t_s']>=touch)
    descent=descent_metrics(raw['uav_pose'],g,actual_deck,start,touch)
    checks['last_half_metre_within_1_5_s']=descent['time_last_0_5m_s']<=1.5
    post=[];initial_offset=None
    for t,p in raw['uav_pose']:
        if t<touch or t>common_end:continue
        deck=actual_deck(t);f=evaluate_airframe(g,RigidMotion.stationary(p),deck)
        relative=sub(p.position_world_m,deck.motion.pose.position_world_m)
        if initial_offset is None:initial_offset=relative
        post.append({'t':t,'gap':f[0].clearance_m,'margin':min(min(x.whole_cylinder_footprint_margins_m) for x in f),
            'tilt_deg':math.degrees(math.acos(max(-1.,min(1.,p.rotate((0,0,1))[2])))),
            'body_displacement_m':math.hypot(relative[0]-initial_offset[0],relative[1]-initial_offset[1])})
    checks.update(postcontact_tilt=max(p['tilt_deg'] for p in post)<=10.,no_rebound=max(p['gap'] for p in post)<=.1,
        postcontact_footprint=min(p['margin'] for p in post)>=.02)
    disarmed=[row for row in rows if row['sim_t_s']>touch and not row['observation']['armed']]
    if disarmed:
        begin=disarmed[0]['sim_t_s'];end=rows[-1]['sim_t_s'];speeds=[]
        for i in range(int((end-begin)/.1)+1):
            t=begin+i*.1;o,_=fit(raw['uav_pose'],t);dd,_=fitted_deck(o.sim_t_s)
            speeds.append(norm(sub(o.motion.velocity_at_origin_world_m_s,dd.motion.velocity_at_origin_world_m_s)))
        checks['three_second_support_after_disarm']=end-begin>=3.-.04 and max(speeds)<=.05
        result['post_disarm']={'duration_s':end-begin,'maximum_relative_speed_m_s':max(speeds)}
    else:checks['three_second_support_after_disarm']=False
    authority=status.get('supported_land_authority_transition',{})
    checks['support_guarded_native_land']=authority.get('parameter_acknowledged',False) and all(authority['support_checks'].values()) and authority['support_dwell_s']>=.4-1e-8
    reaction=[row for row in rows if row['stage']=='BRAKE_AND_HOLD']
    result['reaction']={'detection_delay_s':status['measured_braking_detection']['delay_since_driver_command_s'],
        'maximum_horizontal_offset_m':max(abs(row['observation']['motion']['pose']['position_world_m'][0]-row['measured_deck']['motion']['pose']['position_world_m'][0]) for row in reaction),
        'minimum_foot_gap_m':min(row['minimum_leg_gap_m'] for row in reaction),'maximum_actual_tilt_deg':max(row['tilt_deg'] for row in reaction),
        'minimum_aircraft_forward_speed_m_s':min(row['observation']['motion']['velocity_at_origin_world_m_s'][0] for row in reaction)}
    checks['reaction_envelope']=result['reaction']['maximum_horizontal_offset_m']<=3 and result['reaction']['maximum_actual_tilt_deg']<=20 and result['reaction']['minimum_foot_gap_m']>=.8
    result.update(actual_brake_to_contact_s=touch-epoch,actual_landing_descent_s=touch-start,vehicle_stop_after_brake_s=stop-epoch,
        landing_started_after_brake_s=start-epoch,raw_first_contact_s=touch,raw_vehicle_stop_s=stop,
        vehicle_braking_acceleration_range_m_s2=[min(acceleration),max(acceleration)],
        actual_terminal_descent=descent,reconstruction_max_error=max(reconstruction),
        postcontact={'maximum_tilt_deg':max(p['tilt_deg'] for p in post),'maximum_lowest_foot_gap_m':max(p['gap'] for p in post),
            'minimum_footprint_margin_m':min(p['margin'] for p in post),'maximum_relative_body_displacement_m':max(p['body_displacement_m'] for p in post)},
        peak_solver_net_force_N=max(p['net_force_magnitude_N'] for p in contacts.values()),structural_safety='unknown')
    result['source_sha256']={name:hashlib.sha256((res/name).read_bytes()).hexdigest() for name in ['model_states.jsonl','execution_feedback.jsonl','execution_status.json','attempt_manifest.json']}
    result['passed']=all(checks.values());(out/'verification.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='precontact'},ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);analyze(p.parse_args().run)
