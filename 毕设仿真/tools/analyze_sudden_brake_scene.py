"""Independent measured-state and physical wheel-braking scene audit."""
from pathlib import Path
import sys,json,math,bisect,hashlib,argparse
from dataclasses import asdict
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R),str(R/'tools'),str(R/'work/plot_dependencies')]
from analyze_moving_rapid import raw_data
from analyze_moving_contact import fit
from landing.reference_loader import load_verified_reference
from landing.contact_geometry import Deck,RigidMotion,Pose,add,scale,sub,norm,evaluate_airframe
from landing.sudden_brake_scene import SceneReferences


def analyze(run):
    out=run/'scene_analysis';out.mkdir(exist_ok=False)
    status=json.loads((run/'results/execution_status.json').read_text());raw,contacts=raw_data(run)
    _,g,_,profile=load_verified_reference(R)
    result={'scope':'Physical scene establishment only; no sudden-braking landing success claim',
        'simulation_only':True,'hardware_deployment_allowed':False,'passed':False,'checks':{}}
    checks=result['checks'];checks['scene_execution_completed']=status['completed']
    checks['normal_disarm']=status.get('normal_disarm_observed',False) and not status['armed_at_save']
    checks['no_deck_collision']=not contacts
    checks['no_failure_recovery']=not status.get('recovery_platform_braking_used',False)
    checks['no_landing_claim']=not status['landing_on_braking_platform_verified'] and not status['decision_algorithm_implemented']
    if 'scenario_trigger' not in status:
        (out/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));return
    epoch=status['scenario_trigger']['sim_t_s'];car=raw['car_pose'];cts=[t for t,p in car]
    def exact_car_pose(t):
        i=bisect.bisect_right(cts,t)-1;at,p=car[i]
        if abs(t-at)<1e-10:return p
        bt,q=car[i+1];assert 0<t-at<=.05 and 0<bt-at<=.05
        f=(t-at)/(bt-at);pos=tuple(x+(y-x)*f for x,y in zip(p.position_world_m,q.position_world_m))
        qa=p.orientation_xyzw;qb=q.orientation_xyzw
        if sum(x*y for x,y in zip(qa,qb))<0:qb=tuple(-x for x in qb)
        quat=tuple(x+(y-x)*f for x,y in zip(qa,qb));length=math.sqrt(sum(x*x for x in quat))
        return Pose(pos,tuple(x/length for x in quat))
    def state(t):
        u,uf=fit(raw['uav_pose'],t);c,cf=fit(car,u.sim_t_s,True)
        dt=u.sim_t_s-c.sim_t_s;a=cf['body_acceleration_world_m_s2']
        cp=exact_car_pose(u.sim_t_s);top=cp.transform((0,0,.8))
        d=Deck(RigidMotion(Pose(top,cp.orientation_xyzw),add(c.motion.velocity_at_origin_world_m_s,scale(a,dt)),c.motion.angular_velocity_world_rad_s),(1.,.7))
        feet=evaluate_airframe(g,u.motion,d)
        return {'source_sim_t_s':u.sim_t_s,'since_brake_command_s':u.sim_t_s-epoch,'uav':asdict(u),
            'deck':asdict(d),'feet':[asdict(f) for f in feet],'minimum_foot_gap_m':feet[0].clearance_m,
            'body_offset_from_deck_m':sub(u.motion.pose.position_world_m,d.motion.pose.position_world_m),
            'relative_body_velocity_m_s':sub(u.motion.velocity_at_origin_world_m_s,d.motion.velocity_at_origin_world_m_s),
            'uav_fit':uf,'car_fit':cf,'offline_positions_time_aligned':True}
    trigger=state(epoch);after=state(epoch+.28)
    result['trigger']=trigger;result['braking_frame']=after;result['braking_start_sim_t_s']=epoch
    p=trigger['uav']['motion'];cv=trigger['deck']['motion']['velocity_at_origin_world_m_s'][0]
    checks.update(initial_car_5_m_s=abs(cv-5)<.02,initial_aircraft_5_m_s=abs(p['velocity_at_origin_world_m_s'][0]-5)<.08,
        measured_descent=abs(p['velocity_at_origin_world_m_s'][2]+.4)<.08,
        trigger_height=1.45<=trigger['minimum_foot_gap_m']<=1.52,
        initial_relative_x=abs(trigger['body_offset_from_deck_m'][0])<.2,
        initial_relative_y=abs(trigger['body_offset_from_deck_m'][1])<.05)
    samples=[]
    for i in range(3,19):
        t=epoch+i*.1;c,cf=fit(car,t,True);dt=t-c.sim_t_s;a=cf['body_acceleration_world_m_s2'][0]
        samples.append({'t_s':t,'a_m_s2':a,'v_m_s':c.motion.velocity_at_origin_world_m_s[0]+a*dt})
    increments=[(bt,(q.position_world_m[0]-p.position_world_m[0])/(bt-at))
        for (at,p),(bt,q) in zip(car,car[1:]) if bt>at and at>=epoch]
    stopped=next((t for t,v in increments if t>epoch+1.8 and abs(v)<.01),None)
    x0=exact_car_pose(epoch).position_world_m[0]
    travel=None if stopped is None else exact_car_pose(stopped).position_world_m[0]-x0
    late=[abs(v) for t,v in increments if t>epoch+2.4]
    checks.update(actual_braking_2_5_m_s2=all(abs(p['a_m_s2']+2.5)<.08 for p in samples),
        stops_in_2_seconds=stopped is not None and abs(stopped-epoch-2.)<.2,
        braking_distance_5_m=travel is not None and abs(travel-5.)<.15,
        no_reverse=min(v for t,v in increments)>=-.01,
        remains_stopped=bool(late) and max(late)<.01,
        post_trigger_frame_braking=abs(after['car_fit']['body_acceleration_world_m_s2'][0]+2.5)<.08)
    rows=[json.loads(l) for l in (run/'results/execution_feedback.jsonl').read_text().splitlines()]
    observe=[r for r in rows if epoch<=r['sim_t_s']<epoch+.4-1e-8]
    original=SceneReferences();reference_errors=[]
    for row in observe:
        t=row['sim_t_s'];expected=original.reference(t-status['mission_start_sim_t_s'],t-status['descent_start_sim_t_s'])
        saved=row['body_reference']
        reference_errors.append(max(norm(sub(saved[key],getattr(expected,key))) for key in ['position_world_m','velocity_world_m_s','acceleration_world_m_s2']))
    checks['aircraft_reference_does_not_anticipate_brake']=bool(reference_errors) and max(reference_errors)<1e-9
    result['original_reference_reconstruction_max_pva_error']=max(reference_errors) if reference_errors else None
    checks['aircraft_still_airborne']=min(r['minimum_leg_gap_m'] for r in rows if r['sim_t_s']>=epoch)>.8
    result['vehicle_motion']={'braking_acceleration_range_m_s2':[min(p['a_m_s2'] for p in samples),max(p['a_m_s2'] for p in samples)],
        'raw_stop_elapsed_s':None if stopped is None else stopped-epoch,'travel_to_stop_m':travel,
        'minimum_raw_forward_speed_m_s':min(v for t,v in increments),'maximum_raw_speed_after_stop_m_s':max(late) if late else None,
        'sampled_motion':samples}
    result['flight_tracking']={'maximum_position_error_m':max(r['tracking_position_error_m'] for r in rows),
        'maximum_speed_error_m_s':max(r['tracking_speed_error_m_s'] for r in rows),
        'minimum_foot_gap_after_trigger_m':min(r['minimum_leg_gap_m'] for r in rows if r['sim_t_s']>=epoch)}
    result['source_sha256']={name:hashlib.sha256((run/'results'/name).read_bytes()).hexdigest() for name in ['model_states.jsonl','execution_feedback.jsonl','execution_status.json','attempt_manifest.json']}
    result['passed']=all(checks.values())
    (out/'verification.json').write_text(json.dumps(result,indent=2))
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    seq=[r for r in rows if epoch-1<=r['sim_t_s']<=epoch+3]
    ts=[r['sim_t_s']-epoch for r in seq];fig,axes=plt.subplots(3,1,figsize=(9,8),sharex=True)
    axes[0].plot(ts,[r['observation']['motion']['velocity_at_origin_world_m_s'][0] for r in seq],label='UAV')
    axes[0].plot(ts,[r['measured_deck']['motion']['velocity_at_origin_world_m_s'][0] for r in seq],label='Vehicle');axes[0].legend();axes[0].set_ylabel('Speed (m/s)')
    axes[1].plot(ts,[r['shared_acceleration_m_s2'][0] for r in seq]);axes[1].set_ylabel('Vehicle fitted a (m/s²)')
    axes[2].plot(ts,[r['minimum_leg_gap_m'] for r in seq]);axes[2].set_ylabel('Lowest foot gap (m)');axes[2].set_xlabel('Time since brake command (s)')
    for ax in axes:ax.axvline(0,color='r',linestyle='--');ax.axvline(.4,color='grey',linestyle=':');ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(out/'scene_physics.png',dpi=150)
    print(json.dumps({k:v for k,v in result.items() if k not in ['trigger','braking_frame']},ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);analyze(p.parse_args().run)
