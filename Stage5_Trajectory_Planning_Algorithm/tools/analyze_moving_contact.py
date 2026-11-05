"""Audit moving-deck physics logs, preserving original precision thresholds."""
from pathlib import Path
import argparse,json,sys,math,bisect
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'work/plot_dependencies')]
from dataclasses import asdict
import numpy as np
from landing.contact_geometry import Pose,RigidMotion,Deck,add,sub,scale,norm,evaluate_airframe
from landing.observations import CausalPoseHistory,PoseSample
from landing.moving_platform import plan_interception,translate_deck,check_moving_reference
from landing.reference_loader import load_verified_reference

def stamp(p):
    s=p['header']['stamp'];return float(s.get('sec',0))+float(s.get('nsec',0))*1e-9
def pose(p):
    return Pose(tuple(p['position'].get(k,0.) for k in 'xyz'),tuple(p['orientation'].get(k,0.) for k in 'xyzw'))
def fit(rows,end,car=False):
    h=CausalPoseHistory((0,0,0),span_s=.181 if car else .081,minimum_samples=4 if car else 5,
        minimum_span_s=.08 if car else .04,maximum_sample_gap_s=.05 if car else .025,maximum_sim_age_s=.08,maximum_wall_age_s=1.)
    times=[r[0] for r in rows];i=bisect.bisect_right(times,end)
    for t,p in rows[max(0,i-30):i]:h.push(PoseSample(t,p,0.))
    obs=h.estimate(end,0.,True,'GUIDED',0.)
    return obs,dict(h.last_fit)

def analyze(run):
    result=run/'moving_analysis';result.mkdir(exist_ok=True)
    assert not (result/'verification.json').exists(),'Preserve a completed audit'
    status=json.loads((run/'results/execution_status.json').read_text())
    rows=[json.loads(l) for l in (run/'results/execution_feedback.jsonl').read_text().splitlines()]
    streams={'uav_pose':[],'car_pose':[]};contact_t=None;contact_legs=[]
    for line in (run/'results/model_states.jsonl').open():
        row=json.loads(line);kind=row['kind'];msg=row['message']
        if kind in streams:
            name='uav' if kind=='uav_pose' else 'landing_vehicle'
            for p in msg.get('pose',[]):
                if p['name']==name:streams[kind].append((stamp(p),pose(p)))
        elif kind=='deck_contact' and contact_t is None:
            for c in msg.get('contact',[]):
                names=[c[k]['name'] for k in ('collision1','collision2')]
                if any('landing_deck' in n for n in names) and any(n.startswith('uav::') for n in names):
                    contact_t=stamp(msg);contact_legs=[n.rsplit('::',1)[-1] for n in names if n.startswith('uav::')];break
    for key in streams:streams[key].sort(key=lambda r:r[0])
    report={'simulation_only':True,'hardware_deployment_allowed':False,'controller_status':status,
        'raw_first_contact_sim_t_s':contact_t,'recorded_car_pose_samples':len(streams['car_pose']),
        'scope':'Actual Gazebo/DART rotor dynamics and deck collisions; truth-based observation, no simulated vision/GPS. One scenario is not a robustness study.'}
    _,g,_,profile=load_verified_reference(ROOT)
    if status.get('moving_plan'):
        p=status['moving_plan'];dp=p['deck_at_planning'];motion=dp['motion']
        deck=Deck(RigidMotion(Pose(**motion['pose']),motion['velocity_at_origin_world_m_s'],motion['angular_velocity_world_rad_s']),dp['half_extents_m'])
        # Reconstruct the exact executed polynomial, then independently check it densely.
        snap=p['duration_candidates'][0]['terminal_snap_z_m_s4']
        curve,ideal,rebuilt=plan_interception(ROOT,g,profile,deck,p['lead_s'],durations=[p['duration_s']],snaps=[snap],validate=False)
        coeff=p['normalized_polynomial_coefficients'];assert max(abs(a-b) for ax,cs in zip(curve.axes,coeff) for a,b in zip(ax.coefficients,cs))<1e-10
        limits=json.loads((ROOT/'config/experiments/snap_approach_v2.json').read_text())['design_limits']
        report['executed_reference_dense_check']=check_moving_reference(curve,g,translate_deck(ideal,p['lead_s']),limits,4001)
    if contact_t is not None:
        o,of=fit(streams['uav_pose'],contact_t-.0125)
        co,cf=fit(streams['car_pose'],o.sim_t_s,True)
        dt=o.sim_t_s-co.sim_t_s
        top=add(co.motion.pose.transform((0,0,.8)),scale(co.motion.velocity_at_origin_world_m_s,dt))
        deck=Deck(RigidMotion(Pose(top,co.motion.pose.orientation_xyzw),co.motion.velocity_at_origin_world_m_s,co.motion.angular_velocity_world_rad_s),(1,.7))
        feet=evaluate_airframe(g,o.motion,deck);active=[f for f in feet if f.name in contact_legs]
        nose_up=math.degrees(math.atan2(o.motion.pose.rotate((1,0,0))[2],o.motion.pose.rotate((1,0,0))[0]))
        report['precontact']={'observation_sim_t_s':o.sim_t_s,'guard_to_first_contact_s':contact_t-o.sim_t_s,
            'uav_fit':of,'car_fit':cf,'aircraft':asdict(o),'deck':asdict(deck),'nose_up_deg':nose_up,
            'raw_first_legs':contact_legs,'active_feet':[asdict(f) for f in active],
            'relative_tangential_speed_upper_bound_m_s':max(f.tangential_speed_upper_bound_m_s for f in active),
            'relative_closing_speed_upper_bound_m_s':max(f.closing_speed_upper_bound_m_s for f in active),
            'approach_to_contact_s':contact_t-status['main_approach_start_sim_t_s'] if status.get('main_approach_start_sim_t_s') else None,
            'note':'Causal finite-window estimates ending before contact, not instantaneous impulse velocity.'}
        cfg=json.loads((ROOT/'config/experiments/execution_v1.json').read_text())['handover_limits']
        target_deg=2.;speed=report['precontact']['relative_tangential_speed_upper_bound_m_s']
        closing_low=min(2*f.closing_speed_m_s-f.closing_speed_upper_bound_m_s for f in active)
        closing_high=max(f.closing_speed_upper_bound_m_s for f in active)
        report['independent_contact_checks']={
            'relative_tangential_speed':speed<=cfg['contact_tangential_speed_m_s'],
            'relative_closing_speed':cfg['contact_minimum_closing_speed_m_s']<=closing_low and closing_high<=cfg['contact_normal_speed_m_s'],
            'nose_up_target':abs(nose_up-target_deg)<=math.degrees(cfg['contact_attitude_error_rad']),
            'contact_time':abs(report['precontact']['approach_to_contact_s']-status['reference_approach_duration_s'])<=cfg['contact_time_error_s'],
            'footprint':all(min(f.whole_cylinder_footprint_margins_m)>=cfg['footprint_margin_m'] for f in feet),
            'active_foot_clearance':all(abs(f.clearance_m)<=cfg['contact_clearance_error_m'] for f in active)}
        after=[r for r in rows if r['sim_t_s']>=contact_t and r.get('measured_deck')]
        coords=[]
        for r in after:
            p=r['observation']['motion']['pose']['position_world_m'];d=r['measured_deck']['motion']['pose']['position_world_m']
            coords.append(sub(p,d))
        report['postcontact']={'samples':len(after),'maximum_body_displacement_relative_to_deck_m':max((math.hypot(*sub(p,coords[0])[:2]) for p in coords),default=None),
            'interpretation':'Body displacement includes leveling; not a direct material contact-point slip measurement.',
            'post_disarm_observed_span_s':(after[-1]['sim_t_s']-next((r['sim_t_s'] for r in after if not r['observation']['armed']),after[-1]['sim_t_s'])) if after else None,
            'all_final_support_checks':after[-1]['support_checks'] if after else {}}
    execution=[r for r in rows if r['stage']=='EXECUTION' and r.get('measured_deck')]
    if execution:
        vx=[r['measured_deck']['motion']['velocity_at_origin_world_m_s'][0] for r in execution]
        report['vehicle_motion']={'command_m_s':.3,'measured_x_speed_min_m_s':min(vx),'measured_x_speed_max_m_s':max(vx),
            'measured_forward_displacement_during_execution_m':execution[-1]['measured_deck']['motion']['pose']['position_world_m'][0]-execution[0]['measured_deck']['motion']['pose']['position_world_m'][0],
            'constant_velocity_gate_passed':all(abs(v-.3)<.01 for v in vx),
            'stop_command_only_after_experiment_observation':True}
    report['passed']=bool(status['completed'] and report.get('executed_reference_dense_check',{}).get('passed') and report.get('vehicle_motion',{}).get('constant_velocity_gate_passed') and report.get('independent_contact_checks') and all(report['independent_contact_checks'].values()))
    (result/'verification.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
    ts=[r['sim_t_s'] for r in execution]
    axes[0].plot(ts,[r['observation']['motion']['pose']['position_world_m'][0] for r in execution],label='UAV x')
    axes[0].plot(ts,[r['measured_deck']['motion']['pose']['position_world_m'][0] for r in execution],label='Deck x')
    axes[0].plot(ts,[r['observation']['motion']['pose']['position_world_m'][2] for r in execution],label='UAV z');axes[0].set_ylabel('Position (m)')
    axes[1].plot(ts,[r['observation']['motion']['velocity_at_origin_world_m_s'][0] for r in execution],label='UAV vx')
    axes[1].plot(ts,[r['measured_deck']['motion']['velocity_at_origin_world_m_s'][0] for r in execution],label='Deck vx');axes[1].set_ylabel('World velocity (m/s)')
    pitch=[]
    for r in execution:
        posev=Pose(**r['observation']['motion']['pose']);x=posev.rotate((1,0,0));pitch.append(math.degrees(math.atan2(x[2],x[0])))
    axes[2].plot(ts,pitch,label='Actual nose-up');axes[2].axhline(2,color='gray',ls=':',label='Contact target 2 deg');axes[2].set_ylabel('Nose-up (deg)');axes[2].set_xlabel('Physics simulation time (s)')
    for ax in axes:
        ax.grid(alpha=.3);ax.legend()
        if contact_t:ax.axvline(contact_t,color='red',ls='--')
    fig.suptitle('Moving deck: measured physics flight (truth observation)');fig.tight_layout();fig.savefig(result/'moving_contact.png',dpi=160);plt.close(fig)
    print(json.dumps({k:v for k,v in report.items() if k not in ['controller_status','precontact']},ensure_ascii=False))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);args=ap.parse_args();analyze(args.run)
