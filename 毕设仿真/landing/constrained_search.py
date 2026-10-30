"""Deterministic reference search. No learned response or flight labels used."""
from copy import deepcopy
import math
from .rapid_contact import load_rapid_reference,rapid_motion_metrics
from .moving_platform import check_moving_reference
from .contact_geometry import evaluate_airframe


def candidate_configuration(base,speed,tolerance=.08):
    if not math.isfinite(speed) or not .1<=speed<=.45:
        raise ValueError('Outside this declared simulation search interval [.1,.45]')
    if not math.isfinite(tolerance) or not 0<tolerance<speed:
        raise ValueError('Positive tolerance smaller than speed required')
    cfg=deepcopy(base)
    cfg['target_closing_speed_m_s']=speed
    cfg['contact_closing_speed_interval_m_s']=[round(speed-tolerance,9),round(speed+tolerance,9)]
    cfg['objective']='Deterministic static-deck reference comparison; no learning or hardware permission'
    return cfg


def braking_event(curve,geometry,deck):
    """COM forward-speed maximum, refined at a positive-to-negative ax zero."""
    ts=[curve.T*i/1000 for i in range(1001)]
    states=[curve.at(t) for t in ts]
    k=max(range(len(states)),key=lambda i:states[i]['com_velocity_world_m_s'][0])
    if not 0<k<len(ts)-1:raise ValueError('No interior forward braking transition')
    lo,hi=ts[k-1],ts[k+1]
    if not curve.at(lo)['com_acceleration_world_m_s2'][0]>0>curve.at(hi)['com_acceleration_world_m_s2'][0]:
        raise ValueError('Forward speed maximum not bracketed')
    for _ in range(40):
        mid=(lo+hi)/2
        if curve.at(mid)['com_acceleration_world_m_s2'][0]>0:lo=mid
        else:hi=mid
    t=(lo+hi)/2;s=curve.at(t)
    return {'time_from_approach_start_s':t,'time_remaining_s':curve.T-t,
        'com_world_position_m':s['com_position_world_m'],
        'remaining_forward_com_distance_to_terminal_m':curve.at(curve.T)['com_position_world_m'][0]-s['com_position_world_m'][0],
        'lowest_foot_gap_m':evaluate_airframe(geometry,s['motion'],deck)[0].clearance_m,
        'forward_com_speed_m_s':s['com_velocity_world_m_s'][0],
        'definition':'First global forward COM speed maximum in this reference, not a separate braking switch sent to flight control.'}


def search_references(root,base,speeds,durations,dense_samples=4001):
    if dense_samples<1001:raise ValueError('At least 1001 samples required for final reference screening')
    speeds=sorted(set(speeds));durations=sorted(set(durations))
    if not speeds or not durations or any(not math.isfinite(t) or t<=0 for t in durations):
        raise ValueError('Nonempty finite positive search grid required')
    candidates=[]
    for speed in speeds:
        cfg=candidate_configuration(base,speed)
        for duration in durations:
            curve,g,d,p,plan=load_rapid_reference(root,duration,0.,False,configuration=cfg)
            check=check_moving_reference(curve,g,d,plan['effective_reference_limits'],201)
            motion=rapid_motion_metrics(curve,g,d,cfg['rapid_motion_checks'],501)
            resolution=201
            # Dense evaluation is also performed on rapid-motion rejects so
            # threshold-adjacent cases retain an interpretable dense result.
            if check['passed']:
                check=check_moving_reference(curve,g,d,plan['effective_reference_limits'],dense_samples)
                motion=rapid_motion_metrics(curve,g,d,cfg['rapid_motion_checks'],dense_samples)
                resolution=dense_samples
            failures=list(check.get('failures',[]))
            if not motion['passed']:failures.append('rapid_motion')
            candidates.append({'duration_s':duration,'target_closing_speed_m_s':speed,
                'reference_sampled_feasible':check['passed'] and motion['passed'],
                'design_checks':check,'motion_checks':motion,'failures':failures,
                'design_check_samples':resolution,'braking':braking_event(curve,g,d),
                'terminal_vertical_kinetic_component_J':.5*p['mass_kg']*speed**2,
                'physics_validated':False,'structural_safety_known':False})
    feasible=[c for c in candidates if c['reference_sampled_feasible']]
    pareto=[c for c in feasible if not any(
        other['duration_s']<=c['duration_s'] and other['target_closing_speed_m_s']<=c['target_closing_speed_m_s']
        and (other['duration_s']<c['duration_s'] or other['target_closing_speed_m_s']<c['target_closing_speed_m_s'])
        for other in feasible)]
    return {'algorithm':'Explicit finite grid, dynamics-derived attitude and foot geometry, constraint filtering, Pareto selection',
        'simulation_only':True,'trained_model_used':False,'actuation_enabled':False,
        'candidates':candidates,'pareto':pareto,
        'scope':'Sampled feasibility in a fixed degree-nine planar reference family; no continuous feasibility, global optimum, tracking or force guarantee.',
        'energy_interpretation':'Vertical translation component at target speed only. Excludes rotation, compliance and controller thrust; not predicted force or a hardware safety bound.'}


def select_reference(report,maximum_target_speed):
    if not math.isfinite(maximum_target_speed) or maximum_target_speed<=0:
        raise ValueError('Positive finite target speed cap required')
    allowed=[c for c in report['candidates'] if c['reference_sampled_feasible'] and c['target_closing_speed_m_s']<=maximum_target_speed]
    return min(allowed,key=lambda c:(c['duration_s'],c['target_closing_speed_m_s'])) if allowed else None
