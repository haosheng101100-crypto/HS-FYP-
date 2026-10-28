"""Separate faster-contact experiment. Old soft-contact contracts stay intact."""
from dataclasses import asdict
from copy import deepcopy
import json,math
from .reference_loader import load_verified_reference
from .contact_geometry import evaluate_airframe,norm
from .touchdown_boundary import ContactDesignLimits,TouchdownRequest,solve_touchdown_boundary
from .septic_approach import PlanarSnapMatchedApproach
from .moving_platform import check_moving_reference


def rapid_motion_metrics(curve,geometry,deck,config,samples=1001):
    crossing=None;slow_time=0.;last=None;minimum_down=math.inf
    for i in range(samples):
        t=curve.T*i/(samples-1);s=curve.at(t);feet=evaluate_airframe(geometry,s['motion'],deck)
        gap=feet[0].clearance_m;down=-s['motion'].velocity_at_origin_world_m_s[2]
        if gap<=config['near_deck_height_m']:
            if crossing is None:crossing=t
            minimum_down=min(minimum_down,down)
            if down<config['low_vertical_speed_threshold_m_s'] and last is not None:slow_time+=t-last
        last=t
    return {'time_last_0_5m_s':None if crossing is None else curve.T-crossing,
        'near_deck_low_vertical_speed_duration_s':slow_time,'minimum_downward_body_speed_last_0_5m_m_s':minimum_down,
        'passed':crossing is not None and curve.T-crossing<=config['maximum_time_last_0_5m_s'] and slow_time<=config['maximum_near_deck_low_vertical_speed_duration_s']}


def load_rapid_reference(root,duration=None,snap_z=None,validate=True,configuration=None):
    _,geometry,deck,profile=load_verified_reference(root)
    cfg=deepcopy(configuration) if configuration is not None else json.loads((root/'config/experiments/rapid_contact_v1.json').read_text())
    if not cfg['simulation_only'] or cfg['hardware_deployment_allowed']:raise ValueError('SITL-only configuration required')
    speed=cfg['target_closing_speed_m_s'];lo,hi=cfg['contact_closing_speed_interval_m_s']
    if not all(math.isfinite(x) for x in (speed,lo,hi)) or not 0<lo<=speed<=hi:raise ValueError('Invalid closing speed contract')
    if duration is not None and (not math.isfinite(duration) or duration<=0):raise ValueError('Positive finite duration required')
    base=json.loads((root/'config/experiments/snap_approach_v2.json').read_text())
    limits=dict(base['design_limits']);limits.update(cfg['new_design_limits'])
    boundary=solve_touchdown_boundary(geometry,deck,TouchdownRequest.planar(math.radians(cfg['target_nose_up_deg']),0.,(0.,0.),(0.,0.,-speed)),
        ContactDesignLimits(math.radians(5),math.radians(5),.05,lo,hi,.02))
    offset=profile['nominal_com_in_body_m'];x,z=cfg['initial_com_position_x_z_m']
    start,end,step=cfg['duration_search_s'];durations=[duration] if duration is not None else [round(start+i*step,8) for i in range(round((end-start)/step)+1)]
    snaps=[snap_z] if snap_z is not None else cfg['terminal_snap_z_grid_m_s4'];tried=[]
    for T in durations:
        for snap in snaps:
            curve=PlanarSnapMatchedApproach((x,offset[1],z),cfg['initial_com_velocity_world_m_s'],boundary,offset,base['reference_gravity_m_s2'],profile['mass_kg'],T,terminal_com_snap_world_m_s4=(0.,0.,snap))
            check=check_moving_reference(curve,geometry,deck,limits,201) if validate else {'passed':False,'pending_dense_check':True}
            tried.append({'duration_s':T,'snap_z_m_s4':snap,'failures':check.get('failures',[])})
            if validate and not check['passed']:continue
            motion=rapid_motion_metrics(curve,geometry,deck,cfg['rapid_motion_checks'],501) if validate else None
            if validate and not motion['passed']:continue
            if validate:
                check=check_moving_reference(curve,geometry,deck,limits,4001)
                if not check['passed']:continue
                motion=rapid_motion_metrics(curve,geometry,deck,cfg['rapid_motion_checks'],4001)
                if not motion['passed']:continue
            report={'simulation_only':True,'hardware_deployment_allowed':False,'duration_s':T,'terminal_snap_z_m_s4':snap,
                'experiment_configuration':cfg,'effective_reference_limits':limits,'reference_checks':check,'rapid_motion_checks':motion,
                'terminal_boundary':asdict(boundary),'normalized_polynomial_coefficients':[a.coefficients for a in curve.axes],
                'duration_candidates':tried,'optimality_scope':'First passing finite-grid candidate in this reference family; not globally optimal.'}
            return curve,geometry,deck,profile,report
    raise ValueError('No feasible rapid-contact candidate: '+str(tried[-12:]))
