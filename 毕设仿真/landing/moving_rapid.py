"""Finite-speed contact with a measured, forward constant-velocity deck."""
from dataclasses import asdict
import json,math
from .contact_geometry import Pose,RigidMotion,Deck,norm
from .touchdown_boundary import ContactDesignLimits,TouchdownRequest,solve_touchdown_boundary
from .septic_approach import PlanarSnapMatchedApproach
from .moving_platform import translate_deck,check_moving_reference
from .rapid_contact import rapid_motion_metrics


def plan_moving_rapid(root,geometry,profile,deck_now,lead_s=1.5,durations=None,validate=True):
    cfg=json.loads((root/'config/experiments/moving_rapid_v1.json').read_text())
    if not cfg['simulation_only'] or cfg['hardware_deployment_allowed']:raise ValueError('Simulation configuration required')
    p=deck_now.motion.pose.position_world_m;v=deck_now.motion.velocity_at_origin_world_m_s
    if abs(v[0]-.3)>.01 or abs(v[1])>.005 or abs(v[2])>.005 or norm(deck_now.motion.angular_velocity_world_rad_s)>.005:
        raise ValueError('Forward constant .3 m/s deck required')
    if norm(tuple(a-b for a,b in zip(deck_now.motion.pose.rotate((0,0,1)),(0,0,1))))>.01:
        raise ValueError('Level deck required')
    ideal=Deck(RigidMotion(Pose((p[0],0.,p[2]),(0,0,0,1)),(v[0],0.,0.),(0,0,0)),deck_now.half_extents_m)
    base=json.loads((root/'config/experiments/snap_approach_v2.json').read_text())
    limits={**base['design_limits'],**cfg['new_design_limits']}
    lo,hi=cfg['contact_closing_speed_interval_m_s'];speed=cfg['target_closing_speed_m_s']
    offset=profile['nominal_com_in_body_m'];tried=[]
    for T in (durations if durations is not None else cfg['moving_duration_grid_s']):
        terminal_deck=translate_deck(ideal,lead_s+T)
        boundary=solve_touchdown_boundary(geometry,terminal_deck,
            TouchdownRequest.planar(math.radians(cfg['target_nose_up_deg']),0.,(0.,0.),(0.,0.,-speed)),
            ContactDesignLimits(math.radians(5),math.radians(5),.05,lo,hi,.02))
        curve=PlanarSnapMatchedApproach((0.,offset[1],3.65),(0.,0.,-.4),boundary,offset,
            base['reference_gravity_m_s2'],profile['mass_kg'],T,terminal_com_snap_world_m_s4=(0.,0.,0.))
        main_deck=translate_deck(ideal,lead_s)
        check=check_moving_reference(curve,geometry,main_deck,limits,201) if validate else {'passed':False,'pending_postflight_dense_check':True}
        motion=rapid_motion_metrics(curve,geometry,main_deck,cfg['rapid_motion_checks'],501) if validate else {'passed':False,'pending_postflight_dense_check':True}
        tried.append({'duration_s':T,'failures':check.get('failures',[]),'rapid_motion_passed':motion['passed']})
        if validate and (not check['passed'] or not motion['passed']):continue
        if validate:
            check=check_moving_reference(curve,geometry,main_deck,limits,4001)
            motion=rapid_motion_metrics(curve,geometry,main_deck,cfg['rapid_motion_checks'],4001)
            if not check['passed'] or not motion['passed']:continue
        return curve,ideal,{'simulation_only':True,'hardware_deployment_allowed':False,'duration_s':T,'lead_s':lead_s,
            'terminal_snap_z_m_s4':0.,'experiment_configuration':cfg,'effective_reference_limits':limits,
            'deck_at_planning':asdict(deck_now),'ideal_deck_at_planning':asdict(ideal),
            'target_contact_nose_up_deg':cfg['target_nose_up_deg'],'target_relative_foot_velocity_m_s':[0.,0.,-speed],
            'terminal_boundary':asdict(boundary),'normalized_polynomial_coefficients':[a.coefficients for a in curve.axes],
            'reference_checks':check,'rapid_motion_checks':motion,'duration_candidates':tried,
            'trained_model_used':False,'optimality_scope':'Finite simulation reference grid, not guaranteed trackable or globally optimal.'}
    raise ValueError('No finite-speed moving-deck reference: '+str(tried))
