"""Auditable offline landing proposals; no training, network or actuator effects.

Inputs contain causal entry state only. Contact results are separate labels.
The evidence lookup must not be presented as a learned/generalised policy.
"""
from dataclasses import asdict
import hashlib,json,math
from pathlib import Path
from .contact_geometry import Pose,RigidMotion,Deck,sub,norm,vector
from .touchdown_boundary import quaternion_product,ContactDesignLimits,TouchdownRequest,solve_touchdown_boundary
from .septic_approach import PlanarSnapMatchedApproach
from .moving_platform import check_moving_reference
from .reference_loader import load_verified_reference
from .execution import attitude_error


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def file_sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def motion_from_dict(data):
    return RigidMotion(Pose(**data['pose']),data['velocity_at_origin_world_m_s'],data['angular_velocity_world_rad_s'])


def entry_features(observation,deck_data,profile,context,decision_time_s):
    """COM-relative quantities; deck rotation includes its lever-arm velocity."""
    body=motion_from_dict(observation['motion']);dm=motion_from_dict(deck_data['motion'])
    com=body.pose.transform(profile['nominal_com_in_body_m'])
    q=dm.pose.orientation_xyzw;inverse=(-q[0],-q[1],-q[2],q[3])
    age=decision_time_s-observation['sim_t_s']
    if not math.isfinite(age) or age< -1e-10:raise ValueError('Causal entry observation required')
    acceleration=observation.get('com_acceleration_world_m_s2')
    features={'airframe_profile_id':profile['profile_id'],'airframe_profile_sha256':context['airframe_profile_sha256'],
        'controller_context_id':context['controller_context_id'],'mass_kg':profile['mass_kg'],
        'com_position_deck_m':dm.pose.inverse_rotate(sub(com,dm.pose.position_world_m)),
        'com_relative_velocity_deck_m_s':dm.pose.inverse_rotate(sub(body.velocity_at_world_point(com),dm.velocity_at_world_point(com))),
        'orientation_body_to_deck_xyzw':quaternion_product(inverse,body.pose.orientation_xyzw),
        'relative_angular_velocity_deck_rad_s':dm.pose.inverse_rotate(sub(body.angular_velocity_world_rad_s,dm.angular_velocity_world_rad_s)),
        'com_acceleration_world_m_s2':acceleration,'deck_half_extents_m':deck_data['half_extents_m'],
        'deck_velocity_world_m_s':dm.velocity_at_origin_world_m_s,'deck_angular_velocity_world_rad_s':dm.angular_velocity_world_rad_s,
        'deck_orientation_xyzw':dm.pose.orientation_xyzw,'armed':observation['armed'],'flight_mode':observation['flight_mode'],
        'observation_age_s':max(0.,age)}
    validate_features(features)
    return features


def validate_features(features):
    json.dumps(features,allow_nan=False)
    if not math.isfinite(features['mass_kg']) or features['mass_kg']<=0:raise ValueError('Known positive aircraft mass required')
    for key in ['com_position_deck_m','com_relative_velocity_deck_m_s','relative_angular_velocity_deck_rad_s',
                'deck_velocity_world_m_s','deck_angular_velocity_world_rad_s']:
        vector(features[key],key)
    if features['com_acceleration_world_m_s2'] is not None:vector(features['com_acceleration_world_m_s2'])
    Pose((0,0,0),features['orientation_body_to_deck_xyzw']);Pose((0,0,0),features['deck_orientation_xyzw'])
    size=features['deck_half_extents_m']
    if len(size)!=2 or any(not math.isfinite(x) or x<=0 for x in size):raise ValueError('Known positive deck half extents required')
    if type(features['armed']) is not bool or not isinstance(features['flight_mode'],str):raise ValueError('Explicit mode and armed flag required')
    if not math.isfinite(features['observation_age_s']) or features['observation_age_s']<0:raise ValueError('Nonnegative finite age required')


def controller_context(run,status):
    profile=run/'config/airframes/iris_fixed_gimbal_sim.json'
    cfg=status.get('horizontal_control')
    if cfg:
        values={k:cfg[k] for k in ['native_guided_options_confirmed','position_gain','velocity_gain','acceleration_lead_s','maximum_acceleration_m_s2']}
        controller={'interface':'external_xy_acceleration_feedback',**values,'feedback_source_sha256':file_sha(run/'landing/horizontal_feedback.py')}
    else:controller={'interface':'native_guided_pos_vel_accel','native_guided_options_confirmed':0}
    param_path=run/'results/dataflash_final/parameters.json'
    if not param_path.exists():param_path=run/'results/dataflash/parameters.json'
    params=json.loads(param_path.read_text())
    # Do not include learned hover thrust, timers or other postflight states
    # in the controller key. This subset is configured, non-adaptive gains in
    # these runs, rather than an estimate of every internal controller state.
    controller['native_relevant_parameters']={k:v for k,v in params.items() if k.startswith(('PSC_','ATC_','INS_GYRO_FILTER','INS_ACCEL_FILTER'))}
    context={'airframe_profile_sha256':file_sha(profile),'controller':controller,
        'execution_source_sha256':{n:file_sha(run/'landing'/n) for n in ['execution.py','observations.py','mavlink_reference.py','moving_platform.py']}}
    context['controller_context_id']=digest(context)
    return context


def import_flight(run,root):
    run=Path(run);status_path=run/'results/execution_status.json';audit_path=run/'rapid_analysis/verification.json'
    status=json.loads(status_path.read_text());audit=json.loads(audit_path.read_text())
    if not status['simulation_only'] or status['hardware_deployment_allowed']:raise ValueError('Simulation evidence required')
    raw=run/'results/model_states.jsonl'
    if audit['source_sha256']['model_states']!=file_sha(raw):raise ValueError('Physics source hash mismatch')
    profile_path=run/'config/airframes/iris_fixed_gimbal_sim.json';profile=json.loads(profile_path.read_text());context=controller_context(run,status)
    start=status['main_approach_start_sim_t_s'];touch=audit['contact_sim_t_s']
    entry=None
    with (run/'results/execution_feedback.jsonl').open() as f:
        for line in f:
            row=json.loads(line)
            if row['sim_t_s']==start and row['decision']['phase']=='TRACK_REFERENCE':entry=row;break
    if entry is None or not start<touch:raise ValueError('Precontact approach entry missing')
    features=entry_features(entry['observation'],entry['measured_deck'],profile,context,start)
    plan=status['rapid_plan'];boundary=plan['terminal_boundary']
    speed=plan['experiment_configuration']['target_closing_speed_m_s']
    action={'duration_s':plan['duration_s'],'target_closing_speed_m_s':speed,
        'target_nose_up_deg':plan['experiment_configuration']['target_nose_up_deg'],
        'terminal_snap_z_m_s4':plan['terminal_snap_z_m_s4'],'controller_context_id':context['controller_context_id']}
    # Group every action and controller variation of this nominal scenario
    # together. Different frames or residuals are not independent scenarios.
    scenario={'airframe_profile_sha256':context['airframe_profile_sha256'],
        'initial_com_position_x_z_m':plan['experiment_configuration']['initial_com_position_x_z_m'],
        'initial_com_velocity_world_m_s':plan['experiment_configuration']['initial_com_velocity_world_m_s'],
        'deck_top_center_world_m':[5.,0.,.8],'deck_half_extents_m':entry['measured_deck']['half_extents_m']}
    physical_keys=['tangential_speed','finite_closing_speed','nose_up_target','contact_time','postcontact_tilt','rebound_clearance','postcontact_footprint']
    violations=[k for k in physical_keys if audit['checks'].get(k) is False]
    physics_label=False if violations else (True if audit['passed'] else None)
    labels={'full_experiment_passed':audit['passed'],'observed_motion_feasible':physics_label,
        'observed_motion_violations':violations,'infrastructure_failures':status.get('external_failures',[]),
        'actual_approach_s':audit['actual_approach_s'],'closing_speed_upper_bound_m_s':audit['precontact']['closing_speed_upper_bound_m_s'],
        'tangential_speed_upper_bound_m_s':audit['precontact']['tangential_speed_upper_bound_m_s'],
        'nose_up_deg':audit['precontact']['nose_up_deg'],'solver_force_peak_N':audit['contact_load']['peak_solver_net_force_magnitude_N'],
        'postcontact_maximum_sampled_tilt_deg':audit['postcontact']['maximum_sampled_tilt_deg'],
        'postcontact_maximum_lowest_foot_gap_m':audit['postcontact']['maximum_lowest_foot_clearance_m'],
        'normal_disarm_observed':status['normal_disarm_observed'],'structural_safety_label':None}
    return {'schema_version':1,'record_id':run.relative_to(root).as_posix(),'scenario_group_id':digest(scenario),
        'features':features,'action':action,'labels':labels,'context':context,
        'label_contract':status['contact_limits'],'label_contract_id':digest(status['contact_limits']),
        'causality':{'feature_sim_t_s':entry['observation']['sim_t_s'],'decision_sim_t_s':start,'first_contact_sim_t_s':touch,
            'future_outcomes_not_in_features':True},
        'source_sha256':{'execution_status':file_sha(status_path),'physics_audit':file_sha(audit_path),
            'feedback':file_sha(run/'results/execution_feedback.jsonl'),'model_states':file_sha(raw)}}


def reference_candidate(root,speed,duration,samples=1001):
    if any(not math.isfinite(x) or x<=0 for x in (speed,duration)):raise ValueError('Positive finite speed and time required')
    if speed>.65:raise ValueError('Outside declared simulation speed envelope')
    _,geometry,deck,profile=load_verified_reference(root)
    base=json.loads((root/'config/experiments/snap_approach_v2.json').read_text())
    rapid=json.loads((root/'config/experiments/rapid_contact_v1.json').read_text())
    limits={**base['design_limits'],**rapid['new_design_limits']}
    boundary=solve_touchdown_boundary(geometry,deck,TouchdownRequest.planar(math.radians(2),0.,(0.,0.),(0.,0.,-speed)),
        ContactDesignLimits(math.radians(5),math.radians(5),.05,.001,.65,.02))
    offset=profile['nominal_com_in_body_m']
    curve=PlanarSnapMatchedApproach((0.,offset[1],3.65),(0.,0.,-.4),boundary,offset,base['reference_gravity_m_s2'],profile['mass_kg'],duration,terminal_com_snap_world_m_s4=(0.,0.,0.))
    report=check_moving_reference(curve,geometry,deck,limits,samples)
    return {'candidate_id':f'T{duration:.2f}_vz{speed:.2f}','action':{'duration_s':duration,'target_closing_speed_m_s':speed,'target_nose_up_deg':2.,'terminal_snap_z_m_s4':0.},
        'reference_feasible':report['passed'],'reference_check':report,'physics_validated':False,'structural_safety_label':None,
        'vertical_com_kinetic_energy_proxy_J':.5*profile['mass_kg']*speed**2,
        'proxy_scope':'Vertical translational kinetic energy component at the nominal terminal state; not impact force, absorbed energy or a safety estimate.',
        'normalized_polynomial_coefficients':[a.coefficients for a in curve.axes]}


def match_entry(query,evidence,limits):
    validate_features(query);validate_features(evidence)
    for key in ['airframe_profile_id','airframe_profile_sha256','controller_context_id','mass_kg']:
        if query[key]!=evidence[key]:return False,['context_'+key]
    if not query['armed'] or query['flight_mode']!='GUIDED':return False,['entry_flight_state']
    if not 0<=query['observation_age_s']<=limits['maximum_observation_age_s']:return False,['observation_age']
    if norm(query['deck_velocity_world_m_s'])>limits['maximum_deck_speed_m_s'] or norm(query['deck_angular_velocity_world_rad_s'])>limits['maximum_deck_angular_speed_rad_s']:
        return False,['moving_deck_outside_scope']
    residuals={'position_error_m':norm(sub(query['com_position_deck_m'],evidence['com_position_deck_m'])),
        'velocity_error_m_s':norm(sub(query['com_relative_velocity_deck_m_s'],evidence['com_relative_velocity_deck_m_s'])),
        'attitude_error_rad':attitude_error(Pose((0,0,0),query['orientation_body_to_deck_xyzw']),Pose((0,0,0),evidence['orientation_body_to_deck_xyzw'])),
        'angular_velocity_error_rad_s':norm(sub(query['relative_angular_velocity_deck_rad_s'],evidence['relative_angular_velocity_deck_rad_s']))}
    if query['com_acceleration_world_m_s2'] is None or evidence['com_acceleration_world_m_s2'] is None:return False,['acceleration_missing']
    residuals['com_acceleration_error_m_s2']=norm(sub(query['com_acceleration_world_m_s2'],evidence['com_acceleration_world_m_s2']))
    reasons=[k for k,v in residuals.items() if not math.isfinite(v) or v>limits[k]]
    if max(abs(a-b) for a,b in zip(query['deck_half_extents_m'],evidence['deck_half_extents_m']))>1e-9:reasons.append('deck_size')
    if attitude_error(Pose((0,0,0),query['deck_orientation_xyzw']),Pose((0,0,0),evidence['deck_orientation_xyzw']))>1e-3:reasons.append('deck_orientation')
    return not reasons,reasons


def propose_from_evidence(query,records,limits,maximum_target_closing_speed_m_s=.5):
    """Offline lookup baseline; a proposal is NEVER approval to execute it."""
    validate_features(query)
    if not math.isfinite(maximum_target_closing_speed_m_s) or maximum_target_closing_speed_m_s<=0:raise ValueError('Positive finite research speed preference required')
    accepted=[];rejected=[]
    for record in records:
        reasons=[]
        if not record['labels']['full_experiment_passed'] or record['labels']['observed_motion_feasible'] is not True:reasons.append('not_a_passing_physics_run')
        matched,why=match_entry(query,record['features'],limits)
        if not matched:reasons.extend(why)
        if record['action']['target_closing_speed_m_s']>maximum_target_closing_speed_m_s:reasons.append('target_speed_preference')
        if reasons:rejected.append({'record_id':record['record_id'],'reasons':reasons})
        else:accepted.append(record)
    accepted.sort(key=lambda r:(r['labels']['actual_approach_s'],r['action']['target_closing_speed_m_s']))
    selected=accepted[0] if accepted else None
    return {'status':'OFFLINE_PROPOSAL_REQUIRES_VALIDATION' if selected else 'NEEDS_MATCHED_DATA',
        'selected_action':dict(selected['action']) if selected else None,'evidence_record_id':selected['record_id'] if selected else None,
        'matching_passing_flights':len(accepted),'rejected_records':rejected,'trained_model':False,
        'actuation_allowed':False,'hardware_deployment_allowed':False,
        'scope':'Nearest archived-entry evidence lookup only. Matching tolerances are screening choices, not proof of a validated state neighbourhood. No safety probability or generalisation claim.'}
