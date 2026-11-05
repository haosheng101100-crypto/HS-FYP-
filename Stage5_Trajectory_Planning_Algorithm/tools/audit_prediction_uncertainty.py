"""Rebuild causal calibration and tightened nominal constraints from saved inputs."""
from pathlib import Path
import argparse,json,sys,math,inspect
from dataclasses import asdict,replace
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args()
run=a.run.resolve();sys.path.insert(0,str(run))
import numpy as np
from landing.prediction_uncertainty import DelayedForecastMonitor,ForecastTube,reaction_reserve
from landing.receding_landing import PlatformState,FlightState,BezierLandingPlanner,PlannerLimits
from landing.receding_live import unpack_plan,terminal_body_prediction,DeckAlignmentReference
from landing.swept_envelope import SweptEnvelope
from landing.contact_geometry import AirframeGeometry,evaluate_airframe,norm
from landing.contact_geometry import Pose,RigidMotion
from landing.execution import AircraftObservation,BodyReference
from landing.observations import PoseSample

cfg=json.loads((run/'config/experiments/receding_live_v1.json').read_text())
monitor=DelayedForecastMonitor(**cfg['prediction_uncertainty'])
envelope=SweptEnvelope.load(run/cfg['envelope_profile'])
profile_path=run/'config/airframes/iris_fixed_gimbal_sim.json'
profile=json.loads(profile_path.read_text())
limits=PlannerLimits(**cfg['planner_limits']);reserve=cfg.get('nominal_acceleration_reserve_m_s2',0.)
assert 0<=reserve<limits.acceleration
limits=replace(limits,acceleration=limits.acceleration-reserve,
    entry_acceleration_capture_m_s2=limits.acceleration if reserve else None)
planner=BezierLandingPlanner(AirframeGeometry.load(profile_path),profile['nominal_com_in_body_m'],limits,envelope)
platform_filter=None
if cfg.get('platform_filter'):
    from landing.platform_filter import CausalPlatformKalman
    from landing.platform_prediction import synthetic_pose_samples
    platform_filter=CausalPlatformKalman(**cfg['platform_filter'])
checks=dict(causal_tubes_reconstructed=True,request_uses_recorded_tube=True,
    reaction_reserves_reconstructed=True,landing_points_inside_trust_region=True,
    tightened_nominal_constraints=True,requests_warmed_up=True)
if reserve:
    health=json.loads((run/'runtime/receding_planner/health.json').read_text())
    checks['nominal_feedback_authority_reserved']=(health['nominal_planner_limits']==asdict(limits)
        and health['feedback_acceleration_reserve_m_s2']==reserve)
checks['synthetic_observation_inputs_causal']=True
checks['synthetic_observation_reconstructed']=True
checks['continuous_platform_filter_reconstructed']=True
checks['contact_gain_schedule_reconstructed']=True
checks['actual_feedback_matches_scheduled_gains']=True
tracking_samples=0;tracking_snapshots={}
if cfg.get('tracking_error_feedback'):
    checks.update(tracking_feedback_reconstructed=True,tracking_request_causality=True,tracking_terminal_hulls_reconstructed=True)
if cfg.get('maximum_frame_velocity_correction_m_s'):
    checks['frame_velocity_reserve_reconstructed']=True
    checks['frame_velocity_rejection_wired']=True
frame_reserve_samples=0;frame_reserve_rejections=0
heading_guard_samples=0;heading_guard_rejections=0
if cfg.get('maximum_contact_yaw_acceleration_rad_s2'):
    checks['heading_acceleration_guard_reconstructed']=True
    checks['heading_acceleration_rejection_wired']=True
gain_transition_samples=0
snapshots={};control_rows=0;accepted=0;offsets=[];reserve_values=[]
alignment_generator=None;alignment_rows=0;alignment_complete=None
responses={}
for f in sorted((run/'runtime/receding_planner').glob('response_*.json')):
    result=json.loads(f.read_text())
    if result.get('plan'):responses[result['sequence']]=unpack_plan(result['plan'])
activations=[];pending=None
session_path=run/'results/receding_session.json'
if session_path.exists():events=json.loads(session_path.read_text())['events']
else:
    attempts=json.loads((run/'results/receding_attempts.json').read_text())
    assert len(attempts)==1,'This verifier expects one autonomous attempt per physical cycle'
    events=attempts[0]['events']
for event in events:
    if event['event']=='PLAN_PENDING':pending=event['sequence']
    elif event['event']=='ACTIVATION_REJECTED':pending=None
    elif event['event']=='PLAN_ACTIVATED':
        assert pending in responses
        activations.append((event['time'],responses[pending]));pending=None
alignment_expected=bool(cfg.get('moving_alignment'))
if alignment_expected and cfg['moving_alignment'].get('only_for_degraded_observations'):
    first=json.loads(next((run/'results/receding_control.jsonl').open()))
    quality=first.get('synthetic_platform_observation_stress')
    alignment_expected=bool(quality and any(quality.get(k,0.)>0 for k in ('delay_s','position_sigma_m','yaw_sigma_rad')))
    policy=[e for e in events if e['event']=='PREPARATION_POLICY']
    checks['observation_quality_preparation_selection']=(len(policy)==1 and policy[0]['observation_stress']==quality and policy[0]['moving_alignment_required']==alignment_expected)
if cfg.get('moving_alignment'):
    starts=[e for e in events if e['event']=='ALIGNMENT_STARTED']
    completes=[e for e in events if e['event']=='ALIGNMENT_COMPLETE']
    assert len(starts)==int(alignment_expected) and len(completes)<=len(starts)
    alignment_complete=completes[0] if completes else None
    if alignment_expected and json.loads((run/'results/execution_status.json').read_text())['completed']:assert alignment_complete
    checks['moving_alignment_reconstructed']=True
    checks['moving_initial_state_continuation']=True
checks['terminal_margin_reconstructed']=True
terminal_samples=0
capture_entry_plans=0
checks['arrival_extension_budget_respected']=True
checks['extension_budget_reserved_for_capture']=True
checks['initial_capture_times_declared']=True
reaction=inspect.signature(terminal_body_prediction).parameters['reaction_horizon_s'].default
native_time_samples=0
if cfg.get('maximum_early_native_freeflight_s'):checks['early_native_time_guard_reconstructed']=True
capture_events=[v for v in events if v['event']=='CAPTURE_TIME_WINDOW_OPENED']
capture_opened_at=min((v['time'] for v in capture_events),default=None)
if cfg['planner_limits'].get('directional_corridor'):
    commits=[v for v in events if v['event']=='TERMINAL_PLAN_COMMITTED']
    checks['terminal_contact_commitment_reconstructed']=len(commits)<=1
    for event in commits:
        active=[plan for stamp,plan in activations if stamp<=event['time']+1e-8]
        checks['terminal_contact_commitment_reconstructed'] &= bool(active and math.isclose(event['arrival'],active[-1].arrival,abs_tol=1e-8) and active[-1].arrival-event['time']<=.8+1e-8 and not any(stamp>event['time']+1e-8 for stamp,plan in activations))
    if json.loads((run/'results/execution_status.json').read_text())['completed']:
        checks['terminal_contact_commitment_reconstructed'] &= bool(commits)
for event in capture_events:
    active=[plan for stamp,plan in activations if stamp<=event['time']+1e-8]
    assert active and event['remaining_s']<=cfg['arrival_extension_window_s']+1e-8
    assert math.isclose(active[-1].arrival-event['future_start'],event['remaining_s'],abs_tol=1e-8)
def equal(left,right):
    if isinstance(left,dict):return left.keys()==right.keys() and all(equal(left[k],right[k]) for k in left)
    if isinstance(left,(tuple,list)):return len(left)==len(right) and all(equal(x,y) for x,y in zip(left,right))
    if isinstance(left,bool):return left is right
    if isinstance(left,(float,int)):return math.isclose(left,right,rel_tol=1e-10,abs_tol=1e-10)
    return left==right
for line in (run/'results/receding_control.jsonl').open():
    row=json.loads(line);tube=monitor.update(PlatformState(**row['platform']))
    if cfg.get('tracking_error_feedback'):
        now=row['sim_t_s'];reported=row.get('tracking_contact_envelope')
        prior=[plan for stamp,plan in activations if stamp<now-1e-8]
        if reported is not None:
            tracking_samples+=1
            assert prior,'Tracking feedback without prior executed plan'
            plan=prior[-1];remaining=plan.arrival-now
            settings=cfg['tracking_error_feedback'];assert 0<remaining<=settings.get('window_s',5.)
            m=row['observation']['motion'];pose=Pose(**m['pose']);motion=RigidMotion(**dict(m,pose=pose))
            state=FlightState.from_observation(AircraftObservation(**dict(row['observation'],motion=motion)),planner.offset)
            platform=PlatformState(**row['platform']);model=PlatformState(**plan.audit['platform_prediction_input'])
            predicted=model.predict(now-model.stamp)
            dp=np.array(platform.position)-predicted.position;dv=np.array(platform.velocity)-predicted.velocity;dp[2]=dv[2]=0.
            s=min(1.,max(0.,(5.-remaining)/3.));w=10*s**3-15*s**4+6*s**5;wd=30*s*s*(1-s)**2/3. if s<1 else 0.
            nominal=plan.at(now-plan.start);h=min(settings.get('error_horizon_s',.35),remaining)
            error=np.array(state.position)-nominal['position']-w*dp+h*(np.array(state.velocity)-nominal['velocity']-w*dv-wd*dp);error[2]=0.
            from scipy.spatial.transform import Rotation
            rotation=Rotation.from_quat(pose.orientation_xyzw)*Rotation.from_quat(plan.motion(now-plan.start).pose.orientation_xyzw).inv()
            roll,pitch,yaw=np.abs(rotation.as_euler('xyz'))
            xy=max(float(np.linalg.norm(envelope.points[:,:2],axis=1).max()),*(math.hypot(*r['origin'][:2])+r['swept_radius'] for r in envelope.rotors))
            z=max(float(abs(envelope.points[:,2]).max()),*(abs(r['origin'][2])+r['half_height'] for r in envelope.rotors))
            attitude=2*envelope.radius if max(roll,pitch,yaw)>=math.pi/2 else min(2*envelope.radius,xy*(2-math.cos(roll)-math.cos(pitch)+math.sin(roll)*math.sin(pitch))+z*(math.sin(roll)+math.sin(pitch))+2*envelope.radius*math.sin(yaw/2))
            expected=dict(stamp_s=now,source_plan_start_s=plan.start,source_plan_arrival_s=plan.arrival,propagation_horizon_s=h,position_error_world_m=error.tolist(),attitude_reserve_m=attitude)
            checks['tracking_feedback_reconstructed'] &= equal(expected,reported)
        elif prior and row['decision']['phase'] in ('TRACKING','TERMINAL_TRACKING'):
            checks['tracking_feedback_reconstructed'] &= not 0<prior[-1].arrival-now<=cfg['tracking_error_feedback'].get('window_s',5.)
        tracking_snapshots[round(now,8)]=reported
    if 'observation_quality_preparation_selection' in checks:
        checks['observation_quality_preparation_selection'] &= row.get('synthetic_platform_observation_stress')==quality
    alignment=row.get('alignment_readiness')
    if alignment is not None:
        alignment_rows+=1
        state_data=dict(row['observation']);motion=state_data.pop('motion');motion=RigidMotion(**dict(motion,pose=Pose(**motion['pose'])))
        obs=AircraftObservation(**state_data,motion=motion);state=FlightState.from_observation(obs,planner.offset)
        platform=PlatformState(**row['platform']);local=envelope.centered_body_xy_in_deck(platform.deck());shift=platform.deck().motion.pose.rotate((*local,0.))
        reference=BodyReference(**row['decision']['reference'])
        if alignment_generator is None:
            start=next(e for e in events if e['event']=='ALIGNMENT_STARTED')
            alignment_generator=DeckAlignmentReference(BodyReference(**start['initial_reference']),planner.offset,cfg['moving_alignment']['minimum_leg_gap_m'],cfg['moving_alignment']['dwell_s'])
        expected_ref=alignment_generator.step(row['sim_t_s'],platform,shift)
        actual_angle=2*math.acos(min(1.,abs(obs.motion.pose.orientation_xyzw[3])))
        expected_report=alignment_generator.readiness(row['sim_t_s'],state,platform,shift,actual_angle,norm(obs.motion.angular_velocity_world_rad_s))
        checks['moving_alignment_reconstructed'] &= (row['decision']['phase']!='ALIGNING' or equal(asdict(reference),asdict(expected_ref))) and all(equal(expected_report[k],alignment[k]) for k in expected_report)
        if alignment_complete and abs(alignment_complete['time']-row['sim_t_s'])<1e-8:
            checks['moving_alignment_reconstructed'] &= alignment['ready'] and alignment['original_entry']['ready']
    capture=row.get('terminal_capture_window') or {}
    if cfg.get('maximum_early_native_freeflight_s') and capture.get('active'):
        native_time_samples+=1
        m=row['observation']['motion'];pose=Pose(**m['pose']);motion=RigidMotion(**dict(m,pose=pose))
        from landing.contact_geometry import Deck
        d=row['measured_deck'];dm=d['motion'];deck=Deck(RigidMotion(**dict(dm,pose=Pose(**dm['pose']))),tuple(d['half_extents_m']))
        fs=evaluate_airframe(planner.geometry,motion,deck);gap=fs[0].clearance_m
        nearest=[f for f in fs if f.clearance_m<=gap+.001]
        closing=min(2*f.closing_speed_m_s-f.closing_speed_upper_bound_m_s for f in nearest)
        estimate=gap/closing if gap>=0 and closing>0 else None
        expected_guard=dict(passed=estimate is not None and estimate<=cfg['maximum_early_native_freeflight_s'],gap_m=gap,minimum_closing_speed_m_s=closing,estimated_freeflight_s=estimate,maximum_time_s=cfg['maximum_early_native_freeflight_s'])
        checks['early_native_time_guard_reconstructed'] &= equal(expected_guard,capture.get('native_time_guard',{}))
        original_ready=(0<=capture['remaining_s']<=capture['early_window_s'] and capture['preview_passed'] and capture['whole_body_margin_m']>=capture['required_margin_m'] and all(capture['checks'].values()))
        checks['early_native_time_guard_reconstructed'] &= capture['ready']==(original_ready and expected_guard['passed'])
    if capture.get('ready'):
        checks.setdefault('measured_capture_window_independently_checked',True)
        obs=row['observation'];m=obs['motion'];pose=Pose(**m['pose']);motion=RigidMotion(**dict(m,pose=pose))
        from landing.contact_geometry import Deck
        d=row['measured_deck'];dm=d['motion'];deck=Deck(RigidMotion(**dict(dm,pose=Pose(**dm['pose']))),tuple(d['half_extents_m']))
        fs=evaluate_airframe(planner.geometry,motion,deck);gap=fs[0].clearance_m;closest=[f for f in fs if f.clearance_m<=gap+.001]
        original=json.loads((run/'config/experiments/execution_v1.json').read_text())['handover_limits']
        independent=dict(fresh=0<=row['sim_t_s']-obs['sim_t_s']<=original['observation_age_s'],armed_guided=obs['armed'] and obs['flight_mode']=='GUIDED',gap_0_to_1cm=0<=gap<=.01,
            relative_foot_speed=all(f.tangential_speed_upper_bound_m_s<=original['contact_tangential_speed_m_s'] and f.closing_speed_upper_bound_m_s<=original['contact_normal_speed_m_s'] and 2*f.closing_speed_m_s-f.closing_speed_upper_bound_m_s>=original['contact_minimum_closing_speed_m_s'] for f in closest),
            attitude=2*math.acos(min(1.,abs(pose.orientation_xyzw[3])))<=original['contact_attitude_error_rad'],angular_speed=norm(motion.angular_velocity_world_rad_s)<=original['contact_angular_speed_rad_s'],
            footprint=all(min(f.whole_cylinder_footprint_margins_m)>=original['footprint_margin_m'] for f in fs))
        active=[plan for stamp,plan in activations if stamp<=row['sim_t_s']+1e-8]
        remaining=active[-1].arrival-row['sim_t_s'] if active else -1.
        events_now=[e for e in events if e['event']=='MEASURED_TERMINAL_CAPTURE' and abs(e['time']-row['sim_t_s'])<1e-8]
        checks['measured_capture_window_independently_checked'] &= bool(all(independent.values()) and all(capture['checks'][k]==v for k,v in independent.items()) and 0<=remaining<=original['contact_time_error_s'] and math.isclose(remaining,capture['remaining_s'],abs_tol=1e-8) and envelope.clearance(motion,deck)>=original['footprint_margin_m'] and capture['preview_passed'] and len(events_now)==1 and row['planner_phase']=='CONTACT_HANDOVER')
    if cfg.get('final_command_acceleration_reserve_m_s2') and row['sent_decision']['reference'] is not None:
        checks.setdefault('final_acceleration_response_reserve_reconstructed',True)
        cap=cfg['planner_limits']['acceleration']-cfg['final_command_acceleration_reserve_m_s2']
        diag=row['command_limits']
        checks['final_acceleration_response_reserve_reconstructed'] &= bool(np.linalg.norm(row['sent_decision']['reference']['acceleration_world_m_s2'])<=cap+1e-8 and math.isclose(diag['command_acceleration_cap_m_s2'],cap,abs_tol=1e-10) and math.isclose(diag['physical_acceleration_limit_m_s2'],cfg['planner_limits']['acceleration'],abs_tol=1e-10))
    if cfg.get('maximum_contact_yaw_acceleration_rad_s2'):
        guard=row['heading_acceleration_guard']
        if guard['active']:
            value=row['platform']['yaw_acceleration'];limit=cfg['maximum_contact_yaw_acceleration_rad_s2'];passed=abs(value)<=limit
            checks['heading_acceleration_guard_reconstructed'] &= (equal(value,guard['measured_yaw_acceleration_rad_s2'])
                and equal(limit,guard['maximum_yaw_acceleration_rad_s2']) and passed==guard['passed'])
            checks['heading_acceleration_rejection_wired'] &= min(f['clearance_m'] for f in row['feet'])<=1. and row['observation']['motion']['velocity_at_origin_world_m_s'][2]<0
            if not passed:
                checks['heading_acceleration_rejection_wired'] &= row['stage']=='RECOVERING' and row['planner_phase']=='RECOVERY_REQUIRED'
                heading_guard_rejections+=1
            heading_guard_samples+=1
    if cfg.get('maximum_frame_velocity_correction_m_s'):
        guard=row['frame_velocity_guard']
        if guard['active']:
            now=row['sim_t_s'];prior=[plan for stamp,plan in activations if stamp<now-1e-8]
            assert prior,'Frame reserve guard without a previously active plan'
            plan=prior[-1];model=PlatformState(**plan.audit['platform_prediction_input']);platform=PlatformState(**row['platform'])
            remaining=plan.arrival-now;s=min(1.,max(0.,(5.-remaining)/3.))
            w=10*s**3-15*s**4+6*s**5;wd=30*s*s*(1-s)**2/3. if s<1 else 0.
            predicted=model.predict(now-model.stamp)
            correction=w*(np.array(platform.velocity)-predicted.velocity)+wd*(np.array(platform.position)-predicted.position)
            correction[2]=0.;magnitude=float(np.linalg.norm(correction[:2]));bound=cfg['maximum_frame_velocity_correction_m_s']
            passed=magnitude<=bound+1e-12
            checks['frame_velocity_reserve_reconstructed'] &= (equal(correction.tolist(),guard['velocity_correction_m_s'])
                and equal(magnitude,guard['horizontal_correction_m_s']) and equal(bound,guard['maximum_correction_m_s']) and passed==guard['passed'])
            if not passed:
                checks['frame_velocity_rejection_wired'] &= row['stage']=='RECOVERING' and row['planner_phase']=='RECOVERY_REQUIRED'
                frame_reserve_rejections+=1
            frame_reserve_samples+=1
    feedback=row.get('xy_feedback')
    if cfg.get('terminal_horizontal_feedback') and feedback:
        schedule=row['horizontal_feedback_schedule'];remaining=None
        if row.get('planner_phase') in ('TRACKING','TERMINAL_TRACKING'):
            current_plans=[p for stamp,p in activations if stamp<=row['sim_t_s']+1e-8]
            assert current_plans
            remaining=current_plans[-1].arrival-row['sim_t_s']
        checks['contact_gain_schedule_reconstructed'] &= equal(remaining,schedule['remaining_s'])
        terminal=cfg['terminal_horizontal_feedback'];span=terminal['transition_s']
        measured_speed=math.hypot(*row['platform']['velocity'][:2])
        if terminal.get('speed_scheduling'):
            design=terminal['speed_scheduling'];q=min(1.,max(0.,(measured_speed-design['low_speed_m_s'])/(design['high_speed_m_s']-design['low_speed_m_s'])))
            w_speed=10*q**3-15*q**4+6*q**5
            span=(1-w_speed)*span+w_speed*design['fast_transition_s']
        checks['contact_gain_schedule_reconstructed'] &= equal(span,schedule['transition_s']) and equal(measured_speed,schedule['horizontal_speed_m_s'])
        s=0. if remaining is None else min(1.,max(0.,1.-remaining/span))
        w=10*s**3-15*s**4+6*s**5;expected=dict(cfg['horizontal_feedback'])
        non_descent=cfg.get('stable_non_descent_feedback') and row['decision']['phase'] not in ('TRACKING','TERMINAL_TRACKING')
        if non_descent:
            w=1.
            checks['contact_gain_schedule_reconstructed'] &= schedule.get('policy')=='stable_non_descent_tracking'
        for key in ('position_gain','velocity_gain','acceleration_lead_s'):
            expected[key]=(1-w)*expected[key]+w*terminal[key]
        if non_descent and row['decision']['phase'] in ('PLANNING_HOVER','UNCERTAINTY_WARMUP'):
            expected['acceleration_lead_s']=cfg['horizontal_feedback']['acceleration_lead_s']
        checks['contact_gain_schedule_reconstructed'] &= equal(expected,schedule['gains']) and math.isclose(w,schedule['blend'],abs_tol=1e-10)
        ref=row['decision']['reference'];obs=row['observation']['motion']
        raw=np.array(ref['acceleration_world_m_s2'][:2])+expected['acceleration_lead_s']*np.array(feedback['body_reference_jerk_xy_m_s3'])
        raw+=expected['position_gain']*(np.array(ref['position_world_m'][:2])-obs['pose']['position_world_m'][:2])
        raw+=expected['velocity_gain']*(np.array(ref['velocity_world_m_s'][:2])-obs['velocity_at_origin_world_m_s'][:2])
        capped=raw*min(1.,expected['maximum_acceleration_m_s2']/max(float(np.linalg.norm(raw)),1e-12))
        checks['actual_feedback_matches_scheduled_gains'] &= bool(np.allclose(raw,feedback['raw_acceleration_xy_m_s2'],atol=1e-10,rtol=1e-10)
            and np.allclose(capped,feedback['sent_acceleration_xy_m_s2'],atol=1e-10,rtol=1e-10))
        if s>0:gain_transition_samples+=1
    checks['causal_tubes_reconstructed'] &= equal(asdict(tube),row['forecast_uncertainty'])
    if platform_filter:
        samples=[PoseSample(s['sim_t_s'],Pose(**s['pose']),s['received_monotonic_s']) for s in row['platform_fit_input']]
        stress=row.get('synthetic_platform_observation_stress')
        cutoff=row['sim_t_s']-(stress['delay_s'] if stress else 0.)
        checks['synthetic_observation_inputs_causal'] &= all(s.sim_t_s<=cutoff for s in samples)
        if stress:
            samples=synthetic_pose_samples(samples,cutoff,cfg['platform_fit_span_s'],stress['position_sigma_m'],stress['yaw_sigma_rad'],stress['seed'])
        measured=platform_filter.update(samples,row['sim_t_s'],cfg['deck_half_extents'])
        checks['continuous_platform_filter_reconstructed'] &= equal(asdict(measured),row['platform'])
        if stress:checks['synthetic_observation_reconstructed'] &= equal(asdict(measured),row['platform'])
    elif row.get('synthetic_platform_observation_stress'):
        from landing.platform_prediction import delayed_noisy_platform
        stress=row['synthetic_platform_observation_stress'];cutoff=row['sim_t_s']-stress['delay_s']
        samples=[PoseSample(s['sim_t_s'],Pose(**s['pose']),s['received_monotonic_s']) for s in row['platform_fit_input']]
        checks['synthetic_observation_inputs_causal'] &= all(s.sim_t_s<=cutoff for s in samples)
        measured=delayed_noisy_platform(samples,row['sim_t_s'],cfg['deck_half_extents'],
            span=cfg['platform_fit_span_s'],**stress)
        checks['synthetic_observation_reconstructed'] &= equal(asdict(measured),row['platform'])
    snapshots[round(tube.stamp,8)]=asdict(tube);control_rows+=1
    preview=row.get('terminal_body_prediction') or {}
    if preview.get('active') and row.get('planner_phase') in ('TRACKING','TERMINAL_TRACKING'):
        active=[plan for stamp,plan in activations if stamp<=row['sim_t_s']+1e-8]
        assert active,'Terminal guard without activated plan'
        plan=active[-1];now=row['sim_t_s'];remaining=plan.arrival-now
        obs_data=dict(row['observation']);motion=dict(obs_data.pop('motion'))
        observation=AircraftObservation(**obs_data,motion=RigidMotion(**dict(motion,pose=Pose(**motion['pose']))))
        state=FlightState.from_observation(observation,planner.offset)
        nominal=plan.at(now-plan.start);base=plan.body_reference_at(now)
        ref=BodyReference(**row['decision']['reference'])
        dp=np.array(ref.position_world_m)-base.position_world_m
        dv=np.array(ref.velocity_world_m_s)-base.velocity_world_m_s
        platform=PlatformState(**row['platform']);model=PlatformState(**plan.audit['platform_prediction_input'])
        end_shift=np.array(platform.predict(remaining).position)-model.predict(plan.arrival-model.stamp).position
        end_shift[2]=0.
        error=np.array(state.position)-nominal['position']-dp+min(.35,remaining)*(np.array(state.velocity)-nominal['velocity']-dv)+end_shift
        end=plan.motion(plan.duration)
        predicted=replace(end,pose=replace(end.pose,position_world_m=tuple(np.array(end.pose.position_world_m)+error)))
        dot=abs(float(np.dot(observation.motion.pose.orientation_xyzw,plan.motion(now-plan.start).pose.orientation_xyzw)))
        attitude=2*math.acos(min(1.,dot))*envelope.radius
        if preview.get('attitude_reserve_model')=='axis_bounded_envelope_projection':
            from scipy.spatial.transform import Rotation
            relative=Rotation.from_quat(observation.motion.pose.orientation_xyzw)*Rotation.from_quat(plan.motion(now-plan.start).pose.orientation_xyzw).inv()
            roll,pitch,yaw=np.abs(relative.as_euler('xyz'))
            xy=max(float(np.linalg.norm(envelope.points[:,:2],axis=1).max()),
                *(math.hypot(*r['origin'][:2])+r['swept_radius'] for r in envelope.rotors))
            z=max(float(abs(envelope.points[:,2]).max()),*(abs(r['origin'][2])+r['half_height'] for r in envelope.rotors))
            attitude=2*envelope.radius if max(roll,pitch,yaw)>=math.pi/2 else min(2*envelope.radius,
                xy*(2-math.cos(roll)-math.cos(pitch)+math.sin(roll)*math.sin(pitch))+
                z*(math.sin(roll)+math.sin(pitch))+2*envelope.radius*math.sin(yaw/2))
        bound=tube.at(min(remaining,reaction))
        reserve=bound['position_m']+envelope.radius*bound['heading_rad']
        clearance=envelope.clearance(predicted,platform.predict(remaining).deck())-attitude-reserve
        checks['terminal_margin_reconstructed'] &= math.isclose(clearance,preview['predicted_whole_aircraft_margin_m'],abs_tol=1e-8)
        checks['terminal_margin_reconstructed'] &= preview['passed']==(clearance>=preview['required_margin_m'])
        terminal_samples+=1
for f in sorted((run/'runtime/receding_planner').glob('response_*.json')):
    result=json.loads(f.read_text());req=result['request'];tube=ForecastTube(**req['uncertainty'])
    if cfg.get('tracking_error_feedback'):
        submitted=req['now']-cfg['future_activation_lead_s']
        checks['tracking_request_causality'] &= equal(req.get('tracking_envelope'),tracking_snapshots.get(round(submitted,8),'MISSING'))
    if alignment_complete and req['previous_arrival'] is None:
        ref=BodyReference(**alignment_complete['initial_reference']);dt=req['now']-alignment_complete['time']
        expected=np.array(ref.position_world_m)+planner.offset+np.array(ref.velocity_world_m_s)*dt+.5*np.array(ref.acceleration_world_m_s2)*dt*dt
        checks['moving_initial_state_continuation'] &= bool(dt>=0 and np.allclose(req['state']['position'],expected,atol=1e-9) and np.allclose(req['state']['velocity'],np.array(ref.velocity_world_m_s)+np.array(ref.acceleration_world_m_s2)*dt,atol=1e-9) and np.allclose(req['state']['acceleration'],ref.acceleration_world_m_s2,atol=1e-9))
    if req['previous_arrival'] is None and cfg.get('initial_candidate_durations_s'):
        durations=cfg['moving_alignment'].get('initial_candidate_durations_s',cfg['initial_candidate_durations_s']) if alignment_complete else cfg['initial_candidate_durations_s']
        checks['initial_capture_times_declared'] &= req['durations']==durations
    if req['previous_arrival'] is not None:
        maximum=req['previous_arrival']+cfg['arrival_extension_budget_s']
        checks['arrival_extension_budget_respected'] &= all(req['now']+d<=maximum+1e-8 for d in req['durations'])
        if cfg.get('arrival_extension_window_s'):
            submitted=req['now']-cfg['future_activation_lead_s']
            prior=[plan for stamp,plan in activations if stamp<=submitted+1e-8]
            assert prior
            arrival=prior[-1].arrival
            if capture_opened_at is None or submitted<capture_opened_at-1e-8:
                early=req['previous_arrival']+cfg.get('early_arrival_extension_budget_s',0.)
                checks['extension_budget_reserved_for_capture'] &= all(req['now']+d<=early+1e-8 for d in req['durations'])
    checks['request_uses_recorded_tube'] &= equal(asdict(tube),snapshots.get(round(tube.stamp,8),{}))
    checks['requests_warmed_up'] &= tube.ready and tube.stamp<=req['now']
    if not result.get('plan'):continue
    plan=unpack_plan(result['plan']);accepted+=1
    if cfg.get('tracking_error_feedback'):
        checks['tracking_request_causality'] &= equal(plan.audit.get('tracking_contact_envelope'),req.get('tracking_envelope'))
    if np.linalg.norm(req['state']['acceleration'])>limits.acceleration+limits.tolerance:capture_entry_plans+=1
    reserve=reaction_reserve(tube,plan.start,plan.duration,planner.limits.reaction_horizon_s,envelope.radius)
    checks['reaction_reserves_reconstructed'] &= equal(reserve,plan.audit['uncertainty_reserve'])
    model=PlatformState(**plan.audit['platform_prediction_input'])
    delta=np.array(plan.audit['landing_point_offset_world_m']);offsets.append(float(np.linalg.norm(delta)))
    checks['landing_points_inside_trust_region'] &= offsets[-1]<=planner.limits.landing_point_trust_radius_m+2e-5
    baseline_shift=np.array(plan.target_position)-delta-np.array(model.predict(plan.duration).position)
    baseline_shift[2]=0.
    reused=plan.audit.get('fixed_prediction_allowance_reused_m',0.)
    checks['reaction_reserves_reconstructed'] &= 0<=reused<=min(planner.limits.position_reserve,reserve['reserve_m'])+1e-10
    radius=envelope.safe_radius(model,plan.duration,planner.offset,
        planner.limits.margin+planner.limits.position_reserve,baseline_shift)-reserve['reserve_m']+reused
    checks['reaction_reserves_reconstructed'] &= math.isclose(radius,plan.audit['usable_radius_m'],abs_tol=2e-5)
    audit=planner.audit(plan,FlightState(**req['state']),model,radius)
    checks['tightened_nominal_constraints'] &= audit['passed']
    if cfg['planner_limits'].get('directional_corridor'):
        # Separate numerical reconstruction of signed rectangle certificates.
        # Do not call planner.corridor_facets / the planner's constraint builder.
        checks.setdefault('directional_hulls_independently_reconstructed',True)
        segments=len(plan.points);dt=plan.duration/segments
        count=min(segments,max(1,math.ceil(limits.terminal_coverage_s/dt)))
        for k,c in enumerate(plan.points):
            pp=model.predict(k*dt);angles=[pp.yaw,pp.predict(dt).yaw]
            if pp.yaw_acceleration and 0 < -pp.yaw_rate/pp.yaw_acceleration < dt:
                angles.append(pp.predict(-pp.yaw_rate/pp.yaw_acceleration).yaw)
            mid=(min(angles)+max(angles))/2
            chord=2*math.sin(min(math.pi,(max(angles)-min(angles))/2)/2)
            acceleration=42*np.diff(c,n=2,axis=0)/dt**2
            elevate=np.array([[math.comb(5,j)*math.comb(2,i-j)/math.comb(7,i) if 0<=i-j<=2 else 0. for j in range(6)] for i in range(8)])@acceleration
            slope=0. if k>=segments-count else limits.corridor_slope
            for i in range(8):
                center=np.array(pp.position)+baseline_shift+np.array(pp.velocity)*dt*i/7+np.array(pp.acceleration)*dt**2*i*(i-1)/84+np.array(pp.jerk)*dt**3*i*(i-1)*(i-2)/1260
                e=c[i]-center;gap=c[i,2]-center[2]-(plan.target_position[2]-model.position[2])
                tilt=planner.footprint_tilt_coefficient*np.linalg.norm(elevate[i,:2])
                for half,axis in zip(pp.half_extents,((math.cos(mid),math.sin(mid),0.),(-math.sin(mid),math.cos(mid),0.))):
                    for sign in (1.,-1.):
                        u=sign*np.array(axis);body_support=envelope.support(u,planner.offset)
                        lhs=u@e+body_support+u@baseline_shift+tilt+chord*(np.linalg.norm(e[:2])+envelope.radius+np.linalg.norm(planner.offset)+np.linalg.norm(baseline_shift))+limits.margin+max(limits.position_reserve,reserve['reserve_m'])
                        checks['directional_hulls_independently_reconstructed'] &= bool(gap>=-2e-5 and lhs<=half+slope*gap+2e-5)
                        if getattr(limits,'tracking_error_tightening',False) and k==len(plan.points)-1:
                            feedback=plan.audit.get('tracking_contact_envelope')
                            error=np.zeros(3) if feedback is None else np.array(feedback['position_error_world_m'])
                            attitude=0. if feedback is None else feedback['attitude_reserve_m']
                            corrected=lhs+u@error+chord*np.linalg.norm(error[:2])+attitude
                            checks['tracking_terminal_hulls_reconstructed'] &= bool(corrected<=half+slope*gap+2e-5)
    reserve_values.append(reserve['reserve_m'])
checks['control_rows_present']=control_rows>0
if cfg.get('tracking_error_feedback'):
    checks['tracking_feedback_exercised']=tracking_samples>0 or not json.loads((run/'results/execution_status.json').read_text())['completed']
if cfg.get('maximum_early_native_freeflight_s'):
    checks['early_native_time_guard_exercised']=native_time_samples>0 or not json.loads((run/'results/execution_status.json').read_text())['completed']
if cfg.get('moving_alignment'):checks['moving_alignment_exercised']=(alignment_rows>0)==alignment_expected
checks['accepted_plan_present']=accepted>0
checks['terminal_samples_present']=terminal_samples>0 or not json.loads((run/'results/execution_status.json').read_text())['completed']
checks['contact_gain_transition_exercised']=(not cfg.get('terminal_horizontal_feedback') or gain_transition_samples>0
    or not json.loads((run/'results/execution_status.json').read_text())['completed'])
if cfg.get('maximum_frame_velocity_correction_m_s'):
    checks['frame_velocity_reserve_exercised']=(frame_reserve_samples>0
        or not json.loads((run/'results/execution_status.json').read_text())['completed'])
    if frame_reserve_rejections:
        attempts=json.loads((run/'results/receding_attempts.json').read_text())
        checks['frame_velocity_rejection_wired'] &= attempts[-1]['reason']=='FRAME_VELOCITY_RESERVE'
if cfg.get('maximum_contact_yaw_acceleration_rad_s2'):
    checks['heading_acceleration_guard_exercised']=heading_guard_samples>0
    if heading_guard_rejections:
        attempts=json.loads((run/'results/receding_attempts.json').read_text())
        checks['heading_acceleration_rejection_wired'] &= attempts[-1]['reason']=='CONTACT_YAW_ACCELERATION_RESERVE'
checks={k:bool(v) for k,v in checks.items()}
report=dict(passed=all(checks.values()),checks=checks,accepted_plans=accepted,
    tracking_feedback_samples=tracking_samples,
    early_native_time_guard_samples=native_time_samples,
    terminal_guard_samples=terminal_samples,
    gain_transition_samples=gain_transition_samples,
    frame_velocity_reserve_samples=frame_reserve_samples,frame_velocity_reserve_rejections=frame_reserve_rejections,
    heading_guard_samples=heading_guard_samples,heading_guard_rejections=heading_guard_rejections,
    entry_acceleration_capture_plans=capture_entry_plans,
    largest_landing_point_offset_m=max(offsets,default=None),
    largest_reaction_reserve_m=max(reserve_values,default=None),
    scope='Reconstructed causal empirical calibration and tightened nominal plans. No statistical coverage, invariant tube, or continuous collision guarantee.')
out=run/'results/independent_uncertainty_audit_v24.json';assert not out.exists()
out.write_text(json.dumps(report,indent=2));print(json.dumps(report))
