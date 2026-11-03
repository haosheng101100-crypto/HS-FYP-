"""Replay XY supervision and nominal plans. This never certifies physical flight.

Run alongside the unchanged guided-motion, native-LAND and physical contact/exit
auditors. The historical uncertainty auditor assumes a single alignment epoch;
this auditor handles repeated planar-follow epochs explicitly.
"""
from pathlib import Path
from dataclasses import asdict,replace
import argparse
import json
import math
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from landing.planar_landing import PlanarLandingSupervisor,inflate_forecast_tube,capture_window,precontact_relative_audit
from landing.receding_landing import FlightState,PlatformState,PlannerLimits,BezierLandingPlanner
from landing.receding_live import unpack_plan
from landing.prediction_uncertainty import DelayedForecastMonitor,ForecastTube,reaction_reserve
from landing.contact_geometry import Pose,RigidMotion,AirframeGeometry
from landing.execution import AircraftObservation,BodyReference
from landing.swept_envelope import SweptEnvelope
from landing.platform_filter import CausalPlatformKalman
from landing.observations import PoseSample
from landing.platform_prediction import synthetic_pose_samples


def equal(a,b):
    if isinstance(a,dict):return isinstance(b,dict) and a.keys()==b.keys() and all(equal(a[k],b[k]) for k in a)
    if isinstance(a,(tuple,list)):return isinstance(b,(tuple,list)) and len(a)==len(b) and all(equal(x,y) for x,y in zip(a,b))
    if isinstance(a,bool) or a is None:return a is b
    if isinstance(a,(float,int)):return isinstance(b,(float,int)) and math.isclose(a,b,rel_tol=1e-8,abs_tol=1e-8)
    return a==b


def audit_rows(rows,cfg,com_offset=(0,0,0),events=None):
    policy=PlanarLandingSupervisor(cfg.get('planar_landing'))
    monitor=DelayedForecastMonitor(**cfg['prediction_uncertainty']) if cfg.get('prediction_uncertainty') else None
    snapshots={};failures=[];checked=0;contact_latched=False;previous_reference=None
    platform_filter=CausalPlatformKalman(**cfg['platform_filter']) if cfg.get('platform_filter') else None
    for row in rows:
        now=row['sim_t_s'];platform=PlatformState(**row['platform'])
        tube=monitor.update(platform) if monitor else None
        if platform_filter:
            samples=[PoseSample(s['sim_t_s'],Pose(**s['pose']),s['received_monotonic_s']) for s in row.get('platform_fit_input',[])]
            stress=row.get('synthetic_platform_observation_stress');cutoff=now-(stress['delay_s'] if stress else 0.)
            if any(s.sim_t_s>cutoff+1e-8 for s in samples):failures.append(dict(time=now,reason='FUTURE_PLATFORM_MEASUREMENT'))
            if stress:samples=synthetic_pose_samples(samples,cutoff,cfg['platform_fit_span_s'],stress['position_sigma_m'],stress['yaw_sigma_rad'],stress['seed'])
            try:
                reconstructed=platform_filter.update(samples,now,platform.half_extents)
                if not equal(asdict(reconstructed),asdict(platform)):failures.append(dict(time=now,reason='PLATFORM_FILTER_MISMATCH'))
            except ValueError:failures.append(dict(time=now,reason='PLATFORM_FILTER_INPUT_MISSING'))
        reported=row.get('planar_supervision')
        if reported is not None:
            checked+=1;inputs=reported.get('inputs')
            if not isinstance(inputs,dict) or not {'aligned','descending','gap_m','contact'}<=set(inputs) or set(inputs)-{'aligned','descending','gap_m','contact','braking_reference'}:
                failures.append(dict(time=now,reason='MISSING_POLICY_INPUTS'));continue
            if 'com_state' in row:state=FlightState(**row['com_state'])
            else:
                obs=row['observation'];m=obs['motion']
                motion=RigidMotion(**dict(m,pose=Pose(**m['pose'])))
                state=FlightState.from_observation(AircraftObservation(**dict(obs,motion=motion)),com_offset)
            gap=row.get('measured_leg_gap_m')
            if gap is None:gap=min(f['clearance_m'] for f in row['feet'])
            if not equal(inputs['gap_m'],gap):failures.append(dict(time=now,reason='POLICY_CLEARANCE_MISMATCH'))
            # Native ContactFrame serialization has sim_t_s/leg_names rather
            # than an active flag. Recognize its fresh actual collision names;
            # an empty/future/stale object must not manufacture contact.
            def observed_contact(frame):
                if not isinstance(frame,dict):return False
                if 'active' in frame:return frame['active'] is True
                stamp=frame.get('sim_t_s')
                names=frame.get('leg_names',[])+frame.get('unexpected_uav_collisions',[])
                return bool(isinstance(stamp,(float,int)) and not isinstance(stamp,bool)
                            and math.isfinite(stamp) and -1e-8<=now-stamp<=.06000001
                            and names and all(isinstance(name,str) and name for name in names))
            physical_contact=any(observed_contact(row.get(k)) for k in ('contact_frame','first_new_contact_event'))
            native_started=bool(events and any(e['event']=='NATIVE_CAPTURE_STARTED' and e['time']<now-1e-8 for e in events))
            contact_latched=contact_latched or physical_contact or native_started
            if inputs['contact']!=contact_latched:
                failures.append(dict(time=now,reason='CONTACT_WITHOUT_HANDOVER_EVIDENCE'))
            if events is not None:
                epochs=[e for e in events if e['time']<now-1e-8 and e['event'] in ('PLANAR_INITIAL_FOLLOW','PLANAR_REACQUIRE','ALIGNMENT_COMPLETE')]
                descending=bool(epochs and epochs[-1]['event']=='ALIGNMENT_COMPLETE')
                if inputs['descending']!=descending or inputs['aligned']!=descending:
                    failures.append(dict(time=now,reason='POLICY_EPOCH_MISMATCH'))
            reference_data=inputs.get('braking_reference')
            if 'braking_reference' in inputs and not equal(reference_data,previous_reference):
                failures.append(dict(time=now,reason='BRAKING_REFERENCE_MISMATCH'))
            reference=BodyReference(**reference_data) if reference_data else None
            expected=policy.update(now,state,platform,gap,aligned=inputs['aligned'],descending=inputs['descending'],contact=contact_latched,braking_reference=reference)
            if not equal(expected,{k:v for k,v in reported.items() if k!='inputs'}):
                failures.append(dict(time=now,reason='POLICY_REPLAY_MISMATCH'))
            if events is not None and any(e['event']=='ALIGNMENT_COMPLETE' and abs(e['time']-now)<1e-8 for e in events):
                readiness=row.get('alignment_readiness') or {}
                if not (expected.get('descent_eligible') and readiness.get('ready') and readiness.get('original_entry',{}).get('ready')):
                    failures.append(dict(time=now,reason='ALIGNMENT_WITHOUT_READY_WINDOW'))
            if expected['action'] in ('FOLLOW','REACQUIRE') and row['decision']['phase'] in ('TRACKING','TERMINAL_TRACKING'):
                failures.append(dict(time=now,reason='DESCENT_WITHOUT_WINDOW'))
            if expected['action']=='EXIT' and row['decision']['phase'] not in ('FAILED_ATTEMPT_RETURN','CONTACT_HANDOVER'):
                failures.append(dict(time=now,reason='EXIT_NOT_EXECUTED'))
            tube=inflate_forecast_tube(tube,expected.get('motion'))
        if tube is not None:
            if not equal(asdict(tube),row.get('forecast_uncertainty')):
                failures.append(dict(time=now,reason='FORECAST_RESERVE_MISMATCH'))
            snapshots[round(tube.stamp,8)]=asdict(tube)
        sent_reference=(row.get('sent_decision') or {}).get('reference')
        if sent_reference is not None:previous_reference=sent_reference
    return dict(passed=checked>0 and not failures,policy_rows=checked,failures=failures,
                physics_validated=False,scope='Causal policy and empirical-reserve replay only',snapshots=snapshots)


def audit_saved_plans(run,cfg,snapshots):
    profile_path=run/'config/airframes/iris_fixed_gimbal_sim.json'
    profile=json.loads(profile_path.read_text());envelope=SweptEnvelope.load(run/cfg['envelope_profile'])
    limits=PlannerLimits(**cfg['planner_limits']);reserve=cfg.get('nominal_acceleration_reserve_m_s2',0.)
    limits=replace(limits,acceleration=limits.acceleration-reserve,entry_acceleration_capture_m_s2=limits.acceleration if reserve else None)
    planner=BezierLandingPlanner(AirframeGeometry.load(profile_path),profile['nominal_com_in_body_m'],limits,envelope)
    failures=[];checked=0
    for path in sorted((run/'runtime/receding_planner').glob('response_*.json')):
        result=json.loads(path.read_text());request=result['request']
        if not result.get('plan'):continue
        checked+=1;plan=unpack_plan(result['plan']);model=PlatformState(**plan.audit['platform_prediction_input'])
        tube=ForecastTube(**request['uncertainty'])
        bound=reaction_reserve(tube,plan.start,plan.duration,limits.reaction_horizon_s,envelope.radius)
        if not equal(asdict(tube),snapshots.get(round(tube.stamp,8))):failures.append(path.name+': unrecorded uncertainty')
        shift=np.array(plan.target_position)-np.array(plan.audit['landing_point_offset_world_m'])-np.array(model.predict(plan.duration).position);shift[2]=0.
        radius=envelope.safe_radius(model,plan.duration,planner.offset,limits.margin+limits.position_reserve,shift)-bound['reserve_m']+min(limits.position_reserve,bound['reserve_m'])
        if not planner.audit(plan,FlightState(**request['state']),model,radius)['passed']:failures.append(path.name+': nominal audit')
        screen=capture_window(PlatformState(**request['platform']),plan.duration,cfg['planar_landing'])
        relative=precontact_relative_audit(plan,model,planner.geometry)
        if not screen['passed'] or not relative['passed']:failures.append(path.name+': capture approach')
        if not equal(screen,plan.audit.get('planar_capture')) or not equal(relative,plan.audit.get('planar_precontact')):
            failures.append(path.name+': capture metadata')
    return dict(passed=not failures,accepted_candidate_plans=checked,failures=failures)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args();run=args.run.resolve()
    cfg=json.loads((run/'config/experiments/receding_live_v1.json').read_text())
    if not cfg.get('planar_landing'):parser.error('This auditor requires the explicit planar profile')
    rows=[json.loads(line) for line in (run/'results/receding_control.jsonl').open()]
    session=run/'results/receding_session.json'
    record=json.loads(session.read_text()) if session.exists() else json.loads((run/'results/receding_attempts.json').read_text())[-1]
    events=record['events'];profile=json.loads((run/'config/airframes/iris_fixed_gimbal_sim.json').read_text())
    report=audit_rows(rows,cfg,profile['nominal_com_in_body_m'],events)
    snapshots=report.pop('snapshots');report['nominal_plans']=audit_saved_plans(run,cfg,snapshots)
    report['passed'] &= report['nominal_plans']['passed']
    (run/'results/independent_planar_policy_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2));return 0 if report['passed'] else 1


if __name__=='__main__':sys.exit(main())
