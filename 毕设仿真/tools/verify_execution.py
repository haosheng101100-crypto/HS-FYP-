"""Offline execution-contract tests and causal saved-contact replay.

No new flight or live transport. Recorded flight modes were not available in
the pose/contact log; replay uses UNKNOWN and cannot establish LAND ACK or a
new successful touchdown. Synthetic mode/arming tests are labelled separately.
"""
from pathlib import Path
from dataclasses import asdict
import bisect
import csv
import hashlib
import io
import json
import math
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'work/plot_dependencies'))
import numpy as np
from landing.contact_geometry import Pose,RigidMotion,cross,add,evaluate_airframe
from landing.reference_loader import load_verified_reference
from landing.startup import VerticalLeadIn
from landing.execution import (AircraftObservation,ContactHistory,EntryLimits,HandoverLimits,
    ContactHandover,assess_entry,parse_deck_contacts)

OUT=ROOT/'outputs/model_audit/step08';OUT.mkdir(parents=True,exist_ok=True)
cfg_path=ROOT/'config/experiments/execution_v1.json'
cfg=json.loads(cfg_path.read_text());assert cfg['simulation_only'] and not cfg['hardware_deployment_allowed']
curve,geometry,deck,profile=load_verified_reference(ROOT)
assert cfg['airframe_profile']=='config/airframes/iris_fixed_gimbal_sim.json'
entry_limits=EntryLimits(**cfg['entry_limits']);handover_limits=HandoverLimits(**cfg['handover_limits'])
lead_cfg=cfg['vertical_lead_in'];assert lead_cfg['enabled']
lead=VerticalLeadIn(curve,lead_cfg['duration_s'],lead_cfg['height_above_approach_initial_com_m'])

test_stream=io.StringIO()
tests=unittest.TextTestRunner(stream=test_stream,verbosity=2).run(unittest.defaultTestLoader.discover(str(ROOT/'tests')))
(OUT/'test_output.txt').write_text(test_stream.getvalue(),encoding='utf-8')
assert tests.wasSuccessful(),test_stream.getvalue()

limits=json.loads((ROOT/'config/experiments/snap_approach_v2.json').read_text())['design_limits']
lead_extrema={k:0. for k in ['speed_m_s','acceleration_m_s2','jerk_m_s3','snap_m_s4']}
min_gap=math.inf;lead_rows=[]
for tick in range(1001):
    t=lead.T*tick/1000;state=lead.at(t)
    for name,key in [('speed_m_s','com_velocity_world_m_s'),('acceleration_m_s2','com_acceleration_world_m_s2'),
                     ('jerk_m_s3','com_jerk_world_m_s3'),('snap_m_s4','com_snap_world_m_s4')]:
        lead_extrema[name]=max(lead_extrema[name],float(np.linalg.norm(state[key])))
    min_gap=min(min_gap,min(f.clearance_m for f in evaluate_airframe(geometry,state['motion'],deck)))
    lead_rows.append((t,state))
lead_checks={
    'speed':lead_extrema['speed_m_s']<=limits['max_vertical_speed_m_s'],
    'acceleration':lead_extrema['acceleration_m_s2']<=limits['max_vertical_acceleration_m_s2'],
    'jerk':lead_extrema['jerk_m_s3']<=limits['max_jerk_m_s3'],
    'snap':lead_extrema['snap_m_s4']<=limits['max_snap_m_s4'],
    'leg_plane_clearance':min_gap>limits['edge_height_margin_m'],
    'monotone_descent':all(state['motion'].velocity_at_origin_world_m_s[2]<=1e-10 for _,state in lead_rows)}
jet_keys=['com_position_world_m','com_velocity_world_m_s','com_acceleration_world_m_s2','com_jerk_world_m_s3','com_snap_world_m_s4']
seam_errors={key:max(abs(a-b) for a,b in zip(lead.at(lead.T)[key],curve.at(0)[key])) for key in jet_keys}
lead_checks['position_derivatives_through_snap_match']=max(seam_errors.values())<1e-12
(OUT/'lead_in_check.json').write_text(json.dumps({'passed':all(lead_checks.values()),'checks':lead_checks,
    'duration_s':lead.T,'main_approach_duration_s':curve.T,'combined_reference_duration_s':lead.T+curve.T,
    'initial_com_position_world_m':lead.at(0)['com_position_world_m'],'sample_count':1001,
    'sampled_extrema':lead_extrema,'minimum_leg_plane_gap_m':min_gap,'seam_errors':seam_errors,
    'scope':'Offline reference consistency; does not prove measured aircraft follows the preparation.'},indent=2)+'\n',encoding='utf-8')

RUN=ROOT/'outputs/model_audit/step04/run_004';rs=RUN/'results'
manifest=json.loads((RUN/'attempt_manifest.json').read_text())
old_hashes_match=all(hashlib.sha256((RUN/name).read_bytes()).hexdigest()==digest for name,digest in manifest['source_sha256'].items())
status=json.loads((rs/'cooperation_status.json').read_text())
positions={};frames=[]
mapping={cfg['leg_collision_prefix']+leg.name:leg.name for leg in geometry.legs}

def timestamp(message):
    stamp=message.get('header',{}).get('stamp',{})
    return float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9

with (rs/'model_states.jsonl').open() as stream:
    for line in stream:
        row=json.loads(line);msg=row['message']
        if row['kind']=='uav_pose':
            for p in msg.get('pose',[]):
                if p['name']=='uav':positions[timestamp(p)]=Pose(tuple(p['position'].get(k,0.) for k in 'xyz'),tuple(p['orientation'].get(k,0.) for k in ['x','y','z','w']))
        elif row['kind']=='deck_contact':frames.append(parse_deck_contacts(msg,cfg['deck_collision'],mapping,cfg['uav_model_prefix']))
times=sorted(positions)
frames.sort(key=lambda frame:frame.sim_t_s)
touch=min(frame.sim_t_s for frame in frames if frame.active and frame.sim_t_s>=status['approach_start_sim_t'])
history=ContactHistory(max_frames=max(10000,len(frames)+1))
for frame in frames:history.push(frame)
csv_rows=list(csv.DictReader((rs/'cooperation.csv').open()))
csv_times=[float(row['sim_t']) for row in csv_rows]

def matrix(pose):return np.array([pose.rotate(axis) for axis in [(1,0,0),(0,1,0),(0,0,1)]]).T

def observation(t):
    index=bisect.bisect_right(times,t)-1;actual_t=times[index]
    window=[x for x in times[max(0,index-12):index+1] if x>=actual_t-.081]
    if len(window)<5:raise ValueError('Insufficient causal fit history')
    dt=np.array(window)-actual_t
    P=np.array([positions[x].position_world_m for x in window]);Rs=np.array([matrix(positions[x]) for x in window])
    pc=np.polyfit(dt,P,2);rc=np.polyfit(dt,Rs.reshape(-1,9),2)
    R=matrix(positions[actual_t]);Rdot=rc[1].reshape(3,3);Rddot=2*rc[0].reshape(3,3)
    W=Rdot@R.T;W=(W-W.T)/2
    A=Rddot@R.T;A=(A-A.T)/2
    omega=tuple(float(x) for x in [W[2,1],W[0,2],W[1,0]])
    alpha=tuple(float(x) for x in [A[2,1],A[0,2],A[1,0]])
    lever=positions[actual_t].rotate(curve.com_offset_body_m)
    com_acc=add(tuple(float(x) for x in 2*pc[0]),add(cross(alpha,lever),cross(omega,cross(omega,lever))))
    row=csv_rows[max(0,bisect.bisect_right(csv_times,actual_t)-1)]
    return AircraftObservation(actual_t,RigidMotion(positions[actual_t],tuple(float(x) for x in pc[1]),omega),
        row['armed'].lower()=='true','UNKNOWN',com_acc)

# The old run has different initial conditions. Assess geometry/kinematics
# with its mode explicitly unknown; never relabel it as the new preparation.
old_start=observation(status['approach_start_sim_t'])
entry_report=assess_entry(status['approach_start_sim_t'],old_start,curve.at(0),curve.com_offset_body_m,entry_limits)
kinematic_keys=['position_error_m','velocity_error_m_s','attitude_error_rad','angular_velocity_error_rad_s','com_acceleration_matches']
entry_report['kinematic_checks_only']={k:entry_report['checks'][k] for k in kinematic_keys}
entry_report['actual_recorded_mode_available']=False
(OUT/'legacy_initial_state_gate.json').write_text(json.dumps(entry_report,indent=2)+'\n',encoding='utf-8')

hand=ContactHandover(geometry,deck,curve.terminal_boundary.aircraft,handover_limits)
decisions=[];first_decision=None
for t in times:
    if not touch-.15<=t<=touch+.20:continue
    obs=observation(t);frame,event=history.poll(t,handover_limits.contact_age_s)
    decision=hand.step(t,obs,frame,event)
    decisions.append({'sim_t_s':t,'decision':asdict(decision)})
    if decision.stop_reference and first_decision is None:first_decision=decisions[-1]
assert first_decision is not None
replay_checks={
    'raw_first_contact_time_matches':abs(hand.first_contact['contact_sim_t_s']-touch)<1e-10,
    'first_replay_decision_stops_reference':first_decision['decision']['stop_reference'],
    'first_replay_decision_requests_land':first_decision['decision']['request_land'],
    'no_pva_after_contact':all(row['decision']['reference'] is None for row in decisions if row['sim_t_s']>=first_decision['sim_t_s']),
    'causal_precontact_observation':hand.first_contact['observation_sim_t_s']<touch,
    'precontact_envelope':all(hand.first_contact['checks'].values()),
    'no_claim_of_replayed_mode_ack_or_new_landing_success':not any(row['decision']['complete'] for row in decisions)}
replay={'passed':all(replay_checks.values()),'checks':replay_checks,'source_run':'step04/run_004',
    'new_flight_performed':False,'reference_execution_replayed':False,'recorded_flight_mode_available':False,
    'actual_land_command_sent':False,'first_contact':hand.first_contact,
    'replay_sample_time_offset_from_contact_s':first_decision['sim_t_s']-touch,'decisions':decisions,
    'scope':'Contact-handover-only counterfactual replay using causal pose-fit samples. Transport availability is assumed by simulation timestamp; offset is not measured live command latency. Native LAND response cannot be replayed.'}
(OUT/'contact_handover_replay.json').write_text(json.dumps(replay,indent=2,allow_nan=False)+'\n',encoding='utf-8')

source_paths=[cfg_path,ROOT/'landing/execution.py',ROOT/'landing/startup.py',ROOT/'landing/mavlink_reference.py',ROOT/'landing/reference_loader.py',
    ROOT/'tests/test_execution.py',Path(__file__).resolve(),RUN/'attempt_manifest.json',rs/'model_states.jsonl',rs/'cooperation.csv',rs/'cooperation_status.json',
    ROOT/'outputs/model_audit/step07/verification.json',ROOT/'outputs/model_audit/step07/boundary_densification_check.json',ROOT/'outputs/model_audit/step07/approach_plan.json']
verification={'passed':tests.wasSuccessful() and all(lead_checks.values()) and replay['passed'] and old_hashes_match,
    'simulation_only':True,'new_flight_performed':False,'tests_run':tests.testsRun,
    'lead_in_checks':lead_checks,'contact_replay_checks':replay_checks,'old_simulation_sources_preserved':old_hashes_match,
    'live_gazebo_bridge_connected':False,'live_entry_preparation_tracking_verified':False,'land_mode_ack_verified_in_new_flight':False,
    'native_land_postcontact_dynamics_verified_in_new_flight':False,'hardware_deployment_allowed':False,
    'source_sha256':{str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in source_paths}}
(OUT/'verification.json').write_text(json.dumps(verification,indent=2)+'\n',encoding='utf-8')

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
plt.rcParams['font.family']=font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name()
plt.rcParams['axes.unicode_minus']=False
combined=lead_rows+[(lead.T+curve.T*k/2000,curve.at(curve.T*k/2000)) for k in range(1,2001)]
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True,layout='constrained')
t=[x for x,_ in combined]
axes[0].plot(t,[s['motion'].pose.position_world_m[2] for _,s in combined],label='机体原点高度')
axes[0].set_ylabel('世界高度 / m')
axes[1].plot(t,[s['motion'].velocity_at_origin_world_m_s[0] for _,s in combined],label='向前速度')
axes[1].plot(t,[-s['motion'].velocity_at_origin_world_m_s[2] for _,s in combined],label='下降速度')
axes[1].set_ylabel('速度 / (m/s)')
axes[2].plot(t,[math.degrees(s['nose_up_rad']) for _,s in combined],label='后倾角；负值为前倾')
axes[2].set(ylabel='后倾角 / °',xlabel='从准备参考开始的时间 / s')
for ax in axes:
    ax.axvspan(0,lead.T,color='#ddd',alpha=.5,label='准备参考 1.50 s')
    ax.axvline(lead.T,ls=':',c='#777',label='进入 6.40 s 进场')
    ax.axvline(lead.T+curve.T,ls='--',c='#b44',label='规划首次接触；之后交给 LAND')
    ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('离线参考：悬停准备 → 平滑下降衔接 → 斜向制动进场\n1.50 + 6.40 = 7.90 s；不含起飞、等待及接触后 LAND，无新物理飞行')
fig.savefig(OUT/'execution_reference.png',dpi=160);plt.close(fig)
print(json.dumps({'passed':verification['passed'],'tests_run':tests.testsRun,'lead_extrema':lead_extrema,
    'lead_in_duration_s':lead.T,'main_duration_s':curve.T,'raw_contact_s':touch,
    'replay_decision_offset_s':replay['replay_sample_time_offset_from_contact_s'],
    'replay_first_contact_checks':hand.first_contact['checks'],'legacy_entry_residuals':entry_report['residuals']},indent=2))
assert verification['passed'],'Inspect saved checks; this is not a new flight result'
