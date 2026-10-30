"""Recompute contact kinematics from raw Gazebo poses/contact messages.

No flight commands. Distinguishes raw evidence from live-controller gates.
Finite causal fits and sampled post-contact checks are not continuous proofs.
"""
from pathlib import Path
import argparse,json,math,bisect,hashlib,sys,gzip,xml.etree.ElementTree as ET
from dataclasses import asdict
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from landing.contact_geometry import AirframeGeometry,Pose,RigidMotion,Deck,evaluate_airframe,norm,sub
from landing.execution import parse_deck_contacts,attitude_error
from landing.swept_envelope import SweptEnvelope


def stamp(message):
    s=message.get('header',{}).get('stamp',{})
    return float(s.get('sec',0))+float(s.get('nsec',0))*1e-9


def motion_fit(samples,end,span):
    window=[(t,p) for t,p in samples if end-span-1e-9<=t<=end+1e-9]
    assert len(window)>=5 and window[-1][0]-window[0][0]>=span*.6
    ts=np.array([t for t,p in window]);t=ts[-1];dt=ts-t
    P=np.array([p.position_world_m for _,p in window])
    rs=np.array([np.array([p.rotate(axis) for axis in np.eye(3)]).T for _,p in window])
    pc=np.polyfit(dt,P,2);rc=np.polyfit(dt,rs.reshape(-1,9),2)
    W=rc[1].reshape(3,3)@rs[-1].T;W=.5*(W-W.T)
    return t,RigidMotion(window[-1][1],tuple(pc[1]),(W[2,1],W[0,2],W[1,0]))


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();run=a.run
    results=run/'results';cfg=json.loads((run/'config/experiments/execution_v1.json').read_text());L=cfg['handover_limits']
    live_cfg=json.loads((run/'config/experiments/receding_live_v1.json').read_text())
    halves=tuple(live_cfg.get('deck_half_extents',(1.,.7)))
    envelope=SweptEnvelope.load(run/live_cfg['envelope_profile']) if live_cfg.get('envelope_profile') else None
    geometry=AirframeGeometry.load(run/cfg['airframe_profile'])
    mapping={cfg['leg_collision_prefix']+leg.name:leg.name for leg in geometry.legs}
    streams={'uav_pose':{},'car_pose':{}};contacts={}
    raw=results/'model_states.jsonl'
    raw_open=(lambda mode:raw.open(mode)) if raw.exists() else (lambda mode:gzip.open(raw.with_suffix('.jsonl.gz'),mode))
    incomplete_final_line=None
    stream=raw_open('rt')
    for line_number,line in enumerate(stream,1):
        try:row=json.loads(line)
        except json.JSONDecodeError:
            # A full disk or SIGTERM can leave one incomplete final write. Preserve
            # its bytes/hash and require complete evidence past the flight below.
            assert not stream.read().strip(),'Malformed raw record before end of file'
            incomplete_final_line=dict(line=line_number,characters=len(line));break
        message=row['message'];kind=row['kind']
        if kind in streams:
            name='uav' if kind=='uav_pose' else 'landing_vehicle'
            for v in message.get('pose',[]):
                if v['name']==name:
                    pose=Pose(tuple(v.get('position',{}).get(k,0.) for k in 'xyz'),tuple(v.get('orientation',{}).get(k,0.) for k in 'xyzw'))
                    t=stamp(v)
                    if t in streams[kind]:assert streams[kind][t]==pose
                    streams[kind][t]=pose
        elif kind=='deck_contact':
            frame=parse_deck_contacts(message,cfg['deck_collision'],mapping,cfg['uav_model_prefix'])
            if frame.active:contacts[frame.sim_t_s]=frame
    assert contacts,'No physical deck contact'
    u=sorted(streams['uav_pose'].items());car=sorted(streams['car_pose'].items());cts=[t for t,p in car]
    touch=min(contacts);frame=contacts[touch]
    ut,um=motion_fit(u,touch-.0125,.081);ct,cm=motion_fit(car,ut,.181)
    top=tuple(cm.pose.transform((0,0,.8))[k]+cm.velocity_at_origin_world_m_s[k]*(ut-ct) for k in range(3))
    deck=Deck(RigidMotion(Pose(top,cm.pose.orientation_xyzw),cm.velocity_at_origin_world_m_s,cm.angular_velocity_world_rad_s),halves)
    feet=evaluate_airframe(geometry,um,deck);active=[f for f in feet if f.name in frame.leg_names]
    assert active
    checks=dict(no_unexpected_collision=not frame.unexpected_uav_collisions,
        raw_precontact_fresh=0<touch-ut<=L['observation_age_s'],
        tangential_speed=max(f.tangential_speed_upper_bound_m_s for f in active)<=L['contact_tangential_speed_m_s'],
        closing_speed=max(f.closing_speed_upper_bound_m_s for f in active)<=L['contact_normal_speed_m_s'] and min(2*f.closing_speed_m_s-f.closing_speed_upper_bound_m_s for f in active)>=L['contact_minimum_closing_speed_m_s'],
        active_foot_clearance=all(abs(f.clearance_m)<=L['contact_clearance_error_m'] for f in active),
        footprint=all(min(f.whole_cylinder_footprint_margins_m)>=L['footprint_margin_m'] for f in feet),
        world_level_attitude=attitude_error(um.pose,Pose((0,0,0),(0,0,0,1)))<=L['contact_attitude_error_rad'],
        angular_speed=norm(um.angular_velocity_world_rad_s)<=L['contact_angular_speed_rad_s'])
    if envelope:
        checks['whole_aircraft_contact_margin']=envelope.clearance(um,deck)>=L['footprint_margin_m']
        checks['deck_area_ratio_1p5']=abs(4*np.prod(halves)/envelope.data['bounding_rectangle_area_m2']-1.5)<1e-10
        tree=ET.parse(run/'models/landing_vehicle/model.sdf')
        size=np.fromstring(tree.find('.//collision[@name="landing_deck"]/geometry/box/size').text,sep=' ')
        checks['physical_deck_matches_planning_config']=bool(np.allclose(size[:2],2*np.array(halves),atol=1e-12))
        requests=[json.loads(p.read_text())['request'] for p in (run/'runtime/receding_planner').glob('response_*.json')]
        checks['every_planner_request_has_actual_deck_dimensions']=bool(requests) and all(np.allclose(r['platform']['half_extents'],halves,atol=1e-12) for r in requests)
        asset_checks=[]
        for name,digest in envelope.data['source_sha256'].items():
            name=name.replace('\\','/')
            if name.startswith('work/iris_mesh_assets/'):
                source=run/'vendor/ardupilot_gazebo/models/iris_with_standoffs/meshes'/Path(name).name
            elif name.startswith('work/gimbal_mesh_assets/'):
                source=run/'vendor/ardupilot_gazebo/models/gimbal_small_3d/meshes'/Path(name).name
            else:source=run/name
            asset_checks.append(hashlib.sha256(source.read_bytes()).hexdigest()==digest)
        checks['measured_envelope_matches_physical_assets']=all(asset_checks)
    rows=[json.loads(line) for line in (results/'receding_control.jsonl').open()]
    if incomplete_final_line:
        checks['complete_pose_evidence_through_controller_end']=min(u[-1][0],car[-1][0])>=rows[-1]['sim_t_s']-.025
    disarm=next((r['sim_t_s'] for r in rows if r['sim_t_s']>touch and not r['observation']['armed']),None)
    checks['normal_land_and_disarm_observed']=disarm is not None and any(r['observation']['flight_mode']=='LAND' for r in rows if r['sim_t_s']>=touch)
    checks['three_seconds_postdisarm']=disarm is not None and rows[-1]['sim_t_s']-disarm>=L['post_disarm_dwell_s']-1e-8
    tail=[];body_margins=[]
    for t,pose in u:
        if t<touch:continue
        i=bisect.bisect_right(cts,t)-1;ct,cp=car[i];assert 0<=t-ct<=.06
        d=Deck(RigidMotion.stationary(Pose(cp.transform((0,0,.8)),cp.orientation_xyzw)),halves)
        fs=evaluate_airframe(geometry,RigidMotion.stationary(pose),d)
        tail.append((t,min(f.clearance_m for f in fs),max(f.clearance_m for f in fs),min(min(f.whole_cylinder_footprint_margins_m) for f in fs)))
        if envelope:body_margins.append((t,envelope.clearance(RigidMotion.stationary(pose),d)))
    if disarm is not None:
        support=[v for v in tail if disarm<=v[0]<=disarm+L['post_disarm_dwell_s']+1e-8]
        checks['raw_postdisarm_support']=bool(support) and min(v[1] for v in support)>=-L['support_penetration_m'] and max(v[2] for v in support)<=L['support_maximum_gap_m'] and min(v[3] for v in support)>=L['footprint_margin_m']
    if envelope:checks['whole_aircraft_all_postcontact_margin']=min(v[1] for v in body_margins)>=L['footprint_margin_m']
    status=json.loads((results/'execution_status.json').read_text());checks['live_contract_passed']=status['completed'] and not status['failures'] and not status['external_failures']
    active_times=[r['sim_t_s'] for r in rows if r.get('planner_phase') in ('ALIGNING','TRACKING','TERMINAL_TRACKING')]
    offsets=json.loads((run/cfg['airframe_profile']).read_text())['nominal_com_in_body_m']
    seq=[(t,np.array(p.transform(offsets))) for t,p in u if min(active_times)<=t<touch]
    velocities=[((bt+at)/2,(bp-ap)/(bt-at)) for (at,ap),(bt,bp) in zip(seq,seq[1:]) if bt>at]
    accelerations=[(bv-av)/(bt-at) for (at,av),(bt,bv) in zip(velocities,velocities[1:]) if bt>at]
    peaks=dict(speed_m_s=max(float(np.linalg.norm(v)) for t,v in velocities),acceleration_m_s2=max(float(np.linalg.norm(v)) for v in accelerations))
    checks['sampled_physical_speed_bound']=peaks['speed_m_s']<=live_cfg['planner_limits']['speed']
    checks['sampled_physical_acceleration_bound']=peaks['acceleration_m_s2']<=live_cfg['planner_limits']['acceleration']
    checks['original_contact_limits']=status['contact_limits']==L
    checks={k:bool(v) for k,v in checks.items()}
    raw_hash=hashlib.sha256()
    with raw_open('rb') as stream:
        for chunk in iter(lambda:stream.read(1048576),b''):raw_hash.update(chunk)
    if (results/'raw_compression.json').exists():
        assert raw_hash.hexdigest()==json.loads((results/'raw_compression.json').read_text())['sha256_uncompressed']
    report=dict(passed=all(checks.values()),checks=checks,incomplete_final_raw_record=incomplete_final_line,raw_contact_s=touch,raw_precontact_s=ut,raw_disarm_hint_s=disarm,
        sampled_flight_peaks=peaks,minimum_postcontact_whole_aircraft_margin_m=min((v[1] for v in body_margins),default=None),
        precontact_aircraft=asdict(um),precontact_deck=asdict(deck),active_feet=[asdict(f) for f in active],
        scope='Actual raw Gazebo contact/pose reconstruction; causal fit and sampled support. FCU mode/armed flags from synchronized live heartbeat, not an independent DataFlash disarm-method audit.',
        source_sha256={'model_states.jsonl':raw_hash.hexdigest(),**{n:hashlib.sha256((results/n).read_bytes()).hexdigest() for n in ['receding_control.jsonl','execution_status.json']}})
    out=results/'independent_contact_audit.json';assert not out.exists();out.write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps(dict(passed=report['passed'],checks=checks,contact=touch,disarm=disarm),indent=2))


if __name__=='__main__':main()
