"""Independent raw-pose audit of a rejected deck landing and return home."""
from pathlib import Path
import argparse,bisect,gzip,hashlib,json,sys,xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from landing.contact_geometry import Pose,RigidMotion,Deck,AirframeGeometry,evaluate_airframe,norm
from landing.execution import parse_deck_contacts

def stamp(p):
    s=p.get('header',{}).get('stamp',{});return float(s.get('sec',0))+float(s.get('nsec',0))*1e-9

def main():
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);run=parser.parse_args().run;r=run/'results'
    cfg=json.loads((run/'config/experiments/execution_v1.json').read_text())
    live=json.loads((run/'config/experiments/receding_live_v1.json').read_text())
    geometry=AirframeGeometry.load(run/cfg['airframe_profile'])
    mapping={cfg['leg_collision_prefix']+leg.name:leg.name for leg in geometry.legs}
    rows=[json.loads(line) for line in (r/'receding_control.jsonl').open()]
    recovery=next(v for v in rows if v['stage']=='RECOVERING');abort=recovery['sim_t_s']
    status=json.loads((r/'execution_status.json').read_text())
    bounded_switch=status['scenario']=='switchback' and not (r/'scene_event.json').exists()
    if bounded_switch:
        trigger=next(v['sim_t_s'] for v in rows if v['stage']=='LANDING')
        event=dict(event='BOUNDED_SWITCHING',sim_t_s=trigger,
            measured_leg_gap_m=min(f['clearance_m'] for f in recovery['feet']),
            interpretation='Actual acceleration and deceleration are verified from raw displacement, without a scripted future event input.')
    else:
        event=json.loads((r/'scene_event.json').read_text());trigger=event['sim_t_s']
    returned=rows[-1];home=rows[0]['decision']['reference']['position_world_m']
    raw=r/'model_states.jsonl';opener=(lambda m:raw.open(m)) if raw.exists() else (lambda m:gzip.open(raw.with_suffix('.jsonl.gz'),m))
    poses={'uav_pose':{},'car_pose':{}};contacts=[]
    with opener('rt') as stream:
        for line in stream:
            v=json.loads(line);kind=v['kind'];msg=v['message']
            if kind in poses:
                name='uav' if kind=='uav_pose' else 'landing_vehicle'
                for p in msg.get('pose',[]):
                    if p['name']==name:
                        poses[kind][stamp(p)]=Pose(tuple(p.get('position',{}).get(k,0.) for k in 'xyz'),tuple(p.get('orientation',{}).get(k,0.) for k in 'xyzw'))
            elif kind=='deck_contact':
                f=parse_deck_contacts(msg,cfg['deck_collision'],mapping,cfg['uav_model_prefix'])
                if f.active:contacts.append(f.sim_t_s)
    car=sorted(poses['car_pose'].items());times=[t for t,p in car];clearances=[]
    for t,p in sorted(poses['uav_pose'].items()):
        if not abort<=t<=returned['sim_t_s']:continue
        i=bisect.bisect_right(times,t)-1;ct,cp=car[i];assert 0<=t-ct<=.06
        deck=Deck(RigidMotion.stationary(Pose(cp.transform((0,0,.8)),cp.orientation_xyzw)),tuple(live['deck_half_extents']))
        clearances.append((t,min(f.clearance_m for f in evaluate_airframe(geometry,RigidMotion.stationary(p),deck))))
    # Raw displacement fits, independent of commands and the online estimator.
    def speed(t):
        seq=[(s,p.position_world_m[0]) for s,p in car if t-.2<=s<=t]
        assert len(seq)>=5;return float(np.polyfit(np.array([s-t for s,x in seq]),[x for s,x in seq],1)[0])
    model=run/'models/landing_vehicle/model.sdf'
    plugin=next(p for p in ET.parse(model).findall('.//plugin') if 'DiffDrive' in p.get('name',''))
    physical_acceleration_limit=float(plugin.findtext('max_linear_acceleration'))
    raw_speed_before=speed(abort);raw_speed_after=speed(abort+.5)
    if bounded_switch:
        sample_times=np.arange(trigger+.3,abort+.5,.2)
        accelerations=np.diff([speed(float(t)) for t in sample_times])/.2
        event_motion_ok=len(accelerations)>1 and max(accelerations)>.05 and min(accelerations)<-.05
    elif event['event']=='SURGE':
        event_motion_ok=raw_speed_after-raw_speed_before>=.8*physical_acceleration_limit*.5
    else:
        assert event['event']=='BRAKE'
        event_motion_ok=(raw_speed_after<=max(0.,raw_speed_before-.8*physical_acceleration_limit*.5)+.01
            and speed(trigger-.1)>.15 and raw_speed_after<speed(trigger-.1)-.1)
    attempts=json.loads((r/'receding_attempts.json').read_text())
    expected_error='Automatic trial returned home after failed attempt'
    checks=dict(trigger_during_low_approach=0<event['measured_leg_gap_m']<=event.get('trigger_gap_m',.8),
        abort_after_trigger=abort>=trigger,no_physical_deck_contact=not contacts,
        positive_raw_clearance_through_return=bool(clearances) and min(v for t,v in clearances)>.05,
        platform_motion_matches_declared_event=event_motion_ok,
        return_confirmed=bool(attempts) and returned['stage']=='HOVER',
        home_position_reached=norm(tuple(a-b for a,b in zip(returned['observation']['motion']['pose']['position_world_m'],home)))<.08,
        home_speed_settled=norm(returned['observation']['motion']['velocity_at_origin_world_m_s'])<.08,
        original_rejection_reported=bool(attempts) and not attempts[-1]['success'] and bool(attempts[-1]['reason']),
        deck_landing_not_claimed=not status['completed'],normal_ground_land_finished=not status['armed_at_save'] and status['final_mode']=='LAND',
        no_unexpected_execution_error=status['external_failures']==[expected_error],
        raw_final_pose_on_ground=abs(max(poses['uav_pose'].items())[1].position_world_m[2]-.195)<.03)
    if bounded_switch:
        del checks['trigger_during_low_approach']
        checks['rejected_during_low_measured_descent']=(0<event['measured_leg_gap_m']<=1.
            and recovery['observation']['motion']['velocity_at_origin_world_m_s'][2]<0.)
    checks={k:bool(v) for k,v in checks.items()}
    h=hashlib.sha256()
    with opener('rb') as stream:
        for chunk in iter(lambda:stream.read(1048576),b''):h.update(chunk)
    report=dict(passed=all(checks.values()),checks=checks,trigger=event,abort_s=abort,
        minimum_exit_leg_clearance_m=min(v for t,v in clearances),rejection=attempts,
        platform_physical_acceleration_limit_m_s2=physical_acceleration_limit,
        raw_speed_at_abort_m_s=raw_speed_before,raw_speed_half_second_later_m_s=raw_speed_after,
        physical_model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        interpretation='The actual DiffDrive acceleration limit and raw platform displacement are used, rather than treating command acceleration as measured acceleration. Brake-to-zero and accelerating-away scenes have separate signed checks.',
        raw_source_sha256=h.hexdigest(),scope='One fixed-height level-deck physical simulation; sampled raw clearance, no general invariant escape guarantee. Native disarm audited separately.')
    out=r/('independent_maneuver_exit_audit.json' if bounded_switch else 'independent_exit_audit_v3.json');assert not out.exists();out.write_text(json.dumps(report,indent=2));print(json.dumps({'passed':report['passed'],'checks':checks},indent=2))

if __name__=='__main__':main()
