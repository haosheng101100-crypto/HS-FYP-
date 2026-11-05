"""Independent failed-flight audit: raw events/poses plus original FCU records."""
import sys,json,hashlib,gzip,shutil,math,bisect
from pathlib import Path
from dataclasses import asdict
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R),str(R/'tools')]
from analyze_moving_rapid import raw_data
from landing.outcome_audit import classify_disarm
from landing.reference_loader import load_verified_reference
from landing.contact_geometry import Pose,RigidMotion,Deck,evaluate_airframe

def audit(run):
    res=run/'results';out=run/'self_audit';out.mkdir(exist_ok=True)
    path=res/'model_states.jsonl'
    if not path.exists():
        with gzip.open(res/'model_states.jsonl.gz','rb') as a,path.open('xb') as b:shutil.copyfileobj(a,b)
    post=json.loads((res/'postflight_checks.json').read_text())
    assert hashlib.sha256(path.read_bytes()).hexdigest()==post['raw_sha256']
    status=json.loads((res/'execution_status.json').read_text())
    assert not status['completed'],'Use full touchdown/support audit for an online completed landing'
    rows=[json.loads(l) for l in (res/'execution_feedback.jsonl').read_text().splitlines()]
    df=[json.loads(l) for l in (res/'dataflash_final/control_messages.jsonl').read_text().splitlines()]
    arm=[r for r in df if r.get('mavpackettype')=='ARM']
    events=status.get('events',[])+[r.get('Message','') for r in df if r.get('mavpackettype')=='MSG']
    stop=classify_disarm(status['armed_at_save'],events,status['final_mode'],arm)
    raw,contacts=raw_data(run);_,g,_,profile=load_verified_reference(R)
    car=raw['car_pose'];ts=[t for t,p in car]
    def deck(t):
        i=bisect.bisect_right(ts,t)-1
        if i<0:raise ValueError('No prior car pose')
        if i==len(car)-1:return Deck(RigidMotion.stationary(Pose(car[i][1].transform((0,0,.8)),car[i][1].orientation_xyzw)),(1,.7))
        at,p=car[i];bt,q=car[i+1];f=(t-at)/(bt-at)
        pos=tuple(a+(b-a)*f for a,b in zip(p.position_world_m,q.position_world_m))
        return Deck(RigidMotion.stationary(Pose((pos[0],pos[1],pos[2]+.8),p.orientation_xyzw)),(1,.7))
    detection=status.get('measured_braking_detection',{}).get('sim_t_s')
    flight=[r for r in rows if r['stage']=='DECK_LANDING']
    recovery=status.get('recovery_rows',[])
    last=flight[-1] if flight else None
    # Raw motion after the last tracked reference, including any failed exit.
    tail=[(t,p) for t,p in raw['uav_pose'] if last is not None and t>=last['sim_t_s'] and t<=car[-1][0]]
    tail_tilt=max((math.degrees(math.acos(max(-1.,min(1.,p.rotate((0,0,1))[2])))) for t,p in tail),default=None)
    raw_recovery=[]
    if recovery:
        begin=recovery[0]['sim_t_s'];end=recovery[-1]['sim_t_s']
        for t,p in raw['uav_pose']:
            if begin<=t<=end and t<=car[-1][0]:
                feet=evaluate_airframe(g,RigidMotion.stationary(p),deck(t))
                raw_recovery.append({'t':t,'height_m':p.position_world_m[2],'minimum_foot_gap_m':feet[0].clearance_m,
                    'tilt_deg':math.degrees(math.acos(max(-1.,min(1.,p.rotate((0,0,1))[2]))))})
    recovery_checks={'clearance_stage_completed':bool(status.get('recovery_clearance_completed')),
        'no_crash':not stop['crash_detected'],'normal_land_disarm':stop['normal_disarm_observed'],
        'no_forced_disarm':not stop['forced_disarm_detected'],
        'clear_before_land':bool(recovery) and recovery[-1]['checks']['ready_for_native_land'],
        'no_deck_contact_during_recovery':bool(recovery) and not any(t>=recovery[0]['sim_t_s'] for t in contacts),
        'raw_clearance_positive':bool(raw_recovery) and min(r['minimum_foot_gap_m'] for r in raw_recovery)>.3,
        'raw_upright_recovery':bool(raw_recovery) and max(r['tilt_deg'] for r in raw_recovery)<=25.,
        'remaining_owned_processes_zero':not post['remaining_owned_processes']}
    if recovery:
        final=recovery[-1]['observation']['motion'];d=recovery[-1]['deck']['motion']['pose']['position_world_m']
        recovery_checks['independent_land_entry']=final['pose']['position_world_m'][0]-d[0]>=2.5 and abs(final['pose']['position_world_m'][2]-3.2)<.12 and math.sqrt(sum(a*a for a in final['velocity_at_origin_world_m_s']))<.12
    result={'simulation_only':True,'landing_task_passed':False,'online_reported_completed':status['completed'],
        'stop_classification':stop,'raw_status_normal_disarm_flag':status.get('normal_disarm_observed'),
        'old_normal_disarm_flag_contradicted':bool(status.get('normal_disarm_observed')) and not stop['normal_disarm_observed'],
        'physical_deck_contact_count':len(contacts),'maximum_raw_tail_tilt_deg':tail_tilt,
        'external_failures':status.get('external_failures',[]),'normal_land_recovery_passed':all(recovery_checks.values()),
        'recovery_checks':recovery_checks,
        'recovery_raw_minimum_foot_gap_m':min((r['minimum_foot_gap_m'] for r in raw_recovery),default=None),
        'recovery_raw_maximum_tilt_deg':max((r['tilt_deg'] for r in raw_recovery),default=None),
        'recovery_samples':len(recovery),'arm_records':arm,
        'windows_tests_at_launch':None,'ubuntu_tests_at_launch':post['ubuntu_preflight_tests_passed'],
        'source_sha256':{n:hashlib.sha256((res/n).read_bytes()).hexdigest() for n in ['model_states.jsonl','execution_status.json','execution_feedback.jsonl','attempt_manifest.json']}}
    if last:result['last_tracked_sample']={'time_after_detection_s':last['sim_t_s']-detection,
        'position_error_m':last['tracking_position_error_m'],'speed_error_m_s':last['tracking_speed_error_m_s'],
        'actual_acceleration_m_s2':last['fit']['body_acceleration_world_m_s2'],
        'actual_velocity_m_s':last['observation']['motion']['velocity_at_origin_world_m_s'],
        'actual_tilt_deg':last['tilt_deg'],'command':last.get('attitude_command')}
    target=out/'verification.json';assert not target.exists(),'Preserve completed audit';target.write_text(json.dumps(result,indent=2))
    print(json.dumps(result,ensure_ascii=False))
if __name__=='__main__':audit(Path(sys.argv[1]))
