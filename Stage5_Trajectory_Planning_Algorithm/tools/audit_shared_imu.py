"""Independent causal packet/source check for the physical platform IMU."""
import json,bisect,math

def imu_evidence(run,car):
    from analyze_moving_contact import stamp,fit
    res=run/'results';raw={}
    for line in (res/'model_states.jsonl').open():
        row=json.loads(line)
        if row['kind']=='vehicle_imu':
            m=row['message'];raw[round(stamp(m),9)]=tuple(m.get('linear_acceleration',{}).get(k,0.) for k in 'xyz')
    packets=json.loads((res/'vehicle_imu_shared_packets.json').read_text())
    shared=json.loads((res/'shared_platform_states.json').read_text())
    times=[t for t,p in car];source_error=[];rotation_error=[]
    for p in packets:
        t=p['sample_sim_t_s'];a=p['specific_force_sensor_m_s2'];source=raw.get(round(t,9))
        if source is None:raise ValueError('Shared IMU packet absent from independent raw recorder')
        source_error.append(max(abs(x-y) for x,y in zip(source,a)))
        i=bisect.bisect_right(times,t)-1
        if i<0:continue
        rotation_error.append(abs(car[i][1].rotate(a)[0]-p['forward_acceleration_world_m_s2']))
    packet_map={round(p['sample_sim_t_s'],9):p for p in packets}
    ages=[];query_error=[]
    for p in shared:
        t=p['query_sim_t_s'];at=p['sample_sim_t_s'];ages.append(t-at)
        source=packet_map.get(round(at,9))
        if source is None:raise ValueError('Query source absent from shared IMU packets')
        query_error.append(abs(p['acceleration_m_s2'][0]-source['forward_acceleration_world_m_s2']))
    s=json.loads((res/'execution_status.json').read_text());epoch=s.get('braking_start_sim_t_s');velocity_errors=[]
    if epoch is None:epoch=json.loads((res/('scenario_acceleration_event.json' if s.get('accelerating_decision_trial') else 'scenario_brake_event.json')).read_text())['command_sim_t_s']
    for p in shared:
        t=p['query_sim_t_s']
        # Exclude derivative windows crossing either discontinuity; compare
        # against independent raw vehicle pose fits, not integrated packets.
        if epoch+.2<t<epoch+1.8:
            o,f=fit(car,t,True)
            v=o.motion.velocity_at_origin_world_m_s[0]+f['body_acceleration_world_m_s2'][0]*(t-o.sim_t_s)
            velocity_errors.append(abs(p['velocity_m_s'][0]-v))
    checks={'imu_packets_match_physical_raw':bool(source_error) and max(source_error)<1e-12,
        'imu_world_rotation_matches_car':bool(rotation_error) and max(rotation_error)<1e-5,
        'all_imu_queries_causal_and_fresh':bool(ages) and min(ages)>=-1e-9 and max(ages)<=.05+1e-9,
        'queries_use_recorded_acceleration':bool(query_error) and max(query_error)<1e-12,
        'integrated_braking_velocity_matches_raw':bool(velocity_errors) and max(velocity_errors)<.03}
    return {'checks':checks,'passed':all(checks.values()),'shared_packet_count':len(packets),
        'maximum_query_age_s':max(ages),'raw_source_maximum_error_m_s2':max(source_error),
        'maximum_independent_velocity_error_m_s':max(velocity_errors),
        'sensor_noise_added':False,'real_radio_validated':False}
