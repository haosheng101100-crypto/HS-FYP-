"""Quick post-contact audit of complete static-deck flights, not a safety certification."""
from pathlib import Path
import argparse,hashlib,json,math,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'work/plot_dependencies'))
from landing.contact_geometry import Pose,RigidMotion,evaluate_airframe
from landing.reference_loader import load_verified_reference

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def stamp(msg):
    s=msg.get('header',{}).get('stamp',{});return float(s.get('sec',0))+float(s.get('nsec',0))*1e-9
def analyze(run):
    status=json.loads((run/'results/execution_status.json').read_text())
    previous=json.loads((run/'analysis/flight_verification.json').read_text())
    curve,geometry,deck,profile=load_verified_reference(run)
    for name,digest in previous['source_sha256'].items():assert sha(run/name.replace('\\','/'))==digest
    poses={};contact_frames={};unexpected=set()
    with (run/'results/model_states.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line);msg=row['message']
            if row['kind']=='uav_pose':
                for p in msg.get('pose',[]):
                    if p['name']=='uav':poses[stamp(p)]=Pose(tuple(p.get('position',{}).get(k,0.) for k in 'xyz'),tuple(p['orientation'].get(k,0.) for k in ['x','y','z','w']))
            if row['kind']!='deck_contact':continue
            hits=[];forces=[]
            for c in msg.get('contact',[]):
                names=[c.get(k,{}).get('name','') for k in ['collision1','collision2']]
                uav=[i for i,n in enumerate(names) if n.startswith('uav::')]
                if not uav:continue
                i=uav[0]
                if not any(names[i].endswith(leg.name) for leg in geometry.legs):unexpected.add(names[i])
                hits.append(c)
                for wrench in c.get('wrench',[]):
                    f=wrench.get(f'body_{i+1}_wrench',{}).get('force',{})
                    forces.append(tuple(float(f.get(k,0.)) for k in 'xyz'))
            if hits:
                net=tuple(sum(v[k] for v in forces) for k in range(3))
                contact_frames[stamp(msg)]={'hits':hits,'net_force_magnitude_N':math.sqrt(sum(x*x for x in net)),'force_count':len(forces),
                                           'point_force_magnitude_sum_N':sum(math.sqrt(sum(x*x for x in v)) for v in forces)}
    assert poses and contact_frames
    touch=min(contact_frames);start=status['main_approach_start_sim_t_s'];assert abs(touch-previous['precontact']['raw_first_contact_sim_t_s'])<1e-9
    before=max(t for t in poses if t<=touch);p0=poses[before]
    after=sorted(t for t in poses if t>=touch);assert after[-1]-touch>3
    last=poses[after[-1]];xy=[math.hypot(poses[t].position_world_m[0]-p0.position_world_m[0],poses[t].position_world_m[1]-p0.position_world_m[1]) for t in after]
    tilts=[math.degrees(math.acos(max(-1,min(1,poses[t].rotate((0,0,1))[2])))) for t in after]
    margins=[];gaps=[]
    for t in after:
        f=evaluate_airframe(geometry,RigidMotion.stationary(poses[t]),deck)
        margins.append(min(min(x.whole_cylinder_footprint_margins_m) for x in f));gaps.append(min(x.clearance_m for x in f))
    force_times=sorted(contact_frames);peak_t=max(force_times,key=lambda t:contact_frames[t]['net_force_magnitude_N'])
    diffs=[b-a for a,b in zip(force_times,force_times[1:])]
    force_integral=sum(.5*(contact_frames[a]['net_force_magnitude_N']+contact_frames[b]['net_force_magnitude_N'])*(b-a) for a,b in zip(force_times,force_times[1:]) if a<touch+.5 and b<=touch+.5)
    tail_force=[contact_frames[t]['net_force_magnitude_N'] for t in force_times if t>=force_times[-1]-1.]
    tail_pose=[poses[t] for t in after if t>=after[-1]-1.]
    tail_diameter=max(math.hypot(a.position_world_m[0]-b.position_world_m[0],a.position_world_m[1]-b.position_world_m[1]) for a in tail_pose for b in tail_pose)
    first_points=[p for c in contact_frames[touch]['hits'] for p in c.get('position',[])]
    inside=all(abs(p.get('x',0.)-deck.motion.pose.position_world_m[0])<=deck.half_extents_m[0] and abs(p.get('y',0.)-deck.motion.pose.position_world_m[1])<=deck.half_extents_m[1] for p in first_points)
    checks={'first_contact_points_on_nominal_deck':inside,'no_unexpected_uav_deck_collision':not unexpected,
            'all_sampled_postcontact_footprints_above_original_2cm_margin':min(margins)>=.02,
            'no_sampled_inversion':max(tilts)<90.,'native_land_and_normal_disarm':previous['checks']['land_mode_observed_after_contact'] and status['normal_disarm_observed'] and not status['armed_at_save'],
            'full_support_and_postdisarm_checks_completed':previous['checks']['executor_completed_support_and_post_disarm_checks']}
    return {'run':str(run),'flow_and_gross_support_checks_passed':all(checks.values()),'checks':checks,'original_precision_goal_passed':previous['passed'],
            'approach_to_first_contact_s':touch-start,'contact_sim_t_s':touch,'postcontact_pose_duration_s':after[-1]-touch,
            'precontact_pose_anchor_lead_s':touch-before,'maximum_body_horizontal_excursion_since_precontact_anchor_m':max(xy),
            'final_body_horizontal_displacement_since_precontact_anchor_m':math.hypot(last.position_world_m[0]-p0.position_world_m[0],last.position_world_m[1]-p0.position_world_m[1]),
            'last_one_second_body_xy_diameter_m':tail_diameter,'maximum_sampled_postcontact_tilt_deg':max(tilts),
            'minimum_sampled_postcontact_footprint_margin_m':min(margins),'maximum_sampled_lowest_foot_clearance_m':max(gaps),
            'maximum_raw_contact_depth_m':max(float(d) for frame in contact_frames.values() for hit in frame['hits'] for d in hit.get('depth',[0.])),
            'contact_load':{'peak_solver_net_force_magnitude_N':contact_frames[peak_t]['net_force_magnitude_N'],
                'peak_time_after_first_contact_s':peak_t-touch,'peak_to_nominal_weight_ratio':contact_frames[peak_t]['net_force_magnitude_N']/(profile['mass_kg']*curve.gravity),
                'first_0_5s_integral_of_force_magnitude_N_s':force_integral,'last_one_second_mean_force_magnitude_N':sum(tail_force)/len(tail_force),
                'nominal_weight_N':profile['mass_kg']*curve.gravity,'minimum_contact_message_spacing_s':min(diffs),'maximum_contact_message_spacing_s':max(diffs),
                'wrench_report_frames':len(force_times),'missing_force_frames':sum(contact_frames[t]['force_count']==0 for t in force_times),
                'interpretation':'Magnitude of Gazebo/DART contact-solver wrench summed on UAV collision side; no assumption about wrench normal sign. Not a measured hardware load or structural safety limit. Scalar load integral is not vector net impulse.'},
            'source_sha256':{'model_states.jsonl':sha(run/'results/model_states.jsonl'),'flight_verification.json':sha(run/'analysis/flight_verification.json')},
            'limitations':['Body excursion includes normal leveling and settling, and is NOT a direct measurement of material foot slip.','Positions and footprint margins refer to nominal stationary deck. Vehicle is dynamic under zero velocity command; online controller checks deck top within 1 cm and orientation within 0.01 rad, but full car trajectory was not logged by this logger.','100 Hz poses can miss faster transients; raw contact loads depend on physics step, contact solver and friction. No convergence or hardware contact-capacity test.']}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run-dir',action='append',type=Path,required=True);p.add_argument('--out-dir',type=Path,required=True);a=p.parse_args()
    rows=[analyze(r.resolve()) for r in a.run_dir];a.out_dir.mkdir(parents=True,exist_ok=False)
    report={'sample_size':len(rows),'new_flight_count':sum('step12' in str(r) for r in a.run_dir),'simulation_only':True,'all_gross_support_checks_passed':all(r['flow_and_gross_support_checks_passed'] for r in rows),
            'all_original_precision_goals_passed':all(r['original_precision_goal_passed'] for r in rows),'runs':rows,'statistical_success_rate_estimated':False,'hardware_safety_certified':False,
            'sampling_scope':'Existing complete no-camera runs plus one new unchanged-config no-camera run. Camera-enabled observation-stall attempts are excluded from landing-repeatability subset, not erased or counted as overall system successes.',
            'analysis_script_sha256':sha(Path(__file__))}
    (a.out_dir/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
