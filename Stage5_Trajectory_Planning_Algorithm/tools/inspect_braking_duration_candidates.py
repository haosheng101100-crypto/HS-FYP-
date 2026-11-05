"""Offline height-translation inspection, explicitly not new physical samples."""
from pathlib import Path
import json,sys,math,hashlib
from dataclasses import replace
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R),str(R/'tools')]
from verify_unified_decision_interface import observation,deck
from landing.reference_loader import load_verified_reference
from landing.attitude_braking import attitude_capture_plan
from landing.contact_geometry import Pose,RigidMotion,Deck,evaluate_airframe

source=R/'outputs/model_audit/step28/run_003/results/execution_status.json'
s=json.loads(source.read_text());o=observation(s['decision_observation']);d=deck(s['decision_deck'])
a=s['measured_braking_detection']['measured_acceleration_m_s2'];_,g,_,profile=load_verified_reference(R)
gap=min(f.clearance_m for f in evaluate_airframe(g,o.motion,d));cases=[]
for target_gap in [2.,1.5,1.]:
 p=o.motion.pose.position_world_m
 synthetic=replace(o,motion=replace(o.motion,pose=Pose((p[0],p[1],p[2]+target_gap-gap),o.motion.pose.orientation_xyzw)))
 records=[]
 for i in range(25,45):
  T=i/10
  try:
   curve,terminal,plan=attitude_capture_plan(g,profile,synthetic,d,a,duration=T)
   low=math.inf;margin=math.inf;vx=d.motion.velocity_at_origin_world_m_s[0];stop=-vx/a[0];dp=d.motion.pose.position_world_m
   for j in range(501):
    t=T*j/500;u=min(t,stop);px=dp[0]+vx*u+.5*a[0]*u*u
    moving=Deck(RigidMotion.stationary(Pose((px,dp[1],dp[2]),d.motion.pose.orientation_xyzw)),d.half_extents_m)
    feet=evaluate_airframe(g,curve.at(t)['motion'],moving)
    if j<500:low=min(low,min(f.clearance_m for f in feet))
    margin=min(margin,min(min(f.whole_cylinder_footprint_margins_m) for f in feet))
   records.append({'duration_s':T,'legacy_reference_checks':True,'no_sampled_early_penetration':low>=-1e-7,
                   'whole_footprint_coverage':margin>=.02,'minimum_preterminal_foot_gap_m':low,'minimum_footprint_margin_m':margin,
                   'horizontal_stop_s':plan['horizontal_braking_duration_s'],'physical_validation':False})
  except ValueError as exc:records.append({'duration_s':T,'legacy_reference_checks':False,'reason':str(exc),'physical_validation':False})
 cases.append({'synthetic_foot_gap_m':target_gap,'unchanged_initial_downward_speed_m_s':-o.motion.velocity_at_origin_world_m_s[2],
               'reference_eligible_durations_s':[r['duration_s'] for r in records if r['legacy_reference_checks'] and r['no_sampled_early_penetration'] and r['whole_footprint_coverage']],
               'candidates':records})
out=R/'outputs/model_audit/step30';out.mkdir(exist_ok=True);target=out/'offline_duration_inspection.json';assert not target.exists()
result={'new_physical_samples':0,'synthetic_state_vertical_translation':True,'source_status_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
 'scope':'Translate only z of one actual braking decision state; retain its measured P/V/A and controller. This isolates reference geometry mathematically, not actual different-height normal-descent physics or a certified executable candidate.',
 'cases':cases}
target.write_text(json.dumps(result,indent=2));print(json.dumps([{k:v for k,v in r.items() if k!='candidates'} for r in cases]))
