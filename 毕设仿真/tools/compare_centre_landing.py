"""Compare the same independently reconstructed precontact metric."""
from pathlib import Path
import json,sys,math
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R)]
from landing.contact_geometry import Pose,sub
new_run=Path(sys.argv[1]);old_run=R/'outputs/model_audit/step24/run_005'
def measured(run):
 q=json.loads((run/'landing_analysis/verification.json').read_text());assert q['passed']
 p=q['precontact'];u=p['aircraft']['motion']['pose'];d=p['deck']['motion']['pose']
 deck=Pose(tuple(d['position_world_m']),tuple(d['orientation_xyzw']))
 local=deck.inverse_rotate(sub(u['position_world_m'],d['position_world_m']))
 return {'run':str(run),'body_xy_on_deck_m':local[:2],'body_distance_from_centre_m':math.hypot(*local[:2]),
  'relative_foot_horizontal_speed_m_s':p['relative_foot_tangent_upper_m_s'],
  'attitude_error_deg':p['target_attitude_error_deg'],'brake_to_contact_s':q['actual_brake_to_contact_s'],
  'source_raw_sha256':q['source_sha256']['model_states.jsonl']}
old=measured(old_run);new=measured(new_run)
result={'baseline':old,'centred':new,'centre_error_reduction_m':old['body_distance_from_centre_m']-new['body_distance_from_centre_m'],
 'both_physical_runs':True,'same_declared_initial_scene':True,'single_case_comparison_only':True}
target=new_run.parent/'居中前后实测对比.json';assert not target.exists();target.write_text(json.dumps(result,indent=2,ensure_ascii=False));print(json.dumps(result,ensure_ascii=False))
