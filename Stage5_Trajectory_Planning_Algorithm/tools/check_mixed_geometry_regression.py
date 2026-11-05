from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'work/receding_dependencies')]
from landing.receding_landing import FlightState,PlatformState,PlannerLimits,BezierLandingPlanner
from landing.contact_geometry import AirframeGeometry
from landing.swept_envelope import SweptEnvelope
cfg=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text());profile=json.loads((ROOT/'config/airframes/iris_fixed_gimbal_sim.json').read_text())
planner=BezierLandingPlanner(AirframeGeometry.load(ROOT/'config/airframes/iris_fixed_gimbal_sim.json'),profile['nominal_com_in_body_m'],PlannerLimits(**cfg['planner_limits']),SweptEnvelope.load(ROOT/cfg['envelope_profile']))
records=[]
for number in (3,6):
 old=json.loads((ROOT/f'outputs/receding_physical_20261023/trial_017/runtime/receding_planner/response_{number:06d}.json').read_text());r=old['request'];durations=r['durations']+[max(r['durations'])+.4]
 plan,report=planner.search(FlightState(**r['state']),PlatformState(**r['platform']),r['now'],durations,r['previous_arrival'])
 result=dict(sequence=number,old_status=old['report']['status'],new_status=report['status'],candidates=[dict(duration=p['duration_s'],status=p['status'],failures=p.get('audit',{}).get('failures')) for p in report['candidates']])
 records.append(result);print(json.dumps(result))
 assert plan is not None,result
(ROOT/'outputs/receding_physical_20261023/mixed_geometry_regression.json').write_text(json.dumps(records,indent=2))
