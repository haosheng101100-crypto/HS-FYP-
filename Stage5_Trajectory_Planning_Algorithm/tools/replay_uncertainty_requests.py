"""Check updated candidate feasibility on preserved failed-flight inputs."""
from pathlib import Path
import json,sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'work/receding_dependencies')]
from landing.receding_landing import FlightState,PlatformState,PlannerLimits,BezierLandingPlanner
from landing.prediction_uncertainty import DelayedForecastMonitor
from landing.contact_geometry import AirframeGeometry
from landing.swept_envelope import SweptEnvelope
cfg=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())
profile=json.loads((ROOT/'config/airframes/iris_fixed_gimbal_sim.json').read_text())
planner=BezierLandingPlanner(AirframeGeometry.load(ROOT/'config/airframes/iris_fixed_gimbal_sim.json'),
    profile['nominal_com_in_body_m'],PlannerLimits(**cfg['planner_limits']),
    SweptEnvelope.load(ROOT/cfg['envelope_profile']))
c=ROOT/'outputs/uncertainty_landing_20261024/development_033/cycle_001'
monitor=DelayedForecastMonitor(**cfg['prediction_uncertainty']);snapshots={}
for line in (c/'results/receding_control.jsonl').open():
    row=json.loads(line);tube=monitor.update(PlatformState(**row['platform']))
    snapshots[round(tube.stamp,8)]=tube
results=[]
for number in (3,5,10,14):
    old=json.loads((c/f'runtime/receding_planner/response_{number:06d}.json').read_text());req=old['request']
    tube=snapshots[round(req['uncertainty']['stamp'],8)]
    plan,report=planner.search(FlightState(**req['state']),PlatformState(**req['platform']),
        req['now'],req['durations'],req['previous_arrival'],tube)
    result=dict(sequence=number,old_status=old['report']['status'],new_status=report['status'],
        candidates=[r['status'] for r in report['candidates']],search_s=report['search_s'],
        feasible_replay_only=True)
    print(json.dumps(result),flush=True);results.append(result)
(ROOT/'outputs/uncertainty_landing_20261024/failure_input_replay.json').write_text(json.dumps(results,indent=2))
