"""Replay saved failed planner inputs; not a new physical flight."""
from pathlib import Path
import sys,json
from dataclasses import replace
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'work/receding_dependencies'),str(ROOT)]
from landing.receding_landing import PlannerLimits,BezierLandingPlanner,FlightState,PlatformState
from landing.swept_envelope import SweptEnvelope
from landing.contact_geometry import AirframeGeometry
from landing.prediction_uncertainty import ForecastTube
BASE=ROOT/'outputs/uncertainty_landing_20261024'
cfg=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())
profile=ROOT/'config/airframes/iris_fixed_gimbal_sim.json';metadata=json.loads(profile.read_text())
physical=PlannerLimits(**cfg['planner_limits'])
limits=replace(physical,acceleration=physical.acceleration-cfg['nominal_acceleration_reserve_m_s2'],entry_acceleration_capture_m_s2=physical.acceleration)
planner=BezierLandingPlanner(AirframeGeometry.load(profile),metadata['nominal_com_in_body_m'],limits,SweptEnvelope.load(ROOT/cfg['envelope_profile']))
cases=[]
for session,sequence in ((57,15),(59,15),(59,17)):
    path=BASE/f'batch_{session:03d}/cycle_003/runtime/receding_planner/response_{sequence:06d}.json'
    if not path.exists():continue
    original=json.loads(path.read_text());req=original['request'];previous=req['previous_arrival']
    horizons=list(req['durations'])
    horizons+= [v+1. for v in req['durations'] if previous is None or req['now']+v+1.<=previous+3.+1e-8]
    result,report=planner.search(FlightState(**req['state']),PlatformState(**req['platform']),req['now'],horizons,previous,ForecastTube(**req['uncertainty']))
    row=dict(session=session,sequence=sequence,original_plan_present=bool(original.get('plan')),capture_and_time_candidates_available=result is not None,
        selected_duration_s=report['selected_duration_s'],candidate_statuses=[c['status'] for c in report['candidates']])
    cases.append(row);print(row,flush=True)
out=BASE/'replanning_entry_capture_replay.json'
assert not out.exists();out.write_text(json.dumps(dict(replay_only=True,physical_trial=False,cases=cases),indent=2),encoding='utf-8')
