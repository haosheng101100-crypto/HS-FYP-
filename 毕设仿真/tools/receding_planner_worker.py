"""Isolated optimization worker. Atomic JSON only; no simulator/FCU imports."""
import os
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
if os.environ.get('HS_PLANNER_DEPS'):sys.path.insert(0,os.environ['HS_PLANNER_DEPS'])
import json,time,traceback,hashlib
from dataclasses import replace,asdict
from landing.contact_geometry import AirframeGeometry
from landing.receding_landing import FlightState,PlatformState,PlannerLimits,BezierLandingPlanner
from landing.receding_live import atomic_json,pack_plan
from landing.swept_envelope import SweptEnvelope
from landing.prediction_uncertainty import ForecastTube


def main():
    os.nice(10)  # background optimization yields CPU to the control/physics loop
    directory=ROOT/'runtime/receding_planner';directory.mkdir(parents=True,exist_ok=True)
    cfg=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())
    profile_path=ROOT/'config/airframes/iris_fixed_gimbal_sim.json'
    profile=json.loads(profile_path.read_text())
    envelope=SweptEnvelope.load(ROOT/cfg['envelope_profile']) if cfg.get('envelope_profile') else None
    limits=PlannerLimits(**cfg['planner_limits'])
    reserve=cfg.get('nominal_acceleration_reserve_m_s2',0.)
    if not 0<=reserve<limits.acceleration:raise ValueError('Invalid feedback acceleration reserve')
    limits=replace(limits,acceleration=limits.acceleration-reserve,
        entry_acceleration_capture_m_s2=limits.acceleration if reserve else None)
    planner=BezierLandingPlanner(AirframeGeometry.load(profile_path),profile['nominal_com_in_body_m'],limits,envelope)
    # Compile, solve and audit once on an artificial state before READY. This
    # prepares the reusable SOCP; it neither creates nor executes a flight.
    warm,report=planner.solve(FlightState(0.,(0.,0.,4.),(0,0,0),(0,0,0)),PlatformState(0.,(2.,0.,.8),(.3,0,0),(0,0,0)),0.,10.)
    if warm is None:raise RuntimeError('Worker prewarm failed: '+str(report))
    atomic_json(directory/'warm_plan.json',pack_plan(warm))
    import cvxpy,clarabel
    atomic_json(directory/'health.json',dict(ready=True,pid=os.getpid(),solver='CLARABEL',cvxpy=cvxpy.__version__,clarabel=clarabel.__version__,nominal_planner_limits=asdict(limits),feedback_acceleration_reserve_m_s2=reserve,warmup_s=report['elapsed_s'],source_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ['landing/receding_landing.py','landing/receding_live.py','landing/prediction_uncertainty.py','landing/platform_prediction.py','landing/platform_filter.py','tools/receding_planner_worker.py']}))
    last=None
    while not (directory/'stop.request').exists():
        try:request=json.loads((directory/'request.json').read_text())
        except (FileNotFoundError,json.JSONDecodeError):time.sleep(.02);continue
        if request['sequence']==last:time.sleep(.02);continue
        last=request['sequence'];started=time.monotonic()
        try:
            uncertainty=ForecastTube(**request['uncertainty']) if request.get('uncertainty') else None
            plan,report=planner.search(FlightState(**request['state']),PlatformState(**request['platform']),request['now'],request['durations'],request['previous_arrival'],uncertainty,tracking_envelope=request.get('tracking_envelope'))
            result=dict(sequence=last,plan=pack_plan(plan) if plan else None,report=report,wall_compute_s=time.monotonic()-started)
        except Exception as exc:
            result=dict(sequence=last,plan=None,error=str(exc),traceback=traceback.format_exc(),wall_compute_s=time.monotonic()-started)
        atomic_json(directory/'response.json',result)
        atomic_json(directory/f'response_{last:06d}.json',dict(request=request,**result))


if __name__=='__main__':main()
