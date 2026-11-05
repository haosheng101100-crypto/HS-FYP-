"""Apply the current guard to a past successful flight; no new actuation."""
from pathlib import Path
import sys,json,argparse
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'work/receding_dependencies')]
from landing.receding_live import unpack_plan,terminal_body_prediction
from landing.receding_landing import PlatformState,FlightState
from landing.prediction_uncertainty import DelayedForecastMonitor
from landing.contact_geometry import Pose,RigidMotion
from landing.execution import AircraftObservation,BodyReference
from landing.swept_envelope import SweptEnvelope
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
cfg=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())
envelope=SweptEnvelope.load(ROOT/cfg['envelope_profile'])
offset=json.loads((ROOT/'config/airframes/iris_fixed_gimbal_sim.json').read_text())['nominal_com_in_body_m']
monitor=DelayedForecastMonitor(**cfg['prediction_uncertainty'])
plans={}
for f in (a.run/'runtime/receding_planner').glob('response_*.json'):
    d=json.loads(f.read_text())
    if d.get('plan'):plans[d['sequence']]=unpack_plan(d['plan'])
activations=[];pending=None
for e in json.loads((a.run/'results/receding_session.json').read_text())['events']:
    if e['event']=='PLAN_PENDING':pending=e['sequence']
    elif e['event']=='ACTIVATION_REJECTED':pending=None
    elif e['event']=='PLAN_ACTIVATED':activations.append((e['time'],plans[pending]));pending=None
rows=[]
for line in (a.run/'results/receding_control.jsonl').open():
    row=json.loads(line);platform=PlatformState(**row['platform']);tube=monitor.update(platform)
    if row.get('planner_phase') not in ('TRACKING','TERMINAL_TRACKING'):continue
    plan=[p for stamp,p in activations if stamp<=row['sim_t_s']+1e-8][-1]
    if not 0<plan.arrival-row['sim_t_s']<=1.5:continue
    obs_data=dict(row['observation']);motion=dict(obs_data.pop('motion'))
    obs=AircraftObservation(**obs_data,motion=RigidMotion(**dict(motion,pose=Pose(**motion['pose']))))
    ref=BodyReference(**row['decision']['reference'])
    report=terminal_body_prediction(plan,FlightState.from_observation(obs,offset),obs,platform,envelope,
        row['sim_t_s'],.02,reference_override=ref,frame_model=PlatformState(**plan.audit['platform_prediction_input']),uncertainty=tube)
    rows.append(dict(time=row['sim_t_s'],**report))
result=dict(replay_only=True,source=str(a.run),samples=len(rows),rejected=sum(not r['passed'] for r in rows),
    minimum_margin_m=min(r['predicted_whole_aircraft_margin_m'] for r in rows),
    first_rejections=[r for r in rows if not r['passed']][:3])
assert not a.output.exists();a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result))
