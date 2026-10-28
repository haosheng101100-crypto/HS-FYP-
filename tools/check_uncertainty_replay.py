"""Causal replay of archived observations; not a new physics experiment."""
from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'work/receding_dependencies')]
from landing.prediction_uncertainty import DelayedForecastMonitor, reaction_reserve
from landing.receding_landing import PlatformState
from landing.swept_envelope import SweptEnvelope

envelope=SweptEnvelope.load(ROOT/'config/airframes/iris_swept_envelope_v1.json')
results=[]
for c in sorted((ROOT/'outputs/receding_physical_20261023/batch_032').glob('cycle_*')):
    monitor=DelayedForecastMonitor();samples=[]
    for line in (c/'results/receding_control.jsonl').open(encoding='utf-8'):
        row=json.loads(line);tube=monitor.update(PlatformState(**row['platform']))
        preview=row.get('terminal_body_prediction') or {}
        if (preview.get('active') and row.get('planner_phase') in ('TRACKING','TERMINAL_TRACKING')
                and not (row.get('contact_frame') or {}).get('active')):
            reserve=reaction_reserve(tube,row['sim_t_s'],preview['remaining_s'],.8,envelope.radius)
            samples.append(dict(time=row['sim_t_s'],reserve=reserve['reserve_m'],
                margin=preview['predicted_whole_aircraft_margin_m']-reserve['reserve_m']))
    result=dict(cycle=c.name,scenario=json.loads((c/'results/execution_status.json').read_text())['scenario'],
        nominal_replay_only=True,calibration_ready=tube.ready,terminal_samples=len(samples),
        maximum_reserve_m=max((r['reserve'] for r in samples),default=None),
        minimum_adjusted_margin_m=min((r['margin'] for r in samples),default=None))
    results.append(result);print(json.dumps(result),flush=True)
out=ROOT/'outputs/uncertainty_landing_20261024';out.mkdir(exist_ok=True)
(out/'baseline_replay.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
