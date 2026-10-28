"""Re-estimate preserved noisy-observation inputs, without flying or changing truth."""
from pathlib import Path
import sys,json,argparse
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'work/receding_dependencies')]
import numpy as np
from landing.platform_prediction import delayed_noisy_platform
from landing.prediction_uncertainty import DelayedForecastMonitor
from landing.observations import PoseSample
from landing.contact_geometry import Pose

cfg=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=ROOT/'outputs/uncertainty_landing_20261024/noisy_model_selection_replay.json');args=p.parse_args()
source=ROOT/'outputs/uncertainty_landing_20261024/development_037/cycle_001/results/receding_control.jsonl'
m=DelayedForecastMonitor(**cfg['prediction_uncertainty']);old=[];new=[];position_errors=[];jerks=[]
for line in source.open():
    row=json.loads(line)
    samples=[PoseSample(s['sim_t_s'],Pose(**s['pose']),s['received_monotonic_s']) for s in row['platform_fit_input']]
    state=delayed_noisy_platform(samples,row['sim_t_s'],cfg['deck_half_extents'],
        span=cfg['platform_fit_span_s'],**cfg['synthetic_platform_observation_stress'])
    tube=m.update(state)
    if not tube.ready:continue
    position_errors.append(float(np.linalg.norm(np.array(state.position)[:2]-np.array(row['measured_deck']['motion']['pose']['position_world_m'])[:2])))
    old.append(row['forecast_uncertainty']['position_radius_m'][4]);new.append(tube.at(.6)['position_m'])
    jerks.append(float(np.linalg.norm(state.jerk)))
report=dict(replay_only=True,source=str(source),old_median_forecast_reserve_m=float(np.median(old)),
    new_median_forecast_reserve_m=float(np.median(new)),new_90pct_forecast_reserve_m=float(np.quantile(new,.9)),
    current_position_rms_error_m=float(np.sqrt(np.mean(np.array(position_errors)**2))),
    cubic_fraction=float(np.mean(np.array(jerks)>1e-10)))
out=args.output
assert not out.exists();out.write_text(json.dumps(report,indent=2));print(json.dumps(report))
