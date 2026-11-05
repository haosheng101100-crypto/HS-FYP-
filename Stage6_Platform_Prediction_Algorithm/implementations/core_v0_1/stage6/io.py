"""Strict streaming pose input and deterministic forecast JSONL output."""
import json
import math
from pathlib import Path
from .observation import Observation
from .predictor import VehiclePredictor


FIELDS={'stamp','received_at','position_xy','yaw','covariance'}


def predict_file(source,target,*,every=.2,offset=(0.,0.),particles_per_mode=12,horizon=2.,step_s=.1,seed=0):
    if not math.isfinite(every) or every<=0:
        raise ValueError('Prediction interval must be finite and positive')
    predictor=VehiclePredictor(offset)
    # Validate forecast parameters even for an empty or warming-up stream.
    predictor.forecast(0.,particles_per_mode=particles_per_mode,horizon=horizon,step_s=step_s,seed=seed)
    count=0;emitted=0;next_issue=None;created=False
    try:
        with Path(source).open(encoding='utf-8') as inp,Path(target).open('x',encoding='utf-8') as out:
            created=True
            for line_number,line in enumerate(inp,1):
                if not line.strip():continue
                try:
                    row=json.loads(line)
                    if not isinstance(row,dict) or set(row)!=FIELDS:
                        raise ValueError('Input requires exactly stamp,received_at,position_xy,yaw,covariance; no command/route fields')
                    observation=Observation(**row)
                    predictor.update(observation);count+=1
                    now=observation.received_at
                    if next_issue is None:next_issue=now
                    if now<next_issue-1e-9:continue
                    r=predictor.forecast(now,horizon=horizon,step_s=step_s,
                        particles_per_mode=particles_per_mode,seed=seed+emitted)
                    out.write(json.dumps(r,ensure_ascii=False,allow_nan=False)+'\n')
                    while next_issue<=now+1e-9:next_issue+=every
                    emitted+=1
                except (ValueError,TypeError,OverflowError) as exc:
                    raise ValueError(f'Input line {line_number}: {exc}') from exc
            if not count:raise ValueError('No observations in input')
    except Exception:
        # Remove only a new output created by this call, never an existing file.
        if created:Path(target).unlink(missing_ok=True)
        raise
    return dict(observations=count,predictions=emitted,output=str(target))
