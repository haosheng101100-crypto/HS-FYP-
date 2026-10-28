"""Reproducible numerical preparation only; never starts a flight."""
import argparse
import json
from pathlib import Path
import sys
import math
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
deps = ROOT/'work/receding_dependencies'
if deps.exists(): sys.path.insert(0, str(deps))
import numpy as np
from landing.contact_geometry import AirframeGeometry
from landing.receding_landing import FlightState, PlatformState, PlannerLimits, BezierLandingPlanner


def build():
    config = json.loads((ROOT/'config/experiments/receding_landing_v1.json').read_text(encoding='utf-8'))
    profile = json.loads((ROOT/'config/airframes/iris_fixed_gimbal_sim.json').read_text(encoding='utf-8'))
    return BezierLandingPlanner(AirframeGeometry.load(ROOT/'config/airframes/iris_fixed_gimbal_sim.json'),
        profile['nominal_com_in_body_m'], PlannerLimits(**config['limits']))


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--output', type=Path,
        default=ROOT/'outputs/receding_landing_20261023'); args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    planner = build(); results = []
    for name, velocity, acceleration, yaw_rate in [
        ('stationary', (0.,0.,0.), (0.,0.,0.), 0.),
        ('constant_velocity', (.6,.2,0.), (0.,0.,0.), 0.),
        ('accelerating', (.4,0.,0.), (.15,.05,0.), 0.),
        ('braking', (1.,0.,0.), (-.15,0.,0.), 0.),
        ('turning_local_prediction', (.6,0.,0.), (0.,.15,0.), .1),
        ('fast_deck_rotation_rejected', (.6,0.,0.), (0.,.15,0.), .25),
        ('area_ratio_1p5_declared_disk', (.3,0.,0.), (0.,0.,0.), 0.)]:
        state = FlightState(0., (0.,0.,3.), (0.,0.,0.), (0.,0.,0.))
        deck = PlatformState(0., (1.,.2,.8), velocity, acceleration, yaw_rate=yaw_rate)
        if name == 'area_ratio_1p5_declared_disk':
            from dataclasses import replace
            half = math.sqrt(1.5*math.pi*planner.limits.footprint_radius**2)/2
            deck = replace(deck, half_extents=(half,half))
        plan, report = planner.search(state, deck, 0., (4.,5.,6.,7.))
        expected = 'NO_VALIDATED_CANDIDATE' if name == 'fast_deck_rotation_rejected' else 'PLAN_AVAILABLE'
        report.update(scenario=name, physics_validated=False, expected_status=expected,
            expectation_met=report['status']==expected)
        if plan:
            samples = []
            for t in np.linspace(0., plan.duration, 301):
                s = plan.at(float(t)); samples.append(dict(t=float(t), **{k:v.tolist() for k,v in s.items()},
                    platform_position=list(deck.predict(float(t)).position)))
            (args.output/(name+'_trajectory.json')).write_text(json.dumps(samples), encoding='utf-8')
        results.append(report)
        print(name, report['status'], report['selected_duration_s'], round(report['search_s'],3), flush=True)
    (args.output/'numerical_results.json').write_text(json.dumps(dict(
        scope='Nominal kinematic SOCP preparation, not Gazebo or contact dynamics',
        results=results, all_numerical_expectations_met=all(r['expectation_met'] for r in results)),
        ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


if __name__ == '__main__': main()
