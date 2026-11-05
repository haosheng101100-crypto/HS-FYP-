"""Causal, delayed-error forecast tubes for receding landing.

Inspired by time-lagged adaptive conformal prediction. Sliding dependent
flight samples do NOT establish exchangeability or a probability guarantee.
The tube is an online engineering reserve, not an identified RPI set.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class ForecastTube:
    stamp: float
    horizons: tuple
    position_radius_m: tuple
    velocity_radius_m_s: tuple
    heading_radius_rad: tuple
    sample_counts: tuple
    ready: bool

    def __post_init__(self):
        n = len(self.horizons)
        if n < 2 or self.horizons[0] != 0 or not math.isfinite(self.stamp):
            raise ValueError('Invalid forecast tube time grid')
        if (any(not math.isfinite(h) for h in self.horizons)
                or any(b <= a for a, b in zip(self.horizons, self.horizons[1:]))):
            raise ValueError('Increasing forecast horizons required')
        for name in ('position_radius_m', 'velocity_radius_m_s', 'heading_radius_rad'):
            values = getattr(self, name)
            if len(values) != n or any(not math.isfinite(v) or v < 0 for v in values):
                raise ValueError('Invalid tube radii')
            if any(b < a for a, b in zip(values, values[1:])):
                raise ValueError('Tube must be horizon-monotone')
        if len(self.sample_counts) != n or any(type(v) is not int or v < 0 for v in self.sample_counts):
            raise ValueError('Invalid tube sample counts')
        if type(self.ready) is not bool:
            raise ValueError('Explicit tube readiness required')

    def at(self, horizon):
        # Upper-bin selection avoids understating an intermediate horizon.
        if not math.isfinite(horizon) or not 0 <= horizon <= self.horizons[-1]+1e-9:
            raise ValueError('Forecast tube horizon unavailable')
        i = min(int(np.searchsorted(self.horizons, horizon-1e-9)), len(self.horizons)-1)
        return dict(position_m=self.position_radius_m[i], velocity_m_s=self.velocity_radius_m_s[i],
                    heading_rad=self.heading_radius_rad[i], samples=self.sample_counts[i])


class DelayedForecastMonitor:
    """Score a forecast only after its target time has actually been observed."""
    def __init__(self, horizons=(.12, .24, .4, .6, .8, 1., 1.5, 2.),
                 issue_interval_s=.16, window_s=20., quantile=.9, minimum_samples=12,
                 reaction_horizon_s=.8):
        if (not horizons or any(not math.isfinite(h) or h <= 0 for h in horizons)
                or any(b <= a for a, b in zip(horizons, horizons[1:]))):
            raise ValueError('Increasing positive horizons required')
        if (not 0 < quantile < 1 or not math.isfinite(issue_interval_s) or issue_interval_s <= 0
                or not math.isfinite(window_s) or window_s <= 0
                or type(minimum_samples) is not int or minimum_samples < 2):
            raise ValueError('Invalid calibration design')
        if not 0 < reaction_horizon_s <= horizons[-1]:
            raise ValueError('Invalid reaction horizon')
        self.horizons = tuple(horizons); self.interval = issue_interval_s
        self.window = window_s; self.quantile = quantile; self.minimum = minimum_samples
        self.reaction = reaction_horizon_s
        self.pending = deque(); self.errors = [deque() for _ in horizons]
        self.previous = None; self.last_issue = -math.inf

    @staticmethod
    def context(state):
        # Errors from a high-jerk speed transition need not describe the
        # current constant-speed predictor. Match past prediction contexts.
        return (state.acceleration[0]/.05,state.acceleration[1]/.05,
                state.jerk[0]/.15,state.jerk[1]/.15,
                state.yaw_rate/.03,state.yaw_acceleration/.05)

    def update(self, state):
        if self.previous is not None and state.stamp <= self.previous.stamp:
            raise ValueError('Forecast calibration time must increase')
        remaining = deque()
        for target, i, predicted, context in self.pending:
            if target > state.stamp+1e-9:
                remaining.append((target, i, predicted, context)); continue
            old = self.previous
            if old is None or target < old.stamp-1e-9 or state.stamp-old.stamp > .080001:
                continue  # Never invent data across a missing observation interval.
            w = (target-old.stamp)/(state.stamp-old.stamp)
            p = (1-w)*np.asarray(old.position)+w*np.asarray(state.position)
            v = (1-w)*np.asarray(old.velocity)+w*np.asarray(state.velocity)
            dyaw = math.remainder(state.yaw-old.yaw, 2*math.pi)
            yaw = old.yaw+w*dyaw
            self.errors[i].append((state.stamp,
                float(np.linalg.norm(p[:2]-np.asarray(predicted.position)[:2])),
                float(np.linalg.norm(v[:2]-np.asarray(predicted.velocity)[:2])),
                abs(math.remainder(yaw-predicted.yaw, 2*math.pi)),context))
        self.pending = remaining
        for rows in self.errors:
            while rows and rows[0][0] < state.stamp-self.window: rows.popleft()
        if state.stamp-self.last_issue >= self.interval-1e-9:
            for i, horizon in enumerate(self.horizons):
                self.pending.append((state.stamp+horizon, i, state.predict(horizon),self.context(state)))
            self.last_issue = state.stamp
        self.previous = state
        return self.snapshot()

    def snapshot(self):
        if self.previous is None: raise ValueError('No causal platform state')
        bounds = [[.001, .003, .0005]]; counts = [0]
        current_context=self.context(self.previous)
        for horizon, rows in zip(self.horizons, self.errors):
            # Explicit bootstrap/design floors. These are not physical worst-case bounds.
            floor = [.002+.003*horizon+.0125*horizon*horizon,
                     .006+.025*horizon, .0005+.002*horizon+.003*horizon*horizon]
            matched=[r for r in rows if math.dist(current_context,r[4])<=1.]
            selected=matched if len(matched)>=4 else rows
            if selected:
                values = np.array([r[1:4] for r in selected])
                rank = min(len(selected), math.ceil((len(selected)+1)*self.quantile))-1
                estimate = np.sort(values, axis=0)[rank]
                recent = np.array([r[1:4] for r in selected if r[0] >= self.previous.stamp-.8])
                if len(recent): estimate = np.maximum(estimate, recent.max(axis=0))
                floor = np.maximum(floor, estimate).tolist()
            bounds.append(np.maximum(bounds[-1], floor).tolist()); counts.append(len(selected))
        i = int(np.searchsorted((0.,)+self.horizons, self.reaction))
        return ForecastTube(self.previous.stamp, (0.,)+self.horizons,
            tuple(r[0] for r in bounds), tuple(r[1] for r in bounds),
            tuple(r[2] for r in bounds), tuple(counts), len(self.errors[i-1]) >= self.minimum)


def reaction_reserve(tube, start, duration, reaction_horizon_s, aircraft_radius):
    """Tighten for the next control/replanning interval, not an entire long plan.

    Future contact predictions are revised online. Long plans are conditional
    candidates; this reserve cannot certify their full prediction horizon.
    """
    if tube is None: return dict(position_m=0., velocity_m_s=0., heading_rad=0., reserve_m=0., horizon_s=0.)
    age = start-tube.stamp
    if age < -1e-8: raise ValueError('Future uncertainty observations forbidden')
    horizon = age+min(duration, reaction_horizon_s)
    bound = tube.at(horizon)
    return dict(**bound, reserve_m=bound['position_m']+aircraft_radius*bound['heading_rad'],
                horizon_s=horizon, full_horizon_certified=False, statistically_certified=False)
