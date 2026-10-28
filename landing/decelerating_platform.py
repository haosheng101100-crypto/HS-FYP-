"""Braking-platform interception, with contact strictly before vehicle stop."""
from dataclasses import asdict
import math
from landing.contact_geometry import Deck, RigidMotion, Pose
from landing.accelerating_platform import (predict_deck, acceleration_boundary,
    constrained_snap_axis, check_accelerating_reference, SnapAxis, RollingAccelerationApproach)
from landing.septic_approach import PlanarSnapMatchedApproach


def require_moving_brake_window(deck, acceleration, duration):
    vx = deck.motion.velocity_at_origin_world_m_s[0]
    if not -.7 <= acceleration[0] <= -.3 or abs(acceleration[1])+abs(acceleration[2]) > 1e-8:
        raise ValueError('Outside the declared braking scene')
    if duration <= 0 or vx + acceleration[0]*duration <= .05:
        raise ValueError('Planned contact must precede vehicle stop with positive speed')


def restore_decelerating_plan(geometry, profile, report):
    m = report['deck_at_planning']['motion']
    deck = Deck(RigidMotion(Pose(**m['pose']), m['velocity_at_origin_world_m_s'],
                           m['angular_velocity_world_rad_s']), (1., .7))
    a = report['shared_acceleration_m_s2']; T = report['duration_s']
    require_moving_brake_window(deck, a, T)
    boundary = acceleration_boundary(geometry, predict_deck(deck, a, T), a)
    offset = profile['nominal_com_in_body_m']
    base = PlanarSnapMatchedApproach((0., offset[1], 3.95), (0,0,0), boundary,
        offset, 9.80665, profile['mass_kg'], T, terminal_com_snap_world_m_s4=(0,0,0))
    base.axes = (SnapAxis((0,0,0,0), report['horizontal_snap_intervals'], T), *base.axes[1:])
    return RollingAccelerationApproach(base, geometry, a, report['configuration']['design_limits'])


def plan_decelerating(geometry, profile, deck, cfg):
    if not cfg['simulation_only'] or cfg['hardware_deployment_allowed'] or cfg['lead_s'] != 0:
        raise ValueError('Simulation braking experiment requires direct hover entry')
    a = (cfg['vehicle_acceleration_m_s2'], 0., 0.); tried = []
    for T in cfg['duration_grid_s']:
        try:
            require_moving_brake_window(deck, a, T)
            boundary = acceleration_boundary(geometry, predict_deck(deck, a, T), a)
            offset = profile['nominal_com_in_body_m']
            base = PlanarSnapMatchedApproach((0., offset[1], 3.95), (0,0,0), boundary,
                offset, 9.80665, profile['mass_kg'], T, terminal_com_snap_world_m_s4=(0,0,0))
            axis = constrained_snap_axis((0,0,0,0), base.axes[0].terminal, T, cfg['design_limits'])
            base.axes = (axis, *base.axes[1:])
            check = check_accelerating_reference(base, geometry, deck, a, cfg['design_limits'], 2001)
            tried.append({'duration_s': T, 'checks': check})
            if not check['passed']: continue
            return {'simulation_only': True, 'new_physics_flight': False, 'duration_s': T,
                'lead_s': 0., 'deck_at_planning': asdict(deck), 'shared_acceleration_m_s2': a,
                'target_contact_nose_up_deg': -math.degrees(math.atan2(a[0],9.80665)),
                'terminal_boundary': asdict(boundary), 'configuration': cfg,
                'horizontal_snap_intervals': list(axis.snaps), 'reference_checks': check,
                'duration_candidates': tried, 'horizontal_optimization': axis.optimization}
        except ValueError as exc:
            tried.append({'duration_s': T, 'reason': str(exc)})
    raise ValueError('No feasible moving braking intercept: '+str(tried))


class HoverEntry:
    """Preparation hold only; never supplied as an executor lead-in."""
    T = 0.
    def __init__(self, curve): self.curve = curve
    def at(self, t): return self.curve.at(0.)
