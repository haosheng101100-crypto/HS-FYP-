"""Read-only geometry audit for a pose/twist trajectory supplied by a planner.

This does not integrate forces, solve impact, or certify dynamic feasibility.
Sampling can miss an excursion between samples; the bracket refinement only
improves the time estimate for a crossing that has already been detected.
"""
from dataclasses import asdict
import math
from .contact_geometry import evaluate_airframe


def audit_reference(geometry, aircraft_at, deck_at, duration_s, sample_count=301):
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError('Duration must be positive and finite')
    if not isinstance(sample_count, int) or sample_count < 2:
        raise ValueError('At least two samples are required')

    def evaluate(t):
        aircraft, deck = aircraft_at(t), deck_at(t)
        feet = evaluate_airframe(geometry, aircraft, deck)
        return aircraft, feet

    previous = None
    first = None
    samples = []
    for tick in range(sample_count):
        t = duration_s*tick/(sample_count-1)
        aircraft, feet = evaluate(t)
        rows = {f.name:f for f in feet}
        samples.append({'t_s':t,'minimum_plane_clearance_m':feet[0].clearance_m,
                        'lowest_leg':feet[0].name})
        if first is None:
            candidates = [f for f in feet if f.clearance_m <= 0 and f.projected_point_inside_deck]
            if candidates:
                upper=t
                lower=previous[0] if previous else t
                eligible_names={f.name for f in candidates}
                # Refine only a genuine positive-to-negative crossing already
                # bracketed for an interior foot; never extrapolate through an edge.
                bracket_ok=previous is not None and all(previous[1][name].clearance_m>0 and previous[1][name].projected_point_inside_deck for name in eligible_names)
                if bracket_ok:
                    for _ in range(35):
                        middle=(lower+upper)/2
                        _, mid_feet=evaluate(middle)
                        minimum=min(f.clearance_m for f in mid_feet if f.name in eligible_names)
                        if minimum<=0:upper=middle
                        else:lower=middle
                    t_contact=(lower+upper)/2
                    motion, contact_feet=evaluate(t_contact)
                else:
                    t_contact=t
                    motion, contact_feet=aircraft,feet
                lowest=min((f for f in contact_feet if f.projected_point_inside_deck),key=lambda f:f.clearance_m)
                first={'time_s':t_contact,'bracket_refined':bracket_ok,
                       'sample_bracket_s':[previous[0] if previous else t,t],
                       'lowest_leg':lowest.name,
                       'candidate_legs':[f.name for f in contact_feet if f.clearance_m<=1e-8 and f.projected_point_inside_deck],
                       'aircraft_origin_world_m':motion.pose.position_world_m,
                       'angular_velocity_world_rad_s':motion.angular_velocity_world_rad_s,
                       'feet':[asdict(f) for f in contact_feet]}
        previous=(t,rows)
    end_motion,end_feet=evaluate(duration_s)
    return {'scope':'sampled rigid-geometry audit only', 'sample_count':sample_count,
            'duration_s':duration_s,'first_plane_touch':first,
            'terminal_origin_world_m':end_motion.pose.position_world_m,
            'terminal_angular_velocity_world_rad_s':end_motion.angular_velocity_world_rad_s,
            'terminal_feet':[asdict(f) for f in end_feet], 'samples':samples,
            'limitations':['No dynamics or contact-force validation.',
                           'Only the interior plane is modeled; deck-side/edge collisions and body meshes are excluded.',
                           'A trajectory may leave constraints between samples.']}
