"""Measured centre accuracy; never substitute the planned touchdown target."""
import math
from .contact_geometry import sub,evaluate_airframe

def centre_metrics(geometry,motion,deck,maximum_distance=.15):
    local=deck.motion.pose.inverse_rotate(sub(motion.pose.position_world_m,deck.motion.pose.position_world_m))
    feet=evaluate_airframe(geometry,motion,deck)
    centre=tuple(sum(f.point_world_m[k] for f in feet)/len(feet) for k in range(3))
    foot=deck.motion.pose.inverse_rotate(sub(centre,deck.motion.pose.position_world_m))
    return {'body_xy_on_deck_m':local[:2],'body_horizontal_distance_from_centre_m':math.hypot(*local[:2]),
        'all_leg_footprint_centre_xy_on_deck_m':foot[:2],'all_leg_footprint_centre_distance_m':math.hypot(*foot[:2]),
        'maximum_body_distance_m':maximum_distance,'body_centre_passed':math.hypot(*local[:2])<=maximum_distance}
