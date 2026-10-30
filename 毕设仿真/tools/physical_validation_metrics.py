"""Offline audit metrics; raw poses, target geometry and fitted derivatives stay distinct."""
import math
from landing.contact_geometry import evaluate_airframe,sub,add


def touchdown_target_errors(geometry,motion,deck,boundary,terminal_deck_position):
    offset=sub(boundary['aircraft']['pose']['position_world_m'],terminal_deck_position)
    target=add(deck.motion.pose.position_world_m,offset)
    delta=deck.motion.pose.inverse_rotate(sub(motion.pose.position_world_m,target))
    feet=evaluate_airframe(geometry,motion,deck)
    planned=[f for f in feet if f.name in boundary['active_leg_names']]
    centroid=tuple(sum(f.point_world_m[k] for f in planned)/len(planned) for k in range(3))
    relative=deck.motion.pose.inverse_rotate(sub(centroid,deck.motion.pose.position_world_m))
    return {'body_target_xy_error_m':math.hypot(*delta[:2]),
            'planned_support_centroid_xy_error_m':math.hypot(*relative[:2]),
            'planned_support_leg_names':boundary['active_leg_names']}


def raw_forward_progress(rows,epoch,initial_x):
    sequence=[(t,p.position_world_m[0]) for t,p in rows if t>=epoch]
    chords=[(b[0],b[1]-a[1],(b[1]-a[1])/(b[0]-a[0]))
            for a,b in zip(sequence,sequence[1:]) if b[0]>a[0]]
    stopped=next((r for r in chords if r[0]-epoch>5.7 and abs(r[2])<.01),None)
    x_by_t=dict(sequence)
    return {'minimum_raw_forward_increment_m':min(r[1] for r in chords),
            'minimum_raw_secant_speed_m_s':min(r[2] for r in chords),
            'stop_elapsed_s':None if stopped is None else stopped[0]-epoch,
            'travel_to_stop_m':None if stopped is None else x_by_t[stopped[0]]-initial_x}
