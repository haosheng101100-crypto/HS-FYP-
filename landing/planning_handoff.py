"""Keep reference phase anchored to the measured planning-input timestamp."""
import math

def seed_reference_at_planning_time(executor,planning_t,planning_observation,frame,event,dispatch_t):
    if not all(math.isfinite(x) for x in [planning_t,dispatch_t]):raise ValueError('Finite handoff times required')
    elapsed=dispatch_t-planning_t
    if not 0<=elapsed<=executor.handover.limits.update_gap_s:
        raise ValueError('Planning handoff exceeded original update interval')
    if executor.start_s is not None:raise ValueError('Reference can only be seeded once')
    if elapsed==0:return False
    initial=executor.step(planning_t,planning_observation,frame,event)
    if initial.phase!='TRACK_REFERENCE' or initial.stop_reference:
        raise ValueError('Planning input did not pass original reference entry')
    return True
