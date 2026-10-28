"""Bounded truth-observed height correction for the SITL PVA interface.

This is an outer position feedback term, not a new contact-plane height or a
changed physical trajectory. Keep both physical and transmitted references.
The gain is a simulation controller choice, not an identified hardware gain.
"""
from dataclasses import replace
import math


def compensate_height(decision,observation,gain=2.,maximum_offset_m=.08):
    if not math.isfinite(gain) or gain<0 or not math.isfinite(maximum_offset_m) or maximum_offset_m<=0:
        raise ValueError('Finite nonnegative gain and positive correction bound required')
    if decision.stop_reference or decision.reference is None:return decision
    ref=decision.reference
    error=ref.position_world_m[2]-observation.motion.pose.position_world_m[2]
    dz=max(-maximum_offset_m,min(maximum_offset_m,gain*error))
    position=(*ref.position_world_m[:2],ref.position_world_m[2]+dz)
    return replace(decision,reference=replace(ref,position_world_m=position))


def terminal_height_feedforward(decision,elapsed_s,duration_s,offset_m=.020):
    """Smooth interface compensation calibrated in nominal moving run_001.

    A 20 mm offset is smaller than its measured ~23 mm terminal height deficit.
    This is empirical feedforward, not an identified physical constant or a
    robust feedback controller. It is inactive in preparation and lead-in.
    Command P/V/A are derivatives of the same smooth height correction.
    """
    if decision.stop_reference or decision.reference is None or decision.phase!='TRACK_REFERENCE' or elapsed_s is None:return decision
    if not all(math.isfinite(x) for x in (elapsed_s,duration_s,offset_m)) or duration_s<3 or abs(offset_m)>.03:
        raise ValueError('Finite time, duration >= 3 s and offset <= 3 cm required')
    s=max(0.,min(1.,(elapsed_s-(duration_s-3.))/2.))
    b=s**3*(10-15*s+6*s*s)
    bd=30*s*s*(1-s)**2/2.
    bdd=60*s*(1-s)*(1-2*s)/4.
    ref=decision.reference
    def shift(value,delta):return (*value[:2],value[2]+delta)
    return replace(decision,reference=replace(ref,
        position_world_m=shift(ref.position_world_m,offset_m*b),
        velocity_world_m_s=shift(ref.velocity_world_m_s,offset_m*bd),
        acceleration_world_m_s2=shift(ref.acceleration_world_m_s2,offset_m*bdd)))
