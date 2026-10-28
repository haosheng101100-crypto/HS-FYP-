"""External XY acceleration feedback for the SITL rapid-contact experiment.

Only use after confirming firmware GUID_OPTIONS bit 5 (value 32) removes the
native Guided XY position/velocity feedback. Native vertical, attitude and
motor control remain active. Parameters are simulation design choices.
"""
from dataclasses import replace
import math


class HorizontalFeedback:
    def __init__(self,position_gain=1.5,velocity_gain=2.4,acceleration_lead_s=.12,maximum_acceleration_m_s2=2.5):
        if any(not math.isfinite(v) or v<0 for v in (position_gain,velocity_gain,acceleration_lead_s)) or maximum_acceleration_m_s2<=0:
            raise ValueError('Finite nonnegative gains and positive acceleration limit required')
        self.kp=position_gain;self.kd=velocity_gain;self.lead=acceleration_lead_s;self.limit=maximum_acceleration_m_s2
        self.last_t=None;self.last_a=None;self.diagnostics=None

    def update(self,decision,observation,now):
        if decision.stop_reference or decision.reference is None:
            self.last_t=None;self.last_a=None;self.diagnostics=None
            return decision
        ref=decision.reference
        if not math.isfinite(now) or (self.last_t is not None and now<=self.last_t):raise ValueError('Increasing finite control time required')
        if not 0<=now-observation.sim_t_s<=.06:raise ValueError('Fresh causal observation required')
        dt=None if self.last_t is None else now-self.last_t
        jerk=(0.,0.) if dt is None else tuple((ref.acceleration_world_m_s2[k]-self.last_a[k])/dt for k in range(2))
        ep=tuple(ref.position_world_m[k]-observation.motion.pose.position_world_m[k] for k in range(2))
        ev=tuple(ref.velocity_world_m_s[k]-observation.motion.velocity_at_origin_world_m_s[k] for k in range(2))
        raw=tuple(ref.acceleration_world_m_s2[k]+self.lead*jerk[k]+self.kp*ep[k]+self.kd*ev[k] for k in range(2))
        length=math.hypot(*raw);factor=min(1.,self.limit/max(length,1e-12));acc=tuple(a*factor for a in raw)
        self.diagnostics={'position_error_xy_m':ep,'velocity_error_xy_m_s':ev,'body_reference_jerk_xy_m_s3':jerk,
            'raw_acceleration_xy_m_s2':raw,'sent_acceleration_xy_m_s2':acc,'saturated':factor<1.}
        self.last_t=now;self.last_a=ref.acceleration_world_m_s2
        return replace(decision,reference=replace(ref,acceleration_world_m_s2=(*acc,ref.acceleration_world_m_s2[2])))
