"""Tracking to capture transition; separate from post-contact LAND authority."""
from .horizontal_feedback import HorizontalFeedback
from dataclasses import replace
import math


class CaptureAwareFeedback(HorizontalFeedback):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.nominal_kp=self.kp;self.nominal_lead=self.lead;self.deadline=None;self.approach=None;self.start_s=None
        self.preview_correction_weight=.5

    def update(self,decision,observation,now):
        weight=0.
        if self.deadline is not None and decision.phase=='TRACK_REFERENCE':
            u=max(0.,min(1.,1.-(self.deadline-now)/1.2))
            weight=u*u*(3.-2.*u)
        # Keep velocity correction, smoothly transfer priority from exact COM
        # position tracking to acceleration/velocity matching at the deck.
        self.kp=self.nominal_kp*(1-weight)+.2*weight
        # Preserve actuator-lag feedforward through the velocity capture. Removing
        # it during braking causes a relative-speed deficit at early contact.
        self.lead=self.nominal_lead
        result=super().update(decision,observation,now)
        if self.diagnostics is not None:
            self.diagnostics.update(capture_transition_weight=weight,effective_position_gain=self.kp,effective_acceleration_lead_s=self.lead)
            if self.approach is not None and self.start_s is not None and decision.phase=='TRACK_REFERENCE':
                future_t=min(self.approach.T,max(0.,now-self.start_s)+self.nominal_lead)
                future=self.approach.at(future_t)['body_acceleration_world_m_s2']
                ref=decision.reference;d=self.diagnostics
                # Curvature correction between a first-order lag inverse and
                # exact preview of OUR OWN reference; no continuation past T.
                raw=tuple(d['raw_acceleration_xy_m_s2'][k]+future[k]-ref.acceleration_world_m_s2[k]-self.lead*d['body_reference_jerk_xy_m_s3'][k] for k in range(2))
                raw=tuple((1-self.preview_correction_weight)*d['raw_acceleration_xy_m_s2'][k]+self.preview_correction_weight*raw[k] for k in range(2))
                factor=min(1.,self.limit/max(math.hypot(*raw),1e-12));acc=tuple(x*factor for x in raw)
                d.update(raw_acceleration_xy_m_s2=raw,sent_acceleration_xy_m_s2=acc,saturated=factor<1.,
                         own_reference_preview_t_s=future_t,own_reference_preview_acceleration_xy_m_s2=future[:2],preview_correction_weight=self.preview_correction_weight)
                result=replace(result,reference=replace(result.reference,acceleration_world_m_s2=(*acc,result.reference.acceleration_world_m_s2[2])))
        return result
