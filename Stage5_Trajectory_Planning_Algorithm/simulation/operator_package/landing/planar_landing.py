"""Causal planar-motion supervision; engineering screens, not safety proofs.

The original trajectory, contact and physical audits remain mandatory. Unknown
future maneuvers cannot be certified by past observations. Contact is irreversible.
"""
from collections import deque
from dataclasses import dataclass, asdict, replace
import math
import numpy as np


@dataclass(frozen=True)
class PlanarDesign:
    stable_dwell_s: float = .8
    maximum_wait_s: float = 25.
    maximum_task_s: float = 40.
    maximum_reacquisitions: int = 3
    trusted_prediction_s: float = 1.2
    capture_half_window_s: float = .2
    maximum_platform_speed_m_s: float = .8
    maximum_capture_acceleration_m_s2: float = .11
    maximum_capture_velocity_change_m_s: float = .03
    maximum_yaw_acceleration_rad_s2: float = .04
    maximum_yaw_rate_rad_s: float = .15
    minimum_reacquisition_gap_m: float = 1.25
    braking_clearance_reserve_m: float = .25

    def __post_init__(self):
        for key,value in asdict(self).items():
            if not math.isfinite(value) or value<=0:
                raise ValueError('Positive finite planar design required: '+key)
        if type(self.maximum_reacquisitions) is not int:
            raise ValueError('Integer reacquisition budget required')
        if self.capture_half_window_s>=self.trusted_prediction_s:
            raise ValueError('Capture window must fit trusted prediction span')


class MotionChangeMonitor:
    """Compare matured predictions and meaningful direction changes, in XY.

    Low speeds have no defined direction. A direction anchor survives a brief
    zero-speed crossing. The stability timer resets on every detected change.
    """
    def __init__(self,stable_dwell_s=.8):
        if not math.isfinite(stable_dwell_s) or stable_dwell_s<=0:
            raise ValueError('Positive finite stability dwell required')
        self.dwell=stable_dwell_s;self.history=deque();self.anchor=None
        self.last_change=None;self.first=None

    def update(self,state):
        t=state.stamp
        if self.history and t<=self.history[-1].stamp:
            raise ValueError('Planar observation clock must increase')
        if self.first is None:self.first=t;self.last_change=t
        reasons=[];position_error=velocity_error=acceleration_change=0.
        if self.history:
            previous=self.history[-1]
            if t-previous.stamp>.080001:reasons.append('OBSERVATION_GAP')
            while len(self.history)>1 and self.history[1].stamp<=t-.24:
                self.history.popleft()
            old=self.history[0];predicted=old.predict(t-old.stamp)
            position_error=float(np.linalg.norm(np.subtract(state.position[:2],predicted.position[:2])))
            velocity_error=float(np.linalg.norm(np.subtract(state.velocity[:2],predicted.velocity[:2])))
            acceleration_change=float(np.linalg.norm(np.subtract(state.acceleration[:2],previous.acceleration[:2])))
            if position_error>.015 or velocity_error>.06:reasons.append('PREDICTION_INNOVATION')
            if acceleration_change>.12:reasons.append('ACCELERATION_CHANGE')
        velocity=np.asarray(state.velocity[:2]);speed=float(np.linalg.norm(velocity))
        if speed>=.08:
            if self.anchor is not None and float(velocity@self.anchor)<-.04:
                reasons.append('DIRECTION_REVERSAL')
            self.anchor=velocity/speed
        if reasons:self.last_change=t
        stable=t-self.last_change>=self.dwell-1e-8
        self.history.append(state)
        return dict(changed=bool(reasons),stable=bool(stable),reasons=reasons,
                    stable_for_s=t-self.last_change,position_innovation_m=position_error,
                    velocity_innovation_m_s=velocity_error,acceleration_change_m_s2=acceleration_change,
                    trusted_prediction_s=.6 if not stable else 1.2,
                    statistically_certified=False)


def capture_window(platform,duration,design=None):
    """Screen local acceleration and velocity drift around a contact time.

    Long approach predictions are conditional, never contact authorization.
    Norms of affine acceleration attain maxima at interval endpoints. Velocity
    drift uses the integral bound max|a|*half_window, avoiding sampled extrema.
    """
    design=design or PlanarDesign()
    if isinstance(design,dict):design=PlanarDesign(**design)
    if not math.isfinite(duration) or duration<=0:raise ValueError('Positive finite contact duration required')
    if duration+design.capture_half_window_s>design.trusted_prediction_s+1e-9:
        return dict(passed=True,conditional=True,contact_authorized=False,
                    reason='LONG_APPROACH_REQUIRES_ONLINE_REVALIDATION',duration_s=duration)
    times=(max(0.,duration-design.capture_half_window_s),duration+design.capture_half_window_s)
    states=[platform.predict(t) for t in times]
    acceleration=max(math.hypot(*s.acceleration[:2]) for s in states)
    speed_bound=math.hypot(*platform.predict(duration).velocity[:2])+acceleration*design.capture_half_window_s
    drift=acceleration*design.capture_half_window_s
    checks=dict(capture_acceleration=acceleration<=design.maximum_capture_acceleration_m_s2,
                capture_velocity_change=drift<=design.maximum_capture_velocity_change_m_s,
                platform_speed=speed_bound<=design.maximum_platform_speed_m_s,
                heading_acceleration=abs(platform.yaw_acceleration)<=design.maximum_yaw_acceleration_rad_s2,
                heading_rate=max(abs(s.yaw_rate) for s in states)<=design.maximum_yaw_rate_rad_s)
    passed=all(checks.values())
    return dict(passed=passed,conditional=False,contact_authorized=False,checks=checks,
                duration_s=duration,maximum_acceleration_m_s2=acceleration,
                relative_velocity_drift_bound_m_s=drift,platform_speed_bound_m_s=speed_bound,
                reason='LOCAL_WINDOW_ELIGIBLE' if passed else 'NO_LOCAL_CAPTURE_WINDOW',
                scope='Causal local polynomial screen; original measured contact gates still required')


def inflate_forecast_tube(tube,motion):
    """Add observed change residuals; never invent calibration observations.

    Temporary engineering reserves do not make the tube statistically certified.
    They cannot predict a future unobserved maneuver.
    """
    if tube is None or motion is None or motion.get('stable',False):return tube
    dp=max(.005,motion.get('position_innovation_m',0.))
    dv=max(.015,motion.get('velocity_innovation_m_s',0.))
    da=max(.05,motion.get('acceleration_change_m_s2',0.))
    return replace(tube,position_radius_m=tuple(p+dp+dv*h+.5*da*h*h for p,h in zip(tube.position_radius_m,tube.horizons)),
                   velocity_radius_m_s=tuple(v+dv+da*h for v,h in zip(tube.velocity_radius_m_s,tube.horizons)))


def precontact_relative_audit(plan,platform,geometry):
    """Sample all foot material-point velocities over the final .16 seconds.

    This is an extra rejection screen, not a continuous collision certificate.
    Existing nominal whole-body audits and measured capture checks still decide.
    """
    from .contact_geometry import evaluate_airframe
    maximum=0.
    for elapsed in np.linspace(max(0.,plan.duration-.16),plan.duration,9):
        deck=platform.predict(float(elapsed)).deck()
        feet=evaluate_airframe(geometry,plan.motion(float(elapsed)),deck)
        maximum=max(maximum,max(f.tangential_speed_upper_bound_m_s for f in feet))
    return dict(passed=maximum<=.05+1e-6,maximum_tangential_speed_m_s=maximum,
                interval_s=min(.16,plan.duration),samples=9,continuously_certified=False)


class PlanarLandingSupervisor:
    """Bounded FOLLOW -> DESCEND -> REACQUIRE/EXIT policy, with contact priority."""
    def __init__(self,design=None):
        self.design=design or PlanarDesign()
        if isinstance(self.design,dict):self.design=PlanarDesign(**self.design)
        self.monitor=MotionChangeMonitor(self.design.stable_dwell_s)
        self.started=None;self.last=None;self.phase='FOLLOW';self.reacquisitions=0
        self.wait_elapsed=0.;self.last_waiting=True;self.terminal=False;self.exited=False;self.exit_reason=None

    def update(self,now,state,platform,gap_m,*,aligned=False,descending=False,contact=False,braking_reference=None):
        if contact or self.terminal:
            self.terminal=True;self.phase='CONTACT'
            return dict(action='CONTACT',reason='PHYSICAL_CONTACT_OWNS_CONTROL')
        if not all(math.isfinite(v) for v in (now,gap_m)):
            raise ValueError('Finite planar control time and clearance required')
        if self.last is not None and now<=self.last:raise ValueError('Control clock must increase')
        if self.started is None:self.started=now
        dt=0. if self.last is None else now-self.last
        self.last=now
        if self.last_waiting:self.wait_elapsed+=dt
        reason=None
        if self.exited:reason=self.exit_reason
        elif not 0<=now-state.stamp<=.06 or not 0<=now-platform.stamp<=.06:reason='STALE_PLANAR_OBSERVATION'
        elif now-self.started>self.design.maximum_task_s:reason='PLANAR_TASK_TIMEOUT'
        elif self.wait_elapsed>self.design.maximum_wait_s:reason='PLANAR_WAIT_TIMEOUT'
        if reason:
            self.exited=True;self.exit_reason=reason;self.phase='EXIT'
            return dict(action='EXIT',reason=reason,wait_elapsed_s=self.wait_elapsed)
        motion=self.monitor.update(platform)
        window=capture_window(platform,.8,self.design)
        eligible=motion['stable'] and window['passed'] and not window['conditional']
        action='DESCEND' if eligible and aligned else 'FOLLOW'
        if descending:
            if eligible:action='DESCEND'
            else:
                # Use the same stop-descent design as the runtime; no substitute
                # for the actual whole-body preview and original exit audits.
                from .receding_live import braking_clearance_design
                braking=braking_clearance_design(state,upward_acceleration=.4,vertical_jerk=1.2)['required_clearance_m']
                if braking_reference is not None:
                    reference_state=replace(state,velocity=braking_reference.velocity_world_m_s,acceleration=braking_reference.acceleration_world_m_s2)
                    reference_braking=braking_clearance_design(reference_state,upward_acceleration=.4,vertical_jerk=1.2)['required_clearance_m']
                    # Account for a descending reference even when measured
                    # vertical speed has not yet caught up with that command.
                    braking=max(braking,reference_braking)+abs(braking_reference.position_world_m[2]-state.position[2])
                required=self.design.minimum_reacquisition_gap_m+braking+self.design.braking_clearance_reserve_m
                if gap_m<required or self.reacquisitions>=self.design.maximum_reacquisitions:
                    action='EXIT';reason='PLANAR_CHANGE_WITHOUT_REACQUISITION_CLEARANCE';self.exited=True;self.exit_reason=reason
                else:
                    action='REACQUIRE';reason='PLANAR_WINDOW_LOST';self.reacquisitions+=1
        self.phase=action;self.last_waiting=action in ('FOLLOW','REACQUIRE')
        return dict(action=action,reason=reason,motion=motion,window=window,
                    wait_elapsed_s=self.wait_elapsed,reacquisitions=self.reacquisitions,
                    descent_eligible=bool(eligible),prediction_trust_s=motion['trusted_prediction_s'])
