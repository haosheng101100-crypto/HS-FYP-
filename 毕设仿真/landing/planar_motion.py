"""Test-platform XY commands. Never imported by the landing predictor."""
import math
import numpy as np


SCENARIOS=('planar_forward_backward','planar_lateral','planar_diagonal','planar_reversal','planar_xy')


def desired_planar_velocity(scenario,elapsed):
    if not math.isfinite(elapsed) or elapsed<0:raise ValueError('Finite nonnegative scenario time required')
    if scenario=='planar_forward_backward':return (.3*math.sin(.18*elapsed),0.)
    if scenario=='planar_lateral':return (0.,.3*math.sin(.18*elapsed))
    if scenario=='planar_diagonal':return (.22*math.sin(.18*elapsed),.22*math.sin(.18*elapsed))
    if scenario=='planar_reversal':return ((.3 if int(elapsed/6.)%2==0 else -.3),(.15 if int(elapsed/9.)%2==0 else -.15))
    if scenario=='planar_xy':return (.3*math.sin(.18*elapsed),.25*math.sin(.23*elapsed+.6))
    raise ValueError('Unknown planar scenario: '+str(scenario))


class PlanarMotionCommand:
    """Acceleration/jerk-bounded XY reference, with speed headroom for braking.

    These are command bounds; actual force-driven motion needs independent raw
    pose audits. Native joint control tracks velocity using bounded forces.
    """
    def __init__(self,speed=.8,acceleration=.12,jerk=.3):
        if any(not math.isfinite(v) or v<=0 for v in (speed,acceleration,jerk)):
            raise ValueError('Positive finite planar command limits required')
        self.speed=speed;self.acceleration=acceleration;self.jerk=jerk
        self.headroom=acceleration**2/(2*jerk)+acceleration*.08
        if self.headroom>=speed:raise ValueError('Insufficient speed reserve for jerk-bounded braking')
        self.v=np.zeros(2);self.a=np.zeros(2);self.last=None

    def step(self,now,target):
        target=np.asarray(target,dtype=float)
        if target.shape!=(2,) or not np.isfinite(target).all() or not math.isfinite(now):
            raise ValueError('Finite planar time and XY target required')
        dt=0. if self.last is None else now-self.last
        if self.last is not None and not 0<dt<=.080001:raise ValueError('Planar command update gap')
        self.last=now
        if not dt:return tuple(self.v),tuple(self.a)
        target=target*min(1.,(self.speed-self.headroom)/max(float(np.linalg.norm(target)),1e-12))
        desired=2.*(target-self.v)
        desired*=min(1.,self.acceleration/max(float(np.linalg.norm(desired)),1e-12))
        delta=desired-self.a
        delta*=min(1.,self.jerk*dt/max(float(np.linalg.norm(delta)),1e-12))
        new_a=self.a+delta
        new_v=self.v+.5*(self.a+new_a)*dt
        if np.linalg.norm(new_v)>self.speed+1e-8:raise ValueError('Planar command speed reserve exhausted')
        self.v=new_v;self.a=new_a
        return tuple(float(v) for v in self.v),tuple(float(a) for a in self.a)
