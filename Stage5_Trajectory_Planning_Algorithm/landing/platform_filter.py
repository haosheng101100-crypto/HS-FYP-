"""Causal constant-acceleration Kalman state for the level moving deck.

Pose noise assumptions are simulation design values, not identified hardware
statistics. White jerk process noise permits unannounced acceleration changes.
No platform commands or future pose samples enter this filter.
"""
import math
import numpy as np
from .receding_landing import PlatformState


class CausalPlatformKalman:
    def __init__(self,position_noise_m=.0008,yaw_noise_rad=.0008,
                 linear_jerk_spectral_density=.01,angular_jerk_spectral_density=.0004,
                 maximum_measurement_age_s=.12,innovation_adaptation=None):
        values=(position_noise_m,yaw_noise_rad,linear_jerk_spectral_density,
                angular_jerk_spectral_density,maximum_measurement_age_s)
        if any(not math.isfinite(v) or v<=0 for v in values):raise ValueError('Positive finite filter design required')
        self.R=np.array([position_noise_m**2,position_noise_m**2,yaw_noise_rad**2])
        self.q=np.array([linear_jerk_spectral_density,linear_jerk_spectral_density,angular_jerk_spectral_density])
        self.maximum_age=maximum_measurement_age_s
        self.x=None;self.P=None;self.stamp=None;self.count=0;self.height=None
        self.adaptation=innovation_adaptation
        if self.adaptation:
            if set(self.adaptation)!={'sigma_threshold','process_noise_multiplier','hold_s'} or any(not math.isfinite(v) or v<=0 for v in self.adaptation.values()):
                raise ValueError('Finite positive innovation adaptation design required')
            if self.adaptation['process_noise_multiplier']<1.:raise ValueError('Adaptation must not reduce process noise')
        self.boost_until=-math.inf
        self.innovation_report=dict(process_noise_multiplier=1.,maximum_normalized_xy_innovation=0.)

    @staticmethod
    def matrices(dt):
        F=np.array([[1.,dt,.5*dt*dt],[0.,1.,dt],[0.,0.,1.]])
        Q=np.array([[dt**5/20,dt**4/8,dt**3/6],
                    [dt**4/8,dt**3/3,dt*dt/2],
                    [dt**3/6,dt*dt/2,dt]])
        return F,Q

    def update(self,samples,now,half_extents):
        if not math.isfinite(now):raise ValueError('Finite decision clock required')
        for sample in sorted(samples,key=lambda s:s.sim_t_s):
            if sample.sim_t_s>now:continue
            if self.stamp is not None and sample.sim_t_s<=self.stamp:continue
            px,py,_=sample.pose.position_world_m
            x,y,z,w=sample.pose.orientation_xyzw
            yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            measurement=np.array([px,py,yaw])
            if not np.isfinite(measurement).all():raise ValueError('Nonfinite platform measurement')
            if self.x is None:
                self.x=np.zeros((3,3));self.x[:,0]=measurement
                self.P=np.array([np.diag([r,1.,.5]) for r in self.R])
            else:
                dt=sample.sim_t_s-self.stamp
                if dt>.080001:raise ValueError('Missing platform measurement interval')
                F,Q=self.matrices(dt)
                predicted=self.x@F.T
                measurement[2]=predicted[2,0]+math.remainder(yaw-predicted[2,0],2*math.pi)
                innovation=max(abs(measurement[axis]-predicted[axis,0])/math.sqrt((F@self.P[axis]@F.T+self.q[axis]*Q)[0,0]+self.R[axis]) for axis in range(2))
                if self.adaptation and innovation>self.adaptation['sigma_threshold']:
                    self.boost_until=sample.sim_t_s+self.adaptation['hold_s']
                multiplier=self.adaptation['process_noise_multiplier'] if self.adaptation and sample.sim_t_s<=self.boost_until else 1.
                self.innovation_report=dict(process_noise_multiplier=multiplier,maximum_normalized_xy_innovation=float(innovation),measurement_stamp_s=sample.sim_t_s)
                for axis in range(3):
                    covariance=F@self.P[axis]@F.T+self.q[axis]*Q*(multiplier if axis<2 else 1.)
                    K=covariance[:,0]/(covariance[0,0]+self.R[axis])
                    self.x[axis]=predicted[axis]+K*(measurement[axis]-predicted[axis,0])
                    A=np.eye(3);A[:,0]-=K
                    self.P[axis]=A@covariance@A.T+self.R[axis]*np.outer(K,K)
            self.stamp=sample.sim_t_s;self.count+=1
            self.height=sample.pose.transform((0.,0.,.8))[2]
        if self.count<8:raise ValueError('Platform filter warming up')
        if not 0<=now-self.stamp<=self.maximum_age:raise ValueError('Stale platform filter measurements')
        F,_=self.matrices(now-self.stamp);current=self.x@F.T
        return PlatformState(now,(float(current[0,0]),float(current[1,0]),self.height),
            (float(current[0,1]),float(current[1,1]),0.),
            (float(current[0,2]),float(current[1,2]),0.),
            float(current[2,0]),float(current[2,1]),tuple(half_extents),
            (0.,0.,0.),float(current[2,2]))
