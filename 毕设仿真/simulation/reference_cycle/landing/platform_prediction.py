"""Causal local cubic platform state from past Gazebo pose samples only."""
import math
from dataclasses import replace
import numpy as np
from .receding_landing import PlatformState


def _resolved_polynomial(dt, values):
    """Use cubic derivatives only when the data resolve the extra degree.

    Differentiating millimeter pose noise into a cubic jerk destabilizes long
    forecasts. BIC compares quadratic and cubic residuals on the SAME causal
    window. Clean resolved jerk is retained; noisy unresolvable jerk is zero.
    The numerical floor merely avoids model selection on float roundoff.
    """
    fits={degree:np.polyfit(dt,values,degree) for degree in (2,3)}
    scores={}
    observed=values[:,:2] if values.ndim==2 else values
    dimensions=2 if values.ndim==2 else 1
    count=observed.size
    for degree,c in fits.items():
        predicted=np.array([np.polyval(c[:,k],dt) for k in range(values.shape[1])]).T if values.ndim==2 else np.polyval(c,dt)
        residual=observed-(predicted[:,:2] if values.ndim==2 else predicted)
        scores[degree]=count*math.log(max(float(np.sum(residual*residual))/count,1e-24))+(degree+1)*dimensions*math.log(count)
    # Require a BIC evidence ratio of at least 10 for the noisier extra
    # derivative. Clean resolved dynamics easily clear this threshold.
    if scores[3]+2*math.log(10.)<scores[2]:return fits[3]
    c=fits[2]
    return np.vstack((np.zeros((1,c.shape[1])),c)) if values.ndim==2 else np.r_[0.,c]


def estimate_platform(samples,now,half_extents,span=.35):
    rows=[s for s in samples if now-span<=s.sim_t_s<=now]
    if len(rows)<8 or rows[-1].sim_t_s-rows[0].sim_t_s<span*.7:
        raise ValueError('Cubic platform fit warming up')
    if now-rows[-1].sim_t_s>.06:raise ValueError('Platform pose stale')
    if max(b.sim_t_s-a.sim_t_s for a,b in zip(rows,rows[1:]))>.05:raise ValueError('Platform sample gap')
    dt=np.array([s.sim_t_s-now for s in rows])
    position=np.array([s.pose.position_world_m for s in rows])
    angle=[]
    for s in rows:
        x,y,z,w=s.pose.orientation_xyzw;angle.append(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    pc=_resolved_polynomial(dt,position);yc=_resolved_polynomial(dt,np.unwrap(angle))
    p=pc[3];p[2]=rows[-1].pose.transform((0,0,.8))[2]
    v=pc[2];a=2*pc[1];j=6*pc[0]
    v[2]=a[2]=j[2]=0.
    return PlatformState(now,tuple(p),tuple(v),tuple(a),float(yc[3]),float(yc[2]),tuple(half_extents),tuple(j),float(2*yc[1]))


def delayed_noisy_platform(samples, now, half_extents, span=.65, delay_s=.04,
                          position_sigma_m=.0008, yaw_sigma_rad=.0008, seed=20261024):
    """Declared synthetic sensing stress; never changes the physical platform.

    A timestamp-seeded noise sample remains identical when a historical pose
    is reused. Extrapolation compensates the known sensor timestamp delay.
    """
    from .contact_geometry import quaternion_from_rpy
    if not (0 <= delay_s <= .2 and 0 <= position_sigma_m <= .02 and 0 <= yaw_sigma_rad <= .02):
        raise ValueError('Invalid synthetic observation stress')
    cutoff=now-delay_s
    noisy=synthetic_pose_samples(samples,cutoff,span,position_sigma_m,yaw_sigma_rad,seed)
    return estimate_platform(noisy,cutoff,half_extents,span).predict(delay_s)


def synthetic_pose_samples(samples,cutoff,span=.65,position_sigma_m=.0008,yaw_sigma_rad=.0008,seed=20261024):
    """Timestamp-stable synthetic observations, shared by all estimator tests."""
    from .contact_geometry import quaternion_from_rpy
    noisy=[]
    for sample in samples:
        if not cutoff-span <= sample.sim_t_s <= cutoff:continue
        rng=np.random.default_rng(seed+round(sample.sim_t_s*1000000))
        delta=np.clip(rng.normal(0.,position_sigma_m,2),-3*position_sigma_m,3*position_sigma_m)
        x,y,z,w=sample.pose.orientation_xyzw
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        yaw+=float(np.clip(rng.normal(0.,yaw_sigma_rad),-3*yaw_sigma_rad,3*yaw_sigma_rad))
        p=sample.pose.position_world_m
        pose=replace(sample.pose,position_world_m=(p[0]+delta[0],p[1]+delta[1],p[2]),
                     orientation_xyzw=quaternion_from_rpy(0.,0.,yaw))
        noisy.append(replace(sample,pose=pose))
    return noisy
