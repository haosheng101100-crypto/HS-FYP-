"""Trajectory-preserving sampled futures, explicitly not calibrated probabilities."""
import math
import numpy as np
from .model import derivative, step, wrap


def admissible_batch(states,mode,limits):
    d=derivative(states,mode)
    return (np.all(np.isfinite(states),axis=-1)&
            (np.abs(states[:,3])<=limits.speed+1e-9)&
            (np.abs(states[:,4])<=limits.acceleration+1e-9)&
            (np.abs(states[:,5])<=limits.curvature+1e-9)&
            (np.abs(d[:,4])<=limits.jerk+1e-9)&
            (np.abs(d[:,5])<=limits.curvature_rate+1e-9)&
            (states[:,3]**2*np.abs(states[:,5])<=limits.lateral_acceleration+1e-9))


def sample_paths(filt,times,count,rng,limits):
    """Reject full trajectories, never clip and retain an old probability claim."""
    candidates=20*count
    states=rng.multivariate_normal(filt.mean,filt.covariance,size=candidates,check_valid='raise')
    states[:,2]=wrap(states[:,2])
    alive=admissible_batch(states,filt.mode,limits)
    initial=states.copy();snapshots=[];previous=0.
    for target in times:
        dt=float(target-previous);n=max(1,math.ceil(dt/.05-1e-12));h=dt/n
        for _ in range(n):
            midpoint=step(states,h/2,filt.mode)
            alive &= admissible_batch(midpoint,filt.mode,limits)
            states=step(states,h,filt.mode)
            alive &= admissible_batch(states,filt.mode,limits)
            # Same Euler-discretized driving diffusion as UKF Q=diag(q)*h.
            states[:,6:8]+=rng.normal(size=(candidates,2))*np.sqrt(filt.mode.q[6:8]*h)
            alive &= admissible_batch(states,filt.mode,limits)
        snapshots.append(states.copy());previous=target
    selected=np.flatnonzero(alive)[:count]
    attempted=int(selected[-1]+1) if len(selected)==count else candidates
    paths=np.stack(snapshots,axis=1)[selected]
    trajectories=[]
    for i,path in zip(selected,paths):
        velocity=np.column_stack([path[:,3]*np.cos(path[:,2]),path[:,3]*np.sin(path[:,2])])
        trajectories.append(dict(initial_state=initial[i].tolist(),states=path.tolist(),velocity_xy=velocity.tolist()))
    envelope=None
    if len(paths):
        vel=np.stack([np.array(t['velocity_xy']) for t in trajectories])
        angle_center=paths[0,:,2]
        envelope=dict(position_min=paths[:,:,:2].min(axis=0).tolist(),position_max=paths[:,:,:2].max(axis=0).tolist(),
            velocity_min=vel.min(axis=0).tolist(),velocity_max=vel.max(axis=0).tolist(),
            yaw_center=angle_center.tolist(),yaw_radius=np.abs(wrap(paths[:,:,2]-angle_center)).max(axis=0).tolist())
    return dict(trajectories=trajectories,attempted=attempted,rejected=attempted-len(trajectories),
                requested=count,sample_envelope=envelope)
