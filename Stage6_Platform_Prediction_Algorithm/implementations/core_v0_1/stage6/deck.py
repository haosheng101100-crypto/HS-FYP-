"""Planar rigid mounting geometry. No assumption that the deck is at body origin."""
from dataclasses import dataclass, asdict
from copy import deepcopy
import math
import numpy as np
from .model import state_array, wrap


@dataclass(frozen=True)
class DeckGeometry:
    offset_xy: tuple=(0.,0.)
    half_extents: tuple=(.34,.4)
    mount_yaw: float=0.

    def __post_init__(self):
        for v in (self.offset_xy,self.half_extents):
            a=np.asarray(v,dtype=float)
            if a.shape!=(2,) or not np.all(np.isfinite(a)):raise ValueError('Invalid deck geometry')
        if min(self.half_extents)<=0 or not math.isfinite(self.mount_yaw):raise ValueError('Invalid deck dimensions or yaw')
        object.__setattr__(self,'offset_xy',tuple(self.offset_xy))
        object.__setattr__(self,'half_extents',tuple(self.half_extents))


def body_to_deck(state,geometry):
    s=state_array(state);psi=s[...,2];c=np.cos(psi);sn=np.sin(psi)
    dx,dy=geometry.offset_xy
    r=np.stack([c*dx-sn*dy,sn*dx+c*dy],axis=-1)
    omega=s[...,3]*s[...,5]
    center=s[...,:2]+r
    velocity=np.stack([s[...,3]*c,s[...,3]*sn],axis=-1)+omega[...,None]*np.stack([-r[...,1],r[...,0]],axis=-1)
    yaw=wrap(psi+geometry.mount_yaw)
    hx,hy=geometry.half_extents
    corners=np.array([[hx,hy],[hx,-hy],[-hx,-hy],[-hx,hy]])
    co=np.cos(yaw)[...,None];si=np.sin(yaw)[...,None]
    rotated=np.stack([co*corners[:,0]-si*corners[:,1],si*corners[:,0]+co*corners[:,1]],axis=-1)
    return dict(position_xy=center.tolist(),velocity_xy=velocity.tolist(),yaw=np.asarray(yaw).tolist(),
                yaw_rate=np.asarray(omega).tolist(),corners_xy=(center[...,None,:]+rotated).tolist())


def map_forecast(packet,geometry):
    out=deepcopy(packet);out['deck_geometry']=asdict(geometry)
    out['deck_scope']='Fixed planar mounting only. No roll, pitch, suspension, or vertical motion prediction.'
    for model in out['models']:
        for trajectory in model['trajectories']:
            trajectory['deck']=body_to_deck(trajectory['states'],geometry)
    return out
