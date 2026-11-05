"""The sole online input contract. No future command/route fields exist."""
from dataclasses import dataclass
import math
import numpy as np
from .model import wrap


@dataclass(frozen=True)
class Observation:
    stamp: float
    received_at: float
    position_xy: tuple
    yaw: float
    covariance: np.ndarray

    def __post_init__(self):
        xy=np.asarray(self.position_xy,dtype=float)
        r=np.array(self.covariance,dtype=float,copy=True)
        if xy.shape!=(2,) or not np.all(np.isfinite(xy)) or not all(math.isfinite(t) for t in (self.stamp,self.received_at,self.yaw)):
            raise ValueError('Invalid observation values')
        if self.stamp>self.received_at+1e-9:
            raise ValueError('Observation stamp is in the future')
        if r.shape!=(3,3) or not np.all(np.isfinite(r)) or not np.allclose(r,r.T,rtol=1e-10,atol=1e-12):
            raise ValueError('Observation covariance must be finite symmetric 3x3')
        if np.linalg.eigvalsh(r).min()<=0:
            raise ValueError('Observation covariance must be positive definite')
        r.setflags(write=False)
        object.__setattr__(self,'position_xy',tuple(xy))
        object.__setattr__(self,'covariance',r)
        object.__setattr__(self,'yaw',float(wrap(self.yaw)))

    @property
    def value(self):
        return np.array([*self.position_xy,self.yaw])
