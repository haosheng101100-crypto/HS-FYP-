"""Shared nonholonomic vehicle model. State order: x,y,psi,v,a,kappa,j,eta."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.linalg import expm


def wrap(a):
    return (a+np.pi)%(2*np.pi)-np.pi


@dataclass(frozen=True)
class Mode:
    name: str
    lambda_a: float
    lambda_k: float
    tau_j: float
    tau_eta: float
    q_j: float
    q_eta: float

    def __post_init__(self):
        values=(self.lambda_a,self.lambda_k,self.tau_j,self.tau_eta,self.q_j,self.q_eta)
        if not all(math.isfinite(v) for v in values) or min(values)<0 or min(self.tau_j,self.tau_eta)<=0:
            raise ValueError('Invalid mode parameters')

    @property
    def q(self):
        return np.array([0.,0.,0.,0.,0.,0.,self.q_j,self.q_eta])


MODES=(Mode('CV',3.,3.,.4,.4,.03,.03),Mode('CA',0.,3.,.7,.4,.25,.03),
       Mode('CT',3.,0.,.4,.5,.03,.10),Mode('CTVAR',0.,0.,1.,1.,.25,.25))


@dataclass(frozen=True)
class Limits:
    speed: float=3.
    acceleration: float=1.
    curvature: float=1.
    jerk: float=2.
    curvature_rate: float=1.
    lateral_acceleration: float=2.5

    def __post_init__(self):
        if not all(math.isfinite(v) and v>0 for v in vars(self).values()):
            raise ValueError('Limits must be finite and positive')


def state_array(state):
    s=np.array(state,dtype=float,copy=True)
    if s.ndim<1 or s.shape[-1]!=8 or not np.all(np.isfinite(s)):
        raise ValueError('Expected finite state(s) with last dimension 8')
    return s


def derivative(state,mode):
    s=state_array(state)
    d=np.zeros_like(s)
    d[...,0]=s[...,3]*np.cos(s[...,2]);d[...,1]=s[...,3]*np.sin(s[...,2])
    d[...,2]=s[...,3]*s[...,5];d[...,3]=s[...,4]
    d[...,4]=s[...,6]-mode.lambda_a*s[...,4]
    d[...,5]=s[...,7]-mode.lambda_k*s[...,5]
    d[...,6]=-s[...,6]/mode.tau_j;d[...,7]=-s[...,7]/mode.tau_eta
    return d


def step(state,dt,mode,max_step=.05):
    s=state_array(state)
    if not math.isfinite(dt) or dt<0 or not math.isfinite(max_step) or max_step<=0:
        raise ValueError('Invalid integration time')
    n=max(1,math.ceil(dt/max_step));h=dt/n
    for _ in range(n):
        k1=derivative(s,mode);k2=derivative(s+h*k1/2,mode)
        k3=derivative(s+h*k2/2,mode);k4=derivative(s+h*k3,mode)
        s=s+h*(k1+2*k2+2*k3+k4)/6
    s[...,2]=wrap(s[...,2])
    return s


def violations(state,mode,limits):
    s=state_array(state)
    if s.ndim!=1:raise ValueError('Expected one state for constraint audit')
    d=derivative(s,mode)
    values={'speed':abs(s[3]),'acceleration':abs(s[4]),'curvature':abs(s[5]),
            'jerk':abs(d[4]),'curvature_rate':abs(d[5]),
            'lateral_acceleration':s[3]**2*abs(s[5])}
    return [k for k,v in values.items() if v>getattr(limits,k)+1e-9]


def transition_matrix(dt):
    if not math.isfinite(dt) or dt<0:raise ValueError('Invalid Markov interval')
    generator=np.full((4,4),.15);np.fill_diagonal(generator,-.45)
    return expm(generator*dt)
