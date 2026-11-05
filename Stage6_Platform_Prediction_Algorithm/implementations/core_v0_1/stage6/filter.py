"""Scaled unscented filter; local wrapped angle coordinates, no physical clipping."""
import math
import numpy as np
from .model import step, state_array, wrap


def positive_covariance(matrix):
    p=np.array(matrix,dtype=float,copy=True)
    if p.ndim!=2 or p.shape[0]!=p.shape[1] or not np.all(np.isfinite(p)):
        raise ValueError('Invalid covariance shape or values')
    scale=max(1.,float(np.max(np.abs(p))))
    if not np.allclose(p,p.T,rtol=1e-10,atol=scale*1e-12):
        raise ValueError('Asymmetric covariance')
    p=(p+p.T)/2
    vals,vecs=np.linalg.eigh(p)
    if vals.min() < -1e-9*scale:
        raise ValueError('Covariance lost positive semidefiniteness')
    if vals.min()<1e-12:
        p=(vecs*np.maximum(vals,1e-12))@vecs.T
    return (p+p.T)/2


class LocalAngleError(ValueError):
    """A local Gaussian angle chart cannot represent this uncertainty."""


def require_local_angle(covariance):
    # Conservative support bound for alpha=1,n=8, before any modulo operation.
    if 8*float(covariance[2,2]) >= (math.pi/2)**2:
        raise LocalAngleError('Angular sigma spread must be < pi/2; use a circular/multimodal estimator for a broader prior')


def differences(points,mean,angle_index):
    delta=np.asarray(points)-mean
    delta[...,angle_index]=wrap(delta[...,angle_index])
    return delta


def moments(points,wm,wc,angle_index):
    pts=np.asarray(points)
    mean=np.sum(wm[:,None]*pts,axis=0)
    anchor=pts[0,angle_index]
    mean[angle_index]=wrap(anchor+np.dot(wm,wrap(pts[:,angle_index]-anchor)))
    delta=differences(pts,mean,angle_index)
    covariance=np.einsum('i,ij,ik->jk',wc,delta,delta)
    return mean,(covariance+covariance.T)/2


def sigma_points(mean,covariance):
    mean=state_array(mean)
    if mean.ndim!=1:raise ValueError('Expected one mean')
    p=positive_covariance(covariance)
    if p.shape!=(8,8):raise ValueError('Expected 8x8 covariance')
    require_local_angle(p)
    # alpha=1,beta=2,kappa_UT=0 => lambda=0.
    root=np.linalg.cholesky(8*p)
    pts=np.vstack([mean,mean+root.T,mean-root.T])
    pts[:,2]=wrap(pts[:,2])
    wm=np.full(17,1/16.);wm[0]=0
    wc=wm.copy();wc[0]=2
    return pts,wm,wc


def pose_measurement(state,offset=(0.,0.)):
    s=state_array(state);d=np.asarray(offset,dtype=float)
    if d.shape!=(2,) or not np.all(np.isfinite(d)):raise ValueError('Invalid pose offset')
    c=np.cos(s[...,2]);sn=np.sin(s[...,2])
    return np.stack([s[...,0]+c*d[0]-sn*d[1],s[...,1]+sn*d[0]+c*d[1],s[...,2]],axis=-1)


class UKF:
    def __init__(self,mean,covariance,mode,offset=(0.,0.)):
        self.mean=state_array(mean)
        if self.mean.ndim!=1:raise ValueError('Expected one state mean')
        self.covariance=positive_covariance(covariance)
        if self.covariance.shape!=(8,8):raise ValueError('Expected 8x8 covariance')
        require_local_angle(self.covariance)
        pose_measurement(self.mean,offset)
        self.mode=mode;self.offset=tuple(offset);self.last_nis=None

    def clone(self):
        out=UKF(self.mean,self.covariance,self.mode,self.offset)
        out.last_nis=self.last_nis
        return out

    def predict(self,dt):
        if not math.isfinite(dt) or dt<0:raise ValueError('Invalid prediction interval')
        if dt==0:return
        n=max(1,math.ceil(dt/.05));h=dt/n
        mean=self.mean.copy();covariance=self.covariance.copy()
        for _ in range(n):
            pts,wm,wc=sigma_points(mean,covariance)
            mean,p=moments(step(pts,h,self.mode),wm,wc,2)
            covariance=positive_covariance(p+np.diag(self.mode.q)*h)
            require_local_angle(covariance)
        self.mean=mean;self.covariance=covariance

    def update(self,observation):
        pts,wm,wc=sigma_points(self.mean,self.covariance)
        mapped=pose_measurement(pts,self.offset)
        predicted,s=moments(mapped,wm,wc,2)
        s=positive_covariance(s+observation.covariance)
        dx=differences(pts,self.mean,2);dz=differences(mapped,predicted,2)
        cross=np.einsum('i,ij,ik->jk',wc,dx,dz)
        gain=np.linalg.solve(s,cross.T).T
        innovation=differences(observation.value,predicted,2)
        self.last_nis=float(innovation@np.linalg.solve(s,innovation))
        sign,ld=np.linalg.slogdet(s)
        if sign<=0:raise ValueError('Invalid innovation covariance')
        ll=-.5*(3*math.log(2*math.pi)+ld+self.last_nis)
        self.mean=self.mean+gain@innovation;self.mean[2]=wrap(self.mean[2])
        p=self.covariance-gain@s@gain.T
        self.covariance=positive_covariance((p+p.T)/2)
        return float(ll)


def initialize(observation,mode,offset=(0.,0.)):
    d=np.asarray(offset,dtype=float)
    if d.shape!=(2,) or not np.all(np.isfinite(d)):raise ValueError('Invalid offset')
    c,s=math.cos(observation.yaw),math.sin(observation.yaw)
    rotated=np.array([c*d[0]-s*d[1],s*d[0]+c*d[1]])
    mean=np.zeros(8);mean[:2]=np.asarray(observation.position_xy)-rotated;mean[2]=observation.yaw
    jac=np.zeros((8,3));jac[:3,:3]=np.eye(3)
    jac[0,2]=rotated[1];jac[1,2]=-rotated[0]
    p=jac@observation.covariance@jac.T+np.diag([0,0,0,1,.5,.5,.5,.5])
    return UKF(mean,p,mode,offset)
