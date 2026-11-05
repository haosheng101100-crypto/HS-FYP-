"""Pose-only UKF/IMM; current filtering interaction does not merge future paths."""
import math
from dataclasses import asdict
import numpy as np
from scipy.special import logsumexp
from .filter import UKF, initialize, positive_covariance, LocalAngleError
from .forecast import sample_paths
from .model import MODES, Limits, transition_matrix, violations, wrap


def mix_components(means,covariances,weights):
    ms=np.asarray(means,dtype=float);ps=np.asarray(covariances,dtype=float);w=np.asarray(weights,dtype=float)
    if ms.shape!=(len(w),8) or ps.shape!=(len(w),8,8) or np.any(w<0) or not np.all(np.isfinite(w)) or not np.isclose(w.sum(),1.):
        raise ValueError('Invalid mixture')
    mean=w@ms
    anchor=ms[int(np.argmax(w)),2]
    mean[2]=wrap(anchor+w@wrap(ms[:,2]-anchor))
    delta=ms-mean;delta[:,2]=wrap(delta[:,2])
    p=np.einsum('i,ijk->jk',w,ps)+np.einsum('i,ij,ik->jk',w,delta,delta)
    return mean,positive_covariance(p)


def posterior_weights(prior,log_likelihoods):
    prior=np.asarray(prior,dtype=float);ll=np.asarray(log_likelihoods,dtype=float)
    if prior.shape!=ll.shape or np.any(prior<=0) or not np.all(np.isfinite(ll)):
        raise ValueError('Invalid likelihood or prior')
    values=np.log(prior)+ll
    weights=np.exp(values-logsumexp(values))
    return weights/weights.sum()


class VehiclePredictor:
    def __init__(self,offset=(0.,0.)):
        d=np.asarray(offset,dtype=float)
        if d.shape!=(2,) or not np.all(np.isfinite(d)):raise ValueError('Invalid observation offset')
        self.offset=tuple(d);self.filters=[];self.weights=np.ones(4)/4
        self.latest=None;self.first_stamp=None;self.sample_count=0;self.reset_count=0

    def update(self,observation):
        if self.latest is not None:
            if observation.stamp<=self.latest.stamp or observation.received_at<self.latest.received_at:
                raise ValueError('Out-of-order observations; no implicit smoother/reordering')
        restart=self.latest is None or observation.stamp-self.latest.stamp>1.
        if restart:
            filters=[initialize(observation,m,self.offset) for m in MODES]
            weights=np.ones(4)/4
            if self.latest is not None:self.reset_count+=1
            first_stamp=observation.stamp;count=1
        else:
            dt=observation.stamp-self.latest.stamp
            trans=transition_matrix(dt)
            priors=self.weights@trans
            mixing=self.weights[:,None]*trans/priors[None,:]
            means=np.array([f.mean for f in self.filters]);covs=np.array([f.covariance for f in self.filters])
            filters=[];ll=[]
            for j,mode in enumerate(MODES):
                mean,cov=mix_components(means,covs,mixing[:,j])
                f=UKF(mean,cov,mode,self.offset);f.predict(dt)
                ll.append(f.update(observation));filters.append(f)
            weights=posterior_weights(priors,np.array(ll))
            first_stamp=self.first_stamp;count=self.sample_count+1
        # Commit only after every mode succeeds.
        self.filters=filters;self.weights=weights;self.latest=observation
        self.first_stamp=first_stamp;self.sample_count=count

    def forecast(self,now,horizon=2.,step_s=.1,particles_per_mode=12,seed=0,limits=None):
        if not all(math.isfinite(v) for v in (now,horizon,step_s)) or min(horizon,step_s)<=0 or horizon<step_s:
            raise ValueError('Invalid forecast time grid')
        count=round(horizon/step_s)
        if not math.isclose(count*step_s,horizon,rel_tol=1e-9,abs_tol=1e-9):
            raise ValueError('Horizon must be a multiple of step')
        if not isinstance(particles_per_mode,int) or particles_per_mode<1:
            raise ValueError('Particle count must be a positive integer')
        limits=Limits() if limits is None else limits
        configuration=dict(state_order=['x','y','psi','v','a','kappa','j','eta'],
            modes=[asdict(m) for m in MODES],limits=asdict(limits),observation_offset_xy=list(self.offset),
            max_substep_s=.05,minimum_samples=5,minimum_span_s=.4,max_observation_age_s=.2,reset_gap_s=1.,
            mode_transition_offdiagonal_rate_per_s=.15,ut_alpha=1.,ut_beta=2.,ut_kappa=0.,
            particles_per_mode=particles_per_mode,seed=seed,angular_sigma_bound_rad=math.pi/2)
        result=dict(schema_version=1,configuration=configuration,issued_at=float(now),status='warming_up',trust='uncalibrated',
            observation_age_s=None,observed_duration_s=0.,sample_count=self.sample_count,reset_count=self.reset_count,
            horizons_s=(np.arange(1,count+1)*step_s).tolist(),models=[],physical_reachable_set=None,
            scope='Finite sampled trajectories only; not calibrated probability regions or a deterministic reachable outer bound. Mode weights describe filtering posteriors, not survival-conditioned future probabilities. Constraints audited at discrete substeps, no continuous-time proof.')
        if self.latest is None:return result
        if now<self.latest.received_at-1e-9:raise ValueError('Cannot use an observation before receipt')
        age=max(0.,now-self.latest.stamp)
        result.update(observation_age_s=age,latest_observation_stamp=self.latest.stamp,
                      observed_duration_s=self.latest.stamp-self.first_stamp,estimate_time=float(now))
        if age>.2+1e-9:
            result['status']='stale';return result
        if self.sample_count<5 or self.latest.stamp-self.first_stamp<.4-1e-9:
            return result
        rng=np.random.default_rng(seed)
        status='ready'
        for weight,source in zip(self.weights,self.filters):
            f=source.clone()
            try:
                f.predict(age)
            except LocalAngleError:
                status='degraded'
                result['models'].append(dict(name=f.mode.name,posterior_weight=float(weight),state=None,
                    covariance=None,state_time=None,propagation_error='unsupported_angular_spread',
                    trajectories=[],attempted=0,rejected=0,requested=particles_per_mode,sample_envelope=None))
                continue
            samples=sample_paths(f,np.array(result['horizons_s']),particles_per_mode,rng,limits)
            if len(samples['trajectories'])<particles_per_mode:status='degraded'
            result['models'].append(dict(name=f.mode.name,posterior_weight=float(weight),
                state=f.mean.tolist(),covariance=f.covariance.tolist(),state_time=float(now),mean_constraint_violations=violations(f.mean,f.mode,limits),
                angle_assumption='local Gaussian; sigma spread checked before wrapping',**samples))
        result['status']=status
        return result
