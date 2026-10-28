"""Causal quadratic fit to Gazebo body poses; simulation truth, not a sensor filter."""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np
from .contact_geometry import Pose,RigidMotion,add,cross,vector
from .execution import AircraftObservation


class ObservationUnavailable(ValueError):pass


@dataclass(frozen=True)
class PoseSample:
    sim_t_s:float
    pose:Pose
    received_monotonic_s:float


class CausalPoseHistory:
    def __init__(self,com_offset_body_m,span_s=.081,minimum_samples=5,minimum_span_s=.04,
                 maximum_sample_gap_s=.025,maximum_sim_age_s=.06,maximum_wall_age_s=.5,heartbeat_max_wall_age_s=1.5):
        self.offset=vector(com_offset_body_m);self.samples=deque(maxlen=128)
        self.span=span_s;self.minimum_samples=minimum_samples;self.minimum_span=minimum_span_s
        self.max_gap=maximum_sample_gap_s;self.sim_age=maximum_sim_age_s;self.wall_age=maximum_wall_age_s
        self.last_fit=None
        self.heartbeat_wall_age=heartbeat_max_wall_age_s

    def push(self,sample):
        if not all(math.isfinite(x) for x in [sample.sim_t_s,sample.received_monotonic_s]):raise ValueError('Finite observation stamps required')
        if self.samples and sample.sim_t_s<self.samples[-1].sim_t_s:raise ValueError('Pose clock moved backwards')
        if self.samples and sample.sim_t_s==self.samples[-1].sim_t_s:
            if sample.pose!=self.samples[-1].pose:raise ValueError('Conflicting poses at same timestamp')
            return
        self.samples.append(sample)

    def estimate(self,now_sim,now_wall,armed,mode,heartbeat_received_wall):
        if not all(math.isfinite(x) for x in [now_sim,now_wall,heartbeat_received_wall]):raise ObservationUnavailable('Invalid clock/heartbeat stamp')
        if not 0<=now_wall-heartbeat_received_wall<=self.heartbeat_wall_age:raise ObservationUnavailable('Flight-mode heartbeat stale')
        preceding=[s for s in self.samples if s.sim_t_s<=now_sim]
        if not preceding:raise ObservationUnavailable('No causal body pose')
        latest=preceding[-1]
        if not 0<=now_sim-latest.sim_t_s<=self.sim_age:raise ObservationUnavailable('Body pose stale in simulation time')
        if not 0<=now_wall-latest.received_monotonic_s<=self.wall_age:
            raise ObservationUnavailable('Body pose wall age invalid: '+repr(now_wall-latest.received_monotonic_s))
        rows=[s for s in preceding if s.sim_t_s>=latest.sim_t_s-self.span]
        if len(rows)<self.minimum_samples or rows[-1].sim_t_s-rows[0].sim_t_s<self.minimum_span-1e-9:
            raise ObservationUnavailable('Causal fit warming up')
        if max(b.sim_t_s-a.sim_t_s for a,b in zip(rows,rows[1:]))>self.max_gap:
            raise ObservationUnavailable('Gap in causal pose window')
        dt=np.array([s.sim_t_s-latest.sim_t_s for s in rows])
        def matrix(pose):return np.array([pose.rotate(axis) for axis in [(1,0,0),(0,1,0),(0,0,1)]]).T
        P=np.array([s.pose.position_world_m for s in rows]);Rs=np.array([matrix(s.pose) for s in rows])
        pc=np.polyfit(dt,P,2);rc=np.polyfit(dt,Rs.reshape(-1,9),2)
        R=matrix(latest.pose);Rdot=rc[1].reshape(3,3);Rddot=2*rc[0].reshape(3,3)
        W=Rdot@R.T;W=(W-W.T)/2
        A=Rddot@R.T;A=(A-A.T)/2
        omega=tuple(float(x) for x in [W[2,1],W[0,2],W[1,0]])
        alpha=tuple(float(x) for x in [A[2,1],A[0,2],A[1,0]])
        lever=latest.pose.rotate(self.offset)
        com_acc=add(tuple(float(x) for x in 2*pc[0]),add(cross(alpha,lever),cross(omega,cross(omega,lever))))
        self.last_fit={'end_sim_t_s':latest.sim_t_s,'begin_sim_t_s':rows[0].sim_t_s,'samples':len(rows),
            'span_s':float(-dt[0]),'body_acceleration_world_m_s2':tuple(float(x) for x in 2*pc[0]),
            'angular_acceleration_world_rad_s2':alpha,'fit_kind':'causal quadratic pose/matrix fit; finite-window estimate'}
        return AircraftObservation(latest.sim_t_s,RigidMotion(latest.pose,tuple(float(x) for x in pc[1]),omega),armed,mode,com_acc)
