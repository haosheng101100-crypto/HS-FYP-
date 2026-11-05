"""Causal acceleration sharing and fixed-deadline receding-horizon interception.

Simulation research only. This is analytic endpoint-constrained replanning,
not nonlinear MPC, learned control, or a contact stability proof.
"""
from dataclasses import dataclass, asdict
import math
from .contact_geometry import Pose, RigidMotion, Deck, add, scale, sub, norm, evaluate_airframe
from .touchdown_boundary import ContactDesignLimits, TouchdownRequest, solve_touchdown_boundary
from .septic_approach import PlanarSnapMatchedApproach, PlanarSepticApproach, NinthScalar
from .moving_platform import check_moving_reference


@dataclass(frozen=True)
class SharedDeckState:
    sequence: int
    sample_sim_t_s: float
    position_m: tuple
    velocity_m_s: tuple
    acceleration_m_s2: tuple
    source: str = 'causal quadratic fit of platform truth; simulated shared state'

    def __post_init__(self):
        if not isinstance(self.sequence, int) or self.sequence < 0:
            raise ValueError('Nonnegative sequence required')
        if not math.isfinite(self.sample_sim_t_s): raise ValueError('Finite timestamp required')
        for name in ('position_m', 'velocity_m_s', 'acceleration_m_s2'):
            v = tuple(float(x) for x in getattr(self, name))
            if len(v) != 3 or not all(math.isfinite(x) for x in v): raise ValueError('Finite 3D state required')
            object.__setattr__(self, name, v)
        if abs(self.acceleration_m_s2[0]) > 1.0 or max(abs(x) for x in self.acceleration_m_s2[1:]) > .08:
            raise ValueError('Acceleration outside this level forward scene')

    def deck_at(self, now, half_extents=(1., .7), max_age=.12):
        age = now-self.sample_sim_t_s
        if not 0 <= age <= max_age: raise ValueError('Future or stale shared platform state')
        return predict_deck(Deck(RigidMotion(Pose(self.position_m, (0,0,0,1)), self.velocity_m_s, (0,0,0)), half_extents), self.acceleration_m_s2, age)


class SharedStateReceiver:
    def __init__(self): self.latest=None;self.history=[]
    def accept(self, packet):
        if self.latest is not None and (packet.sequence <= self.latest.sequence or packet.sample_sim_t_s <= self.latest.sample_sim_t_s):
            raise ValueError('Duplicate or out-of-order shared state')
        self.latest=packet;self.history.append(packet);self.history=self.history[-128:]
    def at(self,t):
        packet=next((p for p in reversed(self.history) if p.sample_sim_t_s<=t),None)
        if packet is None or t-packet.sample_sim_t_s>.12:raise ValueError('No fresh causal packet at requested time')
        return packet


def predict_deck(deck, acceleration, dt):
    if not math.isfinite(dt) or dt < 0: raise ValueError('Nonnegative finite horizon required')
    if norm(deck.motion.angular_velocity_world_rad_s) > .01 or norm(sub(deck.motion.pose.rotate((0,0,1)), (0,0,1))) > .01:
        raise ValueError('Level nonrotating platform required')
    if len(acceleration)!=3 or not all(math.isfinite(x) for x in acceleration): raise ValueError('Finite acceleration required')
    p=add(deck.motion.pose.position_world_m, add(scale(deck.motion.velocity_at_origin_world_m_s,dt),scale(acceleration,.5*dt*dt)))
    v=add(deck.motion.velocity_at_origin_world_m_s,scale(acceleration,dt))
    return Deck(RigidMotion(Pose(p,(0,0,0,1)),v,(0,0,0)),deck.half_extents_m)


def acceleration_boundary(geometry,deck,acceleration,closing=.45,gravity=9.80665):
    if abs(acceleration[1])+abs(acceleration[2])>1e-8: raise ValueError('Planar forward acceleration only')
    # Zero relative horizontal acceleration at impact requires a_uav = a_deck.
    theta=-math.atan2(acceleration[0],gravity)
    return solve_touchdown_boundary(geometry,deck,TouchdownRequest.planar(theta,0.,(0.,0.),(0.,0.,-closing)),
        ContactDesignLimits(math.radians(5),math.radians(5),.05,.25,.65,.02))


class _Segments:
    def __init__(self,axis): self.pieces=[(0.,axis)]
    def at(self,t,order=0):
        return sum(a.at(max(0.,t-s),order) for s,a in self.pieces if t>=s-1e-12)
    @property
    def coefficients(self): return self.pieces[0][1].coefficients


class SnapAxis:
    """Exact fourth-order integration of bounded piecewise constant snap."""
    def __init__(self,initial,snaps,duration):
        self.T=duration;self.snaps=tuple(float(x) for x in snaps);self.dt=duration/len(snaps)
        self.jets=[tuple(float(x) for x in initial[:4])]
        for s in self.snaps:
            p,v,a,j=self.jets[-1];h=self.dt
            self.jets.append((p+v*h+a*h*h/2+j*h**3/6+s*h**4/24,
                              v+a*h+j*h*h/2+s*h**3/6,a+j*h+s*h*h/2,j+s*h))
    @property
    def coefficients(self):return self.snaps
    def at(self,t,order=0):
        if t< -1e-9 or t>self.T+1e-9:raise ValueError('No continuation beyond contact')
        i=min(len(self.snaps)-1,int(max(0,t)/self.dt));h=max(0,t)-i*self.dt
        p,v,a,j=self.jets[i];s=self.snaps[i]
        return (p+v*h+a*h*h/2+j*h**3/6+s*h**4/24,v+a*h+j*h*h/2+s*h**3/6,a+j*h+s*h*h/2,j+s*h,s)[order]


def constrained_snap_axis(initial,terminal,duration,limits,near_tail_s=.28):
    """Convex quadratic program: minimum integrated squared jerk + snap.

    Exact linear integration, sampled v/a/j/s constraints, terminal p/v/a/j.
    Solver success is followed by independent dense geometric checks.
    """
    import numpy as np
    from scipy.optimize import minimize,LinearConstraint,linprog
    n=60;dt=duration/n
    zero=SnapAxis(initial,[0.]*n,duration)
    ts=np.linspace(0,duration,2*n+1)
    b=np.array([[zero.at(float(t),o) for o in (0,1,2,3)] for t in ts])
    M=np.empty((len(ts),4,n))
    for k in range(n):
        unit=[0.]*n;unit[k]=1.;axis=SnapAxis((0,0,0,0),unit,duration)
        for i,t in enumerate(ts):M[i,:,k]=[axis.at(float(t),o) for o in (0,1,2,3)]
    end=M[-1];rhs=np.asarray(terminal[:4])-b[-1]
    # Start/end snap zero makes the lead-in and final contact jet consistent.
    E=np.vstack((end,np.eye(n)[0],np.eye(n)[-1]));er=np.r_[rhs,0.,0.]
    matrices=[];lower=[];upper=[]
    for o,lo,hi in [(1,0.,limits['max_horizontal_speed_m_s']-.08),(2,-limits['max_horizontal_acceleration_m_s2']+.06,limits['max_horizontal_acceleration_m_s2']-.06),(3,-3.2,3.2)]:
        matrices.append(M[:,o]);lower.extend(lo-b[:,o]);upper.extend(hi-b[:,o])
    C=np.vstack(matrices);lo=np.array(lower);hi=np.array(upper)
    bound=limits['max_snap_m_s4']-.5;bounds=[(-bound,bound)]*n
    feasible=linprog(np.zeros(n),A_ub=np.vstack((C,-C)),b_ub=np.r_[hi,-lo],A_eq=E,b_eq=er,bounds=bounds,method='highs')
    if not feasible.success:raise ValueError('Bounded snap interception infeasible')
    H=(M[:,3].T@M[:,3])*dt+np.eye(n)*.025
    linear=(M[:,3].T@b[:,3])*dt
    fit=minimize(lambda x:float(.5*x@H@x+linear@x),feasible.x,jac=lambda x:H@x+linear,
        constraints=[LinearConstraint(E,er,er),LinearConstraint(C,lo,hi)],bounds=bounds,method='SLSQP',options={'maxiter':150,'ftol':1e-8})
    # A feasible LP is not substituted for a failed quadratic optimization.
    if not fit.success or np.max(np.abs(E@fit.x-er))>1e-6 or np.max(C@fit.x-hi)>1e-6 or np.max(lo-C@fit.x)>1e-6:
        raise ValueError('Minimum jerk/snap QP did not converge: '+str(fit.message))
    axis=SnapAxis(initial,fit.x,duration)
    axis.optimization={'method':'linear-constrained minimum jerk/snap quadratic objective, SLSQP','intervals':n,'objective':float(fit.fun),'terminal_residual':float(np.max(np.abs(E@fit.x-er))),'solver_success':True}
    return axis


class RollingAccelerationApproach(PlanarSepticApproach):
    """Replan from current commanded derivatives through snap, fixed touchdown.

    Updates use received current p/v/a only. No future platform command is
    exposed. Vertical descent is also retargeted for changed contact geometry.
    """
    def __init__(self,curve,geometry,acceleration,limits):
        self.__dict__.update(curve.__dict__)
        self.axes=tuple(_Segments(a) for a in curve.axes)
        self.geometry=geometry;self.acceleration=tuple(acceleration);self.limits=limits
        self.updates=[];self.last_attempt=-math.inf

    def retarget(self,t,deck_now,acceleration,force=False):
        remaining=self.T-t
        if t<0 or remaining<=0: raise ValueError('Replanning only before touchdown deadline')
        if remaining<1.0 or t-self.last_attempt<.24:return False
        self.last_attempt=t
        terminal=predict_deck(deck_now,acceleration,remaining)
        boundary=acceleration_boundary(self.geometry,terminal,acceleration,gravity=self.gravity)
        p=boundary.aircraft.pose.transform(self.com_offset_body_m)
        v=boundary.aircraft.velocity_at_world_point(p)
        old=self.at(self.T)
        mismatch=norm(sub(p,old['com_position_world_m']))
        velocity_mismatch=norm(sub(v,old['com_velocity_world_m_s']))
        if not force and mismatch<.002 and velocity_mismatch<.003 and norm(sub(acceleration,self.acceleration))<.003:return False
        new=[]
        for k,axis in enumerate(self.axes):
            endpoint=(p[k],v[k],acceleration[k],0.,0.)
            correction=tuple(endpoint[o]-axis.at(self.T,o) for o in range(5))
            new.append(NinthScalar((0.,)*5,correction,remaining))
        # Check candidate remaining segment before changing the active reference.
        probe=PlanarSepticApproach.__new__(PlanarSepticApproach)
        for axis,newaxis in zip(self.axes,new):axis.pieces.append((t,newaxis))
        class Window:
            def __init__(self,axis,start):self.axis=axis;self.start=start
            def at(self,dt,order=0):return self.axis.at(self.start+dt,order)
        probe.__dict__.update(self.__dict__);probe.axes=tuple(Window(a,t) for a in self.axes);probe.T=remaining
        report=check_moving_reference(probe,self.geometry,deck_now,self.limits,61,
                                     lambda dt:predict_deck(deck_now,acceleration,dt))
        if not report['passed']:
            for axis in self.axes:axis.pieces.pop()
            raise ValueError('Replanned remaining segment infeasible: '+str(report['failures']))
        self.terminal_boundary=boundary;self.acceleration=tuple(acceleration)
        self.updates.append({'t_s':t,'remaining_s':remaining,'terminal_position_correction_m':mismatch,
            'velocity_correction_m_s':velocity_mismatch,'acceleration_m_s2':acceleration,
            'segments':[list(a.coefficients) for a in new], 'candidate_checks':report,
            'terminal_boundary':asdict(boundary)})
        return True


def check_accelerating_reference(curve,geometry,deck,acceleration,limits,samples=1001):
    result=check_moving_reference(curve,geometry,deck,limits,samples,
                                 lambda t:predict_deck(deck,acceleration,t))
    crossing=None;slow=0.;last=0.;relative_acc=[]
    for i in range(samples):
        t=curve.T*i/(samples-1);s=curve.at(t)
        actual=predict_deck(deck,acceleration,t)
        gap=evaluate_airframe(geometry,s['motion'],actual)[0].clearance_m
        if gap<=.5:
            if crossing is None:crossing=t
            if -s['motion'].velocity_at_origin_world_m_s[2]<.15:slow+=t-last
        relative_acc.append(s['com_acceleration_world_m_s2'][0]-acceleration[0]);last=t
    rapid={'time_last_0_5m_s':None if crossing is None else curve.T-crossing,'slow_descent_s':slow}
    rapid['passed']=crossing is not None and rapid['time_last_0_5m_s']<=1.5 and slow<=.1
    result['rapid_motion']=rapid;result['passed']=result['passed'] and rapid['passed']
    result['relative_braking_present']=min(relative_acc)<-.05
    result['terminal_world_acceleration_m_s2']=curve.at(curve.T)['com_acceleration_world_m_s2']
    return result


def plan_accelerating(root,geometry,profile,deck_at_planning,acceleration,cfg,durations=None,validate=True):
    if not cfg['simulation_only'] or cfg['hardware_deployment_allowed']:raise ValueError('SITL only')
    if not .3<=acceleration[0]<=.7 or abs(acceleration[1])+abs(acceleration[2])>1e-8:raise ValueError('Acceleration outside v1 scenario')
    limits=cfg['design_limits'];lead=cfg['lead_s'];offset=profile['nominal_com_in_body_m'];tried=[]
    for T in (durations or cfg['duration_grid_s']):
        terminal=predict_deck(deck_at_planning,acceleration,lead+T)
        boundary=acceleration_boundary(geometry,terminal,acceleration)
        base=PlanarSnapMatchedApproach((0.,offset[1],3.65),(0.,0.,-.4),boundary,offset,9.80665,profile['mass_kg'],T,terminal_com_snap_world_m_s4=(0,0,0))
        try:
            end=base.axes[0].terminal
            xaxis=constrained_snap_axis((0,0,0,0),end,T,limits)
            base.axes=(xaxis,*base.axes[1:])
        except ValueError as exc:
            tried.append({'duration_s':T,'reason':str(exc)})
            continue
        main=predict_deck(deck_at_planning,acceleration,lead)
        check=check_accelerating_reference(base,geometry,main,acceleration,limits,201) if validate else {'passed':False,'pending_dense_audit':True}
        tried.append({'duration_s':T,'checks':check})
        if validate and not check['passed']:continue
        if validate:
            check=check_accelerating_reference(base,geometry,main,acceleration,limits,2001)
            if not check['passed']:continue
        curve=RollingAccelerationApproach(base,geometry,acceleration,limits)
        return curve,{'simulation_only':True,'duration_s':T,'lead_s':lead,'deck_at_planning':asdict(deck_at_planning),
            'shared_acceleration_m_s2':acceleration,'target_contact_nose_up_deg':-math.degrees(math.atan2(acceleration[0],9.80665)),
            'terminal_boundary':asdict(boundary),'normalized_polynomial_coefficients':[a.coefficients for a in curve.axes],
            'reference_checks':check,'duration_candidates':tried,'configuration':cfg,'horizontal_optimization':xaxis.optimization,
            'horizontal_snap_intervals':list(xaxis.snaps)}
    raise ValueError('No feasible acceleration-aware intercept: '+str(tried))


def restore_accelerating_plan(geometry,profile,report):
    m=report['deck_at_planning']['motion']
    deck=Deck(RigidMotion(Pose(**m['pose']),m['velocity_at_origin_world_m_s'],m['angular_velocity_world_rad_s']), (1.,.7))
    a=report['shared_acceleration_m_s2'];T=report['duration_s']
    terminal=predict_deck(deck,a,T+report['lead_s']);boundary=acceleration_boundary(geometry,terminal,a)
    offset=profile['nominal_com_in_body_m']
    base=PlanarSnapMatchedApproach((0.,offset[1],3.65),(0,0,-.4),boundary,offset,9.80665,profile['mass_kg'],T,terminal_com_snap_world_m_s4=(0,0,0))
    base.axes=(SnapAxis((0,0,0,0),report['horizontal_snap_intervals'],T),*base.axes[1:])
    return RollingAccelerationApproach(base,geometry,a,report['configuration']['design_limits'])
