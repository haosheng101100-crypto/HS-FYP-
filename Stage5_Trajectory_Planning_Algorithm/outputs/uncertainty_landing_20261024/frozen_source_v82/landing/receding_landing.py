"""Three-dimensional bounded landing planner. Simulation research, no actuation.

Fixed-time septic Bezier SOCPs + finite arrival-time search. Convex-hull
certificates apply to nominal translational limits. Foot/attitude checks are
dense sampled checks, not a continuous collision or robust feasibility proof.
"""
from dataclasses import dataclass, field, replace, asdict
from math import comb, factorial
import math
import time
import numpy as np
from .contact_geometry import (Pose, RigidMotion, Deck, evaluate_airframe,
    quaternion_from_rpy)
from .touchdown_boundary import ContactDesignLimits, TouchdownRequest, solve_touchdown_boundary, InfeasibleTouchdown


def vec(x):
    a = np.asarray(x, dtype=float)
    if a.shape != (3,) or not np.isfinite(a).all():
        raise ValueError('Expected finite world-frame 3-vector')
    return a


@dataclass(frozen=True)
class FlightState:
    stamp: float
    position: tuple  # COM, ENU
    velocity: tuple
    acceleration: tuple

    def __post_init__(self):
        if not math.isfinite(self.stamp): raise ValueError('Invalid timestamp')
        for k in ('position', 'velocity', 'acceleration'):
            object.__setattr__(self, k, tuple(vec(getattr(self, k))))

    @classmethod
    def from_observation(cls, observation, com_offset):
        if observation.com_acceleration_world_m_s2 is None:
            raise ValueError('Measured COM acceleration required')
        m = observation.motion; lever = vec(m.pose.rotate(com_offset))
        return cls(observation.sim_t_s, tuple(vec(m.pose.position_world_m)+lever),
            tuple(vec(m.velocity_at_origin_world_m_s)+np.cross(m.angular_velocity_world_rad_s,lever)),
            observation.com_acceleration_world_m_s2)


@dataclass(frozen=True)
class PlatformState:
    stamp: float
    position: tuple
    velocity: tuple
    acceleration: tuple
    yaw: float = 0.
    yaw_rate: float = 0.
    half_extents: tuple = (1., .7)
    jerk: tuple = (0.,0.,0.)
    yaw_acceleration: float = 0.
    motion_horizon_s: float | None = None

    def __post_init__(self):
        for k in ('position', 'velocity', 'acceleration','jerk'):
            object.__setattr__(self, k, tuple(vec(getattr(self, k))))
        if not all(math.isfinite(x) for x in (self.stamp, self.yaw, self.yaw_rate,self.yaw_acceleration)):
            raise ValueError('Invalid platform time/heading')
        if len(self.half_extents) != 2 or any(not math.isfinite(x) or x <= 0 for x in self.half_extents):
            raise ValueError('Positive platform half extents required')
        if abs(self.velocity[2]) > 1e-8 or abs(self.acceleration[2]) > 1e-8 or abs(self.jerk[2])>1e-8:
            raise ValueError('First implementation requires a level, fixed-height deck')
        if self.motion_horizon_s is not None and (not math.isfinite(self.motion_horizon_s) or self.motion_horizon_s<0):
            raise ValueError('Finite nonnegative motion horizon required')

    def predict(self, horizon):
        """Causal local cubic translation and quadratic heading prediction.

        Turning translations are corrected on every observation; yaw rate alone
        does not supply a future translational route to this predictor.
        """
        if not math.isfinite(horizon) or horizon < 0: raise ValueError('Invalid horizon')
        if self.motion_horizon_s is not None:
            h=min(horizon,self.motion_horizon_s)
            local=replace(self,motion_horizon_s=None).predict(h)
            if horizon>=self.motion_horizon_s:
                return replace(local,stamp=self.stamp+horizon,position=tuple(vec(local.position)+(horizon-h)*vec(local.velocity)),acceleration=(0.,0.,0.),jerk=(0.,0.,0.),yaw_rate=0.,yaw_acceleration=0.,motion_horizon_s=0.)
            return replace(local,motion_horizon_s=self.motion_horizon_s-h)
        p = vec(self.position) + horizon*vec(self.velocity) + .5*horizon*horizon*vec(self.acceleration)+horizon**3*vec(self.jerk)/6
        v = vec(self.velocity) + horizon*vec(self.acceleration)+.5*horizon*horizon*vec(self.jerk)
        a=vec(self.acceleration)+horizon*vec(self.jerk)
        return PlatformState(self.stamp+horizon, tuple(p), tuple(v), tuple(a),
            self.yaw+horizon*self.yaw_rate+.5*horizon*horizon*self.yaw_acceleration, self.yaw_rate+horizon*self.yaw_acceleration, self.half_extents,self.jerk,self.yaw_acceleration)

    def deck(self):
        return Deck(RigidMotion(Pose(self.position, quaternion_from_rpy(0, 0, self.yaw)),
            self.velocity, (0., 0., self.yaw_rate)), self.half_extents)


@dataclass(frozen=True)
class PlannerLimits:
    speed: float = 6.
    acceleration: float = 3.
    jerk: float = 8.
    tilt_rad: float = math.radians(18)
    upward_speed: float = 1.
    downward_speed: float = 1.
    vertical_acceleration: float = 1.5
    contact_speed: float = .1
    footprint_radius: float = .55  # Declared full-body disk envelope; configure for model.
    margin: float = .02
    position_reserve: float = .01  # Design allowance, not identified robust bound.
    corridor_slope: float = 1.
    minimum_vertical_specific_force: float = 5.
    maximum_specific_force: float = 13.
    max_age: float = .06
    max_platform_acceleration: float = 2.6
    max_platform_yaw_rate: float = 1.
    segments: int = 3
    audit_samples: int = 301
    prediction_horizon_s: float = 15.
    terminal_coverage_s: float = .01
    tolerance: float = 2e-5
    landing_point_trust_radius_m: float = .015
    landing_point_weight: float = 100.
    reaction_horizon_s: float = .2
    entry_acceleration_capture_m_s2: float | None = None
    directional_corridor: bool = False
    tracking_error_tightening: bool = False
    tracking_feedback_max_age_s: float = .6

    def __post_init__(self):
        for k, v in vars(self).items():
            if k in ('directional_corridor','tracking_error_tightening'):
                if type(v) is not bool:raise ValueError('Boolean directional corridor required')
                continue
            if k=='entry_acceleration_capture_m_s2' and v is None:continue
            if not math.isfinite(v) or v <= 0: raise ValueError('Positive finite limit required: '+k)
        if self.entry_acceleration_capture_m_s2 is not None and self.entry_acceleration_capture_m_s2<self.acceleration:
            raise ValueError('Entry capture must preserve the nominal acceleration range')
        if type(self.segments) is not int or not 1 <= self.segments <= 8:
            raise ValueError('One to eight segments required')
        if type(self.audit_samples) is not int or self.audit_samples < 101:
            raise ValueError('At least 101 audit samples required')
        if self.tilt_rad >= math.pi/2: raise ValueError('Tilt must be below 90 degrees')


def derivatives(points, duration, order=1):
    n = len(points)-1
    if order > n: return np.zeros((1, 3))
    return factorial(n)/factorial(n-order)/duration**order * np.diff(points, n=order, axis=0)


def basis(n, s):
    return np.array([comb(n, i)*s**i*(1-s)**(n-i) for i in range(n+1)])


def gram(n):
    return np.array([[comb(n, i)*comb(n, j)/((2*n+1)*comb(2*n, i+j))
        for j in range(n+1)] for i in range(n+1)])


def elevation_matrix(n,m):
    if m<n:raise ValueError('Cannot lower Bernstein degree')
    return np.array([[comb(n,i)*comb(m-n,j-i)/comb(m,j) if 0<=j-i<=m-n else 0.
        for i in range(n+1)] for j in range(m+1)])


@dataclass
class LandingPlan:
    start: float
    duration: float
    points: np.ndarray
    target_position: tuple
    target_velocity: tuple
    com_offset: tuple
    cost: float
    audit: dict = field(default_factory=dict)

    @property
    def arrival(self): return self.start+self.duration

    def at(self, elapsed):
        if not math.isfinite(elapsed) or not 0 <= elapsed <= self.duration+1e-9:
            raise ValueError('Airborne reference must not be extrapolated beyond contact')
        dt = self.duration/len(self.points)
        index = min(int(elapsed/dt), len(self.points)-1)
        s = min(1., max(0., (elapsed-index*dt)/dt))
        c = self.points[index]
        return {name: basis(7-order, s) @ derivatives(c, dt, order)
            for name, order in [('position', 0), ('velocity', 1), ('acceleration', 2), ('jerk', 3)]}

    def motion(self, elapsed):
        """Nominal body state derived from COM flat outputs and fixed yaw=0."""
        s = self.at(elapsed); f = s['acceleration'] + np.array([0., 0., 9.80665])
        bz = f/np.linalg.norm(f); xc = np.array([1., 0., 0.])
        by = np.cross(bz, xc); by /= np.linalg.norm(by); bx = np.cross(by, bz)
        R = np.column_stack((bx, by, bz))
        # scipy is imported only when the new planner's geometry audit is used.
        from scipy.spatial.transform import Rotation
        q = tuple(Rotation.from_matrix(R).as_quat())
        dz = (np.eye(3)-np.outer(bz, bz)) @ s['jerk']/np.linalg.norm(f)
        raw_y = np.cross(bz, xc)
        dy = (np.eye(3)-np.outer(by, by)) @ np.cross(dz, xc)/np.linalg.norm(raw_y)
        dx = np.cross(dy, bz)+np.cross(by, dz)
        W = np.column_stack((dx, dy, dz)) @ R.T
        omega = np.array([W[2, 1]-W[1, 2], W[0, 2]-W[2, 0], W[1, 0]-W[0, 1]])/2
        lever = R @ vec(self.com_offset)
        return RigidMotion(Pose(tuple(s['position']-lever), q),
            tuple(s['velocity']-np.cross(omega, lever)), tuple(omega))

    def body_reference_at(self, now):
        """Adapter for existing MAVLink dispatcher; never sends a command."""
        from .execution import BodyReference
        elapsed = now-self.start; m = self.motion(elapsed)
        h = min(1e-4, self.duration/10000)
        lo = max(0.,elapsed-h); hi = min(self.duration,elapsed+h)
        alpha = (vec(self.motion(hi).angular_velocity_world_rad_s)-vec(self.motion(lo).angular_velocity_world_rad_s))/(hi-lo)
        lever = vec(m.pose.rotate(self.com_offset)); omega = vec(m.angular_velocity_world_rad_s)
        acceleration = self.at(elapsed)['acceleration']-np.cross(alpha,lever)-np.cross(omega,np.cross(omega,lever))
        return BodyReference(m.pose.position_world_m, m.velocity_at_origin_world_m_s,tuple(acceleration),0.)


class BezierLandingPlanner:
    def __init__(self, geometry, com_offset, limits=None, envelope=None):
        self.geometry = geometry
        self.offset = tuple(vec(com_offset))
        self.limits = limits or PlannerLimits()
        self.envelope=envelope
        if self.limits.tracking_error_tightening and not (self.limits.directional_corridor and envelope):
            raise ValueError('Tracking feedback requires the full rotating rectangle envelope')
        self.footprint_tilt_coefficient=0. if envelope is None else envelope.tilt_projection_coefficient(self.offset,self.limits.tilt_rad,max(self.limits.minimum_vertical_specific_force,9.80665-self.limits.vertical_acceleration))
        self._template = None
        self._template_limits = None

    def _entry(self, state, platform, now):
        L = self.limits
        if not math.isfinite(now): raise ValueError('Invalid decision time')
        if any(not 0 <= now-t <= L.max_age for t in (state.stamp, platform.stamp)):
            raise ValueError('Stale or future state')
        if np.linalg.norm(state.velocity) > L.speed+L.tolerance:
            raise ValueError('Initial speed outside planning envelope')
        entry_cap=L.entry_acceleration_capture_m_s2 or L.acceleration
        if np.linalg.norm(state.acceleration)>entry_cap+L.tolerance:
            raise ValueError('Initial acceleration outside declared capture bound')
        if np.linalg.norm(platform.acceleration) > L.max_platform_acceleration:
            raise ValueError('Platform acceleration outside declared envelope')
        if abs(platform.yaw_rate) > L.max_platform_yaw_rate:
            raise ValueError('Platform turn rate outside declared envelope')
        # Conservative yaw-independent corridor using an inscribed deck disk.
        radius = min(platform.half_extents)-L.footprint_radius-L.margin-L.position_reserve
        if self.envelope:radius=self.envelope.safe_radius(platform,0.,self.offset,L.margin+L.position_reserve)
        if radius <= 0: raise ValueError('No usable landing area after footprint and reserves')
        age = now-state.stamp
        p = vec(state.position)+age*vec(state.velocity)+.5*age**2*vec(state.acceleration)
        v = vec(state.velocity)+age*vec(state.acceleration)
        return FlightState(now, tuple(p), tuple(v), state.acceleration), platform.predict(now-platform.stamp), radius

    def _tracking_feedback(self,data,now):
        if data is None:return None
        if not self.limits.tracking_error_tightening:raise ValueError('Tracking-error tightening not enabled')
        try:
            error=vec(data['position_error_world_m']);reserve=float(data['attitude_reserve_m']);stamp=float(data['stamp_s'])
            source_start=float(data['source_plan_start_s']);arrival=float(data['source_plan_arrival_s'])
            horizon=float(data['propagation_horizon_s'])
        except (KeyError,TypeError,ValueError):raise ValueError('Valid tracking feedback payload required')
        if abs(error[2])>1e-12 or not all(math.isfinite(v) for v in (reserve,stamp,source_start,arrival,horizon)) or reserve<0 or horizon<0:
            raise ValueError('Finite planar tracking error and nonnegative attitude reserve required')
        if not 0<=now-stamp<=self.limits.tracking_feedback_max_age_s+1e-9 or not source_start<=stamp<arrival:
            raise ValueError('Tracking feedback must precede the future planning start')
        return data

    def solve(self, state, platform, now, duration, uncertainty=None,tracking_envelope=None):
        """One fixed-time SOCP; no flight permissions, no implicit fallback."""
        import cvxpy as cp
        started = time.perf_counter(); L = self.limits
        tracking_envelope=self._tracking_feedback(tracking_envelope,now)
        if self._template_limits != L: self._template = None
        state, platform, radius = self._entry(state, platform, now)
        if not math.isfinite(duration) or duration < .2 or duration > 15:
            raise ValueError('Duration outside [.2,15] s')
        if duration>2.:
            # High derivatives are local estimates, not an eight-second route.
            platform=replace(platform,jerk=(0.,0.,0.),yaw_acceleration=0.)
        if duration>L.prediction_horizon_s:
            dt=duration/L.segments
            cutoff=max(1,math.floor(L.prediction_horizon_s/dt))*dt
            platform=replace(platform,motion_horizon_s=cutoff)
        terminal = platform.predict(duration)
        # Fixed world heading and level touchdown. Relative yaw cancels deck yaw.
        request = TouchdownRequest(quaternion_from_rpy(0, 0, -terminal.yaw),
            (0., 0., -terminal.yaw_rate), (0., 0.), (0., 0., -L.contact_speed))
        contact_limits = ContactDesignLimits(.01, L.max_platform_yaw_rate+.01,
            .05, L.contact_speed*.5, L.contact_speed*1.5, L.margin)
        try:
            boundary = solve_touchdown_boundary(self.geometry, terminal.deck(), request, contact_limits)
            if self.envelope:
                deck=terminal.deck();desired=self.envelope.centered_body_xy_in_deck(deck)
                current=deck.motion.pose.inverse_rotate(vec(boundary.aircraft.pose.position_world_m)-vec(terminal.position))
                request=replace(request,support_centroid_deck_xy_m=tuple(desired[k]-current[k] for k in (0,1)))
                boundary=solve_touchdown_boundary(self.geometry,deck,request,contact_limits)
        except InfeasibleTouchdown as e:
            return None, {'status': 'CONTACT_BOUNDARY_REJECTED', 'duration_s': duration, 'detail': str(e)}
        target = vec(boundary.aircraft.pose.position_world_m)+vec(self.offset)
        target_v = vec(boundary.aircraft.velocity_at_origin_world_m_s)
        if self.envelope:
            shift=target-vec(terminal.position);shift[2]=0.
            radius=self.envelope.safe_radius(platform,duration,self.offset,L.margin+L.position_reserve,shift)
            if radius<=0 and not L.directional_corridor:return None,dict(status='BODY_FOOTPRINT_REJECTED',duration_s=duration)
        from .prediction_uncertainty import reaction_reserve
        reserve = reaction_reserve(uncertainty, now, duration, L.reaction_horizon_s,
                                   self.envelope.radius if self.envelope else L.footprint_radius)
        # The old fixed position allowance and the empirical forecast reserve
        # describe the same prediction-placement error. Keep the larger one;
        # the independent physical/contact margin is unchanged.
        radius -= max(0., reserve['reserve_m']-L.position_reserve)
        if radius <= 0 and not (L.directional_corridor and self.envelope):
            return None, dict(status='UNCERTAINTY_FOOTPRINT_REJECTED', duration_s=duration,
                              uncertainty_reserve=reserve)
        contact_height = target[2]-terminal.position[2]
        dt = duration/L.segments
        if self._template is not None:
            problem, C, P = self._template
            return self._run_template(state, platform, now, duration, radius, target, target_v, contact_height, started, reserve,tracking_envelope)
        P = {key: cp.Parameter(nonneg=True) for key in ('iv', 'ia', 'ij', 'wj', 'ws', 'radius')}
        P.update({key: cp.Parameter(3) for key in ('p0', 'v0_scaled', 'a0_scaled', 'pf', 'vf_scaled')})
        P['centers'] = cp.Parameter((L.segments*8, 3))
        P['height'] = cp.Parameter()
        P['slopes']=cp.Parameter(L.segments,nonneg=True)
        P['corridor_shift']=cp.Parameter(L.segments*8)
        P['offset_velocity_scaled'] = cp.Parameter((3, 2))
        directional=bool(L.directional_corridor and self.envelope)
        if directional:
            P['facet_axes']=cp.Parameter((L.segments*4,2))
            P['facet_chords']=cp.Parameter(L.segments,nonneg=True)
            P['facet_rhs']=cp.Parameter((L.segments*8,4))
            if L.tracking_error_tightening:P['tracking_facet_rhs']=cp.Parameter((8,4))
        C = [cp.Variable((8, 3)) for _ in range(L.segments)]
        landing_xy = cp.Variable(2, name='landing_point_world_offset')
        landing_radius = cp.Variable(nonneg=True, name='landing_point_radius')
        landing_xyz = cp.hstack([landing_xy, 0.])
        # Optimize landing point jointly with all trajectory control points.
        # Charging its norm against the corridor radius preserves the original
        # whole-body center reserve for every point in the trust region.
        cons = [cp.norm(landing_xy) <= landing_radius,
                landing_radius <= L.landing_point_trust_radius_m]
        cost = L.landing_point_weight*cp.sum_squares(landing_xy); ds = []
        G3 = gram(4); G4 = gram(3)
        for k, c in enumerate(C):
            raw = [c]
            for _ in range(4): raw.append(raw[-1][1:]-raw[-1][:-1])
            v = P['iv']*raw[1]; a = P['ia']*raw[2]; j = P['ij']*raw[3]
            ds.append(raw)
            for d, cap in ((v, L.speed), (j, L.jerk)):
                cons.append(cp.norm(d, axis=1) <= cap)
            if k==0 and L.entry_acceleration_capture_m_s2 is not None:
                # This first derivative control point is fixed by the exact
                # C2 entry state. All free acceleration points retain the
                # tighter nominal reserve; no entry state is clipped.
                cons += [cp.norm(a[0])<=L.entry_acceleration_capture_m_s2,
                    cp.norm(a[1:],axis=1)<=L.acceleration]
            else:cons.append(cp.norm(a,axis=1)<=L.acceleration)
            cons += [v[:, 2] <= L.upward_speed, v[:, 2] >= -L.downward_speed,
                cp.abs(a[:, 2]) <= L.vertical_acceleration,
                9.80665+a[:, 2] >= L.minimum_vertical_specific_force,
                cp.norm(a[:, :2], axis=1) <= math.tan(L.tilt_rad)*(9.80665+a[:, 2]),
                cp.norm(a+np.array([0., 0., 9.80665]), axis=1) <= L.maximum_specific_force]
            # Platform CA prediction is exactly representable in Bernstein degree 7.
            # Difference curve control points then certify the translating cone.
            elevated_a=elevation_matrix(5,7)@a
            if directional:
                center_distance=cp.Variable(8,nonneg=True,name=f'center_distance_{k}')
                cons.append(cp.norm(c[:,:2]-P['centers'][k*8:(k+1)*8,:2],axis=1)<=center_distance)
            for i in range(8):
                center = P['centers'][k*8+i]
                gap = c[i, 2]-center[2]-P['height']
                cons.append(gap>=0)
                if directional:
                    for f in range(4):
                        lhs=P['facet_axes'][k*4+f]@c[i,:2]+P['facet_chords'][k]*center_distance[i]+self.footprint_tilt_coefficient*cp.norm(elevated_a[i,:2])
                        cons.append(lhs <= P['facet_rhs'][k*8+i,f]+P['slopes'][k]*c[i,2])
                        if L.tracking_error_tightening and k==L.segments-1:
                            cons.append(lhs <= P['tracking_facet_rhs'][i,f]+P['slopes'][k]*c[i,2])
                else:
                    cons.append(cp.norm(c[i, :2]-center[:2]-landing_xy, 2)+landing_radius+self.footprint_tilt_coefficient*cp.norm(elevated_a[i,:2],2) <= P['radius']+P['slopes'][k]*c[i,2]+P['corridor_shift'][k*8+i])
            cost += sum(P['wj']*cp.quad_form(raw[3][:, ax], G3)+P['ws']*cp.quad_form(raw[4][:, ax], G4) for ax in range(3))
        for a, b in zip(ds, ds[1:]):
            cons += [x[-1] == y[0] for x, y in zip(a[:4], b[:4])]
        cons += [ds[0][0][0] == P['p0'], ds[0][1][0] == P['v0_scaled'],
            ds[0][2][0] == P['a0_scaled'], ds[-1][0][-1] == P['pf']+landing_xyz,
            ds[-1][1][-1] == P['vf_scaled']+P['offset_velocity_scaled']@landing_xy,
            ds[-1][2][-1] == 0, ds[-1][3][-1] == 0]
        problem = cp.Problem(cp.Minimize(cost), cons)
        if not problem.is_dpp(): raise RuntimeError('Expected reusable DPP conic template')
        self._template = (problem, C, P)
        self._landing_xy = landing_xy
        self._template_limits = L
        return self._run_template(state, platform, now, duration, radius, target, target_v, contact_height, started, reserve,tracking_envelope)

    def _run_template(self, state, platform, now, duration, radius, target, target_v, contact_height, started, reserve,tracking_envelope):
        import cvxpy as cp
        problem, C, P = self._template; L = self.limits; dt = duration/L.segments
        for key, value in dict(iv=7/dt, ia=42/dt**2, ij=210/dt**3,
            wj=210**2/dt**5, ws=.01*840**2/dt**7, radius=max(0.,radius), p0=vec(state.position),
            v0_scaled=vec(state.velocity)*dt/7, a0_scaled=vec(state.acceleration)*dt**2/42,
            pf=target, vf_scaled=target_v*dt/7, height=contact_height).items(): P[key].value = value
        omega = platform.predict(duration).yaw_rate
        P['offset_velocity_scaled'].value = np.array([[0., -omega], [omega, 0.], [0., 0.]])*dt/7
        centers = []
        shift=np.zeros(3)
        if self.envelope:
            shift=target-vec(platform.predict(duration).position);shift[2]=0.
        for k in range(L.segments):
            pp = platform.predict(k*dt)
            for i in range(8):
                centers.append(vec(pp.position)+shift+vec(pp.velocity)*(dt*i/7)+vec(pp.acceleration)*(.5*dt*dt*i*(i-1)/42)+vec(pp.jerk)*(dt**3*i*(i-1)*(i-2)/1260))
        P['centers'].value = np.array(centers)
        count=min(L.segments,max(1,math.ceil(L.terminal_coverage_s/dt)))
        slopes=np.array([0. if self.envelope and k>=L.segments-count else L.corridor_slope for k in range(L.segments)])
        P['slopes'].value=slopes
        P['corridor_shift'].value=-np.repeat(slopes,8)*(np.array(centers)[:,2]+contact_height)
        if L.directional_corridor and self.envelope:
            axes=[];chords=[];rhs=[]
            reserve_m=L.margin+max(L.position_reserve,reserve['reserve_m'])
            for k in range(L.segments):
                facets=self.envelope.corridor_facets(platform.predict(k*dt),dt,self.offset,reserve_m,shift)
                axes.extend([f['axis'][:2] for f in facets]);chords.append(facets[0]['heading_chord'])
                for i in range(8):
                    center=np.array(centers[k*8+i])
                    rhs.append([f['allowance_m']+np.array(f['axis'])@center-slopes[k]*(center[2]+contact_height) for f in facets])
            P['facet_axes'].value=np.array(axes);P['facet_chords'].value=np.array(chords);P['facet_rhs'].value=np.array(rhs)
            if L.tracking_error_tightening:
                error=np.zeros(3) if tracking_envelope is None else vec(tracking_envelope['position_error_world_m'])
                attitude=0. if tracking_envelope is None else tracking_envelope['attitude_reserve_m']
                # Fold products of numerical parameters outside CVXPY to retain
                # the reusable DPP template. Triangle inequality covers the
                # heading chord of the shifted terminal curve as well.
                correction=[np.array(f['axis'])@error+f['heading_chord']*np.linalg.norm(error[:2])+attitude for f in facets]
                P['tracking_facet_rhs'].value=np.array(rhs[-8:])-np.array(correction)
        try:
            problem.solve(solver='CLARABEL', max_iter=100, tol_gap_abs=1e-8, tol_feas=1e-8, enforce_dpp=True)
        except cp.error.SolverError as e:
            return None, {'status': 'SOLVER_ERROR', 'duration_s':duration, 'detail': str(e)}
        if problem.status not in ('optimal', 'optimal_inaccurate'):
            return None, {'status': problem.status, 'duration_s': duration,
                'elapsed_s': time.perf_counter()-started}
        delta = np.r_[self._landing_xy.value, 0.]
        target = target+delta
        target_v = target_v+np.cross((0., 0., omega), delta)
        plan = LandingPlan(now, duration, np.array([c.value for c in C]), tuple(target), tuple(target_v),
            self.offset, float(problem.value))
        plan.audit['landing_point_offset_world_m'] = delta.tolist()
        plan.audit['uncertainty_reserve'] = reserve
        if L.tracking_error_tightening:plan.audit['tracking_contact_envelope']=tracking_envelope
        plan.audit = self.audit(plan, state, platform, radius)
        plan.audit['uncertainty_reserve'] = reserve
        plan.audit['fixed_prediction_allowance_reused_m'] = min(L.position_reserve,reserve['reserve_m'])
        plan.audit['platform_prediction_input']=asdict(platform)
        record = {'status': 'ACCEPTED' if plan.audit['passed'] else 'AUDIT_REJECTED',
            'duration_s': duration, 'cost': plan.cost, 'audit': plan.audit,
            'elapsed_s': time.perf_counter()-started}
        record['solver_and_canonicalization_s'] = float((problem.compilation_time or 0.)+(problem.solver_stats.solve_time or 0.))
        return (plan if plan.audit['passed'] else None), record

    def audit(self, plan, state, platform, radius):
        """Independently recompute hull bounds and sampled physical-foot geometry."""
        L = self.limits; dt = plan.duration/len(plan.points); tol = L.tolerance
        peaks = dict(speed=0., acceleration=0., jerk=0., tilt_ratio=0., specific_force=0.)
        failed = []
        corridor_residual = -math.inf
        tracking_residual=-math.inf
        tracking=plan.audit.get('tracking_contact_envelope')
        tracking=self._tracking_feedback(tracking,plan.start)
        tracking_error=np.zeros(3) if tracking is None else vec(tracking['position_error_world_m'])
        tracking_attitude=0. if tracking is None else tracking['attitude_reserve_m']
        shift=np.zeros(3)
        landing_offset = vec(plan.audit.get('landing_point_offset_world_m', (0., 0., 0.)))
        if self.envelope:
            shift=vec(plan.target_position)-landing_offset-vec(platform.predict(plan.duration).position);shift[2]=0.
        contact_height = plan.target_position[2]-platform.position[2]
        for k, c in enumerate(plan.points):
            pp = platform.predict(k*dt)
            facets=None
            if L.directional_corridor and self.envelope:
                reserve_m=L.margin+max(L.position_reserve,plan.audit.get('uncertainty_reserve',{}).get('reserve_m',0.))
                facets=self.envelope.corridor_facets(pp,dt,self.offset,reserve_m,shift)
            elevated_a=elevation_matrix(5,7)@derivatives(c,dt,2)
            for i in range(8):
                center = vec(pp.position)+shift+vec(pp.velocity)*dt*i/7+vec(pp.acceleration)*(.5*dt*dt*i*(i-1)/42)+vec(pp.jerk)*(dt**3*i*(i-1)*(i-2)/1260)
                gap = c[i,2]-center[2]-contact_height
                count=min(L.segments,max(1,math.ceil(L.terminal_coverage_s/dt)))
                slope=0. if self.envelope and k>=L.segments-count else L.corridor_slope
                if facets:
                    e=c[i]-center
                    for f in facets:
                        corridor_residual=max(corridor_residual,-gap,float(np.array(f['axis'])@e)+f['heading_chord']*np.linalg.norm(e[:2])+self.footprint_tilt_coefficient*np.linalg.norm(elevated_a[i,:2])-f['allowance_m']-slope*gap)
                        if L.tracking_error_tightening and k==len(plan.points)-1:
                            tracking_residual=max(tracking_residual,float(np.array(f['axis'])@(e+tracking_error))+f['heading_chord']*(np.linalg.norm(e[:2])+np.linalg.norm(tracking_error[:2]))+self.footprint_tilt_coefficient*np.linalg.norm(elevated_a[i,:2])+tracking_attitude-f['allowance_m']-slope*gap)
                else:
                    corridor_residual = max(corridor_residual, -gap,
                        np.linalg.norm(c[i,:2]-center[:2]-landing_offset[:2])+np.linalg.norm(landing_offset)+self.footprint_tilt_coefficient*np.linalg.norm(elevated_a[i,:2])-radius-slope*gap)
            v, a, j = [derivatives(c, dt, n) for n in (1, 2, 3)]
            reserved=a[1:] if k==0 and L.entry_acceleration_capture_m_s2 is not None else a
            if np.linalg.norm(reserved,axis=1).max()>L.acceleration+tol:failed.append('post_entry_acceleration_reserve')
            for key, d in [('speed', v), ('acceleration', a), ('jerk', j)]:
                peaks[key] = max(peaks[key], float(np.linalg.norm(d, axis=1).max()))
            peaks['specific_force'] = max(peaks['specific_force'], float(np.linalg.norm(a+[0, 0, 9.80665], axis=1).max()))
            peaks['tilt_ratio'] = max(peaks['tilt_ratio'], float((np.linalg.norm(a[:, :2], axis=1)/(9.80665+a[:, 2])).max()))
            if max(v[:, 2]) > L.upward_speed+tol or min(v[:, 2]) < -L.downward_speed-tol: failed.append('vertical_speed')
            if max(abs(a[:, 2])) > L.vertical_acceleration+tol: failed.append('vertical_acceleration')
            if min(9.80665+a[:, 2]) < L.minimum_vertical_specific_force-tol: failed.append('vertical_force')
        for key, cap in [('speed', L.speed), ('acceleration', L.acceleration), ('jerk', L.jerk),
                         ('tilt_ratio', math.tan(L.tilt_rad)), ('specific_force', L.maximum_specific_force)]:
            if key=='acceleration' and L.entry_acceleration_capture_m_s2 is not None:cap=L.entry_acceleration_capture_m_s2
            if peaks[key] > cap+tol: failed.append(key)
        start = plan.at(0); end = plan.at(plan.duration)
        residual = max(np.linalg.norm(start[n]-vec(getattr(state, n))) for n in ('position', 'velocity', 'acceleration'))
        residual = max(residual, np.linalg.norm(end['position']-plan.target_position),
            np.linalg.norm(end['velocity']-plan.target_velocity), np.linalg.norm(end['acceleration']), np.linalg.norm(end['jerk']))
        for a, b in zip(plan.points, plan.points[1:]):
            residual = max(residual, *(np.linalg.norm(derivatives(a, dt, n)[-1]-derivatives(b, dt, n)[0]) for n in range(4)))
        if residual > tol: failed.append('boundary_or_continuity')
        if np.linalg.norm(landing_offset)>L.landing_point_trust_radius_m+tol: failed.append('landing_point_trust_region')
        if corridor_residual > tol: failed.append('continuous_corridor')
        if L.tracking_error_tightening and tracking_residual>tol:failed.append('tracking_terminal_corridor')
        min_gap = math.inf; min_margin = math.inf; max_rate = 0.
        for t in np.linspace(0, plan.duration, L.audit_samples):
            motion = plan.motion(float(t)); deck = platform.predict(float(t)).deck()
            feet = evaluate_airframe(self.geometry, motion, deck)
            gap = min(f.clearance_m for f in feet); min_gap = min(min_gap, gap)
            max_rate = max(max_rate, np.linalg.norm(motion.angular_velocity_world_rad_s))
            if gap < -tol: failed.append('sampled_foot_penetration')
            if gap <= .1:
                min_margin = min(min_margin, *(min(f.whole_cylinder_footprint_margins_m) for f in feet))
                if self.envelope:min_margin=min(min_margin,self.envelope.clearance(motion,deck))
        if min_margin < L.margin-tol: failed.append('sampled_low_footprint')
        # Fixed heading tilt-rate audit; separate from the translational certificate.
        if max_rate > math.radians(70)+tol: failed.append('sampled_angular_speed')
        result={'passed': not failed, 'failures': sorted(set(failed)), 'hull_upper_bounds': peaks,
            'corridor_model':'rotating_rectangle_support_bound' if L.directional_corridor and self.envelope else 'inscribed_disk',
            'entry_acceleration_capture_m_s2':L.entry_acceleration_capture_m_s2,
            'landing_point_offset_world_m': landing_offset.tolist(),
            'boundary_residual': float(residual), 'sampled_minimum_foot_gap_m': float(min_gap),
            'corridor_control_point_residual_m': float(corridor_residual),
            'sampled_low_altitude_foot_margin_m': float(min_margin), 'sampled_maximum_angular_speed_rad_s': float(max_rate),
            'geometry_samples': L.audit_samples, 'collision_continuously_certified': False,
            'physics_validated': False, 'usable_radius_m': radius}
        if L.tracking_error_tightening:result.update(tracking_contact_envelope=tracking,tracking_terminal_corridor_residual_m=float(tracking_residual))
        return result

    def search(self, state, platform, now, durations=(3., 4., 5., 6.), previous_arrival=None, uncertainty=None,tracking_envelope=None):
        started = time.perf_counter(); records = []; best = None; best_score = math.inf
        for duration in sorted(set(durations)):
            plan, rec = self.solve(state, platform, now, duration, uncertainty,tracking_envelope); records.append(rec)
            if plan is not None:
                score = plan.cost + .5*duration
                if previous_arrival is not None: score += 2.*(plan.arrival-previous_arrival)**2
                if score < best_score: best, best_score = plan, score
        return best, {'status': 'PLAN_AVAILABLE' if best else 'NO_VALIDATED_CANDIDATE',
            'candidates': records, 'search_s': time.perf_counter()-started,
            'selected_duration_s': best.duration if best else None,
            'selected_score': best_score if best else None,
            'global_infeasibility_claim': False, 'actuation_enabled': False}


class RecedingLandingSession:
    """Explicit operator gate and irreversible real-contact handover.

    Missing plans request recovery assessment; this class never invents a hover
    or return command. It does not arm, take off, send LAND, or command motors.
    """
    def __init__(self, planner, durations=(3., 4., 5., 6.), max_task_time=25., max_compute_s=1.):
        if not math.isfinite(max_task_time) or max_task_time <= 0: raise ValueError('Invalid task timeout')
        self.planner = planner; self.durations = durations; self.timeout = max_task_time
        if not math.isfinite(max_compute_s) or max_compute_s <= 0: raise ValueError('Invalid computation deadline')
        self.max_compute = max_compute_s
        self.started = None; self.plan = None; self.contact = False; self.last_now = None

    def update(self, state, platform, now, landing_requested=False, contact_event=False):
        if type(landing_requested) is not bool or type(contact_event) is not bool: raise ValueError('Explicit boolean gates required')
        if not math.isfinite(now) or (self.last_now is not None and now < self.last_now): raise ValueError('Decision clock reversed')
        self.last_now = now
        if contact_event: self.contact = True; self.plan = None
        if self.contact: return {'action': 'CONTACT_HANDOVER', 'reference': None, 'report': None}
        if not landing_requested: return {'action': 'WAIT_OPERATOR', 'reference': None, 'report': None}
        if self.started is None: self.started = now
        if now-self.started > self.timeout:
            self.plan = None
            return {'action': 'REQUIRE_RECOVERY_ASSESSMENT', 'reference': None, 'reason': 'TASK_TIMEOUT'}
        arrival = self.plan.arrival if self.plan else None
        durations = self.durations
        if arrival is not None:
            remaining = arrival-now
            # Avoid ill-conditioned rescaling into tens-of-ms Bezier pieces.
            # The final validated reference is checked against new measurements
            # instead of repeatedly replacing its near-contact boundary.
            if remaining < .8:
                try:
                    aligned, deck, radius = self.planner._entry(state, platform, now)
                    if remaining <= 0: raise ValueError('REFERENCE_EXPIRED_WITHOUT_CONTACT')
                    old = self.plan.at(now-self.plan.start)
                    if np.linalg.norm(old['position']-aligned.position) > .03 or np.linalg.norm(old['velocity']-aligned.velocity) > .1:
                        raise ValueError('TERMINAL_TRACKING_ERROR')
                    end_deck = deck.predict(remaining)
                    body = self.plan.motion(self.plan.duration)
                    feet = evaluate_airframe(self.planner.geometry, body, end_deck.deck())
                    centroid = np.mean([f.point_world_m for f in feet],axis=0)
                    # A jointly optimized contact point need not be the deck
                    # center. Check drift from its planned deck-relative point.
                    original=PlatformState(**self.plan.audit['platform_prediction_input'])
                    planned_deck=original.predict(self.plan.arrival-original.stamp)
                    planned_relative=centroid-vec(planned_deck.position)
                    if (np.linalg.norm(centroid-vec(end_deck.position)-planned_relative) > .005
                            or max(f.tangential_speed_upper_bound_m_s for f in feet) > .05
                            or min(min(f.whole_cylinder_footprint_margins_m) for f in feet)<self.planner.limits.margin):
                        raise ValueError('TERMINAL_PREDICTION_CHANGED')
                    return {'action': 'TRACK_TERMINAL_CANDIDATE', 'reference': self.plan, 'report': None}
                except ValueError as e:
                    self.plan = None
                    return {'action': 'REQUIRE_RECOVERY_ASSESSMENT', 'reference': None, 'reason': str(e)}
            durations = tuple(t for t in (remaining-.2, remaining, remaining+.2) if .2 <= t <= 15)
        try:
            plan, report = self.planner.search(state, platform, now, durations, arrival)
        except ValueError as e:
            self.plan = None
            return {'action': 'REQUIRE_RECOVERY_ASSESSMENT', 'reference': None, 'reason': str(e)}
        self.plan = plan
        if report['search_s'] > self.max_compute:
            self.plan = None
            return {'action':'REQUIRE_RECOVERY_ASSESSMENT','reference':None,'reason':'COMPUTATION_DEADLINE', 'report':report}
        return {'action': 'TRACK_CANDIDATE' if plan else 'REQUIRE_RECOVERY_ASSESSMENT',
            'reference': plan if plan else None, 'report': report}
