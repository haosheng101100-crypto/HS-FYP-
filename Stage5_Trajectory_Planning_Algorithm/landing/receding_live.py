"""Nonblocking, simulation-only execution of asynchronous Bezier plans.

The worker has no MAVLink connection. Only this control-side adapter selects
references; the existing dispatcher and native FCU retain actuation ownership.
Future plan starts continue the current reference (C2), rather than replaying a
stale measured initial condition. Measured tracking gates remain mandatory.
"""
from dataclasses import asdict,replace
from pathlib import Path
import json
import math
import numpy as np
from .receding_landing import FlightState, PlatformState, LandingPlan, vec
from .execution import BodyReference, ExecutionDecision,attitude_error
from .contact_geometry import norm, sub


def atomic_json(path, value):
    path = Path(path)
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    temp.replace(path)


def pack_plan(plan):
    return dict(start=plan.start, duration=plan.duration, points=plan.points.tolist(),
                target_position=plan.target_position, target_velocity=plan.target_velocity,
                com_offset=plan.com_offset, cost=plan.cost, audit=plan.audit)


def unpack_plan(data):
    points = np.asarray(data['points'], dtype=float)
    if points.ndim != 3 or points.shape[1:] != (8, 3) or not 1 <= len(points) <= 8:
        raise ValueError('Invalid plan dimensions')
    if not np.isfinite(points).all() or not data['audit'].get('passed'):
        raise ValueError('Unverified worker result')
    plan = LandingPlan(**dict(data, points=points))
    if not all(math.isfinite(x) for x in (plan.start, plan.duration, plan.cost)) or not .2 <= plan.duration <= 15:
        raise ValueError('Invalid plan time/cost')
    for name in ('target_position', 'target_velocity', 'com_offset'): vec(getattr(plan, name))
    return plan


def terminal_feedback_gains(base,terminal,remaining_s=None,horizontal_speed_m_s=None):
    """Smoothly trade aggressive position tracking for level contact capture."""
    keys=('position_gain','velocity_gain','acceleration_lead_s')
    if not terminal:return dict(base),dict(active=False,blend=0.,remaining_s=remaining_s,gains=dict(base))
    span=terminal['transition_s']
    if not math.isfinite(span) or span<=0:raise ValueError('Positive contact gain transition required')
    if terminal.get('speed_scheduling'):
        schedule=terminal['speed_scheduling']
        low,high,fast=(schedule[k] for k in ('low_speed_m_s','high_speed_m_s','fast_transition_s'))
        if not all(math.isfinite(v) for v in (low,high,fast)) or not 0<=low<high or fast<span:
            raise ValueError('Ordered finite speed/transition design required')
        if horizontal_speed_m_s is not None and (not math.isfinite(horizontal_speed_m_s) or horizontal_speed_m_s<0):
            raise ValueError('Nonnegative finite measured horizontal speed required')
        q=0. if horizontal_speed_m_s is None else min(1.,max(0.,(horizontal_speed_m_s-low)/(high-low)))
        weight=10*q**3-15*q**4+6*q**5
        span=(1-weight)*span+weight*fast
    for key in keys:
        if any(not math.isfinite(v) or v<0 for v in (base[key],terminal[key])):
            raise ValueError('Finite nonnegative contact gains required')
    if remaining_s is not None and not math.isfinite(remaining_s):raise ValueError('Finite remaining time required')
    s=0. if remaining_s is None else min(1.,max(0.,1.-remaining_s/span))
    blend=10*s**3-15*s**4+6*s**5
    gains=dict(base)
    for key in keys:gains[key]=(1-blend)*base[key]+blend*terminal[key]
    return gains,dict(active=bool(s>0),blend=blend,remaining_s=remaining_s,gains=gains,
        transition_s=span,horizontal_speed_m_s=horizontal_speed_m_s)


def native_capture_time_guard(gap_m,minimum_closing_speed_m_s,maximum_time_s):
    """Bound the engineering free-flight interval after an early LAND switch.

    LAND may change horizontal attitude before touchdown. Use the existing
    conservative foot closing-speed bound, keeping guided tracking until the
    remaining measured gap is sufficiently short. This is not a future-contact
    guarantee; real contact and the original kinematic checks still decide.
    """
    if not all(math.isfinite(v) for v in (gap_m,minimum_closing_speed_m_s,maximum_time_s)) or maximum_time_s<=0:
        raise ValueError('Finite gap/speed and positive native capture interval required')
    estimate=gap_m/minimum_closing_speed_m_s if gap_m>=0 and minimum_closing_speed_m_s>0 else None
    return dict(passed=estimate is not None and estimate<=maximum_time_s,gap_m=gap_m,minimum_closing_speed_m_s=minimum_closing_speed_m_s,
        estimated_freeflight_s=estimate,maximum_time_s=maximum_time_s)


def terminal_capture_window(remaining_s,early_window_s,checks,whole_body_margin_m,required_margin_m,preview_passed,native_time_guard=None):
    """Allow measured native capture inside the original early-contact window.

    Every existing <=1cm, speed, attitude and footprint capture condition still
    has to hold. This only removes the requirement to wait for the exact nominal
    timestamp when actual safe capture is already available.
    """
    required=('fresh','armed_guided','gap_0_to_1cm','relative_foot_speed','attitude','angular_speed','target_xy','footprint')
    values=(remaining_s,early_window_s,whole_body_margin_m,required_margin_m)
    if not all(math.isfinite(v) for v in values) or early_window_s<=0 or required_margin_m<=0:
        raise ValueError('Finite terminal time window and physical margin required')
    active=0<=remaining_s<=early_window_s
    ready=active and bool(preview_passed) and whole_body_margin_m>=required_margin_m and all(k in checks and bool(checks[k]) for k in required)
    if native_time_guard is not None:ready=ready and native_time_guard['passed']
    result=dict(active=active,ready=bool(ready),remaining_s=remaining_s,early_window_s=early_window_s,
        checks=dict(checks),whole_body_margin_m=whole_body_margin_m,required_margin_m=required_margin_m,preview_passed=bool(preview_passed))
    if native_time_guard is not None:result['native_time_guard']=native_time_guard
    return result


def execution_feedback_gains(base,terminal,remaining_s,horizontal_speed_m_s,phase):
    if phase in ('TRACKING','TERMINAL_TRACKING') or not terminal:
        return terminal_feedback_gains(base,terminal,remaining_s,horizontal_speed_m_s)
    gains,report=terminal_feedback_gains(base,terminal,0.,horizontal_speed_m_s)
    # The alignment reference follows a causal acceleration estimate. Its
    # finite-difference jerk contains estimation changes, unlike a polynomial
    # descent reference; derivative lead amplifies them into attitude chatter.
    if phase in ('PLANNING_HOVER','UNCERTAINTY_WARMUP'):
        gains['acceleration_lead_s']=base['acceleration_lead_s']
        report['gains']=dict(gains)
    report.update(policy='stable_non_descent_tracking',phase=phase,remaining_s=None)
    return gains,report


class FilePlannerClient:
    """One request in flight, atomic files, no waits in the flight loop."""
    def __init__(self, directory):
        self.directory = Path(directory); self.directory.mkdir(parents=True, exist_ok=True)
        self.sequence = 0; self.outstanding = None; self.request_data = None

    def submit(self, state, platform, durations, previous_arrival, uncertainty=None,tracking_envelope=None):
        if self.outstanding is not None: raise RuntimeError('Planner already busy')
        self.sequence += 1; self.outstanding = self.sequence
        data = dict(sequence=self.sequence, state=asdict(state), platform=asdict(platform),
                    now=state.stamp, durations=list(durations), previous_arrival=previous_arrival)
        if uncertainty is not None: data['uncertainty'] = asdict(uncertainty)
        if tracking_envelope is not None:data['tracking_envelope']=tracking_envelope
        self.request_data = data
        # Retain complete input before publishing the worker mailbox. Never replace history.
        with (self.directory/f'request_{self.sequence:06d}.json').open('x', encoding='utf-8') as archive:
            json.dump(data, archive, ensure_ascii=False, allow_nan=False)
        atomic_json(self.directory/'request.json', data)
        return data

    def poll(self):
        if self.outstanding is None: return None
        try: result = json.loads((self.directory/'response.json').read_text(encoding='utf-8'))
        except (FileNotFoundError, json.JSONDecodeError): return None
        if result.get('sequence') != self.outstanding: return None
        self.outstanding = None
        return result

    def abandon(self):
        # Worker responses cannot authorize future control after cancellation.
        self.outstanding = None; self.request_data = None


class AsyncLandingReference:
    def __init__(self, client, com_offset, hold_reference, lead_s=1.2, task_timeout=40.,maximum_extension_s=1.,extension_window_s=None,initial_durations=(6.,8.,10.),early_extension_s=0.,moving_hold_start_s=None):
        self.client = client; self.offset = tuple(com_offset); self.hold = hold_reference
        self.lead = lead_s; self.timeout = task_timeout
        self.maximum_extension_s=maximum_extension_s
        if extension_window_s is not None and (not math.isfinite(extension_window_s) or extension_window_s<=0):
            raise ValueError('Positive finite extension window required')
        self.extension_window_s=extension_window_s
        if not math.isfinite(early_extension_s) or not 0<=early_extension_s<=maximum_extension_s:
            raise ValueError('Early extension must fit inside total budget')
        self.early_extension_s=early_extension_s;self.extension_window_opened=False
        try:self.initial_durations=tuple(sorted(set(float(v) for v in initial_durations)))
        except (TypeError,ValueError):raise ValueError('Finite initial approach durations required')
        if not self.initial_durations or any(not math.isfinite(v) or not .8<=v<=15. for v in self.initial_durations):
            raise ValueError('Initial approach durations must be within [.8,15] seconds')
        self.plan = None; self.pending = None; self.started = None
        self.last_submission = -math.inf; self.request = None
        self.phase = 'WAIT_OPERATOR'; self.failure = None; self.contact = False
        self.events = []; self.last_report = None
        self.committed_arrival=None;self.maximum_arrival=None
        self.terminal_committed=False
        if moving_hold_start_s is not None and not math.isfinite(moving_hold_start_s):raise ValueError('Finite moving hold start required')
        self.moving_hold_start=moving_hold_start_s

    def hold_at(self,now):
        if self.moving_hold_start is None:return self.hold
        dt=now-self.moving_hold_start
        if dt<0:raise ValueError('Moving initial reference queried before start')
        p=vec(self.hold.position_world_m);v=vec(self.hold.velocity_world_m_s);a=vec(self.hold.acceleration_world_m_s2)
        return replace(self.hold,position_world_m=tuple(p+v*dt+.5*a*dt*dt),velocity_world_m_s=tuple(v+a*dt))

    def cancel(self, reason):
        self.failure = reason; self.phase = 'RECOVERY_REQUIRED'
        self.plan = None; self.pending = None; self.client.abandon()

    def touch(self):
        self.contact = True; self.plan = None; self.pending = None
        self.client.abandon(); self.phase = 'CONTACT_HANDOVER'

    def _future_state(self, now, platform=None):
        start = now+self.lead
        if self.plan:
            if start >= self.plan.arrival: return None
            s = self.plan.at(start-self.plan.start)
            if platform is not None:
                base=BodyReference(tuple(s['position']),tuple(s['velocity']),tuple(s['acceleration']),0.)
                corrected,_=measured_frame_reference(self.plan,self.model,platform.predict(start-platform.stamp),start,base)
                return FlightState(start,corrected.position_world_m,corrected.velocity_world_m_s,corrected.acceleration_world_m_s2)
            return FlightState(start, tuple(s['position']), tuple(s['velocity']), tuple(s['acceleration']))
        # Level hover reference; body and COM origins stay explicitly distinct.
        hold=self.hold_at(start)
        return FlightState(start,tuple(vec(hold.position_world_m)+vec(self.offset)),hold.velocity_world_m_s,hold.acceleration_world_m_s2)

    def _prediction_changed(self, platform, model, now, position_cap=.08, velocity_cap=.12):
        if now < model.stamp: return False
        predicted = model.predict(now-model.stamp)
        return norm(sub(predicted.position,platform.position)) > position_cap or norm(sub(predicted.velocity,platform.velocity)) > velocity_cap

    def step(self, now, state, platform, landing_requested=False, contact_event=False, measured_frame_guard=False, continue_measured_frame=False, uncertainty=None,tracking_envelope=None):
        if contact_event: self.touch()
        if self.contact: return None
        if not landing_requested: return None
        if self.failure: return None
        if self.started is None: self.started=now
        if not 0 <= now-state.stamp <= .06 or not 0 <= now-platform.stamp <= .06:
            self.cancel('STALE_OBSERVATION'); return None
        if now-self.started > self.timeout:
            self.cancel('TASK_TIMEOUT'); return None
        if self.moving_hold_start is not None and self.plan is None and now-self.moving_hold_start>2.:
            self.cancel('INITIAL_MOVING_PLAN_TIMEOUT');return None

        if self.plan and self.plan.arrival-now<=.8 and not self.terminal_committed:
            # Actual contact can precede the nominal endpoint. Do not let a
            # late replacement move the handover clock after terminal capture
            # is already under way. Tracking and geometry/recovery gates remain.
            self.terminal_committed=True;self.pending=None
            self.client.abandon();self.request=None
            self.events.append(dict(time=now,event='TERMINAL_PLAN_COMMITTED',arrival=self.plan.arrival))

        response = self.client.poll()
        if response is not None:
            self.last_report = response.get('report')
            try:
                if not response.get('plan'): raise ValueError(response.get('error') or 'NO_VALIDATED_CANDIDATE')
                plan = unpack_plan(response['plan'])
                if abs(plan.start-self.request['now']) > 1e-8: raise ValueError('START_MISMATCH')
                if not any(abs(plan.duration-d)<1e-7 for d in self.request['durations']):raise ValueError('UNREQUESTED_DURATION')
                if self.maximum_arrival is not None and plan.arrival>self.maximum_arrival+1e-7:raise ValueError('ARRIVAL_EXTENSION_BUDGET')
                if now > plan.start+.04: raise ValueError('SOLVER_RESULT_LATE')
                if not np.allclose(plan.com_offset,self.offset,atol=1e-10): raise ValueError('COM_PROFILE_MISMATCH')
                model = PlatformState(**plan.audit.get('platform_prediction_input',self.request['platform']))
                # Alignment at a future start uses the same causal predictor.
                check = platform.predict(max(0.,plan.start-platform.stamp))
                if self._prediction_changed(check, model, plan.start): raise ValueError('RESULT_PLATFORM_CHANGED')
                self.pending=(plan,model)
                self.events.append(dict(time=now, event='PLAN_PENDING', sequence=response['sequence'], arrival=plan.arrival))
            except (ValueError,KeyError,TypeError) as exc:
                self.events.append(dict(time=now,event='RESULT_REJECTED',reason=str(exc)))
                # Keep a still valid old plan, or keep hovering before descent.
            self.request=None

        if self.pending and now >= self.pending[0].start:
            plan,model=self.pending; self.pending=None
            if now > plan.start+.08 or self._prediction_changed(platform,model,now):
                self.events.append(dict(time=now,event='ACTIVATION_REJECTED'))
            else:
                nominal=plan.at(min(now-plan.start,plan.duration))
                if norm(sub(state.position,nominal['position'])) <= .12 and norm(sub(state.velocity,nominal['velocity'])) <= .25:
                    self.plan=plan;self.model=model;self.phase='TRACKING'
                    if self.committed_arrival is None:
                        self.committed_arrival=plan.arrival;self.maximum_arrival=plan.arrival+self.maximum_extension_s
                    self.events.append(dict(time=now,event='PLAN_ACTIVATED',arrival=plan.arrival))
                else:self.events.append(dict(time=now,event='ACTIVATION_TRACKING_REJECTED'))

        if self.plan:
            remaining=self.plan.arrival-now
            if remaining <= 0:
                self.phase='TERMINAL_EXPIRED';return None
            nominal=self.plan.at(now-self.plan.start)
            if continue_measured_frame or measured_frame_guard:
                # Compare the measured aircraft with the reference actually
                # executed, rather than the uncorrected optimization reference.
                # A separate geometric preview must not silently switch the
                # tracking metric back to an obsolete platform prediction.
                base=BodyReference(tuple(nominal['position']),tuple(nominal['velocity']),tuple(nominal['acceleration']),0.)
                corrected,_=measured_frame_reference(self.plan,self.model,platform,now,base)
                nominal=dict(nominal,position=corrected.position_world_m,velocity=corrected.velocity_world_m_s)
            if norm(sub(state.position,nominal['position'])) > .35 or norm(sub(state.velocity,nominal['velocity'])) > .5:
                self.cancel('TRACKING_ENVELOPE');return None
            if remaining < .8:
                # Freeze only a terminal candidate that still matches the latest
                # endpoint position AND velocity; changed targets cause recovery.
                endpoint=platform.predict(remaining)
                expected=self.model.predict(self.plan.arrival-self.model.stamp)
                if (norm(sub(endpoint.position,expected.position)) > .03 or norm(sub(endpoint.velocity,expected.velocity)) > .04) and not (continue_measured_frame or measured_frame_guard):
                    self.cancel('TERMINAL_PLATFORM_CHANGED');return None
                self.phase='TERMINAL_TRACKING'

        busy=self.client.outstanding is not None
        if busy and self.request and now > self.request['now']+.08:
            self.client.abandon();self.request=None
            self.events.append(dict(time=now,event='REQUEST_DEADLINE_MISSED'))
            busy=False
        if not self.terminal_committed and not busy and self.pending is None and now-self.last_submission >= .4:
            future=self._future_state(now,platform if continue_measured_frame else None)
            if future is not None and self.plan and self.plan.arrival-future.stamp<.8-1e-8:
                future=None
            if future is not None:
                if self.plan:
                    remaining=self.plan.arrival-future.stamp
                    changes=(-.1,0.)
                    if self.extension_window_s is None:changes+=(.1,.5)
                    else:
                        if not self.extension_window_opened and remaining<=self.extension_window_s:
                            self.extension_window_opened=True
                            self.events.append(dict(time=now,event='CAPTURE_TIME_WINDOW_OPENED',remaining_s=remaining,future_start=future.stamp))
                        changes+=(.1,.5,1.,1.5) if self.extension_window_opened else (.1,.5)
                    durations=tuple(remaining+d for d in changes if .8<=remaining+d<=15)
                    maximum=self.maximum_arrival
                    if self.extension_window_s is not None and not self.extension_window_opened:
                        maximum=min(maximum,self.committed_arrival+self.early_extension_s)
                    durations=tuple(x for x in durations if future.stamp+x<=maximum+1e-8)
                else:durations=self.initial_durations
                if durations:
                    if uncertainty is not None and not uncertainty.ready:
                        self.phase='UNCERTAINTY_WARMUP'
                        return self.plan.body_reference_at(now) if self.plan else self.hold_at(now)
                    model=platform.predict(future.stamp-platform.stamp)
                    feedback={} if tracking_envelope is None else dict(tracking_envelope=tracking_envelope)
                    self.request=self.client.submit(future,model,durations,self.committed_arrival,uncertainty,**feedback)
                    self.last_submission=now
        if self.plan:return self.plan.body_reference_at(now)
        self.phase='PLANNING_HOVER'
        return self.hold_at(now)


def moving_alignment_required(configuration,observation_stress):
    """Choose extra preparation from declared current measurement quality.

    Accurate simulator poses can use the direct optimized intercept. A known
    delayed/noisy observation stream first matches the moving deck; no future
    vehicle command or scenario name is used by this policy.
    """
    if not configuration:return False
    quality=observation_stress or {}
    values=[quality.get(k,0.) for k in ('delay_s','position_sigma_m','yaw_sigma_rad')]
    if any(not math.isfinite(v) or v<0 for v in values):raise ValueError('Finite nonnegative observation quality required')
    return not configuration.get('only_for_degraded_observations',False) or any(v>0 for v in values)


def initial_plan_durations(configuration,aligned=False):
    durations=configuration.get('initial_candidate_durations_s',(6.,8.,10.))
    if aligned:durations=configuration.get('moving_alignment',{}).get('initial_candidate_durations_s',durations)
    return tuple(durations)


class DeckAlignmentReference:
    """Jerk-bounded coarse approach above the deck; descent starts on readiness.

    Uses only the current causal platform estimate. These engineering reference
    bounds do not replace the actual-motion, collision or contact audits.
    """
    def __init__(self,initial,com_offset,minimum_leg_gap_m=1.5,dwell_s=.6,start_s=None,reserve_vertical=False):
        if minimum_leg_gap_m<1. or dwell_s<=0:raise ValueError('Safe alignment height and positive dwell required')
        self.offset=vec(com_offset);self.p=vec(initial.position_world_m)+self.offset
        self.v=vec(initial.velocity_world_m_s);self.a=vec(initial.acceleration_world_m_s2)
        if start_s is not None and not math.isfinite(start_s):raise ValueError('Finite alignment start required')
        self.gap=minimum_leg_gap_m;self.dwell=dwell_s;self.last=start_s;self.stable_since=None
        self.reserve_vertical=reserve_vertical

    def target(self,platform,center_shift):
        p=vec(platform.position)+vec(center_shift)+self.offset
        p[2]=platform.position[2]+.195+self.gap+self.offset[2]
        return p

    def step(self,now,platform,center_shift):
        dt=.04 if self.last is None else now-self.last
        if dt==0 and self.last==now:
            return BodyReference(tuple(self.p-self.offset),tuple(self.v),tuple(self.a),0.)
        if not 0<dt<=.080001:raise ValueError('Alignment reference update gap')
        error=self.target(platform,center_shift)-self.p
        relative=error[:2];relative*=min(1.,1.2/max(np.linalg.norm(relative),1e-12))
        velocity=np.r_[np.array(platform.velocity[:2])+relative,np.clip(.5*error[2],-.3,.3)]
        # Unsaturated error dynamics: XY e''+4e'+4e=0; Z e''+2e'+e=0.
        # Critically damped reference shaping avoids a coarse-approach overshoot.
        desired=vec(platform.acceleration)+np.array([4.,4.,2.])*(velocity-self.v)
        desired[2]=np.clip(desired[2],-.4,.4)
        if self.reserve_vertical:
            # Independent Z authority for the planar reacquisition stop-distance
            # calculation. XY cannot consume the upward acceleration or jerk.
            desired[:2]*=min(1.,1./max(np.linalg.norm(desired[:2]),1e-12))
            delta=desired-self.a
            delta[:2]*=min(1.,1.6*dt/max(np.linalg.norm(delta[:2]),1e-12))
            delta[2]=np.clip(delta[2],-1.2*dt,1.2*dt)
        else:
            desired*=min(1.,1.1/max(np.linalg.norm(desired),1e-12))
            delta=desired-self.a;delta*=min(1.,2.*dt/max(np.linalg.norm(delta),1e-12))
        a=self.a+delta
        self.p+=self.v*dt+(2*self.a+a)*dt*dt/6
        self.v+=.5*(self.a+a)*dt;self.a=a;self.last=now
        return BodyReference(tuple(self.p-self.offset),tuple(self.v),tuple(self.a),0.)

    def readiness(self,now,state,platform,center_shift,attitude_error_rad,angular_speed_rad_s):
        error=vec(state.position)-self.target(platform,center_shift)
        relative=vec(state.velocity)-vec(platform.velocity)
        residuals=dict(horizontal_position_m=float(np.linalg.norm(error[:2])),height_m=abs(float(error[2])),relative_speed_m_s=float(np.linalg.norm(relative[:2])),vertical_speed_m_s=abs(float(state.velocity[2])),
            reference_position_m=float(np.linalg.norm(vec(state.position)-self.p)),reference_velocity_m_s=float(np.linalg.norm(vec(state.velocity)-self.v)),acceleration_m_s2=float(np.linalg.norm(vec(state.acceleration)-vec(platform.acceleration))),attitude_rad=float(attitude_error_rad),angular_speed_rad_s=float(angular_speed_rad_s))
        limits=dict(horizontal_position_m=.02,height_m=.06,relative_speed_m_s=.03,vertical_speed_m_s=.04,reference_position_m=.04,reference_velocity_m_s=.06,acceleration_m_s2=.08,attitude_rad=math.radians(1.),angular_speed_rad_s=.08)
        checks={k:v<=limits[k] for k,v in residuals.items()}
        checks['fresh']=0<=now-state.stamp<=.06 and 0<=now-platform.stamp<=.06
        passed=all(checks.values())
        self.stable_since=(now if self.stable_since is None else self.stable_since) if passed else None
        dwell=0. if self.stable_since is None else now-self.stable_since
        return dict(ready=bool(passed and dwell>=self.dwell-1e-8),checks=checks,residuals=residuals,limits=limits,dwell_s=dwell,required_dwell_s=self.dwell,target_com_world_m=self.target(platform,center_shift).tolist())


class FinalCommandLimiter:
    """Bound the PVA sent after feedback; native tracking still needs raw audit.

    Acceleration/vertical/tilt constraints form a convex set. Slewing from the
    previous feasible command toward a feasible target keeps those bounds.
    This does not certify the corrected P/V/A as a new Bezier trajectory.
    """
    def __init__(self,limits,interval=.04,acceleration_reserve_m_s2=0.):
        if not math.isfinite(acceleration_reserve_m_s2) or not 0<=acceleration_reserve_m_s2<limits['acceleration']:
            raise ValueError('Final command reserve must fit inside original physical acceleration limit')
        self.acceleration_reserve=acceleration_reserve_m_s2
        self.limits=limits;self.interval=interval;self.last=None;self.a=np.zeros(3);self.diagnostics=None

    def update(self,decision,now):
        if decision.stop_reference or decision.reference is None:
            self.last=None;self.a=np.zeros(3);self.diagnostics=None;return decision
        dt=self.interval if self.last is None else now-self.last
        if not 0<dt<=.080001:raise ValueError('Final command limiter update gap')
        ref=decision.reference;requested=vec(ref.acceleration_world_m_s2)
        if not np.isfinite(requested).all():raise ValueError('Nonfinite final command')
        a=requested.copy();cap=self.limits['acceleration']-self.acceleration_reserve
        vertical=min(cap,self.limits['vertical_acceleration']);a[2]=np.clip(a[2],-vertical,vertical)
        horizontal=min(math.sqrt(max(0.,cap*cap-a[2]**2)),(9.80665+a[2])*math.tan(self.limits['tilt_rad']))
        a[:2]*=min(1.,horizontal/max(float(np.linalg.norm(a[:2])),1e-12))
        delta=a-self.a;change=float(np.linalg.norm(delta));fraction=min(1.,self.limits['jerk']*dt/max(change,1e-12))
        a=self.a+fraction*delta;jerk=(a-self.a)/dt
        v=vec(ref.velocity_world_m_s);v*=min(1.,self.limits['speed']/max(float(np.linalg.norm(v)),1e-12))
        self.diagnostics=dict(requested_acceleration_m_s2=requested.tolist(),sent_acceleration_m_s2=a.tolist(),
            command_acceleration_cap_m_s2=cap,physical_acceleration_limit_m_s2=self.limits['acceleration'],
            acceleration_response_reserve_m_s2=self.acceleration_reserve,
            sent_acceleration_norm_m_s2=float(np.linalg.norm(a)),sent_jerk_norm_m_s3=float(np.linalg.norm(jerk)),
            acceleration_projected=bool(np.linalg.norm(a-requested)>1e-9))
        self.a=a;self.last=now
        return replace(decision,reference=replace(ref,acceleration_world_m_s2=tuple(a),velocity_world_m_s=tuple(v)))


class RecoveryReference:
    """Bounded brake/climb then return over the flat, empty simulation world.

    This is an explicit failed-attempt controller, not a robust escape proof.
    It does not land or disarm and is not included as successful deck landing.
    """
    def __init__(self, observation, home=(0.,0.,4.185), *, initial_reference=None, start_s=None):
        # A live transition must continue the PVA that was actually sent. Using
        # measured position and zero acceleration can step the target downward
        # and discard existing upward braking close to the deck (M18).
        self.p=vec(initial_reference.position_world_m if initial_reference else observation.motion.pose.position_world_m).copy()
        self.v=vec(initial_reference.velocity_world_m_s if initial_reference else observation.motion.velocity_at_origin_world_m_s).copy()
        self.a=vec(initial_reference.acceleration_world_m_s2).copy() if initial_reference else np.zeros(3)
        self.yaw=initial_reference.yaw_enu_rad if initial_reference else 0.
        self.home=vec(home);self.last=start_s;self.stable_since=None
        self.initial_pending=start_s is not None
        if start_s is not None and (not math.isfinite(start_s) or abs(start_s-observation.sim_t_s)>1e-8):
            raise ValueError('Recovery must start at the current measured control tick')

    def step(self, now, observation):
        if self.initial_pending:
            if now!=self.last:raise ValueError('First recovery output must own its declared start tick')
            self.initial_pending=False
            return BodyReference(tuple(self.p),tuple(self.v),tuple(self.a),self.yaw),False
        dt=.04 if self.last is None else now-self.last
        if not 0 < dt <= .080001:raise ValueError('Recovery update gap')
        target=np.array([0.,0.,min(.6,max(0.,self.home[2]-self.p[2]))])
        clear=observation.motion.pose.position_world_m[2] >= self.home[2]-.15
        if clear:target=np.clip(.6*(self.home-self.p),-.8,.8)
        desired=1.5*(target-self.v)
        desired[2]=np.clip(desired[2],-1.,1.)
        desired[:2]*=min(1.,.65/max(np.linalg.norm(desired[:2]),1e-12))
        delta=desired-self.a
        delta[2]=np.clip(delta[2],-2.*dt,2.*dt)
        delta[:2]*=min(1.,2.*dt/max(np.linalg.norm(delta[:2]),1e-12))
        # Separate caps reserve upward authority during simultaneous XY braking:
        # Desired caps are XY=.65, Z=1 with jerk<3. An admitted initial
        # command retains its PVA; the live final limiter still enforces the
        # original global acceleration/speed/jerk bounds during transition.
        next_a=self.a+delta
        self.p+=self.v*dt+.5*self.a*dt*dt
        self.v+=.5*(self.a+next_a)*dt;self.a=next_a;self.last=now
        stable=norm(sub(observation.motion.pose.position_world_m,self.home)) < .08 and norm(observation.motion.velocity_at_origin_world_m_s) < .08
        self.stable_since=(now if self.stable_since is None else self.stable_since) if stable else None
        done=self.stable_since is not None and now-self.stable_since >= 1.
        return BodyReference(tuple(self.p),tuple(self.v),tuple(self.a),self.yaw),done


def tracking_contact_envelope(plan,state,observation,platform,now,envelope,window_s=5.,error_horizon_s=.35):
    """Causal frozen tracking residual for terminal landing-point constraints.

    Platform frame displacement is removed here: the new optimization already
    targets its latest causal platform model. This is an engineering feedback
    envelope, not an invariant or statistical tracking-error certificate.
    """
    if plan is None or not 0<plan.arrival-now<=window_s:return None
    if not all(0<=now-t<=.06000001 for t in (state.stamp,observation.sim_t_s,platform.stamp)):
        raise ValueError('Fresh causal tracking feedback required')
    nominal=plan.at(now-plan.start);base=plan.body_reference_at(now)
    model=PlatformState(**plan.audit['platform_prediction_input'])
    ref,_=measured_frame_reference(plan,model,platform,now,base)
    horizon=min(error_horizon_s,plan.arrival-now)
    error=vec(state.position)-nominal['position']-(vec(ref.position_world_m)-vec(base.position_world_m))
    error+=horizon*(vec(state.velocity)-nominal['velocity']-(vec(ref.velocity_world_m_s)-vec(base.velocity_world_m_s)))
    error[2]=0.
    from scipy.spatial.transform import Rotation
    actual=Rotation.from_quat(observation.motion.pose.orientation_xyzw)
    rotation=Rotation.from_quat(plan.motion(now-plan.start).pose.orientation_xyzw)
    component=tuple(abs(float(v)) for v in (actual*rotation.inv()).as_euler('xyz'))
    return dict(stamp_s=now,source_plan_start_s=plan.start,source_plan_arrival_s=plan.arrival,
        propagation_horizon_s=horizon,position_error_world_m=error.tolist(),attitude_reserve_m=envelope.rotation_projection_bound(component))


def terminal_body_prediction(plan,state,observation,platform,envelope,now,margin,reference_override=None,frame_model=None,uncertainty=None,reaction_horizon_s=.2):
    """Measured-error contact preview, not an invariant escape guarantee.

    A bounded residual is propagated for at most .35 s; reference terminal
    geometry is evaluated on the latest causal platform prediction. This adds
    an early guard to the post-contact checks, without relaxing their limits.
    """
    remaining=plan.arrival-now
    if not 0<remaining<=1.5:return dict(active=False,passed=True)
    nominal=plan.at(now-plan.start);end=plan.motion(plan.duration)
    correction_p=np.zeros(3);correction_v=np.zeros(3);end_shift=np.zeros(3)
    if reference_override is not None:
        base=plan.body_reference_at(now)
        correction_p=vec(reference_override.position_world_m)-vec(base.position_world_m)
        correction_v=vec(reference_override.velocity_world_m_s)-vec(base.velocity_world_m_s)
    if frame_model is not None:
        end_shift=vec(platform.predict(remaining).position)-vec(frame_model.predict(plan.arrival-frame_model.stamp).position)
        end_shift[2]=0.
    error=vec(state.position)-nominal['position']-correction_p+min(.35,remaining)*(vec(state.velocity)-nominal['velocity']-correction_v)
    error+=end_shift
    predicted=replace(end,pose=replace(end.pose,position_world_m=tuple(vec(end.pose.position_world_m)+error)))
    deck=platform.predict(remaining).deck()
    from scipy.spatial.transform import Rotation
    actual=Rotation.from_quat(observation.motion.pose.orientation_xyzw)
    nominal_rotation=Rotation.from_quat(plan.motion(now-plan.start).pose.orientation_xyzw)
    component_bounds=tuple(abs(float(a)) for a in (actual*nominal_rotation.inv()).as_euler('xyz'))
    attitude_reserve=envelope.rotation_projection_bound(component_bounds)
    clearance=envelope.clearance(predicted,deck)-attitude_reserve
    from .prediction_uncertainty import reaction_reserve
    reserve=reaction_reserve(uncertainty,now,remaining,reaction_horizon_s,envelope.radius)
    clearance-=reserve['reserve_m']
    return dict(active=True,passed=bool(clearance>=margin),remaining_s=remaining,
        predicted_whole_aircraft_margin_m=float(clearance),required_margin_m=margin,
        uncertainty_reserve=reserve,
        attitude_error_component_bounds_rad=component_bounds,
        attitude_reserve_model='axis_bounded_envelope_projection',
        propagated_position_error_m=error.tolist(),attitude_reserve_m=float(attitude_reserve))


def measured_frame_reference(plan,model,platform,now,reference):
    """Quintic blend into measured-deck-relative terminal PVA control.

    This feedback correction is logged separately from the nominal SOCP
    certificate. The dispatcher still sends ordinary bounded PVA commands.
    """
    remaining=plan.arrival-now
    if not 0<remaining<5.:return reference,dict(active=False)
    predicted=model.predict(now-model.stamp)
    dp=vec(platform.position)-vec(predicted.position);dv=vec(platform.velocity)-vec(predicted.velocity);da=vec(platform.acceleration)-vec(predicted.acceleration)
    dp[2]=dv[2]=da[2]=0.
    # Correct the measured platform frame while there is still maneuvering
    # time. A three-second blend bounds the curvature term to a quarter of
    # the previous 1.5-second blend for the same position discrepancy.
    span=3.
    s=min(1.,max(0.,(5.-remaining)/span))
    w=10*s**3-15*s**4+6*s**5
    wd=30*s*s*(1-s)**2/span if s<1 else 0.
    wdd=60*s*(1-s)*(1-2*s)/span**2 if s<1 else 0.
    cp=w*dp;cv=w*dv+wd*dp;ca=w*da+2*wd*dv+wdd*dp
    ref=replace(reference,position_world_m=tuple(vec(reference.position_world_m)+cp),velocity_world_m_s=tuple(vec(reference.velocity_world_m_s)+cv),acceleration_world_m_s2=tuple(vec(reference.acceleration_world_m_s2)+ca))
    return ref,dict(active=True,blend=float(w),position_correction_m=cp.tolist(),velocity_correction_m_s=cv.tolist(),acceleration_correction_m_s2=ca.tolist())


def frame_velocity_reserve_check(correction,maximum_correction_m_s=.3):
    """Bound dependence on a stale nominal prediction during low descent.

    This optional experimental check is disabled in the current runtime:
    a difference from a stale nominal velocity is not itself an aircraft
    tracking error or an actuation-limit violation. Actual measured tracking,
    final PVA limits, geometry and turning-domain gates retain ownership.
    """
    if not math.isfinite(maximum_correction_m_s) or maximum_correction_m_s<=0:
        raise ValueError('Positive finite frame velocity reserve required')
    if not correction or not correction.get('active'):
        return dict(active=False,passed=True)
    value=vec(correction['velocity_correction_m_s'])
    speed=float(np.linalg.norm(value[:2]))
    return dict(active=True,passed=bool(speed<=maximum_correction_m_s+1e-12),
        velocity_correction_m_s=value.tolist(),horizontal_correction_m_s=speed,
        maximum_correction_m_s=maximum_correction_m_s)


def contact_heading_acceleration_guard(platform,maximum_rad_s2=.04):
    """Declared turning-domain guard for fixed-world-heading small-deck contact.

    A rapidly reversing deck heading consumes the small projection allowance
    before a late position-error alarm. This measured-state design guard does
    not predict an unknown turn schedule or prove a general escape invariant.
    """
    value=float(platform.yaw_acceleration)
    if not math.isfinite(maximum_rad_s2) or maximum_rad_s2<=0 or not math.isfinite(value):
        raise ValueError('Finite positive heading acceleration design required')
    return dict(active=True,passed=bool(abs(value)<=maximum_rad_s2),
        measured_yaw_acceleration_rad_s2=value,maximum_yaw_acceleration_rad_s2=maximum_rad_s2)


def braking_clearance_design(state,response_s=.2,upward_acceleration=1.,vertical_jerk=2.,reserve=.05):
    """Declared stopping-distance design for fixed-height deck recovery.

    .2 s response allowance is a simulation guard choice, not identified as a
    uniform physical guarantee. Independent recovery runs must validate it.
    """
    speed=max(0.,-state.velocity[2]);down_acc=max(0.,-state.acceleration[2])
    loss=speed*response_s+.5*down_acc*response_s**2
    speed+=down_acc*response_s
    ramp=(upward_acceleration+down_acc)/vertical_jerk
    stop=(down_acc+math.sqrt(down_acc**2+2*vertical_jerk*speed))/vertical_jerk
    t=min(ramp,stop)
    loss+=speed*t+.5*down_acc*t*t-vertical_jerk*t**3/6
    if stop>ramp:
        remaining=speed+down_acc*ramp-.5*vertical_jerk*ramp*ramp
        loss+=remaining**2/(2*upward_acceleration)
    return dict(stopping_loss_m=float(loss),required_clearance_m=float(loss+reserve),response_allowance_s=response_s,
        upward_acceleration_design_m_s2=upward_acceleration,vertical_jerk_design_m_s3=vertical_jerk)
