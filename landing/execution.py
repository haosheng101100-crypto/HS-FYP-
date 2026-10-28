"""Simulation execution contract; no network, simulator or actuator side effects.

The entry gate permits bounded residuals, not exact C2/C3 state matching.
Observations must come from a synchronized, fresh state/mode provider. Native
LAND owns post-contact attitude and motors; this code does not solve impact.
"""
from collections import deque
from dataclasses import dataclass, asdict
import math
from .contact_geometry import RigidMotion, add, sub, norm, vector, evaluate_airframe, dot


@dataclass(frozen=True)
class AircraftObservation:
    sim_t_s: float
    motion: RigidMotion  # body origin, world axes, never the old virtual foot
    armed: bool
    flight_mode: str  # provider must synchronize and check heartbeat freshness
    com_acceleration_world_m_s2: tuple | None = None

    def __post_init__(self):
        if not math.isfinite(self.sim_t_s):raise ValueError('Finite simulation timestamp required')
        if not isinstance(self.armed,bool) or not isinstance(self.flight_mode,str):raise ValueError('Explicit armed flag and mode required')
        if self.com_acceleration_world_m_s2 is not None:
            object.__setattr__(self,'com_acceleration_world_m_s2',vector(self.com_acceleration_world_m_s2))


@dataclass(frozen=True)
class ContactFrame:
    sim_t_s: float
    leg_names: tuple = ()
    unexpected_uav_collisions: tuple = ()

    def __post_init__(self):
        if not math.isfinite(self.sim_t_s):raise ValueError('Finite contact timestamp required')
        for field in ['leg_names','unexpected_uav_collisions']:
            object.__setattr__(self,field,tuple(sorted(set(getattr(self,field)))))

    @property
    def active(self):return bool(self.leg_names or self.unexpected_uav_collisions)


def parse_deck_contacts(message,deck_collision,leg_collision_names,uav_model_prefix):
    """Exact collision-pair matching, including explicit empty frames."""
    stamp=message['header']['stamp'];t=float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9
    legs=set();unexpected=set()
    for contact in message.get('contact',[]):
        a=contact['collision1']['name'];b=contact['collision2']['name']
        other=b if a==deck_collision else a if b==deck_collision else None
        if other is None:continue
        if other in leg_collision_names:legs.add(leg_collision_names[other])
        elif other.startswith(uav_model_prefix):unexpected.add(other)
    return ContactFrame(t,tuple(legs),tuple(unexpected))


class ContactHistory:
    """One canonical contact stream; future frames stay buffered, never used early."""
    def __init__(self,max_frames=10000):
        if not isinstance(max_frames,int) or max_frames<2:raise ValueError('Positive history capacity required')
        self.frames=deque(maxlen=max_frames)
        self.last_poll_t=None
        self.discarded_through_s=None

    def push(self,frame):
        if self.frames and frame.sim_t_s<self.frames[-1].sim_t_s:raise ValueError('Out-of-order contact stream')
        if self.frames and frame.sim_t_s==self.frames[-1].sim_t_s:
            if frame!=self.frames[-1]:raise ValueError('Conflicting contact frames at the same time')
            return
        if len(self.frames)==self.frames.maxlen:self.discarded_through_s=self.frames[0].sim_t_s
        self.frames.append(frame)

    def at(self,t,max_age_s):
        if not math.isfinite(t) or not math.isfinite(max_age_s) or max_age_s<=0:raise ValueError('Finite causal query required')
        for frame in reversed(self.frames):
            if frame.sim_t_s<=t:return frame if t-frame.sim_t_s<=max_age_s else None
        return None

    def poll(self,t,max_age_s):
        """Return current support AND the earliest new active collision event.

        A short collision/bounce must not be overwritten by a later empty or
        multi-leg support frame before the slower control loop runs.
        """
        current=self.at(t,max_age_s)
        if self.last_poll_t is not None and t<=self.last_poll_t:raise ValueError('Contact polls must advance')
        lower=self.last_poll_t if self.last_poll_t is not None else t-max_age_s-1e-12
        if self.discarded_through_s is not None and lower<self.discarded_through_s<=t:
            raise ValueError('Unconsumed contact history was overwritten; first collision is unknown')
        event=next((f for f in self.frames if lower<f.sim_t_s<=t and f.active),None)
        self.last_poll_t=t
        return current,event


@dataclass(frozen=True)
class EntryLimits:
    position_error_m:float
    velocity_error_m_s:float
    attitude_error_rad:float
    angular_velocity_error_rad_s:float
    com_acceleration_error_m_s2:float
    observation_age_s:float

    def __post_init__(self):_positive(self)


def _positive(value):
    if any(not math.isfinite(x) or x<=0 for x in asdict(value).values()):raise ValueError('Positive finite experiment conditions required')


def attitude_error(a,b):
    cosine=abs(sum(x*y for x,y in zip(a.orientation_xyzw,b.orientation_xyzw)))
    return 2*math.acos(min(1.,max(0.,cosine)))


def assess_entry(now,observation,reference,com_offset_body_m,limits,contact=None):
    offset=vector(com_offset_body_m);actual=observation.motion;target=reference['motion']
    com=actual.pose.transform(offset);target_com=target.pose.transform(offset)
    residuals={
        'position_error_m':norm(sub(com,target_com)),
        'velocity_error_m_s':norm(sub(actual.velocity_at_world_point(com),target.velocity_at_world_point(target_com))),
        'attitude_error_rad':attitude_error(actual.pose,target.pose),
        'angular_velocity_error_rad_s':norm(sub(actual.angular_velocity_world_rad_s,target.angular_velocity_world_rad_s))}
    checks={k:v<=getattr(limits,k) for k,v in residuals.items()}
    age=now-observation.sim_t_s
    checks.update(observation_fresh=math.isfinite(now) and 0<=age<=limits.observation_age_s,
                  armed=observation.armed,guided_mode=observation.flight_mode=='GUIDED')
    if observation.com_acceleration_world_m_s2 is None:
        checks['com_acceleration_available']=False
    else:
        value=norm(sub(observation.com_acceleration_world_m_s2,reference['com_acceleration_world_m_s2']))
        residuals['com_acceleration_error_m_s2']=value
        checks['com_acceleration_matches']=value<=limits.com_acceleration_error_m_s2
    checks['no_recent_deck_contact']=not(contact and 0<=now-contact.sim_t_s<=limits.observation_age_s and contact.active)
    return {'ready':all(checks.values()),'checks':checks,'residuals':residuals,
            'observation_age_s':age,'actual_com_position_world_m':com,
            'scope':'Bounded initial residual gate; does not prepare the aircraft or guarantee derivative continuity.'}


@dataclass(frozen=True)
class BodyReference:
    position_world_m:tuple
    velocity_world_m_s:tuple
    acceleration_world_m_s2:tuple
    yaw_enu_rad:float

    @classmethod
    def from_state(cls,state):
        motion=state['motion'];x,y,z,w=motion.pose.orientation_xyzw
        return cls(motion.pose.position_world_m,motion.velocity_at_origin_world_m_s,
                   state['body_acceleration_world_m_s2'],math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


@dataclass(frozen=True)
class ExecutionDecision:
    phase:str
    stop_reference:bool
    reference:BodyReference|None=None
    request_land:bool=False
    pending_land_velocity_world_m_s:tuple|None=None
    complete:bool=False
    failures:tuple=()


@dataclass(frozen=True)
class HandoverLimits:
    observation_age_s:float
    contact_age_s:float
    update_gap_s:float
    contact_time_error_s:float
    contact_xy_error_m:float
    contact_clearance_error_m:float
    contact_tangential_speed_m_s:float
    contact_normal_speed_m_s:float
    contact_minimum_closing_speed_m_s:float
    contact_attitude_error_rad:float
    contact_tilt_rad:float
    contact_angular_speed_rad_s:float
    footprint_margin_m:float
    support_minimum_legs:int
    support_maximum_gap_m:float
    support_penetration_m:float
    support_tilt_rad:float
    support_speed_m_s:float
    support_angular_speed_rad_s:float
    support_dwell_s:float
    post_disarm_dwell_s:float
    land_request_interval_s:float
    land_ack_timeout_s:float
    settle_timeout_s:float
    disarm_timeout_s:float

    def __post_init__(self):
        _positive(self)
        if not isinstance(self.support_minimum_legs,int) or self.support_minimum_legs<3:
            raise ValueError('At least three simultaneous known leg contacts required')
        if self.contact_minimum_closing_speed_m_s>self.contact_normal_speed_m_s:raise ValueError('Invalid closing-speed interval')


class ContactHandover:
    def __init__(self,geometry,deck,terminal_motion,limits,expected_contact_s=None):
        self.geometry=geometry;self.deck=deck;self.target=terminal_motion;self.limits=limits
        if not geometry.simulation_only:raise ValueError('This execution prototype is simulation-only')
        if norm(deck.motion.velocity_at_origin_world_m_s)+norm(deck.motion.angular_velocity_world_rad_s)>1e-10:
            raise ValueError('Static deck required')
        if limits.support_minimum_legs>len(geometry.legs):raise ValueError('Support count exceeds configured legs')
        self.expected_contact_s=expected_contact_s;self.phase='AIRBORNE';self.first_contact=None
        self.started=None;self.last_t=None;self.last_land_request=None
        self.stable_since=None;self.disarm_wait_since=None;self.post_disarm_since=None
        self.failures=[];self.last_support_checks={}
        self.observation_history=deque(maxlen=128)

    def fail(self,reason):
        if reason not in self.failures:self.failures.append(reason)

    def begin(self,now,reason=None):
        if self.started is None:self.started=now;self.phase='LAND_SETTLE'
        if reason:self.fail(reason)

    def _fresh_contact(self,now,contact):
        return contact if contact and 0<=now-contact.sim_t_s<=self.limits.contact_age_s else None

    def _capture(self,now,observation,contact):
        self.begin(now)
        preceding=[o for o in self.observation_history if o.sim_t_s<contact.sim_t_s-1e-10]
        if preceding:observation=preceding[-1]
        feet={f.name:f for f in evaluate_airframe(self.geometry,observation.motion,self.deck)}
        known=[feet[name] for name in contact.leg_names if name in feet]
        normal=self.deck.motion.pose.rotate((0,0,1))
        tilt=math.acos(max(-1.,min(1.,dot(normal,observation.motion.pose.rotate((0,0,1))))))
        delta=self.deck.motion.pose.inverse_rotate(sub(observation.motion.pose.position_world_m,self.target.pose.position_world_m))
        checks={
            'precontact_observation_available':bool(preceding),
            'known_legs_only':bool(known) and len(known)==len(contact.leg_names) and not contact.unexpected_uav_collisions,
            'observation_fresh':0<=now-observation.sim_t_s<=self.limits.observation_age_s,
            'target_xy':math.hypot(*delta[:2])<=self.limits.contact_xy_error_m,
            'tilt':tilt<=self.limits.contact_tilt_rad,
            'attitude_target':attitude_error(observation.motion.pose,self.target.pose)<=self.limits.contact_attitude_error_rad,
            'angular_speed':norm(observation.motion.angular_velocity_world_rad_s)<=self.limits.contact_angular_speed_rad_s,
            'leg_clearance':bool(known) and all(abs(f.clearance_m)<=self.limits.contact_clearance_error_m for f in known),
            'leg_speed':bool(known) and all(f.tangential_speed_upper_bound_m_s<=self.limits.contact_tangential_speed_m_s and
                f.closing_speed_upper_bound_m_s<=self.limits.contact_normal_speed_m_s and
                2*f.closing_speed_m_s-f.closing_speed_upper_bound_m_s>=self.limits.contact_minimum_closing_speed_m_s for f in known),
            'footprint':all(min(f.whole_cylinder_footprint_margins_m)>=self.limits.footprint_margin_m for f in feet.values())}
        if self.expected_contact_s is not None:
            checks['contact_time']=abs(contact.sim_t_s-self.expected_contact_s)<=self.limits.contact_time_error_s
        self.first_contact={'contact_sim_t_s':contact.sim_t_s,'decision_sim_t_s':now,'observation_sim_t_s':observation.sim_t_s,
            'legs':contact.leg_names,'unexpected_uav_collisions':contact.unexpected_uav_collisions,'checks':checks,
            'scope':'Latest causal observation strictly before reported contact, not exact instantaneous impact velocity.'}
        for key,passed in checks.items():
            if not passed:self.fail('first_contact_'+key)

    def _support(self,now,observation,contact):
        feet=evaluate_airframe(self.geometry,observation.motion,self.deck)
        names=set(contact.leg_names) if contact else set()
        known={leg.name for leg in self.geometry.legs}
        fresh=0<=now-observation.sim_t_s<=self.limits.observation_age_s
        tilt=math.acos(max(-1.,min(1.,dot(self.deck.motion.pose.rotate((0,0,1)),observation.motion.pose.rotate((0,0,1))))))
        self.last_support_checks={
            'observation_fresh':fresh,'land_mode_acknowledged':observation.flight_mode=='LAND',
            'simultaneous_known_legs':len(names)>=self.limits.support_minimum_legs and names<=known,
            'no_unexpected_collision':contact is not None and not contact.unexpected_uav_collisions,
            'all_leg_gaps':all(-self.limits.support_penetration_m<=f.clearance_m<=self.limits.support_maximum_gap_m for f in feet),
            'footprint':all(min(f.whole_cylinder_footprint_margins_m)>=self.limits.footprint_margin_m for f in feet),
            'tilt':tilt<=self.limits.support_tilt_rad,
            'speed':norm(observation.motion.velocity_at_origin_world_m_s)<=self.limits.support_speed_m_s,
            'angular_speed':norm(observation.motion.angular_velocity_world_rad_s)<=self.limits.support_angular_speed_rad_s}
        return all(self.last_support_checks.values())

    def step(self,now,observation,contact=None,contact_event=None):
        if not math.isfinite(now):raise ValueError('Finite simulation time required')
        if self.last_t is not None and now<=self.last_t:raise ValueError('Execution time must increase')
        gap=self.last_t is not None and now-self.last_t>self.limits.update_gap_s
        self.last_t=now;contact=self._fresh_contact(now,contact)
        if not self.observation_history or observation.sim_t_s>self.observation_history[-1].sim_t_s:self.observation_history.append(observation)
        event=contact_event if contact_event and contact_event.sim_t_s<=now and contact_event.active else contact
        if self.first_contact is None and event and event.active:self._capture(now,observation,event)
        if self.started is None:return ExecutionDecision('AIRBORNE',False)
        if contact and contact.unexpected_uav_collisions:self.fail('unexpected_collision_during_handover')
        fresh=0<=now-observation.sim_t_s<=self.limits.observation_age_s
        if not fresh:self.fail('observation_stale_during_handover')
        if gap:self.stable_since=None
        if gap and self.phase=='WAIT_DISARM':self.phase='LAND_SETTLE'
        support=self._support(now,observation,contact)
        if self.phase=='LAND_SETTLE':
            if not observation.armed:self.fail('disarmed_before_support_gate');self.phase='FAILED'
            elif support:
                if self.stable_since is None:self.stable_since=now
                if now-self.stable_since>=self.limits.support_dwell_s-1e-10:
                    self.phase='WAIT_DISARM';self.disarm_wait_since=now
            else:self.stable_since=None
            if self.phase=='LAND_SETTLE' and now-self.started>self.limits.settle_timeout_s:
                self.fail('support_gate_timeout');self.phase='FAILED'
        elif self.phase=='WAIT_DISARM':
            if not support:
                self.phase='LAND_SETTLE';self.stable_since=None
                if not observation.armed:self.fail('support_lost_at_disarm');self.phase='FAILED'
            elif not observation.armed:
                self.phase='POST_DISARM';self.post_disarm_since=now
            elif now-self.disarm_wait_since>self.limits.disarm_timeout_s:
                self.fail('normal_disarm_timeout');self.phase='FAILED'
        elif self.phase=='POST_DISARM':
            if observation.armed or not support or gap:
                self.fail('post_disarm_support_lost');self.phase='FAILED'
            elif now-self.post_disarm_since>=self.limits.post_disarm_dwell_s-1e-10:self.phase='COMPLETE'
        request=False
        if observation.armed and observation.flight_mode!='LAND':
            if now-self.started>self.limits.land_ack_timeout_s:self.fail('land_mode_ack_timeout');self.phase='FAILED'
            if self.last_land_request is None or now-self.last_land_request>=self.limits.land_request_interval_s-1e-10:
                request=True;self.last_land_request=now
        # While LAND is not acknowledged, replace the old PVA with a zero
        # velocity-only command in GUIDED. In LAND, send no external setpoints.
        fallback=(0.,0.,0.) if observation.armed and observation.flight_mode=='GUIDED' else None
        return ExecutionDecision(self.phase,True,request_land=request,pending_land_velocity_world_m_s=fallback,
            complete=self.phase=='COMPLETE' and not self.failures,failures=tuple(self.failures))


class LandingExecutor:
    """Entry gate -> reference -> irreversible native-LAND handover."""
    def __init__(self,approach,geometry,deck,entry_limits,handover_limits,lead_in=None):
        self.approach=approach;self.entry_limits=entry_limits;self.start_s=None;self.last_t=None
        self.handover=ContactHandover(geometry,deck,approach.terminal_boundary.aircraft,handover_limits)
        self.entry_report=None;self.approach_entry_report=None;self.lead_in=lead_in;self.lead_start_s=None

    def experiment_can_finish(self,now,armed,disarmed_since):
        """Stopping capture is distinct from passing touchdown goals.

        Goal failures must not end capture just before the post-disarm state
        completes its own dwell. Failed safety exits retain a bounded tail.
        """
        if armed:return False
        if self.handover.phase=='COMPLETE':return True
        return self.handover.phase=='FAILED' and disarmed_since is not None and now-disarmed_since>=self.handover.limits.post_disarm_dwell_s

    def step(self,now,observation,contact=None,contact_event=None):
        if not math.isfinite(now) or self.last_t is not None and now<=self.last_t:raise ValueError('Execution time must increase')
        was_tracking=self.start_s is not None or self.lead_start_s is not None
        update_gap=was_tracking and self.last_t is not None and now-self.last_t>self.handover.limits.update_gap_s
        self.last_t=now
        event_active=contact_event and contact_event.sim_t_s<=now and contact_event.active
        # Process physical contact before reference expiry; never evaluate the
        # flight polynomial after its terminal time.
        if self.start_s is None and self.lead_start_s is None and self.handover.started is None:
            reference=(self.lead_in or self.approach).at(0)
            self.entry_report=assess_entry(now,observation,reference,self.approach.com_offset_body_m,self.entry_limits,contact)
            if event_active or contact and 0<=now-contact.sim_t_s<=self.handover.limits.contact_age_s and contact.active:
                self.handover.begin(now,'contact_before_reference_start')
            elif not self.entry_report['ready']:return ExecutionDecision('WAIT_ENTRY',True)
            else:
                if self.lead_in is None:self.start_s=now
                else:self.lead_start_s=now
                self.handover.expected_contact_s=now+self.approach.T+(0. if self.lead_in is None else self.lead_in.T)
        if self.lead_start_s is not None and self.start_s is None and self.handover.started is None:
            if now-self.lead_start_s>=self.lead_in.T:
                self.approach_entry_report=assess_entry(now,observation,self.approach.at(0),self.approach.com_offset_body_m,self.entry_limits,contact)
                if self.approach_entry_report['ready']:
                    self.start_s=now;self.handover.expected_contact_s=now+self.approach.T
                else:self.handover.begin(now,'lead_in_finished_without_matching_entry')
        active=self.start_s is not None or self.lead_start_s is not None
        if active and self.handover.started is None:
            age=now-observation.sim_t_s
            if not 0<=age<=self.entry_limits.observation_age_s:self.handover.begin(now,'tracking_observation_stale')
            if not observation.armed or observation.flight_mode!='GUIDED':self.handover.begin(now,'tracking_flight_state_changed')
            if update_gap:self.handover.begin(now,'tracking_update_gap')
            if self.start_s is None and (event_active or contact and 0<=now-contact.sim_t_s<=self.handover.limits.contact_age_s and contact.active):
                self.handover.begin(now,'contact_during_lead_in')
        elapsed=None if self.start_s is None else now-self.start_s
        recent_contact=event_active or contact and 0<=now-contact.sim_t_s<=self.handover.limits.contact_age_s and contact.active
        if elapsed is not None and elapsed>=self.approach.T and (self.handover.started is None or self.handover.started==now) and not recent_contact:
            self.handover.begin(now,'reference_ended_without_contact')
        handover=self.handover.step(now,observation,contact,contact_event)
        if handover.stop_reference:return handover
        if self.start_s is None:
            return ExecutionDecision('LEAD_IN',False,BodyReference.from_state(self.lead_in.at(now-self.lead_start_s)))
        return ExecutionDecision('TRACK_REFERENCE',False,BodyReference.from_state(self.approach.at(elapsed)))
