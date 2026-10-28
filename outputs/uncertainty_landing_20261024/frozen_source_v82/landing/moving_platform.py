"""Constant-velocity interception and measured moving-deck handover (SITL only).

The ninth-degree reference is in the inertial world frame. Its terminal foot
velocity is the predicted deck velocity plus the prescribed relative closing
velocity. No hovering or speed-matching segment precedes the descent.
"""
from dataclasses import replace, asdict
import json, math
from .contact_geometry import Pose, RigidMotion, Deck, add, sub, scale, norm, evaluate_airframe
from .touchdown_boundary import ContactDesignLimits, TouchdownRequest, solve_touchdown_boundary
from .septic_approach import PlanarSnapMatchedApproach
from .execution import ContactHandover,LandingExecutor,attitude_error


def translate_deck(deck, dt):
    if norm(deck.motion.angular_velocity_world_rad_s)>1e-3:
        raise ValueError('Constant-velocity planner requires a non-rotating deck')
    p=add(deck.motion.pose.position_world_m,scale(deck.motion.velocity_at_origin_world_m_s,dt))
    return Deck(RigidMotion(Pose(p,deck.motion.pose.orientation_xyzw),
        deck.motion.velocity_at_origin_world_m_s,(0.,0.,0.)),deck.half_extents_m)


def check_moving_reference(curve,geometry,deck_at_main_start,limits,samples=501):
    """Sample world dynamics and relative foot constraints; not a continuous proof."""
    extrema={}; failures=[]; last_relative_fast=None
    for i in range(samples):
        t=curve.T*i/(samples-1);s=curve.at(t);deck=translate_deck(deck_at_main_start,t)
        v=s['com_velocity_world_m_s'];a=s['com_acceleration_world_m_s2']
        values={'horizontal_speed_m_s':(math.hypot(*v[:2]),limits['max_horizontal_speed_m_s']),
            'vertical_speed_m_s':(abs(v[2]),limits['max_vertical_speed_m_s']),
            'horizontal_acceleration_m_s2':(math.hypot(*a[:2]),limits['max_horizontal_acceleration_m_s2']),
            'vertical_acceleration_m_s2':(abs(a[2]),limits['max_vertical_acceleration_m_s2']),
            'jerk_m_s3':(norm(s['com_jerk_world_m_s3']),limits['max_jerk_m_s3']),
            'snap_m_s4':(norm(s['com_snap_world_m_s4']),limits['max_snap_m_s4']),
            'tilt_deg':(math.degrees(abs(s['nose_up_rad'])),limits['max_tilt_deg']),
            'pitch_rate_deg_s':(math.degrees(abs(s['nose_up_rate_rad_s'])),limits['max_pitch_rate_deg_s']),
            'pitch_acceleration_deg_s2':(math.degrees(abs(s['nose_up_acceleration_rad_s2'])),limits['max_pitch_acceleration_deg_s2'])}
        for name,(value,maximum) in values.items():
            extrema[name]=max(extrema.get(name,0.),value)
            if value>maximum+1e-8:failures.append(name)
        if a[2]+curve.gravity<limits['minimum_upward_specific_force_m_s2']:failures.append('upward_force')
        if v[0]<-1e-4 or v[2]>.01:failures.append('reversal_or_climb')
        feet=evaluate_airframe(geometry,s['motion'],deck)
        if i<samples-1 and feet[0].clearance_m< -1e-8:failures.append('early_plane_penetration')
        for f in feet:
            if f.clearance_m<limits['edge_height_margin_m'] and min(f.whole_cylinder_footprint_margins_m)<limits['footprint_margin_m']-1e-8:
                failures.append('edge_margin')
            if f.clearance_m<=limits['near_contact_height_m']:
                if f.tangential_speed_upper_bound_m_s>limits['near_contact_tangential_speed_m_s']+1e-8:failures.append('near_tangent')
                if f.closing_speed_upper_bound_m_s>limits['near_contact_closing_speed_m_s']+1e-8:failures.append('near_closing')
                if abs(math.degrees(s['nose_up_rad']))>limits['near_contact_tilt_deg']+1e-8:failures.append('near_tilt')
        if math.hypot(*sub(v,deck.motion.velocity_at_origin_world_m_s)[:2])>=.12:last_relative_fast=t
    if last_relative_fast is None or curve.T-last_relative_fast>limits['max_low_horizontal_speed_lead_s']+1e-8:
        failures.append('long_speed_matching_lead')
    return {'passed':not failures,'failures':sorted(set(failures)),'sample_count':samples,'extrema':extrema,
        'relative_low_speed_lead_s':None if last_relative_fast is None else curve.T-last_relative_fast}


def plan_interception(root,geometry,profile,deck_now,lead_s=1.5,durations=None,snaps=None,validate=True):
    cfg=json.loads((root/'config/experiments/snap_approach_v2.json').read_text())
    bc=json.loads((root/cfg['boundary_config']).read_text());b=bc['design_limits']
    # Planar prediction uses a level deck. Measured roll/pitch/yaw must be
    # within the live scene gate; impact is assessed against measured motion.
    p=deck_now.motion.pose.position_world_m;v=deck_now.motion.velocity_at_origin_world_m_s
    if abs(v[1])>.005 or abs(v[2])>.005:raise ValueError('Forward constant-velocity scene required')
    ideal=Deck(RigidMotion(Pose((p[0],0.,p[2]),(0.,0.,0.,1.)),(v[0],0.,0.),(0.,0.,0.)),deck_now.half_extents_m)
    limits=ContactDesignLimits(math.radians(b['max_relative_tilt_deg']),math.radians(b['max_relative_angular_speed_deg_s']),
        b['max_tangential_speed_m_s'],b['min_closing_speed_m_s'],b['max_closing_speed_m_s'],b['footprint_margin_m'])
    offset=profile['nominal_com_in_body_m'];tried=[]
    for T,snap_z in ((T,snap) for T in (durations or [tick/10 for tick in range(64,121)]) for snap in (snaps or (0.,1.,2.,3.))):
        terminal_deck=translate_deck(ideal,lead_s+T)
        boundary=solve_touchdown_boundary(geometry,terminal_deck,TouchdownRequest.planar(math.radians(2.),0.,(0.,0.),(0.,0.,-.08)),limits)
        curve=PlanarSnapMatchedApproach((0.,offset[1],3.65),(0.,0.,-.4),boundary,offset,
            cfg['reference_gravity_m_s2'],profile['mass_kg'],T,terminal_com_snap_world_m_s4=(0.,0.,snap_z))
        report=check_moving_reference(curve,geometry,translate_deck(ideal,lead_s),cfg['design_limits']) if validate else {'passed':False,'failures':[],'pending_postflight_dense_check':True}
        tried.append({'duration_s':T,'terminal_snap_z_m_s4':snap_z,'failures':report['failures']})
        if report['passed'] or not validate:
            dense=check_moving_reference(curve,geometry,translate_deck(ideal,lead_s),cfg['design_limits'],2001) if validate else report
            if validate and not dense['passed']:continue
            return curve,ideal,{'simulation_only':True,'hardware_deployment_allowed':False,
                'duration_s':T,'lead_s':lead_s,'deck_at_planning':asdict(deck_now),'ideal_deck_at_planning':asdict(ideal),
                'target_contact_nose_up_deg':2.,'target_relative_foot_velocity_m_s':[0.,0.,-.08],
                'terminal_boundary':asdict(boundary),'normalized_polynomial_coefficients':[a.coefficients for a in curve.axes],
                'checks':dense,'duration_candidates':tried}
    raise ValueError('No feasible moving-deck reference in 6.4..12.0 s: '+str(tried))


class MovingContactHandover(ContactHandover):
    """Keep the original thresholds, using causal measured deck motion.

    Native LAND owns motors after collision. Its ability to remain on a moving
    deck is tested rather than assumed. No forced disarm or continuing flight
    polynomial is permitted after contact.
    """
    def __init__(self,geometry,terminal_boundary,terminal_deck,limits,deck_provider):
        stationary=Deck(RigidMotion.stationary(terminal_deck.motion.pose),terminal_deck.half_extents_m)
        super().__init__(geometry,stationary,terminal_boundary.aircraft,limits)
        self.deck_provider=deck_provider
        self.target_offset=sub(terminal_boundary.aircraft.pose.position_world_m,terminal_deck.motion.pose.position_world_m)
        self.target_q=terminal_boundary.aircraft.pose.orientation_xyzw

    def _capture(self,now,observation,contact):
        preceding=[o for o in self.observation_history if o.sim_t_s<contact.sim_t_s-1e-10]
        sample=preceding[-1] if preceding else observation
        self.deck=self.deck_provider(sample.sim_t_s)
        self.target=RigidMotion(Pose(add(self.deck.motion.pose.position_world_m,self.target_offset),self.target_q),
            self.deck.motion.velocity_at_origin_world_m_s,(0.,0.,0.))
        super()._capture(now,observation,contact)

    def _support(self,now,observation,contact):
        self.deck=self.deck_provider(observation.sim_t_s)
        relative=sub(observation.motion.velocity_at_origin_world_m_s,
            self.deck.motion.velocity_at_world_point(observation.motion.pose.position_world_m))
        # Base support gate uses only this velocity's magnitude, not foot speeds.
        relative_observation=replace(observation,motion=replace(observation.motion,velocity_at_origin_world_m_s=relative))
        return super()._support(now,relative_observation,contact)

    def step(self,now,observation,contact=None,contact_event=None):
        decision=super().step(now,observation,contact,contact_event)
        if decision.pending_land_velocity_world_m_s is not None:
            return replace(decision,pending_land_velocity_world_m_s=self.deck_provider(observation.sim_t_s).motion.velocity_at_origin_world_m_s)
        return decision


class MovingLandingExecutor(LandingExecutor):
    """Optional guarded native-LAND completion of the final <= 1 cm gap.

    At the planned terminal time the flight reference stops. LAND may finish
    contact only inside the original velocity/attitude/footprint/time gates.
    No airborne polynomial is evaluated beyond T, and no below-deck target is
    sent. Missing the capture funnel retains the original expiry failure.
    """
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.native_contact_completion=None

    def terminal_capture_checks(self,now,observation):
        h=self.handover;deck=h.deck_provider(observation.sim_t_s);feet=evaluate_airframe(h.geometry,observation.motion,deck)
        closest=[f for f in feet if f.clearance_m<=feet[0].clearance_m+.001]
        target=Pose(add(deck.motion.pose.position_world_m,h.target_offset),h.target_q)
        delta=deck.motion.pose.inverse_rotate(sub(observation.motion.pose.position_world_m,target.position_world_m))
        return {'fresh':0<=now-observation.sim_t_s<=h.limits.observation_age_s,
            'armed_guided':observation.armed and observation.flight_mode=='GUIDED',
            'gap_0_to_1cm':0<=feet[0].clearance_m<=.01,
            'relative_foot_speed':all(f.tangential_speed_upper_bound_m_s<=h.limits.contact_tangential_speed_m_s and
                f.closing_speed_upper_bound_m_s<=h.limits.contact_normal_speed_m_s and
                2*f.closing_speed_m_s-f.closing_speed_upper_bound_m_s>=h.limits.contact_minimum_closing_speed_m_s for f in closest),
            'attitude':attitude_error(observation.motion.pose,target)<=h.limits.contact_attitude_error_rad,
            'angular_speed':norm(observation.motion.angular_velocity_world_rad_s)<=h.limits.contact_angular_speed_rad_s,
            'target_xy':math.hypot(*delta[:2])<=h.limits.contact_xy_error_m,
            'footprint':all(min(f.whole_cylinder_footprint_margins_m)>=h.limits.footprint_margin_m for f in feet)}

    def step(self,now,observation,contact=None,contact_event=None):
        elapsed=None if self.start_s is None else now-self.start_s
        recent=(contact_event is not None and contact_event.sim_t_s<=now and contact_event.active) or (contact is not None and 0<=now-contact.sim_t_s<=self.handover.limits.contact_age_s and contact.active)
        if elapsed is not None and elapsed>=self.approach.T and self.handover.started is None and not recent:
            checks=self.terminal_capture_checks(now,observation)
            if self.last_t is not None and 0<now-self.last_t<=self.handover.limits.update_gap_s and all(checks.values()):
                self.native_contact_completion={'start_sim_t_s':now,'checks':checks,'maximum_gap_m':.01,
                    'latest_contact_sim_t_s':self.start_s+self.approach.T+self.handover.limits.contact_time_error_s,
                    'policy':'Stop reference at T; native LAND finishes guarded gap, still subject to original first-contact tests.'}
                self.handover.begin(now);self.last_t=now
                return self.handover.step(now,observation,contact,contact_event)
        decision=super().step(now,observation,contact,contact_event)
        if self.native_contact_completion and not self.handover.first_contact and now>self.native_contact_completion['latest_contact_sim_t_s']:
            self.handover.fail('native_contact_completion_timeout')
            decision=replace(decision,complete=False,failures=tuple(self.handover.failures))
        return decision
