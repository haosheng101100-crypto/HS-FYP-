"""Cached derivative/Horner evaluation for the new candidate search only.

Keep pinned legacy reference classes untouched. Endpoints, sample counts and
flight/contact checks retain their definitions; equivalence is tested.
"""
import math
from .septic_approach import PlanarSnapMatchedApproach
from .contact_geometry import evaluate_airframe

class CachedScalar:
    def __init__(self,source,seconds=False):
        self.source=source;self.seconds=seconds;self.cache={}
    def __getattr__(self,name):return getattr(self.source,name)
    def at(self,t,order=0):
        if not math.isfinite(t) or not isinstance(order,int) or order<0:
            raise ValueError('Finite time and valid polynomial derivative required')
        if self.seconds and order>=len(self.source.c):return 0.
        if not self.seconds and order>=len(self.source.coefficients):raise ValueError('Derivative exceeds degree')
        if t < -1e-12 or t>self.T+1e-12:raise ValueError('No continuation beyond planned polynomial interval')
        t=min(self.T,max(0.,t))
        if not self.seconds:
            if order<=3 and t==0:return self.initial[order]
            if order<=3 and t==self.T:return self.terminal[order]
            if order==4 and t==0:return self.first_snap
            if order==4 and t==self.T:return self.last_snap
        if order not in self.cache:
            coefficients=self.source.c if self.seconds else self.source.coefficients
            self.cache[order]=tuple(coefficients[k]*math.factorial(k)/math.factorial(k-order) for k in range(order,len(coefficients)))
        x=t if self.seconds else t/self.T;value=0.
        for c in reversed(self.cache[order]):value=value*x+c
        return value if self.seconds else value/self.T**order

class CachedBrakeAxis:
    def __init__(self,source,duration):
        self.source=source;self.T=duration;self.pieces=[(t,CachedScalar(p,seconds=True)) for t,p in source.pieces]
    def __getattr__(self,name):return getattr(self.source,name)
    def at(self,t,order=0):
        if t < -1e-9 or t>self.T+1e-9:raise ValueError('No continuation beyond planned contact')
        if t>=self.stop_s:return self.target if order==0 else 0.
        start,piece=next((s,p) for s,p in reversed(self.pieces) if t>=s)
        return piece.at(t-start,order)

def cached_attitude_capture_plan(geometry,profile,observation,deck,acceleration,duration,terminal,boundary,axis):
    offset=profile['nominal_com_in_body_m'];com=observation.motion.pose.transform(offset)
    v=observation.motion.velocity_at_world_point(com)
    y=com[1]-offset[1]-terminal.motion.pose.position_world_m[1]
    curve=PlanarSnapMatchedApproach(com,(v[0],0,v[2]),boundary,offset,9.80665,profile['mass_kg'],duration,
        terminal_com_snap_world_m_s4=(0,0,0))
    curve.axes=(CachedBrakeAxis(axis,duration),CachedScalar(curve.axes[1]),CachedScalar(curve.axes[2]))
    # These gates only consume COM velocities. Evaluate those exact components
    # at all original 201 timestamps; reserve full rigid-body transforms for
    # the separate 81-point moving-footprint check.
    times=[duration*i/200 for i in range(201)]
    vx=[curve.axes[0].at(t,1) for t in times]
    vz=[curve.axes[2].at(t,1) for t in times]
    for t in times:
        ax=curve.axes[0].at(t,2);up=curve.gravity+curve.axes[2].at(t,2)
        if up<=0 or ax*ax+up*up<1e-10:raise ValueError('Invalid point-mass thrust direction')
    checks={'reference_never_reverses':min(vx)>=-1e-8,
        'reference_keeps_descending':max(vz)<-.15,
        'reference_braking_acceleration':axis.peak_deceleration<=3.4,
        'reference_vertical_speed':max(abs(v) for v in vz)<=1.2,
        'finite_deck_terminal_footprint':all(min(f.whole_cylinder_footprint_margins_m)>=.02 for f in evaluate_airframe(geometry,boundary.aircraft,terminal)),
        'braking_release_before_contact':axis.stop_s<duration-.25}
    if not all(checks.values()):raise ValueError('Cached attitude candidate rejected: '+str(checks))
    return curve,terminal,{'reference_checks':checks,'measured_deck':deck,'measured_acceleration_m_s2':acceleration,
        'predicted_stop':terminal,'duration_s':duration,'horizontal_braking_duration_s':axis.stop_s,
        'peak_reference_deceleration_m_s2':axis.peak_deceleration,'horizontal_pieces':axis.coefficients,
        'vertical_coefficients':curve.axes[2].coefficients,'target_contact_x_on_deck_m':0.,
        'centre_landing_contract':{'maximum_body_horizontal_distance_m':.15,'target_xy_on_deck_m':[0.,y],
            'lateral_strategy':'Preserve measured lateral position within 2 cm for the planar approach'},
        'control_interface':'SET_ATTITUDE_TARGET with GUID_OPTIONS bit 3 clear: normalized climb-rate field',
        'reference_deceleration_limit_m_s2':3.4,
        'command_limits':{'horizontal_acceleration_m_s2':3.5,'tilt_deg':19.5,'angular_slew_deg_s':70.,'climb_rate_m_s':[-1.2,.5]},
        'actual_flight_limits':{'tilt_deg':20.,'angular_speed_deg_s':80.},
        'no_claim_of_old_PVA_jerk_snap_envelope':True,'physics_verification_required':True}
