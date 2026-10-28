"""One continuous braking descent, with a one-way horizontal reference.

This nominal SITL case predicts the stopping point from *measured* current
vehicle velocity/acceleration. No brake-command epoch is an input. It is not
a proof for arbitrary future braking changes, sensing delays or real hardware.
"""
import math
from .contact_geometry import Pose,RigidMotion,Deck,sub,evaluate_airframe
from .septic_approach import PlanarSnapMatchedApproach,NinthScalar
from .sudden_brake_landing import level_boundary
from .accelerating_feedback import CaptureAwareFeedback
from dataclasses import replace


class AccelerationPiece:
    def __init__(self,p,v,acceleration_coefficients,duration):
        self.T=duration
        self.c=(p,v,*[a/((i+1)*(i+2)) for i,a in enumerate(acceleration_coefficients)])
    def at(self,t,order=0):
        t=min(self.T,max(0.,t))
        return sum(self.c[i]*math.factorial(i)/math.factorial(i-order)*t**(i-order)
                   for i in range(order,len(self.c)))


class MonotoneBrakeAxis:
    """Smoothly enter braking, sustain it, smoothly release it; never reverse.

    For each plateau duration, area under deceleration fixes its magnitude.
    A scalar bisection matches stopping distance, rather than a polynomial
    that can overshoot and reverse to meet a distant endpoint.
    """
    def __init__(self,p,v,target,total_duration,ramp_in=.22,ramp_out=.65):
        if v<=0 or target<=p:raise ValueError('Forward stopping distance required')
        self.T=total_duration;self.target=target
        def build(hold):
            A=v/(hold+(ramp_in+ramp_out)/2)
            smooth=lambda r:[0.,0.,0.,10/r**3,-15/r**4,6/r**5]
            a=AccelerationPiece(p,v,[-A*x for x in smooth(ramp_in)],ramp_in)
            b=AccelerationPiece(a.at(a.T),a.at(a.T,1),[-A],hold)
            c=AccelerationPiece(b.at(b.T),b.at(b.T,1),[-A,0.,0.,* [A*x for x in smooth(ramp_out)[3:]]],ramp_out)
            return A,[(0.,a),(ramp_in,b),(ramp_in+hold,c)],c.at(c.T)
        lo=0.;hi=total_duration-ramp_in-ramp_out
        if hi<=0 or not build(lo)[2]<=target<=build(hi)[2]:raise ValueError('No monotone braking solution inside horizon')
        for _ in range(60):
            mid=(lo+hi)/2
            if build(mid)[2]<target:lo=mid
            else:hi=mid
        self.peak_deceleration,self.pieces,_=build((lo+hi)/2)
        self.stop_s=ramp_in+(lo+hi)/2+ramp_out
        self.coefficients=[{'start_s':t,'duration_s':a.T,'position_coefficients_seconds':a.c} for t,a in self.pieces]
    def at(self,t,order=0):
        if t< -1e-9 or t>self.T+1e-9:raise ValueError('No continuation beyond planned contact')
        if t>=self.stop_s:return self.target if order==0 else 0.
        start,piece=next((s,a) for s,a in reversed(self.pieces) if t>=s)
        return piece.at(t-start,order)


def predict_stopped_deck(deck,acceleration):
    vx=deck.motion.velocity_at_origin_world_m_s[0];ax=acceleration[0]
    if not -2.65<ax<-2.1 or not 4.5<vx<5.:
        raise ValueError('Not the measured onset of this 5 m/s sudden-braking case')
    p=deck.motion.pose.position_world_m
    target=(p[0]+vx*vx/(-2*ax),p[1],p[2])
    return Deck(RigidMotion.stationary(Pose(target,(0,0,0,1))),deck.half_extents_m)


def direct_braking_plan(geometry,profile,observation,deck,acceleration,duration=2.7,samples=1001,enforce_dynamic_limits=True):
    terminal=predict_stopped_deck(deck,acceleration)
    com_offset=profile['nominal_com_in_body_m'];com=observation.motion.pose.transform(com_offset)
    v=observation.motion.velocity_at_world_point(com)
    y=com[1]-com_offset[1]-terminal.motion.pose.position_world_m[1]
    boundary=level_boundary(geometry,terminal,y)
    curve=PlanarSnapMatchedApproach(com,(v[0],0.,v[2]),boundary,com_offset,9.80665,profile['mass_kg'],duration,
                                  terminal_com_snap_world_m_s4=(0,0,0))
    x=MonotoneBrakeAxis(com[0],v[0],curve.axes[0].terminal[0],duration)
    curve.axes=(x,*curve.axes[1:])
    values=[curve.at(duration*i/(samples-1)) for i in range(samples)]
    checks={'no_reverse_reference':min(s['com_velocity_world_m_s'][0] for s in values)>=-1e-8,
        'continuous_descent':max(s['com_velocity_world_m_s'][2] for s in values)<-.15,
        'horizontal_braking_bound':x.peak_deceleration<=3.2,
        'vertical_speed':max(abs(s['com_velocity_world_m_s'][2]) for s in values)<=1.2,
        'tilt':max(abs(s['nose_up_rad']) for s in values)<=math.radians(20),
        'no_early_penetration':all(evaluate_airframe(geometry,s['motion'],terminal)[0].clearance_m>=-1e-7 for s in values),
        'braking_release_before_contact':x.stop_s<duration-.25}
    if not all(checks.values()):raise ValueError('Continuous braking reference infeasible: '+str(checks))
    # The first physical prototype omitted these checks; its immutable failed
    # package is retained. Do not submit that fast shape for flight again.
    dynamic={'jerk_m_s3':max(math.sqrt(sum(a*a for a in s['com_jerk_world_m_s3'])) for s in values),
             'snap_m_s4':max(math.sqrt(sum(a*a for a in s['com_snap_world_m_s4'])) for s in values),
             'pitch_rate_deg_s':max(math.degrees(abs(s['nose_up_rate_rad_s'])) for s in values),
             'pitch_acceleration_deg_s2':max(math.degrees(abs(s['nose_up_acceleration_rad_s2'])) for s in values)}
    dynamic_checks={k:dynamic[k]<=bound for k,bound in [('jerk_m_s3',8.),('snap_m_s4',15.),('pitch_rate_deg_s',20.),('pitch_acceleration_deg_s2',90.)]}
    if enforce_dynamic_limits and not all(dynamic_checks.values()):
        raise ValueError('Reject before flight: braking shape exceeds existing dynamic design bounds: '+str(dynamic))
    return curve,terminal,{'checks':checks,'measured_deck':deck,'measured_acceleration_m_s2':acceleration,
        'predicted_stop':terminal,'duration_s':duration,'horizontal_braking_duration_s':x.stop_s,
        'peak_reference_deceleration_m_s2':x.peak_deceleration,'horizontal_pieces':x.coefficients,
        'vertical_coefficients':curve.axes[2].coefficients,'no_hold_no_recenter':True,
        'dynamic_extrema':dynamic,'dynamic_checks':dynamic_checks,
        'eligible_for_physics_trial':all(dynamic_checks.values()),'validation_bypass_for_offline_inspection_only':not enforce_dynamic_limits}


class OneWayCaptureFeedback(CaptureAwareFeedback):
    """Do not request a backward recovery when braking consumes the margin.

    A velocity-dependent lower acceleration bound releases braking near zero
    speed. Actual no-reversal and landing precision remain independently
    checked; this bound does not edit measured state or guarantee feasibility.
    """
    def update(self,decision,observation,now):
        result=super().update(decision,observation,now)
        if result.reference is None or decision.stop_reference:return result
        vx=observation.motion.velocity_at_origin_world_m_s[0]
        a=result.reference.acceleration_world_m_s2
        lower=-max(0.,vx)/.25
        bounded=max(a[0],lower)
        self.diagnostics.update(one_way_acceleration_lower_bound_m_s2=lower,
                                one_way_bound_active=bounded>a[0])
        return replace(result,reference=replace(result.reference,acceleration_world_m_s2=(bounded,*a[1:])))
