"""Separate failed-flight recovery: gain clearance, brake, then normal LAND.

Recovery is never a successful deck landing. No resetting physical state and
no forced disarm. Native attitude/altitude/motor control remains active.
"""
import math
from .contact_geometry import quaternion_from_rpy

def predictive_tracking_guard(reference,motion,horizon_s=.35,position_limit_m=.35):
    """First-order reference-error preview, not a full reachable-set proof.

    The reserve horizon is a declared guard design choice, validated in the
    recovery regression. It is not an identified actuator time constant.
    """
    ep=tuple(p-q for p,q in zip(motion.pose.position_world_m,reference.position_world_m))
    ev=tuple(v-w for v,w in zip(motion.velocity_at_origin_world_m_s,reference.velocity_world_m_s))
    error=math.sqrt(sum(x*x for x in ep))
    growth=0. if error<1e-9 else sum(p*v for p,v in zip(ep,ev))/error
    predicted=error+max(0.,growth)*horizon_s
    return {'horizon_s':horizon_s,'current_error_m':error,'radial_error_growth_m_s':growth,
            'predicted_error_m':predicted,'position_limit_m':position_limit_m,
            'reject_now':predicted>=position_limit_m}

class ForwardClearRecovery:
    def __init__(self,initial_motion,up_speed=2.5,down_speed=1.5):
        z=initial_motion.pose.rotate((0,0,1));self.pitch=-math.atan2(z[0],z[2]);self.roll=0.
        self.up=up_speed;self.down=down_speed;self.last_t=None;self.ready_since=None
    def step(self,now,motion,deck_x):
        dt=.04 if self.last_t is None else now-self.last_t
        if not 0<dt<=.08:raise ValueError('Recovery update interval exceeded')
        p=motion.pose.position_world_m;v=motion.velocity_at_origin_world_m_s
        forward=p[0]-deck_x
        move_clear=forward<2.5 and v[0]<.25
        ax=max(-1.5,min(1.5,.8*((.6 if move_clear else 0.)-v[0])))
        ay=max(-.5,min(.5,-.8*v[1]))
        target_pitch=-math.atan2(ax,9.80665);target_roll=-math.atan2(ay,9.80665)
        slew=math.radians(70)*dt;dp=target_pitch-self.pitch;dr=target_roll-self.roll
        factor=min(1.,slew/max(math.hypot(dp,dr),1e-12));self.pitch+=factor*dp;self.roll+=factor*dr
        climb=max(-.3,min(.8,1.2*(3.2-p[2])))
        q=quaternion_from_rpy(self.roll,self.pitch,math.pi/2)
        ready=forward>=2.5 and abs(p[2]-3.2)<.12 and math.hypot(*v[:2])<.08 and abs(v[2])<.08 and abs(self.pitch)<math.radians(3)
        self.ready_since=now if ready and self.ready_since is None else self.ready_since if ready else None
        self.last_t=now
        command={'quaternion_ned_wxyz':(q[3],q[0],q[1],q[2]),'type_mask':7,
            'normalized_climb_field':.5+climb/(2*(self.up if climb>=0 else self.down)),
            'climb_rate_command_m_s':climb,'roll_command_rad':self.roll,'pitch_command_rad':self.pitch,
            'horizontal_acceleration_command_m_s2':[ax,ay]}
        return command,{'phase':'MOVE_CLEAR_FORWARD' if move_clear else 'BRAKE_AND_GAIN_CLEARANCE',
            'forward_clearance_from_deck_centre_m':forward,'ready_for_native_land':self.ready_since is not None and now-self.ready_since>=.4}
