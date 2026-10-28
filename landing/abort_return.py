"""Abort descent behind a forward-accelerating car, then return through a known flat SITL corridor."""
import math
from .contact_geometry import quaternion_from_rpy,norm
from .execution import BodyReference
from .septic_approach import NinthScalar

class ClearBehindAbort:
    def __init__(self,motion,up_speed=2.5,down_speed=1.5,target_height=3.6):
        z=motion.pose.rotate((0,0,1));self.pitch=-math.atan2(z[0],z[2]);self.roll=0.
        self.up=up_speed;self.down=down_speed;self.target_height=target_height;self.last_t=None;self.stable_since=None
    def step(self,now,motion,deck):
        dt=.04 if self.last_t is None else now-self.last_t
        if not 0<dt<=.08:raise ValueError('Abort update interval exceeded')
        p=motion.pose.position_world_m;v=motion.velocity_at_origin_world_m_s
        ax=max(-1.5,min(1.5,-v[0]));ay=max(-.5,min(.5,-1.2*v[1]))
        target_pitch=-math.atan2(ax,9.80665);target_roll=-math.atan2(ay,9.80665)
        dp=target_pitch-self.pitch;dr=target_roll-self.roll
        f=min(1.,math.radians(70)*dt/max(math.hypot(dp,dr),1e-12));self.pitch+=f*dp;self.roll+=f*dr
        climb=max(-.3,min(.8,1.2*(self.target_height-p[2])))
        q=quaternion_from_rpy(self.roll,self.pitch,math.pi/2)
        behind=deck.motion.pose.position_world_m[0]-p[0]
        ready=behind>=3. and abs(p[2]-self.target_height)<.10 and norm(v)<.08 and abs(self.pitch)<math.radians(3.)
        self.stable_since=now if ready and self.stable_since is None else self.stable_since if ready else None
        self.last_t=now
        return {'quaternion_ned_wxyz':(q[3],q[0],q[1],q[2]),'type_mask':7,
            'normalized_climb_field':.5+climb/(2*(self.up if climb>=0 else self.down)),
            'climb_rate_command_m_s':climb,'roll_command_rad':self.roll,'pitch_command_rad':self.pitch,
            'horizontal_acceleration_command_m_s2':[ax,ay]}, {'phase':'ARREST_DESCENT_BRAKE_AND_CLEAR_BEHIND',
            'measured_vehicle_ahead_m':behind,'vehicle_forward_speed_m_s':deck.motion.velocity_at_origin_world_m_s[0],
            'ready_for_return_corridor_check':self.stable_since is not None and now-self.stable_since>=.5}

class HomeReturnReference:
    def __init__(self,motion,home=(0.,0.,3.6)):
        p=motion.pose.position_world_m;v=motion.velocity_at_origin_world_m_s
        if norm(v)>.08 or p[2]<3.4:raise ValueError('Return requires measured clear stable hover')
        if len(home)!=3 or not all(math.isfinite(x) for x in home) or home[2]<3.4:raise ValueError('Known clear elevated home reference required')
        self.home=tuple(home);self.T=max(20.,2.6*math.dist(p,home)/4.)
        self.axes=tuple(NinthScalar((p[k],v[k],0.,0.,0.),(home[k],0.,0.,0.,0.),self.T) for k in range(3))
        samples=[self.at(self.T*i/400) for i in range(401)]
        self.checks={'reference_speed':max(norm(s.velocity_world_m_s) for s in samples)<=4.,
            'reference_acceleration':max(norm(s.acceleration_world_m_s2) for s in samples)<=1.5,
            'reference_above_clearance':min(s.position_world_m[2] for s in samples)>=3.4}
        if not all(self.checks.values()):raise ValueError('Return corridor reference outside declared envelope')
    def at(self,t):
        if t<0 or t>self.T+1e-9:raise ValueError('Return reference cannot be extrapolated')
        return BodyReference(*(tuple(a.at(min(t,self.T),k) for a in self.axes) for k in range(3)),0.)
