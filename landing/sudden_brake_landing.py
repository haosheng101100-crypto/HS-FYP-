"""One-case reactive braking, arrest descent, then verified stopped-deck landing.

No guarantee of landing before the vehicle stops. No scheduled fly-away exit.
"""
import math
from .contact_geometry import sub,add,norm
from .execution import BodyReference
from .septic_approach import NinthScalar,PlanarSnapMatchedApproach
from .touchdown_boundary import TouchdownRequest,ContactDesignLimits,solve_touchdown_boundary


def level_boundary(geometry,deck,contact_y=0.):
    return solve_touchdown_boundary(geometry,deck,TouchdownRequest.planar(0.,0.,(0.,contact_y),(0.,0.,-.45)),
        ContactDesignLimits(math.radians(5),math.radians(5),.05,.25,.65,.02))


class BrakeAndHold:
    def __init__(self,now,current_reference,geometry,deck):
        self.start=now;self.stop_s=.8
        p=current_reference.position_world_m;v=current_reference.velocity_world_m_s;a=current_reference.acceleration_world_m_s2
        self.z=NinthScalar((p[2],v[2],a[2],0.,0.),(p[2]+.5*v[2]*self.stop_s,0.,0.,0.,0.),self.stop_s)
        boundary=level_boundary(geometry,deck)
        self.offset=sub(boundary.aircraft.pose.position_world_m,deck.motion.pose.position_world_m)

    def reference(self,now,deck,measured_acceleration):
        u=min(self.stop_s,max(0.,now-self.start));p=add(deck.motion.pose.position_world_m,self.offset)
        vx=max(0.,deck.motion.velocity_at_origin_world_m_s[0])
        ax=max(-2.5,min(2.5,measured_acceleration[0])) if vx>.05 else 0.
        return BodyReference((p[0],p[1],self.z.at(u)),(vx,0.,self.z.at(u,1)),(ax,0.,self.z.at(u,2)),0.)

    def ready(self,now,obs,deck,measured_acceleration):
        ref=self.reference(now,deck,measured_acceleration)
        ep=sub(ref.position_world_m,obs.motion.pose.position_world_m)
        ev=sub(obs.motion.velocity_at_origin_world_m_s,deck.motion.velocity_at_origin_world_m_s)
        tilt=math.acos(max(-1.,min(1.,obs.motion.pose.rotate((0,0,1))[2])))
        return {'vehicle_stopped':abs(deck.motion.velocity_at_origin_world_m_s[0])<.02,
            'vehicle_acceleration_settled':abs(measured_acceleration[0])<.05,
            'descent_arrested':now-self.start>=self.stop_s and abs(obs.motion.velocity_at_origin_world_m_s[2])<.04,
            'horizontal_position':math.hypot(ep[0],ep[1])<.08,'relative_speed':norm(ev)<.04,
            'height_settled':abs(ep[2])<.06,'attitude_settled':tilt<math.radians(.7),
            'angular_speed':norm(obs.motion.angular_velocity_world_rad_s)<math.radians(3.)}


def stopped_deck_reference(geometry,profile,observation,deck,duration=3.):
    offset=profile['nominal_com_in_body_m'];com=observation.motion.pose.transform(offset)
    # Keep the tiny measured lateral offset rather than creating a planar-y jump.
    y=com[1]-offset[1]-deck.motion.pose.position_world_m[1]
    boundary=level_boundary(geometry,deck,y)
    v=observation.motion.velocity_at_world_point(com)
    curve=PlanarSnapMatchedApproach(com,(v[0],0.,v[2]),boundary,offset,9.80665,profile['mass_kg'],duration,
        terminal_com_snap_world_m_s4=(0.,0.,0.))
    return curve,boundary
