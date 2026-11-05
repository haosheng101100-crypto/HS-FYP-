"""Full nominal descent towards a currently uniform platform, before any event."""
import math
from dataclasses import dataclass
from .contact_geometry import Pose,RigidMotion,Deck,evaluate_airframe
from .touchdown_boundary import TouchdownRequest,ContactDesignLimits,solve_touchdown_boundary
from .septic_approach import PlanarSnapMatchedApproach
from .execution import BodyReference
from .septic_approach import NinthScalar

def nominal_descent_executor(curve,geometry,terminal_deck,entry_limits,handover_limits,deck_provider):
    from .moving_platform import MovingLandingExecutor,MovingContactHandover
    static=Deck(RigidMotion.stationary(terminal_deck.motion.pose),terminal_deck.half_extents_m)
    executor=MovingLandingExecutor(curve,geometry,static,entry_limits,handover_limits,None)
    executor.handover=MovingContactHandover(geometry,curve.terminal_boundary,terminal_deck,handover_limits,deck_provider)
    return executor

@dataclass(frozen=True)
class UniformPreparationSpec:
    cruise_speed_m_s:float=5.
    acceleration_duration_s:float=5.
    minimum_cruise_s:float=1.5
    hover_body_height_m:float=4.

class CruisePreparation:
    def __init__(self):
        self.spec=UniformPreparationSpec();s=self.spec
        self.x=NinthScalar((0,0,0,0,0),(12.5,5,0,0,0),5.)
    def reference(self,t,descent_elapsed=None):
        if descent_elapsed is not None:raise ValueError('Normal descent has its own complete plan')
        if t<=5.:
            x,v,a=(self.x.at(max(0.,t),k) for k in range(3))
        else:x=12.5+5*(t-5);v=5.;a=0.
        return BodyReference((x,0,4),(v,0,0),(a,0,0),0.)

def nominal_uniform_descent(geometry,profile,observation,deck,duration=8.):
    speed=deck.motion.velocity_at_origin_world_m_s[0]
    if abs(speed-5.)>.03:raise ValueError('Measured uniform 5 m/s entry required')
    offset=profile['nominal_com_in_body_m'];com=observation.motion.pose.transform(offset)
    v=observation.motion.velocity_at_world_point(com)
    if abs(v[0]-speed)>.08 or abs(v[2])>.06:raise ValueError('Matched horizontal flight before normal descent required')
    p=deck.motion.pose.position_world_m
    end=Deck(RigidMotion(Pose((p[0]+speed*duration,p[1],p[2]),(0,0,0,1)),(speed,0,0),(0,0,0)),deck.half_extents_m)
    y=com[1]-offset[1]-p[1]
    boundary=solve_touchdown_boundary(geometry,end,TouchdownRequest.planar(0.,0.,(0.,y),(0.,0.,-.45)),
        ContactDesignLimits(math.radians(5),math.radians(5),.05,.25,.65,.02))
    curve=PlanarSnapMatchedApproach(com,(v[0],0,v[2]),boundary,offset,9.80665,profile['mass_kg'],duration,
        terminal_com_snap_world_m_s4=(0,0,0))
    samples=[curve.at(duration*i/400) for i in range(401)]
    checks={'uniform_horizontal_speed':max(abs(s['com_velocity_world_m_s'][0]-5.) for s in samples)<.08,
        'never_climbs':max(s['com_velocity_world_m_s'][2] for s in samples)<=.06,
        'vertical_speed':max(abs(s['com_velocity_world_m_s'][2]) for s in samples)<1.2,
        'terminal_relative_velocity':all(abs(a-b)<1e-8 for a,b in zip(boundary.aircraft.velocity_at_origin_world_m_s,(speed,0,-.45))),
        'finite_terminal_footprint':all(min(f.whole_cylinder_footprint_margins_m)>=.02 for f in evaluate_airframe(geometry,boundary.aircraft,end))}
    if not all(checks.values()):raise ValueError('Nominal uniform descent reference rejected: '+str(checks))
    return curve,end,{'duration_s':duration,'checks':checks,'axes_coefficients':[a.coefficients for a in curve.axes],
        'initial_measured_observation':observation,'initial_measured_deck':deck,'terminal_boundary':boundary,
        'contains_future_braking_input':False,'scope':'Full nominal uniform-car landing reference, subsequently interruptible by measured braking'}

def replanning_join(old_reference,new_reference,observation):
    def distance(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
    return {'old_to_new_position_jump_m':distance(old_reference.position_world_m,new_reference.position_world_m),
        'old_to_new_velocity_jump_m_s':distance(old_reference.velocity_world_m_s,new_reference.velocity_world_m_s),
        'old_to_new_acceleration_jump_m_s2':distance(old_reference.acceleration_world_m_s2,new_reference.acceleration_world_m_s2),
        'new_to_measured_position_error_m':distance(new_reference.position_world_m,observation.motion.pose.position_world_m),
        'new_to_measured_velocity_error_m_s':distance(new_reference.velocity_world_m_s,observation.motion.velocity_at_origin_world_m_s),
        'new_still_descends':new_reference.velocity_world_m_s[2]<-.15,
        'claim_of_exact_reference_C2_continuity':False}
