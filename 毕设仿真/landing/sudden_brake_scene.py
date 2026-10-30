"""Fixed physical scene preparation; no emergency landing decision policy."""
from dataclasses import dataclass
from .septic_approach import NinthScalar
from .execution import BodyReference


@dataclass(frozen=True)
class SuddenBrakeSceneSpec:
    cruise_speed_m_s: float = 5.
    vehicle_acceleration_limit_m_s2: float = 2.5
    initial_vehicle_x_m: float = -7.5
    hover_body_height_m: float = 4.
    acceleration_duration_s: float = 5.
    minimum_cruise_s: float = 1.5
    descent_duration_s: float = 5.
    nominal_trigger_body_height_m: float = 2.495
    trigger_foot_gap_m: float = 1.5
    descent_speed_m_s: float = .4
    observe_after_brake_s: float = .4
    fixed_exit_duration_s: float = 5.


class SceneReferences:
    """Body-origin references, physically executed by ArduCopter."""
    def __init__(self, spec=None):
        self.spec = spec or SuddenBrakeSceneSpec();s=self.spec
        self.x_ramp=NinthScalar((0,0,0,0,0),(.5*s.cruise_speed_m_s*s.acceleration_duration_s,
             s.cruise_speed_m_s,0,0,0),s.acceleration_duration_s)
        self.z_descent=NinthScalar((s.hover_body_height_m,0,0,0,0),
            (s.nominal_trigger_body_height_m,-s.descent_speed_m_s,0,0,0),s.descent_duration_s)

    def horizontal(self,t,order=0):
        s=self.spec
        if t<=s.acceleration_duration_s:return self.x_ramp.at(max(0.,t),order)
        return (self.x_ramp.at(s.acceleration_duration_s)+s.cruise_speed_m_s*(t-s.acceleration_duration_s)
            if order==0 else s.cruise_speed_m_s if order==1 else 0.)

    def reference(self,t,descent_elapsed=None):
        s=self.spec
        z=(s.hover_body_height_m,0.,0.)
        if descent_elapsed is not None:
            u=max(0.,descent_elapsed)
            z=tuple(self.z_descent.at(u,k) for k in range(3)) if u<=s.descent_duration_s else (
                s.nominal_trigger_body_height_m-s.descent_speed_m_s*(u-s.descent_duration_s),-s.descent_speed_m_s,0.)
        return BodyReference((self.horizontal(t),0.,z[0]),(self.horizontal(t,1),0.,z[1]),
                             (self.horizontal(t,2),0.,z[2]),0.)

    def fixed_exit(self,initial):
        s=self.spec;T=s.fixed_exit_duration_s
        target=(initial.position_world_m[0]+.5*s.cruise_speed_m_s*T,0.,s.hover_body_height_m)
        axes=list(NinthScalar((initial.position_world_m[k],initial.velocity_world_m_s[k],
                        initial.acceleration_world_m_s2[k],0.,0.),(target[k],0,0,0,0),T) for k in range(3))
        axes[2]=StopDescentThenClimb(initial.position_world_m[2],initial.velocity_world_m_s[2],target[2],T)
        return tuple(axes)


class StopDescentThenClimb:
    """Predeclared exit, arrest descent before the slower clearance climb."""
    def __init__(self,z,v,target,duration):
        self.T=duration;self.stop_s=1.;level=z+.5*v*self.stop_s
        self.stop=NinthScalar((z,v,0,0,0),(level,0,0,0,0),self.stop_s)
        self.climb=NinthScalar((level,0,0,0,0),(target,0,0,0,0),duration-self.stop_s)
    def at(self,t,order=0):
        return self.stop.at(t,order) if t<=self.stop_s else self.climb.at(t-self.stop_s,order)


def reference_from_axes(axes,t):
    return BodyReference(*(tuple(a.at(t,k) for a in axes) for k in range(3)),0.)
