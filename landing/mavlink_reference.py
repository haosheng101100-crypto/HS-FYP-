"""Body-origin world ENU references -> local NED MAVLink command arguments.

No pymavlink import or connection creation. dispatch() requires an explicitly
provided transport and LAND callback, and never arms/disarms or writes poses.
"""
from dataclasses import dataclass
import math
from .contact_geometry import vector, sub, add

MAV_FRAME_LOCAL_NED=1
PVA_YAW_MASK=2048
VELOCITY_YAW_MASK=2503


def enu_to_ned(value):
    x,y,z=vector(value)
    return (y,x,-z)


def yaw_to_ned(value):
    if not math.isfinite(value):raise ValueError('Finite yaw required')
    return math.atan2(math.sin(math.pi/2-value),math.cos(math.pi/2-value))


@dataclass(frozen=True)
class LocalNedAlignment:
    offset_ned_m:tuple

    def __post_init__(self):object.__setattr__(self,'offset_ned_m',vector(self.offset_ned_m))

    @classmethod
    def from_body_measurement(cls,body_position_world_m,fcu_position_ned_m):
        return cls(sub(vector(fcu_position_ned_m),enu_to_ned(body_position_world_m)))

    def position(self,body_position_world_m):return add(enu_to_ned(body_position_world_m),self.offset_ned_m)


def dispatch(decision,transport,request_land,alignment,sim_t_s,yaw_enu_rad=0.,system_id=1,component_id=1):
    if not math.isfinite(sim_t_s) or sim_t_s<0:raise ValueError('Nonnegative simulation time required')
    if decision.stop_reference and decision.reference is not None:raise ValueError('Handover must clear the PVA reference')
    if decision.reference is not None and decision.pending_land_velocity_world_m_s is not None:raise ValueError('Ambiguous command')
    if decision.request_land:request_land()
    if decision.reference is not None:
        ref=decision.reference
        p=alignment.position(ref.position_world_m);v=enu_to_ned(ref.velocity_world_m_s);a=enu_to_ned(ref.acceleration_world_m_s2)
        transport.mav.set_position_target_local_ned_send(int(sim_t_s*1000)&0xffffffff,system_id,component_id,
            MAV_FRAME_LOCAL_NED,PVA_YAW_MASK,*p,*v,*a,yaw_to_ned(ref.yaw_enu_rad),0.)
    elif decision.pending_land_velocity_world_m_s is not None:
        transport.mav.set_position_target_local_ned_send(int(sim_t_s*1000)&0xffffffff,system_id,component_id,
            MAV_FRAME_LOCAL_NED,VELOCITY_YAW_MASK,0.,0.,0.,*enu_to_ned(decision.pending_land_velocity_world_m_s),
            0.,0.,0.,yaw_to_ned(yaw_enu_rad),0.)
