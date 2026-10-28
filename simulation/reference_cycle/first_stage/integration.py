"""Small ROS-independent integration helpers, verified without ROS runtime."""
import math
from dataclasses import dataclass


def rotate_quaternion(vector, quaternion):
    """Rotate a body-fixed vector into local ENU; quaternion is x,y,z,w."""
    if len(vector) != 3 or len(quaternion) != 4 or not all(
            math.isfinite(v) for v in (*vector,*quaternion)):
        raise ValueError('invalid vector or quaternion')
    magnitude = math.sqrt(sum(v*v for v in quaternion))
    if magnitude < 1e-8:
        raise ValueError('zero quaternion')
    x,y,z,w = (v/magnitude for v in quaternion)
    vx,vy,vz = vector
    # q v q^-1 = v + 2w(q_xyz cross v) + 2(q_xyz cross(q_xyz cross v))
    cx,cy,cz = y*vz-z*vy,z*vx-x*vz,x*vy-y*vx
    return (vx+2*w*cx+2*(y*cz-z*cy),
            vy+2*w*cy+2*(z*cx-x*cz),
            vz+2*w*cz+2*(x*cy-y*cx))


@dataclass
class FrameTransform:
    offset: tuple = (0.,0.,0.)
    yaw: float = 0.
    reference_offset_body: tuple = (0.,0.,0.)

    def __post_init__(self):
        if len(self.offset) != 3 or len(self.reference_offset_body) != 3 or not all(
                math.isfinite(v) for v in (*self.offset,self.yaw,*self.reference_offset_body)):
            raise ValueError('invalid frame transform')

    def position_to_world(self, position, quaternion=(0.,0.,0.,1.)):
        if len(position) != 3 or not all(math.isfinite(v) for v in position):
            raise ValueError('invalid position')
        relative = rotate_quaternion(self.reference_offset_body,quaternion)
        x,y,z = (a+b for a,b in zip(position,relative))
        c,s = math.cos(self.yaw),math.sin(self.yaw)
        return (self.offset[0]+c*x-s*y,self.offset[1]+s*x+c*y,self.offset[2]+z)

    def velocity_to_local(self, velocity):
        x,y,z = velocity
        c,s = math.cos(self.yaw),math.sin(self.yaw)
        return (c*x+s*y,-s*x+c*y,z)


class FreshSignal:
    """Boolean value expires by wall time even if the ROS clock is paused."""
    def __init__(self, timeout):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('invalid signal timeout')
        self.timeout = timeout
        self.received = None
        self.value = False

    def update(self, value, wall_time):
        self.value,self.received = bool(value),wall_time

    def active(self, wall_time):
        return (self.value and self.received is not None and
                0 <= wall_time-self.received <= self.timeout)

    def reset(self):
        self.received,self.value = None,False
