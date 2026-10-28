"""Level, vertical simulation lead-in to the fixed planar approach initial jet."""
from .contact_geometry import Pose,RigidMotion
from .septic_approach import NinthScalar


class VerticalLeadIn:
    def __init__(self,approach,duration_s,height_above_initial_com_m):
        if height_above_initial_com_m<=0:raise ValueError('Positive lead-in height required')
        initial=approach.at(0)
        if any(abs(initial['com_velocity_world_m_s'][k])>1e-12 for k in [0,1]):
            raise ValueError('Vertical lead-in requires zero initial horizontal velocity')
        if any(abs(v)>1e-12 for key in ['com_acceleration_world_m_s2','com_jerk_world_m_s3','com_snap_world_m_s4'] for v in initial[key]):
            raise ValueError('Version 1 requires zero approach initial acceleration/jerk/snap')
        if abs(initial['nose_up_rad'])+abs(initial['nose_up_rate_rad_s'])>1e-12:raise ValueError('Level initial approach required')
        self.approach=approach;self.T=duration_s
        self.com_offset_body_m=approach.com_offset_body_m
        self.x,self.y,z=initial['com_position_world_m']
        self.z=NinthScalar((z+height_above_initial_com_m,0.,0.,0.,0.),
            (z,initial['com_velocity_world_m_s'][2],0.,0.,0.),duration_s)

    def at(self,t):
        z,v,a,j,s=(self.z.at(t,k) for k in range(5))
        if a+self.approach.gravity<=0:raise ValueError('Invalid vertical lead-in thrust')
        com_pose=Pose((self.x,self.y,z),(0,0,0,1))
        motion=RigidMotion.from_com(com_pose,(0,0,v),(0,0,0),self.com_offset_body_m)
        return {'motion':motion,'body_acceleration_world_m_s2':(0.,0.,a),
            'com_position_world_m':com_pose.position_world_m,'com_velocity_world_m_s':(0.,0.,v),
            'com_acceleration_world_m_s2':(0.,0.,a),'com_jerk_world_m_s3':(0.,0.,j),'com_snap_world_m_s4':(0.,0.,s),
            'nose_up_rad':0.,'nose_up_rate_rad_s':0.,'nose_up_acceleration_rad_s2':0.,
            'point_mass_required_thrust_N':self.approach.mass*(self.approach.gravity+a)}
