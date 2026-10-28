"""Conservative box vertices + exact swept rotor cylinder projections."""
import json,math
from pathlib import Path
import numpy as np


class SweptEnvelope:
    def __init__(self,data):
        self.data=data;self.points=np.array(data['fixed_points_body_m'],dtype=float)
        self.rotors=data['rotors']
        assert self.points.ndim==2 and self.points.shape[1]==3 and np.isfinite(self.points).all()
        self.radius=max(float(np.linalg.norm(self.points,axis=1).max()),
            *(math.sqrt(sum(v*v for v in r['origin']))+math.hypot(r['swept_radius'],r['half_height']) for r in self.rotors))
        self.horizontal_radius=max(float(np.linalg.norm(self.points[:,:2],axis=1).max()),
            *(math.hypot(*r['origin'][:2])+r['swept_radius'] for r in self.rotors))
        self.vertical_extent=max(float(abs(self.points[:,2]).max()),
            *(abs(r['origin'][2])+r['half_height'] for r in self.rotors))

    def rotation_projection_bound(self,angles):
        """XY displacement bound for a ZYX Euler-component error box.

        Bounds concern rotations with |roll/pitch/yaw| no larger than the
        supplied values, around a level, world-yaw-zero terminal body pose.
        Rx followed by Ry gives the two tilt terms below; Rz adds at most
        2*R*sin(yaw/2). This is a geometry bound, not a future attitude bound.
        """
        roll,pitch,yaw=(abs(float(a)) for a in angles)
        if not all(math.isfinite(a) for a in (roll,pitch,yaw)):
            raise ValueError('Finite attitude error bounds required')
        if max(roll,pitch,yaw)>=math.pi/2:return 2*self.radius
        tilt=self.horizontal_radius*(1-math.cos(roll)+1-math.cos(pitch)+math.sin(roll)*math.sin(pitch))
        tilt+=self.vertical_extent*(math.sin(roll)+math.sin(pitch))
        return min(2*self.radius,tilt+2*self.radius*math.sin(yaw/2))

    @classmethod
    def load(cls,path):return cls(json.loads(Path(path).read_text()))

    def support(self,axis,com_offset=(0,0,0)):
        axis=np.asarray(axis);offset=np.array(com_offset)
        values=[float(((self.points-offset)@axis).max())]
        for r in self.rotors:
            values.append(float((np.array(r['origin'])-offset)@axis)+r['swept_radius']*math.hypot(*axis[:2])+r['half_height']*abs(axis[2]))
        return max(values)

    def safe_radius(self,platform,duration,offset,reserve,center_shift=(0,0,0)):
        # Support functions are R-Lipschitz in heading. Subtract half the grid
        # spacing bound, so this heading-interval bound covers between samples.
        angles=[platform.yaw,platform.predict(duration).yaw]
        alpha=getattr(platform,'yaw_acceleration',0.)
        if alpha and 0< -platform.yaw_rate/alpha <duration:angles.append(platform.predict(-platform.yaw_rate/alpha).yaw)
        start=min(angles);end=max(angles);span=end-start;n=max(1,math.ceil(span/.01))
        lower=math.inf
        for yaw in np.linspace(start,end,n+1):
            axes=((math.cos(yaw),math.sin(yaw),0),(-math.sin(yaw),math.cos(yaw),0))
            for half,axis in zip(platform.half_extents,axes):
                shift=float(np.array(center_shift)@axis)
                lower=min(lower,half-self.support(axis,offset)-shift,half-self.support(tuple(-v for v in axis),offset)+shift)
        return lower-reserve-(self.radius+np.linalg.norm(offset)+np.linalg.norm(center_shift))*span/(2*n)

    def corridor_facets(self,platform,duration,offset,reserve,center_shift=(0,0,0)):
        """Fixed signed halfspaces sufficient for a rotating rectangular deck.

        The midpoint axes cover the entire quadratic heading interval using
        ||u(t)-u_mid|| <= chord. Both center displacement and body support
        receive this reserve; it is not just an endpoint/sample check.
        """
        angles=[platform.yaw,platform.predict(duration).yaw]
        alpha=platform.yaw_acceleration
        if alpha and 0 < -platform.yaw_rate/alpha < duration:
            angles.append(platform.predict(-platform.yaw_rate/alpha).yaw)
        lo,hi=min(angles),max(angles);mid=.5*(lo+hi)
        chord=2*math.sin(min(math.pi,.5*(hi-lo))/2)
        body_radius=self.radius+float(np.linalg.norm(offset))
        shift=np.asarray(center_shift,dtype=float)
        facets=[]
        for half,local in zip(platform.half_extents,((1,0,0),(0,1,0))):
            axis=np.array((math.cos(mid)*local[0]-math.sin(mid)*local[1],math.sin(mid)*local[0]+math.cos(mid)*local[1],0.))
            for sign in (1.,-1.):
                u=sign*axis;support=self.support(u,offset)
                facets.append(dict(axis=u.tolist(),local_axis=(sign*np.array(local)).tolist(),
                    heading_chord=chord,body_radius_m=body_radius,support_m=support,
                    allowance_m=float(half-support-u@shift-reserve-chord*(body_radius+np.linalg.norm(shift)))))
        return facets

    def clearance(self,motion,deck):
        delta=np.array(motion.pose.position_world_m)-np.array(deck.motion.pose.position_world_m)
        margins=[]
        for half,local in zip(deck.half_extents_m,((1,0,0),(0,1,0))):
            axis=np.array(deck.motion.pose.rotate(local));body_axis=motion.pose.inverse_rotate(tuple(axis))
            margins.extend((half-float(delta@axis)-self.support(body_axis),
                            half+float(delta@axis)-self.support(tuple(-v for v in body_axis))))
        return min(margins)

    def tilt_projection_coefficient(self,offset,tilt_limit,minimum_vertical_force):
        points=self.points-np.array(offset)
        xy=max(float(np.linalg.norm(points[:,:2],axis=1).max()),
            *(math.hypot(r['origin'][0]-offset[0],r['origin'][1]-offset[1])+r['swept_radius'] for r in self.rotors))
        z=max(float(abs(points[:,2]).max()),*(abs(r['origin'][2]-offset[2])+r['half_height'] for r in self.rotors))
        # For the planner's fixed world heading, with u=a_xy/f_z, its rotation
        # satisfies ||R_xy-I||_2 <= |u|² and ||R_xz,yz|| <= |u|. Thus projected
        # point displacement <= (R_xy*tan(tilt_limit)+Z)*|a_xy|/f_z_min.
        return (xy*math.tan(tilt_limit)+z)/minimum_vertical_force

    def centered_body_xy_in_deck(self,deck):
        # Center the full asymmetric rotor/body occupancy, rather than fixing
        # the leg centroid at the deck center. World body heading is still zero.
        offsets=[]
        for local in ((1,0,0),(0,1,0)):
            axis=deck.motion.pose.rotate(local)
            offsets.append(.5*(self.support(tuple(-v for v in axis))-self.support(axis)))
        return tuple(offsets)
