"""Solve a rigid, pre-impact touchdown boundary from physical foot targets.

The target position and velocity refer to the centroid of the active legs'
minimum-support representative points, expressed in the deck frame. All
minimum-support sets are then checked against explicit design constraints.
This is an algebraic kinematic solver, not a reachability/force/impact solver.
"""
from dataclasses import asdict, dataclass
import math
from .contact_geometry import (
    Pose, RigidMotion, evaluate_airframe, vector, quaternion_from_rpy,
    add, sub, scale, dot, norm, cross,
)


def quaternion_product(a, b):
    ax,ay,az,aw=a;bx,by,bz,bw=b
    return (aw*bx+ax*bw+ay*bz-az*by,aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,aw*bw-ax*bx-ay*by-az*bz)


def mean_vector(values):
    rows=tuple(values)
    if not rows:raise ValueError('At least one point is required')
    return tuple(sum(row[k] for row in rows)/len(rows) for k in range(3))


@dataclass(frozen=True)
class ContactDesignLimits:
    """User/experiment design bounds; these do not establish aircraft capability."""
    max_relative_tilt_rad: float
    max_relative_angular_speed_rad_s: float
    max_tangential_speed_m_s: float
    min_closing_speed_m_s: float
    max_closing_speed_m_s: float
    footprint_margin_m: float

    def __post_init__(self):
        if any(not math.isfinite(x) or x<0 for x in asdict(self).values()):
            raise ValueError('All design bounds must be finite and nonnegative')
        if self.max_closing_speed_m_s<=0 or self.max_closing_speed_m_s<self.min_closing_speed_m_s:
            raise ValueError('Closing-speed interval must have a positive upper bound')


@dataclass(frozen=True)
class TouchdownRequest:
    orientation_relative_deck_xyzw: tuple[float,float,float,float]
    angular_velocity_relative_deck_rad_s: tuple[float,float,float]
    support_centroid_deck_xy_m: tuple[float,float]
    support_centroid_relative_velocity_deck_m_s: tuple[float,float,float]

    def __post_init__(self):
        # Validate/normalize only roundoff-sized quaternion errors.
        rotation=Pose((0,0,0),self.orientation_relative_deck_xyzw)
        object.__setattr__(self,'orientation_relative_deck_xyzw',rotation.orientation_xyzw)
        object.__setattr__(self,'angular_velocity_relative_deck_rad_s',vector(self.angular_velocity_relative_deck_rad_s))
        object.__setattr__(self,'support_centroid_relative_velocity_deck_m_s',vector(self.support_centroid_relative_velocity_deck_m_s))
        xy=tuple(float(x) for x in self.support_centroid_deck_xy_m)
        if len(xy)!=2 or not all(math.isfinite(x) for x in xy):
            raise ValueError('Contact centroid needs two finite deck coordinates')
        object.__setattr__(self,'support_centroid_deck_xy_m',xy)

    @classmethod
    def planar(cls, nose_up_rad, nose_up_rate_rad_s, support_centroid_deck_xy_m,
               support_centroid_relative_velocity_deck_m_s):
        """Forward is deck +x. Positive nose-up rate is deck -y angular velocity."""
        if not math.isfinite(nose_up_rate_rad_s):raise ValueError('Pitch rate must be finite')
        return cls(quaternion_from_rpy(0,-nose_up_rad,0),(0,-nose_up_rate_rad_s,0),
                   support_centroid_deck_xy_m,support_centroid_relative_velocity_deck_m_s)


@dataclass(frozen=True)
class TouchdownBoundary:
    aircraft: RigidMotion
    active_leg_names: tuple[str,...]
    contact_centroid_world_m: tuple[float,float,float]
    contact_centroid_deck_m: tuple[float,float,float]
    contact_centroid_relative_velocity_deck_m_s: tuple[float,float,float]
    feet: tuple
    relative_tilt_rad: float
    residuals: dict
    minimum_footprint_margin_m: float
    maximum_active_tangential_speed_upper_bound_m_s: float
    active_closing_speed_bounds_m_s: tuple[float,float]

    def report(self):
        return dict(asdict(self),scope='Pre-impact rigid kinematic boundary only',
                    dynamic_reachability_checked=False,impact_stability_checked=False)


class InfeasibleTouchdown(ValueError):
    def __init__(self,reasons):
        self.reasons=tuple(reasons)
        super().__init__('; '.join(self.reasons))


def solve_touchdown_boundary(geometry, deck, request, limits):
    """Find body-origin pose and velocity for the specified first-contact state.

    1. Translate along the deck normal until the lowest physical leg touches.
    2. Translate within the deck so active support-point centroid meets target.
    3. Invert v_foot=v_origin+omega x Rr for the centroid velocity.
    4. Validate every active support set and the whole landing-leg footprint.

    No claim is made that an aircraft can reach this boundary under its thrust,
    torque or controller limits, or remain stable after touching rear legs.
    """
    if not geometry.legs:raise ValueError('Airframe has no landing legs')
    position_tol=1e-8
    speed_tol=1e-9
    deck_pose=deck.motion.pose
    relative_rotation=Pose((0,0,0),request.orientation_relative_deck_xyzw)
    tilt=math.acos(max(-1.,min(1.,dot(relative_rotation.rotate((0,0,1)),(0,0,1)))))
    reasons=[]
    if tilt>limits.max_relative_tilt_rad+1e-12:reasons.append('relative tilt exceeds design limit')
    relative_omega=request.angular_velocity_relative_deck_rad_s
    if norm(relative_omega)>limits.max_relative_angular_speed_rad_s+1e-12:
        reasons.append('relative angular speed exceeds design limit')
    if reasons:raise InfeasibleTouchdown(reasons)

    orientation=quaternion_product(deck_pose.orientation_xyzw,request.orientation_relative_deck_xyzw)
    omega=add(deck.motion.angular_velocity_world_rad_s,deck_pose.rotate(relative_omega))
    normal=deck_pose.rotate((0,0,1))
    initial=RigidMotion(Pose(deck_pose.position_world_m,orientation),(0,0,0),omega)
    initial_feet=evaluate_airframe(geometry,initial,deck)
    lowest_gap=initial_feet[0].clearance_m
    active_names=tuple(sorted(f.name for f in initial_feet if abs(f.clearance_m-lowest_gap)<=position_tol))
    height_shift=scale(normal,-lowest_gap)
    level_origin=add(deck_pose.position_world_m,height_shift)
    level_centroid=add(mean_vector(f.point_world_m for f in initial_feet if f.name in active_names),height_shift)
    level_centroid_deck=deck_pose.inverse_rotate(sub(level_centroid,deck_pose.position_world_m))
    in_plane_shift=deck_pose.rotate((request.support_centroid_deck_xy_m[0]-level_centroid_deck[0],
                                    request.support_centroid_deck_xy_m[1]-level_centroid_deck[1],0.))
    origin=add(level_origin,in_plane_shift)
    centroid=add(level_centroid,in_plane_shift)
    centroid_lever=sub(centroid,origin)
    desired_centroid_velocity=add(deck.motion.velocity_at_world_point(centroid),
                                  deck_pose.rotate(request.support_centroid_relative_velocity_deck_m_s))
    origin_velocity=sub(desired_centroid_velocity,cross(omega,centroid_lever))
    aircraft=RigidMotion(Pose(origin,orientation),origin_velocity,omega)
    feet=evaluate_airframe(geometry,aircraft,deck)
    active_feet=tuple(f for f in feet if f.name in active_names)
    actual_centroid=mean_vector(f.point_world_m for f in active_feet)
    actual_centroid_deck=deck_pose.inverse_rotate(sub(actual_centroid,deck_pose.position_world_m))
    actual_relative_velocity=mean_vector(f.relative_velocity_deck_m_s for f in active_feet)
    requested_position=(*request.support_centroid_deck_xy_m,0.)
    residuals={
        'contact_centroid_position_error_m':norm(sub(actual_centroid_deck,requested_position)),
        'contact_centroid_relative_velocity_error_m_s':norm(sub(actual_relative_velocity,request.support_centroid_relative_velocity_deck_m_s)),
        'maximum_active_absolute_plane_gap_m':max(abs(f.clearance_m) for f in active_feet),
        'minimum_all_leg_plane_gap_m':min(f.clearance_m for f in feet),
    }
    margin=min(min(f.whole_cylinder_footprint_margins_m) for f in feet)
    max_tangent=max(f.tangential_speed_upper_bound_m_s for f in active_feet)
    closing_lower=[];closing_upper=[]
    relative_omega_world=deck_pose.rotate(relative_omega)
    for foot in active_feet:
        # Same conservative support-set bound as the geometry module, both signs.
        spread=norm(cross(normal,relative_omega_world))*foot.support_set_radius_m
        closing_lower.append(foot.closing_speed_m_s-spread)
        closing_upper.append(foot.closing_speed_upper_bound_m_s)
    lower,upper=min(closing_lower),max(closing_upper)
    if residuals['contact_centroid_position_error_m']>position_tol:reasons.append('contact position residual exceeds numerical tolerance')
    if residuals['contact_centroid_relative_velocity_error_m_s']>speed_tol:reasons.append('contact velocity residual exceeds numerical tolerance')
    if residuals['maximum_active_absolute_plane_gap_m']>position_tol:reasons.append('an active leg misses the contact plane')
    if residuals['minimum_all_leg_plane_gap_m'] < -position_tol:reasons.append('another landing leg penetrates before the selected contact')
    if margin < limits.footprint_margin_m-position_tol:reasons.append('landing-leg footprint does not fit inside deck design margin')
    if any(not f.projected_point_inside_deck for f in active_feet):reasons.append('active contact lies outside deck')
    if max_tangent>limits.max_tangential_speed_m_s+speed_tol:reasons.append('an active support set exceeds tangential-speed design limit')
    if lower<limits.min_closing_speed_m_s-speed_tol or upper>limits.max_closing_speed_m_s+speed_tol:
        reasons.append('an active support set exceeds closing-speed design interval')
    if reasons:raise InfeasibleTouchdown(reasons)
    return TouchdownBoundary(aircraft,active_names,actual_centroid,actual_centroid_deck,
                             actual_relative_velocity,feet,tilt,residuals,margin,max_tangent,(lower,upper))
