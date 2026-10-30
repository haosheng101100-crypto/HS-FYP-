"""Rigid landing-leg kinematics above a finite, possibly moving planar deck.

SI units. All poses map local coordinates to world coordinates. Aircraft body
axes are forward/left/up. Twists are in world axes and velocities refer to the
stated pose origin, never implicitly to the centre of mass. Cylindrical legs
are aligned with body z; meshes, deck edges/side collisions, compliance and
post-contact forces are outside this module's scope.
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path

Vector = tuple[float, float, float]


def vector(value, name='vector'):
    result = tuple(float(x) for x in value)
    if len(result) != 3 or not all(math.isfinite(x) for x in result):
        raise ValueError(f'{name} must contain three finite numbers')
    return result


def add(a, b): return tuple(x+y for x, y in zip(a, b))
def sub(a, b): return tuple(x-y for x, y in zip(a, b))
def scale(a, factor): return tuple(x*factor for x in a)
def dot(a, b): return sum(x*y for x, y in zip(a, b))
def norm(a): return math.sqrt(dot(a, a))
def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def quaternion_from_rpy(roll, pitch, yaw):
    """Body-to-world Rz(yaw) Ry(pitch) Rx(roll); nose-up planar theta=-pitch."""
    if not all(math.isfinite(x) for x in (roll, pitch, yaw)):
        raise ValueError('RPY angles must be finite radians')
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


@dataclass(frozen=True)
class Pose:
    position_world_m: Vector
    orientation_xyzw: tuple[float, float, float, float]

    def __post_init__(self):
        object.__setattr__(self, 'position_world_m', vector(self.position_world_m, 'position'))
        q = tuple(float(x) for x in self.orientation_xyzw)
        if len(q) != 4 or not all(math.isfinite(x) for x in q):
            raise ValueError('Quaternion requires four finite xyzw components')
        length = math.sqrt(sum(x*x for x in q))
        if abs(length-1) > 1e-6:
            raise ValueError('Quaternion must be unit length; check frame/source')
        object.__setattr__(self, 'orientation_xyzw', tuple(x/length for x in q))

    def rotate(self, local):
        x, y, z, w = self.orientation_xyzw
        # Unit-quaternion rotation: v + 2w(qv x v) + 2(qv x(qv x v)).
        uv = cross((x, y, z), local)
        return add(local, add(scale(uv, 2*w), scale(cross((x, y, z), uv), 2)))

    def inverse_rotate(self, world_vector):
        x, y, z, w = self.orientation_xyzw
        return Pose((0, 0, 0), (-x, -y, -z, w)).rotate(world_vector)

    def transform(self, local):
        return add(self.position_world_m, self.rotate(local))


@dataclass(frozen=True)
class RigidMotion:
    pose: Pose
    velocity_at_origin_world_m_s: Vector
    angular_velocity_world_rad_s: Vector

    def __post_init__(self):
        object.__setattr__(self, 'velocity_at_origin_world_m_s', vector(self.velocity_at_origin_world_m_s))
        object.__setattr__(self, 'angular_velocity_world_rad_s', vector(self.angular_velocity_world_rad_s))

    def velocity_at_world_point(self, point):
        return add(self.velocity_at_origin_world_m_s,
                   cross(self.angular_velocity_world_rad_s, sub(point, self.pose.position_world_m)))

    @classmethod
    def stationary(cls, pose):
        return cls(pose, (0, 0, 0), (0, 0, 0))

    @classmethod
    def from_com(cls, com_pose, com_velocity_world_m_s, angular_velocity_world_rad_s, com_offset_body_m):
        """Convert COM state to a body-origin state using an explicit offset."""
        r = com_pose.rotate(vector(com_offset_body_m))
        omega = vector(angular_velocity_world_rad_s)
        body_pose = Pose(sub(com_pose.position_world_m, r), com_pose.orientation_xyzw)
        return cls(body_pose, sub(vector(com_velocity_world_m_s), cross(omega, r)), omega)


@dataclass(frozen=True)
class CylinderLeg:
    name: str
    center_body_m: Vector
    radius_m: float
    length_m: float

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name:
            raise ValueError('Leg name must be nonempty')
        object.__setattr__(self, 'center_body_m', vector(self.center_body_m))
        if any(not math.isfinite(x) or x <= 0 for x in (self.radius_m, self.length_m)):
            raise ValueError('Cylinder radius and length must be positive finite values')


@dataclass(frozen=True)
class AirframeGeometry:
    profile_id: str
    simulation_only: bool
    legs: tuple[CylinderLeg, ...]

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        frame = data.get('body_frame')
        if not frame or [frame.get(k) for k in ['x', 'y', 'z']] != ['forward', 'left', 'up']:
            raise ValueError('An explicit forward/left/up body frame is required')
        rows = data.get('landing_legs')
        if not isinstance(rows, list) or not rows:
            raise ValueError('Landing geometry is unspecified; do not fill from another airframe')
        legs = tuple(CylinderLeg(row['name'], row['cylinder_center_body_m'],
                                 float(row['radius_m']), float(row['length_m'])) for row in rows)
        if len({leg.name for leg in legs}) != len(legs):
            raise ValueError('Leg names must be unique')
        return cls(data['profile_id'], data['simulation_only'], legs)


@dataclass(frozen=True)
class Deck:
    """Pose origin is the deck top centre; local +z is its outward normal."""
    motion: RigidMotion
    half_extents_m: tuple[float, float]

    def __post_init__(self):
        values = tuple(float(x) for x in self.half_extents_m)
        if len(values) != 2 or any(not math.isfinite(x) or x <= 0 for x in values):
            raise ValueError('Deck half extents must be two positive finite numbers')
        object.__setattr__(self, 'half_extents_m', values)


@dataclass(frozen=True)
class FootKinematics:
    name: str
    point_body_m: Vector
    point_world_m: Vector
    projection_on_deck_world_m: Vector
    clearance_m: float
    velocity_world_m_s: Vector
    relative_velocity_deck_m_s: Vector
    closing_speed_m_s: float  # positive toward the deck, negative away
    tangential_speed_m_s: float
    support_set_kind: str
    support_set_radius_m: float
    closing_speed_upper_bound_m_s: float
    tangential_speed_upper_bound_m_s: float
    projected_point_inside_deck: bool
    whole_cylinder_footprint_margins_m: tuple[float, float]


def evaluate_leg(leg, aircraft, deck):
    """Return support geometry and material-point velocity, before contact.

    The geometric minimizer can change as attitude changes. Its derivative
    is not the material-point velocity: use v_origin + omega x Rr instead.
    When a whole cap/side is equally low, report a representative point and
    conservative velocity upper bounds over that minimum support set.
    """
    body_pose, deck_pose = aircraft.pose, deck.motion.pose
    n = deck_pose.rotate((0., 0., 1.))
    n_body = body_pose.inverse_rotate(n)
    radial = math.hypot(n_body[0], n_body[1])
    axial = n_body[2]
    local = list(leg.center_body_m)
    if abs(axial) > 1e-12:
        local[2] -= math.copysign(leg.length_m/2, axial)
    if radial > 1e-12:
        local[0] -= leg.radius_m*n_body[0]/radial
        local[1] -= leg.radius_m*n_body[1]/radial
    if radial <= 1e-12:
        kind, support_radius = 'cap_disc', leg.radius_m
    elif abs(axial) <= 1e-12:
        kind, support_radius = 'side_segment', leg.length_m/2
    else:
        kind, support_radius = 'unique_rim_point', 0.
    point_body = tuple(local)
    point_world = body_pose.transform(point_body)
    clearance = dot(sub(point_world, deck_pose.position_world_m), n)
    projection = sub(point_world, scale(n, clearance))
    deck_local = deck_pose.inverse_rotate(sub(projection, deck_pose.position_world_m))
    point_velocity = aircraft.velocity_at_world_point(point_world)
    surface_velocity = deck.motion.velocity_at_world_point(projection)
    relative_world = sub(point_velocity, surface_velocity)
    relative_deck = deck_pose.inverse_rotate(relative_world)
    closing = -relative_deck[2]
    tangent = math.hypot(relative_deck[0], relative_deck[1])
    omega_relative = sub(aircraft.angular_velocity_world_rad_s, deck.motion.angular_velocity_world_rad_s)
    closing_bound = closing + norm(cross(n, omega_relative))*support_radius
    tangent_bound = tangent + norm(omega_relative)*support_radius

    # Exact projected cylindrical support extents along the rectangle axes.
    center_world = body_pose.transform(leg.center_body_m)
    center_deck = deck_pose.inverse_rotate(sub(center_world, deck_pose.position_world_m))
    axis_deck = deck_pose.inverse_rotate(body_pose.rotate((0., 0., 1.)))
    extents = tuple(leg.length_m/2*abs(axis_deck[k]) +
                    leg.radius_m*math.sqrt(max(0., 1-axis_deck[k]**2)) for k in (0, 1))
    margins = tuple(deck.half_extents_m[k]-abs(center_deck[k])-extents[k] for k in (0, 1))
    inside = all(abs(deck_local[k]) <= deck.half_extents_m[k]+1e-12 for k in (0, 1))
    return FootKinematics(leg.name, point_body, point_world, projection, clearance,
                          point_velocity, relative_deck, closing, tangent, kind,
                          support_radius, closing_bound, tangent_bound, inside, margins)


def evaluate_airframe(geometry, aircraft, deck):
    """Sorted by plane clearance. A zero gap outside the deck is not a hit."""
    return tuple(sorted((evaluate_leg(leg, aircraft, deck) for leg in geometry.legs),
                        key=lambda state: (state.clearance_m, state.name)))
