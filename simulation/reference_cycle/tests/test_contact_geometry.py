"""Physical identities, degeneracies, parameter isolation and frame invariance."""
import math
from pathlib import Path
import random
import unittest
from landing.contact_geometry import (
    AirframeGeometry, CylinderLeg, Deck, Pose, RigidMotion, evaluate_airframe,
    evaluate_leg, quaternion_from_rpy, add, sub, scale, norm, cross,
)

ROOT=Path(__file__).resolve().parents[1]


class ContactGeometryChecks(unittest.TestCase):
    def setUp(self):
        self.geometry=AirframeGeometry.load(ROOT/'config/airframes/iris_fixed_gimbal_sim.json')
        self.deck=Deck(RigidMotion.stationary(Pose((0,0,0),(0,0,0,1))),(1.,.7))

    def assertVector(self, actual, expected, tolerance=1e-10):
        self.assertLess(norm(sub(actual,expected)),tolerance)

    def test_level_legs_on_deck(self):
        aircraft=RigidMotion.stationary(Pose((0,0,.195),(0,0,0,1)))
        for foot in evaluate_airframe(self.geometry,aircraft,self.deck):
            self.assertAlmostEqual(foot.clearance_m,0,places=12)
            self.assertEqual(foot.support_set_kind,'cap_disc')
            self.assertVector(foot.velocity_world_m_s,(0,0,0))

    def test_nose_up_rear_first_and_cylinder_radius(self):
        theta=math.radians(5)
        height=.195*math.cos(theta)+(.14+.005)*math.sin(theta)
        aircraft=RigidMotion.stationary(Pose((0,0,height),quaternion_from_rpy(0,-theta,0)))
        feet=evaluate_airframe(self.geometry,aircraft,self.deck)
        self.assertTrue(feet[0].name.startswith('rear'))
        self.assertAlmostEqual(feet[0].clearance_m,0,places=12)
        front=next(f for f in feet if f.name.startswith('front'))
        self.assertAlmostEqual(front.clearance_m,.263*math.sin(theta),places=12)

    def test_zero_origin_speed_does_not_mean_zero_foot_speed(self):
        aircraft=RigidMotion(Pose((0,0,.195),(0,0,0,1)),(0,0,0),(0,-.5,0))
        rear=next(f for f in evaluate_airframe(self.geometry,aircraft,self.deck) if f.name=='rear_left_leg_collision')
        self.assertVector(rear.velocity_world_m_s,(.0975,0,-.07))
        self.assertAlmostEqual(rear.closing_speed_m_s,.07)

    def test_com_and_body_origin_give_identical_point_velocity(self):
        q=quaternion_from_rpy(.1,-.2,.5)
        body=RigidMotion(Pose((2,3,4),q),(.3,-.2,-.1),(.1,.2,.3))
        c=(.01,-.003,-.014)
        com_pose=Pose(body.pose.transform(c),q)
        recovered=RigidMotion.from_com(com_pose,body.velocity_at_world_point(com_pose.position_world_m),body.angular_velocity_world_rad_s,c)
        self.assertVector(recovered.pose.position_world_m,body.pose.position_world_m)
        point=body.pose.transform((-.14,.21,-.195))
        self.assertVector(recovered.velocity_at_world_point(point),body.velocity_at_world_point(point))

    def test_velocity_matches_finite_difference_of_same_material_point(self):
        theta=.17;rate=.6;eps=1e-6
        pose=Pose((0,0,1),quaternion_from_rpy(0,-theta,0))
        aircraft=RigidMotion(pose,(.4,0,-.2),(0,-rate,0))
        foot=evaluate_airframe(self.geometry,aircraft,self.deck)[0]
        before=Pose(scale(aircraft.velocity_at_origin_world_m_s,-eps),quaternion_from_rpy(0,-theta+rate*eps,0))
        after=Pose(scale(aircraft.velocity_at_origin_world_m_s,eps),quaternion_from_rpy(0,-theta-rate*eps,0))
        measured=scale(sub(after.transform(foot.point_body_m),before.transform(foot.point_body_m)),1/(2*eps))
        self.assertVector(measured,foot.velocity_world_m_s,1e-8)

    def test_common_rigid_motion_has_zero_relative_contact_speed(self):
        q=quaternion_from_rpy(.2,-.1,.4)
        deck_motion=RigidMotion(Pose((2,3,4),q),(.4,.2,.1),(.1,.2,-.3))
        aircraft_pose=Pose(deck_motion.pose.transform((0,0,.195)),q)
        aircraft=RigidMotion(aircraft_pose,deck_motion.velocity_at_world_point(aircraft_pose.position_world_m),deck_motion.angular_velocity_world_rad_s)
        deck=Deck(deck_motion,(1,.7))
        for foot in evaluate_airframe(self.geometry,aircraft,deck):
            self.assertAlmostEqual(foot.clearance_m,0,places=12)
            self.assertVector(foot.relative_velocity_deck_m_s,(0,0,0))

    def test_outside_deck_plane_zero_does_not_imply_valid_touch(self):
        aircraft=RigidMotion.stationary(Pose((3,0,.195),(0,0,0,1)))
        for foot in evaluate_airframe(self.geometry,aircraft,self.deck):
            self.assertAlmostEqual(foot.clearance_m,0)
            self.assertFalse(foot.projected_point_inside_deck)
            self.assertLess(foot.whole_cylinder_footprint_margins_m[0],0)

    def test_exact_projection_bound_catches_overhanging_leg(self):
        leg=CylinderLeg('edge',(0,0,-.11),.005,.17)
        aircraft=RigidMotion.stationary(Pose((.997,0,.195),(0,0,0,1)))
        foot=evaluate_leg(leg,aircraft,self.deck)
        self.assertTrue(foot.projected_point_inside_deck)
        self.assertAlmostEqual(foot.whole_cylinder_footprint_margins_m[0],-.002)

    def test_cap_contact_patch_bounds_cover_rim_velocities(self):
        aircraft=RigidMotion(Pose((0,0,.195),(0,0,0,1)),(.1,.2,-.1),(.3,-.4,2.))
        leg=self.geometry.legs[0]
        foot=evaluate_leg(leg,aircraft,self.deck)
        for i in range(36):
            angle=i*math.pi/18
            rim=add(foot.point_body_m,(leg.radius_m*math.cos(angle),leg.radius_m*math.sin(angle),0))
            v=aircraft.velocity_at_world_point(aircraft.pose.transform(rim))
            self.assertLessEqual(math.hypot(v[0],v[1]),foot.tangential_speed_upper_bound_m_s+1e-12)
            self.assertLessEqual(-v[2],foot.closing_speed_upper_bound_m_s+1e-12)

    def test_side_contact_degeneracy_is_explicit(self):
        aircraft=RigidMotion(Pose((0,0,1),quaternion_from_rpy(0,math.pi/2,0)),(0,0,0),(0,0,0))
        foot=evaluate_leg(self.geometry.legs[0],aircraft,self.deck)
        self.assertEqual(foot.support_set_kind,'side_segment')

    def test_global_rotation_translation_invariance(self):
        aircraft=RigidMotion(Pose((.2,-.1,.6),quaternion_from_rpy(.03,-.1,.2)),(.3,.1,-.2),(.1,.2,.3))
        original=evaluate_airframe(self.geometry,aircraft,self.deck)
        transform=Pose((4,5,6),quaternion_from_rpy(.2,-.3,.8))
        # Quaternion product, independent of the module's rotate implementation.
        def product(a,b):
            ax,ay,az,aw=a;bx,by,bz,bw=b
            return (aw*bx+ax*bw+ay*bz-az*by,aw*by-ax*bz+ay*bw+az*bx,
                    aw*bz+ax*by-ay*bx+az*bw,aw*bw-ax*bx-ay*by-az*bz)
        def move(motion):
            return RigidMotion(Pose(transform.transform(motion.pose.position_world_m),product(transform.orientation_xyzw,motion.pose.orientation_xyzw)),
                               transform.rotate(motion.velocity_at_origin_world_m_s),transform.rotate(motion.angular_velocity_world_rad_s))
        moved=evaluate_airframe(self.geometry,move(aircraft),Deck(move(self.deck.motion),(1,.7)))
        for a,b in zip(original,moved):
            self.assertEqual(a.name,b.name)
            self.assertAlmostEqual(a.clearance_m,b.clearance_m,places=12)
            self.assertVector(a.relative_velocity_deck_m_s,b.relative_velocity_deck_m_s)

    def test_support_height_matches_independent_closed_form_random_poses(self):
        rng=random.Random(20261018)
        leg=self.geometry.legs[0]
        for _ in range(100):
            pose=Pose((.1,.2,.7),quaternion_from_rpy(*(rng.uniform(-math.pi,math.pi) for _ in range(3))))
            foot=evaluate_leg(leg,RigidMotion.stationary(pose),self.deck)
            axis=pose.rotate((0,0,1))
            center=pose.transform(leg.center_body_m)
            expected=center[2]-leg.length_m/2*abs(axis[2])-leg.radius_m*math.sqrt(max(0,1-axis[2]**2))
            self.assertAlmostEqual(foot.clearance_m,expected,places=11)

    def test_missing_hardware_geometry_and_invalid_input_are_rejected(self):
        with self.assertRaises(ValueError):AirframeGeometry.load(ROOT/'config/airframes/real_airframe_pending.json')
        with self.assertRaises(ValueError):Pose((0,0,0),(0,0,0,0))
        with self.assertRaises(ValueError):Pose((math.nan,0,0),(0,0,0,1))
        with self.assertRaises(ValueError):CylinderLeg('bad',(0,0,0),-.1,.2)
        with self.assertRaises(ValueError):Deck(self.deck.motion,(0,.7))


if __name__=='__main__':unittest.main()
