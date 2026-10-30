from dataclasses import replace
import math
from pathlib import Path
import unittest
from landing.contact_geometry import AirframeGeometry,Deck,Pose,RigidMotion,quaternion_from_rpy,sub,norm,add,scale,evaluate_airframe
from landing.touchdown_boundary import ContactDesignLimits,TouchdownRequest,InfeasibleTouchdown,solve_touchdown_boundary

ROOT=Path(__file__).resolve().parents[1]


class TouchdownBoundaryChecks(unittest.TestCase):
    def setUp(self):
        self.geometry=AirframeGeometry.load(ROOT/'config/airframes/iris_fixed_gimbal_sim.json')
        self.deck=Deck(RigidMotion.stationary(Pose((5,0,.8),(0,0,0,1))),(1,.7))
        self.limits=ContactDesignLimits(math.radians(5),math.radians(5),.05,.02,.12,.02)

    def request(self,angle=2,rate=0,velocity=(0,0,-.08),xy=(0,0)):
        return TouchdownRequest.planar(math.radians(angle),math.radians(rate),xy,velocity)

    def solve(self,request=None,deck=None,limits=None):
        return solve_touchdown_boundary(self.geometry,deck or self.deck,request or self.request(),limits or self.limits)

    def assertVector(self,a,b,tolerance=1e-10):self.assertLess(norm(sub(a,b)),tolerance)

    def test_level_contact_touches_all_four_caps(self):
        result=self.solve(self.request(angle=0))
        self.assertEqual(len(result.active_leg_names),4)
        self.assertAlmostEqual(result.aircraft.pose.position_world_m[2]-.8,.195)
        self.assertVector(result.contact_centroid_deck_m,(0,0,0))
        self.assertVector(result.aircraft.velocity_at_origin_world_m_s,(0,0,-.08))

    def test_nose_up_touches_rear_pair_without_penetration(self):
        result=self.solve()
        theta=math.radians(2)
        expected_height=.195*math.cos(theta)+(.14+.005)*math.sin(theta)
        self.assertEqual(set(result.active_leg_names),{'rear_left_leg_collision','rear_right_leg_collision'})
        self.assertAlmostEqual(result.aircraft.pose.position_world_m[2]-.8,expected_height,places=12)
        for foot in result.feet:
            self.assertGreaterEqual(foot.clearance_m,-1e-12)
            if foot.name.startswith('front'):self.assertAlmostEqual(foot.clearance_m,.263*math.sin(theta),places=12)

    def test_nose_down_touches_front_pair(self):
        result=self.solve(self.request(angle=-2))
        self.assertEqual(set(result.active_leg_names),{'front_left_leg_collision','front_right_leg_collision'})

    def test_pitch_rotation_requires_origin_velocity_compensation(self):
        result=self.solve(self.request(rate=3))
        self.assertLess(result.aircraft.velocity_at_origin_world_m_s[0],-.01)
        self.assertVector(result.contact_centroid_relative_velocity_deck_m_s,(0,0,-.08))
        for foot in result.feet:
            if foot.name in result.active_leg_names:
                self.assertLess(foot.tangential_speed_m_s,1e-10)

    def test_moving_deck_velocity_is_matched_at_contact(self):
        deck=Deck(RigidMotion(Pose((5,0,.8),(0,0,0,1)),(.4,.1,0),(0,0,0)),(1,.7))
        result=self.solve(deck=deck)
        self.assertVector(result.aircraft.velocity_at_origin_world_m_s,(.4,.1,-.08))
        self.assertVector(result.contact_centroid_relative_velocity_deck_m_s,(0,0,-.08))

    def test_rotating_tilted_deck_uses_world_axes_and_relative_rates(self):
        deck=Deck(RigidMotion(Pose((2,3,4),quaternion_from_rpy(.1,-.15,.4)),(.3,-.1,.05),(.02,-.03,.04)),(1,.7))
        result=self.solve(self.request(angle=3,rate=1),deck=deck)
        self.assertVector(result.contact_centroid_deck_m,(0,0,0))
        self.assertVector(result.contact_centroid_relative_velocity_deck_m_s,(0,0,-.08))
        self.assertAlmostEqual(result.relative_tilt_rad,math.radians(3),places=12)

    def test_only_centroid_target_is_insufficient_for_rotating_cap_sets(self):
        # With a flat attitude all four caps touch. Their velocities vary with
        # pitch rate, even though the requested centroid tangential speed is zero.
        limits=replace(self.limits,min_closing_speed_m_s=.079,max_closing_speed_m_s=.081)
        with self.assertRaisesRegex(InfeasibleTouchdown,'support set'):
            self.solve(self.request(angle=0,rate=3),limits=limits)

    def test_cap_rim_speed_bound_is_enforced(self):
        request=TouchdownRequest(quaternion_from_rpy(0,0,0),(0,0,.05),(0,0),(0,0,-.08))
        with self.assertRaisesRegex(InfeasibleTouchdown,'tangential'):
            self.solve(request,limits=replace(self.limits,max_tangential_speed_m_s=0.))

    def test_excess_tilt_rate_and_edge_target_are_rejected(self):
        with self.assertRaisesRegex(InfeasibleTouchdown,'tilt'):self.solve(self.request(angle=6))
        with self.assertRaisesRegex(InfeasibleTouchdown,'angular speed'):self.solve(self.request(rate=6))
        with self.assertRaisesRegex(InfeasibleTouchdown,'footprint'):self.solve(self.request(xy=(.95,0)))

    def test_receding_or_excessive_descent_is_rejected(self):
        with self.assertRaisesRegex(InfeasibleTouchdown,'closing'):self.solve(self.request(velocity=(0,0,.1)))
        with self.assertRaisesRegex(InfeasibleTouchdown,'closing'):self.solve(self.request(velocity=(0,0,-.2)))

    def test_common_translation_shifts_solution_without_changing_velocities(self):
        original=self.solve()
        delta=(3,4,5)
        deck=Deck(RigidMotion.stationary(Pose(add(self.deck.motion.pose.position_world_m,delta),(0,0,0,1))),(1,.7))
        shifted=self.solve(deck=deck)
        self.assertVector(shifted.aircraft.pose.position_world_m,add(original.aircraft.pose.position_world_m,delta))
        self.assertVector(shifted.aircraft.velocity_at_origin_world_m_s,original.aircraft.velocity_at_origin_world_m_s)

    def test_local_zero_rate_braking_example_contacts_only_at_endpoint(self):
        theta=math.radians(2)
        result=self.solve()
        acceleration=(-9.80665*math.tan(theta),0,0)
        for i in range(21):
            dt=-.2+i*.01
            p=add(result.aircraft.pose.position_world_m,add(scale(result.aircraft.velocity_at_origin_world_m_s,dt),scale(acceleration,.5*dt*dt)))
            v=add(result.aircraft.velocity_at_origin_world_m_s,scale(acceleration,dt))
            state=RigidMotion(Pose(p,result.aircraft.pose.orientation_xyzw),v,(0,0,0))
            feet=evaluate_airframe(self.geometry,state,self.deck)
            if i<20:
                self.assertGreater(feet[0].clearance_m,0)
                self.assertGreater(state.velocity_at_origin_world_m_s[0],0)
            else:
                self.assertAlmostEqual(feet[0].clearance_m,0,places=12)
                self.assertAlmostEqual(state.velocity_at_origin_world_m_s[0],0,places=12)

    def test_nonfinite_and_inconsistent_requests_are_rejected(self):
        with self.assertRaises(ValueError):self.request(rate=math.nan)
        with self.assertRaises(ValueError):self.request(xy=(0,math.inf))
        with self.assertRaises(ValueError):replace(self.limits,min_closing_speed_m_s=.2)
        with self.assertRaises(ValueError):replace(self.limits,max_tangential_speed_m_s=-1)


if __name__=='__main__':unittest.main()
