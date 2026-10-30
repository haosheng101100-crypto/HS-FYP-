import math
from pathlib import Path
import unittest
from landing.contact_geometry import AirframeGeometry,Deck,Pose,RigidMotion,sub,norm
from landing.touchdown_boundary import ContactDesignLimits,TouchdownRequest,solve_touchdown_boundary
from landing.septic_approach import SepticScalar,NinthScalar,PlanarSepticApproach,PlanarSnapMatchedApproach,ApproachDesignLimits,check_approach_reference,check_height_band_entries
from dataclasses import replace
import json

ROOT=Path(__file__).resolve().parents[1]

class SepticApproachChecks(unittest.TestCase):
    def test_arbitrary_known_polynomial_is_reconstructed_through_snap(self):
        c=(.2,-.3,.1,.04,-.1,.03,.02,-.01);T=1.7
        def f(t,o):return sum(c[k]*math.factorial(k)/math.factorial(k-o)*t**(k-o) for k in range(o,8))
        curve=SepticScalar(tuple(f(0,o) for o in range(4)),tuple(f(T,o) for o in range(4)),T)
        for t in [0,.13,.7,1.2,T]:
            for o in range(5):self.assertAlmostEqual(curve.at(t,o),f(t,o),places=9)

    def test_no_post_contact_reference_is_silently_extrapolated(self):
        curve=SepticScalar((0,0,0,0),(1,0,0,0),2.)
        with self.assertRaises(ValueError):curve.at(2.01)
        with self.assertRaises(ValueError):curve.at(-.01)
        with self.assertRaises(ValueError):SepticScalar((0,0,0,0),(1,0,0,0),0)

    def test_known_degree_nine_polynomial_is_reconstructed(self):
        c=(.2,-.3,.1,.04,-.1,.03,.02,-.01,.003,-.001);T=1.7
        def f(t,o):return sum(c[k]*math.factorial(k)/math.factorial(k-o)*t**(k-o) for k in range(o,10))
        curve=NinthScalar(tuple(f(0,o) for o in range(5)),tuple(f(T,o) for o in range(5)),T)
        for t in [0,.13,.7,1.2,T]:
            for o in range(6):self.assertAlmostEqual(curve.at(t,o),f(t,o),places=8)
        with self.assertRaises(ValueError):curve.at(T+.01)

    def make_approach(self):
        geometry=AirframeGeometry.load(ROOT/'config/airframes/iris_fixed_gimbal_sim.json')
        deck=Deck(RigidMotion.stationary(Pose((5,0,.8),(0,0,0,1))),(1,.7))
        boundary=solve_touchdown_boundary(geometry,deck,TouchdownRequest.planar(math.radians(2),0,(0,0),(0,0,-.08)),
            ContactDesignLimits(math.radians(5),math.radians(5),.05,.02,.12,.02))
        offset=(0.,-.0011616161616161613,-.013349641414141416)
        return PlanarSepticApproach((0,offset[1],3.65),(0,0,-.4),boundary,offset,9.80665,1.98,7.),boundary

    def test_terminal_pose_twist_acceleration_and_jerk_are_consistent(self):
        approach,boundary=self.make_approach();state=approach.at(approach.T)
        self.assertLess(norm(sub(state['motion'].pose.position_world_m,boundary.aircraft.pose.position_world_m)),1e-12)
        self.assertLess(norm(sub(state['motion'].velocity_at_origin_world_m_s,boundary.aircraft.velocity_at_origin_world_m_s)),1e-12)
        self.assertAlmostEqual(state['nose_up_rad'],math.radians(2),places=12)
        self.assertAlmostEqual(state['nose_up_rate_rad_s'],0,places=12)
        self.assertLess(state['com_acceleration_world_m_s2'][0],0)
        self.assertEqual(state['com_jerk_world_m_s3'],(0.,0.,0.))

    def test_pitch_rate_and_acceleration_match_finite_differences(self):
        approach,_=self.make_approach();h=1e-5
        for t in [1.,3.,5.]:
            before=approach.at(t-h);current=approach.at(t);after=approach.at(t+h)
            self.assertAlmostEqual((after['nose_up_rad']-before['nose_up_rad'])/(2*h),current['nose_up_rate_rad_s'],places=7)
            self.assertAlmostEqual((after['nose_up_rate_rad_s']-before['nose_up_rate_rad_s'])/(2*h),current['nose_up_acceleration_rad_s2'],places=7)

    def test_body_velocity_and_acceleration_match_material_origin_motion(self):
        approach,_=self.make_approach();h=1e-5
        for t in [1.,3.,5.]:
            before=approach.at(t-h);current=approach.at(t);after=approach.at(t+h)
            v=tuple((b-a)/(2*h) for a,b in zip(before['motion'].pose.position_world_m,after['motion'].pose.position_world_m))
            a=tuple((b-a)/(2*h) for a,b in zip(before['motion'].velocity_at_origin_world_m_s,after['motion'].velocity_at_origin_world_m_s))
            self.assertLess(norm(sub(v,current['motion'].velocity_at_origin_world_m_s)),1e-7)
            self.assertLess(norm(sub(a,current['body_acceleration_world_m_s2'])),1e-7)

    def test_com_force_direction_matches_com_acceleration(self):
        approach,_=self.make_approach()
        for t in [0,1,3,5,7]:
            state=approach.at(t)
            thrust=state['point_mass_required_thrust_N']
            direction=state['motion'].pose.rotate((0,0,1))
            acceleration=tuple(thrust/approach.mass*x-(approach.gravity if k==2 else 0) for k,x in enumerate(direction))
            self.assertLess(norm(sub(acceleration,state['com_acceleration_world_m_s2'])),1e-12)

    def test_snap_variant_preserves_physical_contact_and_endpoint_snap(self):
        previous,boundary=self.make_approach()
        curve=PlanarSnapMatchedApproach((0,previous.com_offset_body_m[1],3.65),(0,0,-.4),boundary,
            previous.com_offset_body_m,9.80665,1.98,6.5,terminal_com_snap_world_m_s4=(0,0,3))
        self.assertEqual(curve.at(0)['com_snap_world_m_s4'],(0.,0.,0.))
        end=curve.at(curve.T)
        self.assertEqual(end['com_snap_world_m_s4'],(0.,0.,3.))
        self.assertLess(norm(sub(end['motion'].pose.position_world_m,boundary.aircraft.pose.position_world_m)),1e-12)
        self.assertAlmostEqual(end['nose_up_rad'],math.radians(2),places=12)
        self.assertAlmostEqual(end['nose_up_rate_rad_s'],0,places=12)
        h=1e-5;t=5.
        self.assertAlmostEqual((curve.at(t+h)['nose_up_rate_rad_s']-curve.at(t-h)['nose_up_rate_rad_s'])/(2*h),
            curve.at(t)['nose_up_acceleration_rad_s2'],places=7)

    def test_tighter_foot_speed_bound_rejects_otherwise_valid_reference(self):
        previous,boundary=self.make_approach()
        geometry=AirframeGeometry.load(ROOT/'config/airframes/iris_fixed_gimbal_sim.json')
        deck=Deck(RigidMotion.stationary(Pose((5,0,.8),(0,0,0,1))),(1,.7))
        curve=PlanarSnapMatchedApproach((0,previous.com_offset_body_m[1],3.65),(0,0,-.4),boundary,
            previous.com_offset_body_m,9.80665,1.98,6.5,terminal_com_snap_world_m_s4=(0,0,3))
        values=json.loads((ROOT/'config/experiments/septic_approach_v1.json').read_text())['design_limits']
        for a,b in [('max_tilt_deg','max_tilt_rad'),('max_pitch_rate_deg_s','max_pitch_rate_rad_s'),
                    ('max_pitch_acceleration_deg_s2','max_pitch_acceleration_rad_s2'),('near_contact_tilt_deg','near_contact_tilt_rad')]:
            values[b]=math.radians(values.pop(a))
        limits=ApproachDesignLimits(**values)
        self.assertTrue(check_approach_reference(curve,geometry,deck,limits,1001)['passed'])
        rejected=check_approach_reference(curve,geometry,deck,replace(limits,near_contact_tangential_speed_m_s=.1),1001)
        self.assertFalse(rejected['passed'])
        self.assertEqual(rejected['reason'],'near-contact foot tangential speed')

    def test_band_entry_catches_speed_violation_between_time_samples(self):
        previous,boundary=self.make_approach()
        geometry=AirframeGeometry.load(ROOT/'config/airframes/iris_fixed_gimbal_sim.json')
        deck=Deck(RigidMotion.stationary(Pose((5,0,.8),(0,0,0,1))),(1,.7))
        curve=PlanarSnapMatchedApproach((0,previous.com_offset_body_m[1],3.65),(0,0,-.4),boundary,
            previous.com_offset_body_m,9.80665,1.98,6.35,terminal_com_snap_world_m_s4=(0,0,3))
        values=json.loads((ROOT/'config/experiments/septic_approach_v1.json').read_text())['design_limits']
        for a,b in [('max_tilt_deg','max_tilt_rad'),('max_pitch_rate_deg_s','max_pitch_rate_rad_s'),
                    ('max_pitch_acceleration_deg_s2','max_pitch_acceleration_rad_s2'),('near_contact_tilt_deg','near_contact_tilt_rad')]:
            values[b]=math.radians(values.pop(a))
        limits=ApproachDesignLimits(**values)
        rejected=check_approach_reference(curve,geometry,deck,limits,2001)
        self.assertFalse(rejected['passed'])
        self.assertEqual(rejected['reason'],'height-band entry constraint')
        events=check_height_band_entries(curve,geometry,deck,limits,401)
        failed=[e for e in events['events'] if not e['passed']]
        self.assertTrue(failed)
        self.assertAlmostEqual(failed[0]['boundary_t_s'],5.6080570084,places=8)
        self.assertGreater(failed[0]['tangential_speed_upper_bound_m_s'],.28)

if __name__=='__main__':unittest.main()
