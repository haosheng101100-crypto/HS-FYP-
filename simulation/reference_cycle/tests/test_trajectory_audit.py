import unittest
from pathlib import Path
from landing.contact_geometry import AirframeGeometry,CylinderLeg,Deck,Pose,RigidMotion,quaternion_from_rpy
from landing.trajectory_audit import audit_reference

ROOT=Path(__file__).resolve().parents[1]

class TrajectoryAuditChecks(unittest.TestCase):
    def setUp(self):
        self.geometry=AirframeGeometry.load(ROOT/'config/airframes/iris_fixed_gimbal_sim.json')
        self.deck=Deck(RigidMotion.stationary(Pose((0,0,0),(0,0,0,1))),(1,.7))

    def test_known_level_descent_contact_time_and_all_legs(self):
        def trajectory(t):return RigidMotion(Pose((0,0,.395-.1*t),(0,0,0,1)),(0,0,-.1),(0,0,0))
        result=audit_reference(self.geometry,trajectory,lambda t:self.deck,3,41)
        self.assertAlmostEqual(result['first_plane_touch']['time_s'],2.,places=9)
        self.assertEqual(len(result['first_plane_touch']['candidate_legs']),4)
        self.assertAlmostEqual(result['terminal_feet'][0]['clearance_m'],-.1)

    def test_hover_has_no_contact(self):
        result=audit_reference(self.geometry,lambda t:RigidMotion.stationary(Pose((0,0,1),(0,0,0,1))),lambda t:self.deck,2,5)
        self.assertIsNone(result['first_plane_touch'])

    def test_crossing_plane_outside_platform_is_not_counted_as_landing(self):
        result=audit_reference(self.geometry,lambda t:RigidMotion(Pose((3,0,.395-.1*t),(0,0,0,1)),(0,0,-.1),(0,0,0)),lambda t:self.deck,3,41)
        self.assertIsNone(result['first_plane_touch'])

    def test_initial_intersection_is_reported_without_fake_time_refinement(self):
        result=audit_reference(self.geometry,lambda t:RigidMotion.stationary(Pose((0,0,.1),(0,0,0,1))),lambda t:self.deck,1,5)
        self.assertEqual(result['first_plane_touch']['time_s'],0)
        self.assertFalse(result['first_plane_touch']['bracket_refined'])

    def test_lower_outside_leg_does_not_replace_interior_contact_leg(self):
        geometry=AirframeGeometry('two_legs_for_edge_test',True,(
            CylinderLeg('inside',(0,0,-.11),.005,.17),
            CylinderLeg('outside',(2,0,-.11),.005,.17)))
        def trajectory(t):
            return RigidMotion(Pose((0,0,.395-.1*t),quaternion_from_rpy(0,.1,0)),(0,0,-.1),(0,0,0))
        result=audit_reference(geometry,trajectory,lambda t:self.deck,3,41)
        self.assertEqual(result['first_plane_touch']['lowest_leg'],'inside')
        self.assertEqual(result['first_plane_touch']['candidate_legs'],['inside'])

if __name__=='__main__':unittest.main()
