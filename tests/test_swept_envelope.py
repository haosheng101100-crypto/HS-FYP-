from pathlib import Path
import sys,unittest,math,json
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'work/receding_dependencies'))
import numpy as np
from landing.swept_envelope import SweptEnvelope
from landing.receding_landing import PlatformState,elevation_matrix,basis
from landing.contact_geometry import Pose,RigidMotion,Deck,quaternion_from_rpy


class SweptEnvelopeTests(unittest.TestCase):
    def setUp(self):self.envelope=SweptEnvelope.load(ROOT/'config/airframes/iris_swept_envelope_v1.json')

    def test_degree_elevation_preserves_acceleration_curve(self):
        a=np.arange(18).reshape(6,3)/10;raised=elevation_matrix(5,7)@a
        for t in np.linspace(0,1,29):np.testing.assert_allclose(basis(5,t)@a,basis(7,t)@raised,atol=1e-13)

    def test_rotor_support_bounds_all_sampled_phases_and_tilts(self):
        e=self.envelope
        for roll,pitch,yaw in ((.1,-.15,.3),(-.2,.12,-.2),(0,0,1.)):
            motion=RigidMotion.stationary(Pose((.01,-.02,1.),quaternion_from_rpy(roll,pitch,yaw)))
            deck=Deck(RigidMotion.stationary(Pose((0,0,.8),quaternion_from_rpy(0,0,.4))),(.34,.41))
            points=list(e.points)
            for r in e.rotors:
                for phase in np.linspace(0,2*math.pi,361):
                    for z in (-r['half_height'],r['half_height']):points.append(np.array(r['origin'])+[r['swept_radius']*math.cos(phase),r['swept_radius']*math.sin(phase),z])
            positions=[deck.motion.pose.inverse_rotate(np.array(motion.pose.transform(p))-np.array(deck.motion.pose.position_world_m)) for p in points]
            sampled=min(min(deck.half_extents_m[k]-abs(p[k]) for k in (0,1)) for p in positions)
            self.assertLessEqual(e.clearance(motion,deck),sampled+1e-10)
            self.assertLess(sampled-e.clearance(motion,deck),1e-5)

    def test_heading_grid_bound_covers_intermediate_headings(self):
        p=PlatformState(0,(0,0,.8),(0,0,0),(0,0,0),yaw_rate=.025,half_extents=(.34,.4082))
        bound=self.envelope.safe_radius(p,8.,(0,0,0),.03)
        for t in np.linspace(0,8.,1001):
            yaw=.025*t
            for h,axis in zip(p.half_extents,((math.cos(yaw),math.sin(yaw),0),(-math.sin(yaw),math.cos(yaw),0))):
                self.assertLessEqual(bound,h-self.envelope.support(axis)-.03+1e-12)

    def test_small_deck_area_uses_measured_rectangle_convention(self):
        d=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())
        self.assertAlmostEqual(4*np.prod(d['deck_half_extents'])/self.envelope.data['bounding_rectangle_area_m2'],1.5,places=12)

    def test_attitude_projection_bound_covers_rotor_phases_and_error_box(self):
        from scipy.spatial.transform import Rotation
        e=self.envelope;points=list(e.points)
        for r in e.rotors:
            for phase in np.linspace(0,2*math.pi,97):
                for z in (-r['half_height'],r['half_height']):
                    points.append(np.array(r['origin'])+[r['swept_radius']*math.cos(phase),r['swept_radius']*math.sin(phase),z])
        points=np.array(points);rng=np.random.default_rng(1108)
        for bounds in ((.01,.02,.003),(.15,.12,.2),(.0,.1,.0)):
            reserve=e.rotation_projection_bound(bounds)
            for angles in rng.uniform(-1,1,(100,3))*bounds:
                moved=Rotation.from_euler('xyz',angles).apply(points)
                self.assertLessEqual(np.linalg.norm((moved-points)[:,:2],axis=1).max(),reserve+1e-12)
        self.assertLess(e.rotation_projection_bound((0,.01,0)),e.radius*.01)
        self.assertEqual(e.rotation_projection_bound((0,0,0)),0.)


if __name__=='__main__':unittest.main()
