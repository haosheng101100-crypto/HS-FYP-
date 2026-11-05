import math
import unittest
import numpy as np
from stage6.model import MODES, Mode
from stage6.observation import Observation
from stage6.filter import UKF, sigma_points, moments, pose_measurement, initialize, positive_covariance


def obs(t=0.,xy=(0.,0.),yaw=0.,r=None,received=None):
    return Observation(t,t if received is None else received,xy,yaw,np.diag([.04,.04,.01]) if r is None else r)


class FilterTests(unittest.TestCase):
    def test_sigma_reconstructs_covariance(self):
        m=np.zeros(8);p=np.diag([1,2,.01,.3,.1,.1,.2,.2])
        pts,wm,wc=sigma_points(m,p)
        actual,cov=moments(pts,wm,wc,angle_index=2)
        np.testing.assert_allclose(actual,m,atol=1e-12)
        np.testing.assert_allclose(cov,p,atol=1e-12)

    def test_linear_measurement_update(self):
        p=np.diag([1,1,.01,1,1,1,1,1])
        f=UKF(np.zeros(8),p,MODES[3])
        ll=f.update(obs(xy=(1.,0.)))
        self.assertTrue(math.isfinite(ll))
        self.assertAlmostEqual(f.mean[0],1/1.04,places=9)
        self.assertAlmostEqual(f.covariance[0,0],.04/1.04,places=9)
        self.assertAlmostEqual(f.covariance[2,2],.005,places=9)

    def test_yaw_wrap_short_innovation(self):
        s=np.zeros(8);s[2]=math.pi-.02
        f=UKF(s,np.diag([.1,.1,.001,.1,.1,.1,.1,.1]),MODES[3])
        f.update(obs(yaw=-math.pi+.02))
        self.assertLess(abs(abs(f.mean[2])-math.pi),.03)
        self.assertLess(f.last_nis,.2)

    def test_pose_offset_and_initial_covariance(self):
        s=np.zeros(8);s[2]=math.pi/2
        np.testing.assert_allclose(pose_measurement(s,(1.,0.)),[0,1,math.pi/2],atol=1e-12)
        f=initialize(obs(xy=(1.,0.)),MODES[0],offset=(1.,0.))
        np.testing.assert_allclose(f.mean[:3],[0,0,0])
        self.assertAlmostEqual(f.covariance[1,2],-.01,places=9)
        self.assertAlmostEqual(f.covariance[1,1],.05,places=9)

    def test_stationary_curvature_not_divided_by_speed(self):
        f=initialize(obs(),MODES[3])
        initial=f.covariance[5,5]
        for i in range(1,11):
            f.predict(.1);f.update(obs(t=i/10))
        self.assertTrue(np.all(np.isfinite(f.mean)))
        self.assertGreater(f.covariance[5,5],initial*.6)

    def test_process_noise_matches_substep_and_no_state_clipping(self):
        f=UKF(np.zeros(8),np.eye(8)*.001,MODES[3]);g=f.clone()
        f.predict(.2)
        for _ in range(4):g.predict(.05)
        np.testing.assert_allclose(f.mean,g.mean,atol=1e-9)
        np.testing.assert_allclose(f.covariance,g.covariance,atol=1e-9)
        self.assertGreater(f.covariance[6,6],.02)
        self.assertGreater(np.linalg.eigvalsh(f.covariance).min(),0.)

    def test_invalid_observations_and_covariance(self):
        for r in [np.zeros((3,3)),np.diag([1,1,-1]),np.ones((2,2))]:
            with self.assertRaises(ValueError):obs(r=r)
        with self.assertRaises(ValueError):obs(t=1.,received=.9)
        with self.assertRaises(ValueError):obs(xy=(float('nan'),0))
        with self.assertRaises(ValueError):positive_covariance(np.diag([1.,-1.]))
        with self.assertRaises(ValueError):positive_covariance(np.array([[1.,.5],[0.,1.]]))

    def test_clone_does_not_alias(self):
        f=initialize(obs(),MODES[0]);g=f.clone();g.mean[0]=9
        self.assertEqual(f.mean[0],0.)
        g.covariance[0,0]=99
        self.assertNotEqual(f.covariance[0,0],99)

    def test_broad_heading_covariance_cannot_alias_to_precise(self):
        p=np.eye(8)*1e-10;p[2,2]=(2*math.pi)**2/8
        with self.assertRaisesRegex(ValueError,'[Aa]ngle|[Aa]ngular'):
            UKF(np.zeros(8),p,MODES[3])
        with self.assertRaisesRegex(ValueError,'[Aa]ngle|[Aa]ngular'):
            initialize(obs(r=np.diag([.04,.04,p[2,2]])),MODES[3])

    def test_unsupported_prediction_does_not_partially_change_filter(self):
        p=np.eye(8)*.001;p[2,2]=.3;p[5,5]=.5
        mean=np.zeros(8);mean[3]=2.
        f=UKF(mean,p,MODES[3]);original=f.clone()
        with self.assertRaises(ValueError):f.predict(.3)
        np.testing.assert_array_equal(f.mean,original.mean)
        np.testing.assert_array_equal(f.covariance,original.covariance)
