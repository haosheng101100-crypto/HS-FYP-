import json
import math
import unittest
import numpy as np
from stage6.observation import Observation
from stage6.model import MODES, Limits
from stage6.predictor import VehiclePredictor, mix_components, posterior_weights


def obs(t,x=None,y=0.,yaw=0.,received=None):
    return Observation(t,t if received is None else received,(t if x is None else x,y),yaw,np.diag([.0025,.0025,.0004]))


class PredictorTests(unittest.TestCase):
    def trained(self):
        p=VehiclePredictor()
        for i in range(11):p.update(obs(i/10))
        return p

    def test_mixing_includes_between_model_covariance_and_wrap(self):
        ms=np.zeros((2,8));ms[:,0]=[-2,2];ms[:,2]=[math.pi-.02,-math.pi+.02]
        covs=np.repeat((np.eye(8)*.01)[None],2,axis=0)
        m,p=mix_components(ms,covs,np.array([.5,.5]))
        self.assertAlmostEqual(m[0],0)
        self.assertAlmostEqual(p[0,0],4.01)
        self.assertLess(abs(abs(m[2])-math.pi),.001)
        self.assertLess(p[2,2],.011)

    def test_extreme_likelihoods_stay_normalized(self):
        w=posterior_weights(np.ones(4)/4,np.array([-1e8,-1e8-1,-1e8-2,-1e8-3]))
        self.assertAlmostEqual(w.sum(),1.)
        self.assertTrue(np.all(np.isfinite(w)));self.assertGreater(w[0],w[3])

    def test_pose_only_estimation_and_positive_covariances(self):
        p=self.trained()
        self.assertEqual(len(p.filters),4)
        self.assertAlmostEqual(p.weights.sum(),1.)
        for f in p.filters:
            self.assertLess(abs(f.mean[3]-1.),.3)
            self.assertGreater(np.linalg.eigvalsh(f.covariance).min(),0)
        self.assertEqual(p.forecast(1.,particles_per_mode=2,seed=2)['trust'],'uncalibrated')

    def test_lifecycle_and_delayed_receipt(self):
        p=VehiclePredictor()
        self.assertEqual(p.forecast(0)['status'],'warming_up')
        p.update(obs(0,received=.1))
        self.assertEqual(p.forecast(.1)['status'],'warming_up')
        with self.assertRaises(ValueError):p.forecast(.05)
        p=self.trained()
        self.assertEqual(p.forecast(1.3)['status'],'stale')
        p.update(obs(3.))
        self.assertEqual(p.forecast(3.)['status'],'warming_up')
        self.assertEqual(p.reset_count,1)

    def test_rejected_order_does_not_modify_state(self):
        p=self.trained();before=[f.mean.copy() for f in p.filters]
        with self.assertRaises(ValueError):p.update(obs(.5))
        for a,b in zip(before,p.filters):np.testing.assert_array_equal(a,b.mean)
        with self.assertRaises(ValueError):p.update(obs(1.1,received=.9))

    def test_forecast_is_repeatable_and_does_not_mutate_filter_or_global_rng(self):
        p=self.trained();before=[(f.mean.copy(),f.covariance.copy()) for f in p.filters]
        np.random.seed(123);expected=np.random.random(3);np.random.seed(123)
        a=p.forecast(1.05,particles_per_mode=3,seed=7)
        b=p.forecast(1.05,particles_per_mode=3,seed=7)
        self.assertEqual(json.dumps(a,sort_keys=True),json.dumps(b,sort_keys=True))
        np.testing.assert_array_equal(np.random.random(3),expected)
        for (m,c),f in zip(before,p.filters):
            np.testing.assert_array_equal(m,f.mean);np.testing.assert_array_equal(c,f.covariance)
        self.assertEqual([m['name'] for m in a['models']],['CV','CA','CT','CTVAR'])
        self.assertAlmostEqual(sum(m['posterior_weight'] for m in a['models']),1.)
        self.assertIsNone(a['physical_reachable_set'])
        for m in a['models']:
            self.assertEqual(m['attempted'],m['rejected']+len(m['trajectories']))
            for tr in m['trajectories']:self.assertEqual(len(tr['states']),20)
        json.dumps(a,allow_nan=False)

    def test_no_samples_does_not_erase_mode(self):
        p=self.trained()
        a=p.forecast(1.,particles_per_mode=2,seed=3,limits=Limits(speed=.00001,acceleration=.00001,curvature=.00001,jerk=.00001,curvature_rate=.00001,lateral_acceleration=.00001))
        self.assertEqual(a['status'],'degraded')
        self.assertEqual(len(a['models']),4)
        self.assertTrue(all(len(m['trajectories'])==0 for m in a['models']))
        self.assertAlmostEqual(sum(m['posterior_weight'] for m in a['models']),1.)

    def test_invalid_forecast_grid(self):
        p=self.trained()
        for kwargs in [dict(horizon=-1),dict(step_s=0),dict(particles_per_mode=0),dict(horizon=.25,step_s=.1)]:
            with self.assertRaises(ValueError):p.forecast(1.,**kwargs)

    def test_invalid_age_propagation_retains_modes_and_degrades(self):
        from stage6.filter import UKF
        p=self.trained()
        for j,f in enumerate(p.filters):
            mean=np.zeros(8);mean[3]=2.
            cov=np.eye(8)*.001;cov[2,2]=.3;cov[5,5]=.5
            p.filters[j]=UKF(mean,cov,f.mode)
        r=p.forecast(1.2,particles_per_mode=2)
        self.assertEqual(r['status'],'degraded')
        self.assertEqual(len(r['models']),4)
        for branch in r['models']:
            self.assertEqual(branch['propagation_error'],'unsupported_angular_spread')
            self.assertEqual(branch['trajectories'],[])
            self.assertIsNone(branch['state'])
