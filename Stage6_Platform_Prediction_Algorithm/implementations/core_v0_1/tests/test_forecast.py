import math
import unittest
import numpy as np
from stage6.filter import UKF
from stage6.forecast import sample_paths, admissible_batch
from stage6.model import Mode, MODES, Limits

class ForecastTests(unittest.TestCase):
    def test_particle_circle_analytic_and_velocity_heading_link(self):
        mode=Mode('analytic',0,0,1,1,0,0)
        state=np.array([0.,0.,0.,1.,0.,.5,0.,0.])
        f=UKF(state,np.eye(8)*1e-12,mode)
        r=sample_paths(f,np.array([1.,2.]),2,np.random.default_rng(7),Limits())
        self.assertEqual(len(r['trajectories']),2)
        for path in r['trajectories']:
            s=np.array(path['states']);v=np.array(path['velocity_xy'])
            np.testing.assert_allclose(s[-1,:3],[2*math.sin(1),2*(1-math.cos(1)),1.],atol=2e-5)
            np.testing.assert_allclose(v[-1],[math.cos(1),math.sin(1)],atol=2e-5)
            self.assertTrue(np.all(admissible_batch(s,mode,Limits())))

    def test_all_rejections_reported(self):
        state=np.zeros(8);state[3]=20
        f=UKF(state,np.eye(8)*1e-12,MODES[0])
        r=sample_paths(f,np.array([.1,.2]),2,np.random.default_rng(7),Limits())
        self.assertEqual(r['trajectories'],[])
        self.assertEqual(r['attempted'],40)
        self.assertEqual(r['rejected'],40)
        self.assertIsNone(r['sample_envelope'])
