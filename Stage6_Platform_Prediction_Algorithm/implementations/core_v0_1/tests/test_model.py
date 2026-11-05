import math
import unittest
import numpy as np
from stage6.model import MODES, Limits, step, derivative, violations, transition_matrix

class ModelTests(unittest.TestCase):
    def test_constant_velocity_and_reverse(self):
        s=np.array([1,2,math.pi/2,-2,0,0,0,0.])
        actual=step(s,1.,MODES[3])
        np.testing.assert_allclose(actual[:4],[1,0,math.pi/2,-2],atol=1e-9)
        np.testing.assert_array_equal(s[:2],[1,2])

    def test_acceleration_and_jerk_analytic(self):
        s=np.array([0,0,0,1,.5,0,0,0.])
        r=step(s,2.,MODES[1])
        np.testing.assert_allclose(r[:5],[3,0,0,2,.5],atol=1e-9)

    def test_circle(self):
        s=np.array([0,0,0,1,0,.5,0,0.])
        r=step(s,math.pi,MODES[2])
        np.testing.assert_allclose(r[:3],[2,2,math.pi/2],atol=2e-8)

    def test_batch_matches_scalar_and_substeps(self):
        s=np.array([0,0,3.1,1,.2,.1,.1,.03])
        batched=step(np.array([s,s]),.37,MODES[3])
        np.testing.assert_allclose(batched[0],step(s,.37,MODES[3]))
        np.testing.assert_allclose(step(s,.37,MODES[3]),step(s,.37,MODES[3],max_step=.01),atol=1e-7)

    def test_limits_use_actual_jerk(self):
        s=np.array([0,0,0,1,1,0,0,0.])
        self.assertIn('jerk',violations(s,MODES[0],Limits()))
        self.assertNotIn('jerk',violations(s,MODES[1],Limits()))
        s[3]=3;s[5]=1
        self.assertIn('lateral_acceleration',violations(s,MODES[3],Limits()))

    def test_transitions_are_stochastic_and_time_consistent(self):
        p=transition_matrix(.17)
        self.assertTrue(np.all(p>=0));np.testing.assert_allclose(p.sum(axis=1),1)
        np.testing.assert_allclose(p@transition_matrix(.31),transition_matrix(.48),atol=1e-12)
        np.testing.assert_allclose(transition_matrix(0),np.eye(4))

    def test_invalid_inputs(self):
        for dt in [-1,float('nan')]:
            with self.assertRaises(ValueError):step(np.zeros(8),dt,MODES[0])
        with self.assertRaises(ValueError):step(np.zeros(7),.1,MODES[0])
        with self.assertRaises(ValueError):step(np.zeros(8),.1,MODES[0],max_step=0)
        with self.assertRaises(ValueError):transition_matrix(-1)
