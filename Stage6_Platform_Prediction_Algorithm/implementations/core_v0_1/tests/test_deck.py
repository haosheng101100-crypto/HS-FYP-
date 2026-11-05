import math
import unittest
import numpy as np
from stage6.deck import DeckGeometry, body_to_deck, map_forecast

class DeckTests(unittest.TestCase):
    def test_rotating_offset_velocity(self):
        s=np.array([0,0,math.pi/2,2,0,.5,0,0.])
        d=body_to_deck(s,DeckGeometry(offset_xy=(1.,0.),half_extents=(.5,.25)))
        np.testing.assert_allclose(d['position_xy'],[0,1],atol=1e-12)
        np.testing.assert_allclose(d['velocity_xy'],[-1,2],atol=1e-12)
        self.assertAlmostEqual(d['yaw_rate'],1.)
        np.testing.assert_allclose(d['corners_xy'][0],[-.25,1.5],atol=1e-12)

    def test_mount_yaw_and_batch_preserve_samples(self):
        states=np.zeros((2,8));states[:,3]=1;states[1,0]=2
        geom=DeckGeometry(mount_yaw=math.pi/2,half_extents=(.5,.25))
        d=body_to_deck(states,geom)
        self.assertEqual(np.asarray(d['corners_xy']).shape,(2,4,2))
        np.testing.assert_allclose(np.array(d['velocity_xy']),[[1,0],[1,0]])
        self.assertAlmostEqual(d['yaw'][0],math.pi/2)
        packet=dict(status='ready',models=[dict(name='CT',posterior_weight=.7,trajectories=[dict(states=states.tolist())])])
        mapped=map_forecast(packet,geom)
        self.assertNotIn('deck',packet['models'][0]['trajectories'][0])
        np.testing.assert_allclose(mapped['models'][0]['trajectories'][0]['deck']['position_xy'],[[0,0],[2,0]])
        self.assertEqual(mapped['models'][0]['posterior_weight'],.7)

    def test_invalid_geometry(self):
        with self.assertRaises(ValueError):DeckGeometry(half_extents=(-1,.3))
        with self.assertRaises(ValueError):DeckGeometry(offset_xy=(1,))
