from pathlib import Path
import sys,unittest,math
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'work/receding_dependencies'))
import numpy as np
from landing.platform_filter import CausalPlatformKalman
from landing.contact_geometry import Pose,quaternion_from_rpy
from landing.observations import PoseSample


class PlatformFilterTests(unittest.TestCase):
    def sample(self,t,position,yaw=0.):
        return PoseSample(t,Pose((float(position),0.,0.),quaternion_from_rpy(0.,0.,float(yaw))),0.)

    def test_noisy_constant_velocity_and_causal_prediction(self):
        f=CausalPlatformKalman();rng=np.random.default_rng(147);rows=[];errors=[]
        for t in np.arange(0.,5.,.034):
            rows.append(self.sample(float(t),.3*t+rng.normal(0,.0008)))
            if len(rows)<8:continue
            state=f.update(rows[-20:],float(t),(.34,.4))
            if t>1.:errors.append(abs(state.predict(.6).position[0]-.3*(t+.6)))
        self.assertLess(np.quantile(errors,.9),.02)
        before=f.x.copy();count=f.count
        future=self.sample(100.,999.)
        state=f.update(rows[-20:]+[future],float(rows[-1].sim_t_s),(.34,.4))
        np.testing.assert_array_equal(before,f.x);self.assertEqual(f.count,count)

    def test_unannounced_acceleration_recovered_from_pose_only(self):
        f=CausalPlatformKalman();rows=[]
        for t in np.arange(0.,5.,.034):
            u=max(0.,t-2.)
            rows.append(self.sample(float(t),.3*t+.06*u*u))
            if len(rows)>=8:state=f.update(rows[-20:],float(t),(.34,.4))
        self.assertAlmostEqual(state.acceleration[0],.12,delta=.01)
        self.assertAlmostEqual(state.velocity[0],.3+.12*(state.stamp-2.),delta=.01)

    def test_heading_wrap_and_known_timestamp_delay(self):
        f=CausalPlatformKalman();rows=[]
        for t in np.arange(0.,4.,.034):
            rows.append(self.sample(float(t),.3*t,math.remainder(3.1+.1*t,2*math.pi)))
            if len(rows)>=8:state=f.update(rows[-20:],float(t+.04),(.34,.4))
        self.assertAlmostEqual(state.position[0],.3*state.stamp,delta=.001)
        self.assertAlmostEqual(math.remainder(state.yaw-(3.1+.1*state.stamp),2*math.pi),0.,delta=.001)
        self.assertAlmostEqual(state.yaw_rate,.1,delta=.001)

    def test_missing_data_rejected(self):
        f=CausalPlatformKalman();rows=[self.sample(float(t),0.) for t in np.arange(0.,.4,.034)]
        state=f.update(rows,rows[-1].sim_t_s,(.34,.4))
        with self.assertRaises(ValueError):f.update([],state.stamp+.2,(.34,.4))
        with self.assertRaises(ValueError):f.update([self.sample(state.stamp+.3,0.)],state.stamp+.3,(.34,.4))


if __name__=='__main__':unittest.main()
