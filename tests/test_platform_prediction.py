from pathlib import Path
import sys,unittest,math
from dataclasses import replace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'work/receding_dependencies'))
import numpy as np
from landing.platform_prediction import estimate_platform
from landing.receding_landing import PlatformState,basis
from landing.observations import PoseSample
from landing.contact_geometry import Pose,quaternion_from_rpy


class PlatformPredictionTests(unittest.TestCase):
    def samples(self):
        return [PoseSample(float(t),Pose((3+.4*t+.03*t*t+.004*t**3,-.02*t*t,0.),quaternion_from_rpy(0,0,.1+.02*t+.003*t*t)),0.) for t in np.linspace(-.35,0,36)]

    def test_causal_cubic_identifies_current_jerk_and_acceleration(self):
        state=estimate_platform(self.samples(),0.,(.34,.4))
        np.testing.assert_allclose(state.velocity,(.4,0,0),atol=1e-10)
        np.testing.assert_allclose(state.acceleration,(.06,-.04,0),atol=1e-10)
        np.testing.assert_allclose(state.jerk,(.024,0,0),atol=1e-9)
        self.assertAlmostEqual(state.yaw_acceleration,.006,places=10)

    def test_future_samples_never_leak_into_estimate(self):
        rows=self.samples();rows.append(PoseSample(.01,Pose((999,999,0),(0,0,0,1)),0.))
        a=estimate_platform(rows,0.,(.34,.4));b=estimate_platform(self.samples(),0.,(.34,.4))
        np.testing.assert_allclose(a.position,b.position,atol=1e-12)

    def test_cubic_bernstein_center_representation_is_exact(self):
        p=PlatformState(0.,(3.,.2,.8),(.4,.1,0),(.06,-.04,0),jerk=(.024,.03,0))
        dt=1.3
        C=np.array([np.array(p.position)+np.array(p.velocity)*dt*i/7+np.array(p.acceleration)*dt*dt*i*(i-1)/84+np.array(p.jerk)*dt**3*i*(i-1)*(i-2)/1260 for i in range(8)])
        for t in np.linspace(0,dt,31):np.testing.assert_allclose(basis(7,t/dt)@C,p.predict(float(t)).position,atol=1e-12)

    def test_fit_rejects_missing_causal_history(self):
        with self.assertRaises(ValueError):estimate_platform(self.samples()[:3],0.,(.34,.4))

    def test_derivative_window_rejects_command_step_ripple(self):
        t=np.arange(0.,8.001,.001)
        # Physical-loop callbacks occasionally advance by 50/60 ms. A perfectly
        # uniform command clock aliases differently and misses this ripple.
        ticks=np.r_[0.,np.cumsum(np.resize(np.array([.04,.05,.04,.04,.06]),200))]
        held=ticks[np.searchsorted(ticks,t,side='right')-1]
        v=.3+.12*np.sin(.6*held+.7);x=3+np.cumsum(v)*.001
        rows=[PoseSample(float(t[i]),Pose((float(x[i]),0,0),(0,0,0,1)),0.) for i in range(0,len(t),10)]
        errors={.35:[],.65:[]}
        for now in np.arange(6.,7.,.04):
            true=-.12*.6**2*math.sin(.6*now+.7)
            for span in errors:
                s=estimate_platform(rows,float(now),(.34,.4),span=span);errors[span].append((s.jerk[0]-true)**2)
        self.assertLess(np.mean(errors[.65]),np.mean(errors[.35]))

    def test_delayed_observations_ignore_future_and_compensate_timestamp(self):
        from landing.platform_prediction import delayed_noisy_platform
        rows=self.samples()
        # Data later than the sensor cutoff must not influence the estimate.
        a=delayed_noisy_platform(rows,0.,(.34,.4),span=.25,delay_s=.04,position_sigma_m=0.,yaw_sigma_rad=0.)
        rows=[replace(s,pose=Pose((999.,999.,0.),(0,0,0,1))) if s.sim_t_s>-.04 else s for s in rows]
        b=delayed_noisy_platform(rows,0.,(.34,.4),span=.25,delay_s=.04,position_sigma_m=0.,yaw_sigma_rad=0.)
        np.testing.assert_allclose(a.position,b.position,atol=1e-12)
        np.testing.assert_allclose(a.position,(3.,0.,.8),atol=1e-10)
        self.assertEqual(a.stamp,0.)

    def test_synthetic_noise_is_repeatable_and_actually_changes_observation(self):
        from landing.platform_prediction import delayed_noisy_platform
        a=delayed_noisy_platform(self.samples(),0.,(.34,.4),span=.25)
        b=delayed_noisy_platform(self.samples(),0.,(.34,.4),span=.25)
        clean=delayed_noisy_platform(self.samples(),0.,(.34,.4),span=.25,position_sigma_m=0.,yaw_sigma_rad=0.)
        np.testing.assert_array_equal(a.position,b.position)
        self.assertGreater(np.linalg.norm(np.array(a.position)-clean.position),1e-9)
        self.assertLess(np.linalg.norm(np.array(a.position)-clean.position),.005)

    def test_unresolved_cubic_noise_does_not_dominate_short_forecasts(self):
        # Match the actual platform pose stream (~30 Hz), rather than assuming
        # the higher-rate UAV body observation stream.
        t=np.linspace(-.65,0.,20);new_errors=[];cubic_errors=[]
        for seed in range(24):
            rng=np.random.default_rng(901+seed)
            xy=np.c_[.3*t,np.zeros(len(t))]+rng.normal(0,.0008,(len(t),2))
            rows=[PoseSample(float(s),Pose((float(p[0]),float(p[1]),0.),(0,0,0,1)),0.) for s,p in zip(t,xy)]
            estimate=estimate_platform(rows,0.,(.34,.4),span=.65)
            c=np.polyfit(t,xy,3);old=np.array([np.polyval(c[:,k],.6) for k in range(2)])
            target=np.array([.18,0.])
            new_errors.append(np.linalg.norm(np.array(estimate.predict(.6).position)[:2]-target)**2)
            cubic_errors.append(np.linalg.norm(old-target)**2)
        self.assertLess(np.mean(new_errors),.6*np.mean(cubic_errors))


if __name__=='__main__':unittest.main()
