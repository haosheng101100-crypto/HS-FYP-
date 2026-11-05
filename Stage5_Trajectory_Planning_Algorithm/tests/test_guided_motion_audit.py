from pathlib import Path
import sys,unittest
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'work/receding_dependencies'),str(ROOT)]
import numpy as np
import tools.audit_guided_motion as audit

class GuidedMotionAuditTests(unittest.TestCase):
    def test_nonuniform_raw_differences_reconstruct_quadratic_motion(self):
        t=np.array([0.,.01,.019,.029,.04,.05]);p=np.c_[.4*t+.6*t*t,np.zeros((len(t),2))]
        result=audit.motion_peaks(t,p,3.,1.5)
        self.assertTrue(result['passed']);self.assertAlmostEqual(result['peak_acceleration_m_s2'],1.2,places=9)

    def test_actual_excess_fails_even_with_bounded_endpoints(self):
        t=np.arange(20)*.01;p=np.c_[t*t,np.zeros((len(t),2))]
        result=audit.motion_peaks(t,p,3.,1.5)
        self.assertFalse(result['passed']);self.assertFalse(result['checks']['actual_acceleration'])
        self.assertTrue(result['checks']['actual_speed'])

    def test_missing_duplicate_and_nonfinite_samples_fail_closed(self):
        for t in ([0.,.01,.04,.05],[0.,.01,.01,.02],[0.,.01,float('nan'),.03]):
            with self.assertRaises(ValueError):audit.motion_peaks(t,np.zeros((4,3)),3.,1.5)

if __name__=='__main__':unittest.main()
