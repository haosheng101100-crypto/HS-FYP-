import unittest
import numpy as np
from audit_planar_physics import axis_evidence
class RawPlanarEvidenceTests(unittest.TestCase):
 def test_two_axis_speed_uses_vector_norm(self):
  t=np.arange(0.,1.,.01);p=np.column_stack((.6*t,.6*t,np.zeros(len(t))))
  report,_,exercised=axis_evidence(t,p,'planar_diagonal',.8)
  self.assertTrue(exercised);self.assertFalse(report['checks']['actual_speed']);self.assertAlmostEqual(report['peak_speed_m_s'],2**.5*.6,places=9)
 def test_motion_on_one_axis_does_not_prove_two_axis_fixture(self):
  t=np.arange(0.,1.,.01);p=np.column_stack((.3*t,np.zeros(len(t)),np.zeros(len(t))))
  _,_,exercised=axis_evidence(t,p,'planar_xy',.8);self.assertFalse(exercised)
 def test_actual_lateral_motion_is_required(self):
  t=np.arange(0.,1.,.01);p=np.column_stack((np.zeros(len(t)),.3*t,np.zeros(len(t))))
  report,_,exercised=axis_evidence(t,p,'planar_lateral',.8)
  self.assertTrue(exercised);self.assertAlmostEqual(report['peak_speed_m_s'],.3,places=9)
 def test_declared_30hz_platform_samples_are_valid(self):
  t=np.arange(0.,1.,.034);p=np.column_stack((np.zeros(len(t)),.3*t,np.zeros(len(t))))
  report,_,exercised=axis_evidence(t,p,'planar_lateral',.8)
  self.assertTrue(exercised);self.assertAlmostEqual(report['peak_speed_m_s'],.3,places=9)
 def test_pose_gaps_are_rejected(self):
  t=np.arange(0.,1.,.1);p=np.column_stack((.3*t,np.zeros(len(t)),np.zeros(len(t))))
  with self.assertRaises(ValueError):axis_evidence(t,p,'planar_forward_backward',.8)
if __name__=='__main__':unittest.main()
