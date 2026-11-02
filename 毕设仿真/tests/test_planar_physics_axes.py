import unittest
from tools.audit_planar_physics import axis_evidence


class ReacquisitionPhysicalAxesTests(unittest.TestCase):
    def test_single_axis_trial_requires_its_actual_axis_only(self):
        times=[0.,.033,.066,.099,.132]
        for scenario,points in [('planar_reacquire_x',[(t*.2,0.,0.) for t in times]),('planar_reacquire_y',[(0.,t*.2,0.) for t in times])]:
            peaks,ranges,passed=axis_evidence(times,points,scenario,.8)
            self.assertTrue(passed)
            self.assertTrue(peaks['checks']['actual_speed'])
        self.assertFalse(axis_evidence(times,[(t*.2,0.,0.) for t in times],'planar_reacquire_y',.8)[2])

if __name__=='__main__':unittest.main()
