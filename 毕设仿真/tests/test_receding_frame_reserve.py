from pathlib import Path
import sys,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'work/receding_dependencies'))
from landing.receding_live import frame_velocity_reserve_check

class FrameReserveTests(unittest.TestCase):
    def test_inactive_prediction_does_not_require_correction(self):
        self.assertEqual(frame_velocity_reserve_check(None),dict(active=False,passed=True))

    def test_two_axis_speed_uses_joint_feedback_budget(self):
        at=frame_velocity_reserve_check(dict(active=True,velocity_correction_m_s=(.24,.18,0.)))
        beyond=frame_velocity_reserve_check(dict(active=True,velocity_correction_m_s=(.24,.181,0.)))
        self.assertTrue(at['passed']);self.assertFalse(beyond['passed'])

    def test_detects_failed_physical_switch_before_old_late_exit(self):
        measured=frame_velocity_reserve_check(dict(active=True,velocity_correction_m_s=(-.33716469964056844,.02010199080005874,0.)))
        self.assertFalse(measured['passed'])
        self.assertGreater(measured['horizontal_correction_m_s'],.3)

    def test_nonfinite_input_and_invalid_budget_rejected(self):
        for budget in (0.,-1.,float('nan')):
            with self.assertRaises(ValueError):frame_velocity_reserve_check(None,budget)
        with self.assertRaises(ValueError):frame_velocity_reserve_check(dict(active=True,velocity_correction_m_s=(float('nan'),0.,0.)))

if __name__=='__main__':unittest.main()
