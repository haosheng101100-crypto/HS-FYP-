from pathlib import Path
import sys,unittest
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'work/receding_dependencies'))
from landing.receding_live import contact_heading_acceleration_guard

class HeadingGuardTests(unittest.TestCase):
    def test_measured_reverse_turn_rejected_before_late_tracking_failure(self):
        self.assertFalse(contact_heading_acceleration_guard(SimpleNamespace(yaw_acceleration=.112))['passed'])

    def test_smooth_and_noisy_recorded_ranges_remain_eligible(self):
        for value in (.019,-.026,.04,-.04):
            self.assertTrue(contact_heading_acceleration_guard(SimpleNamespace(yaw_acceleration=value))['passed'])

    def test_invalid_input_fails_closed(self):
        for value,bound in ((float('nan'),.04),(.01,0.),(.01,float('inf'))):
            with self.assertRaises(ValueError):contact_heading_acceleration_guard(SimpleNamespace(yaw_acceleration=value),bound)

if __name__=='__main__':unittest.main()
