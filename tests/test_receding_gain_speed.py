from pathlib import Path
import sys,unittest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'work/receding_dependencies'))
import landing.receding_live as live
from landing.receding_live import terminal_feedback_gains

class SpeedScheduledGainTests(unittest.TestCase):
    base=dict(position_gain=6.25,velocity_gain=5.,acceleration_lead_s=.12)
    terminal=dict(transition_s=2.,position_gain=1.5,velocity_gain=2.4,acceleration_lead_s=0.,speed_scheduling=dict(low_speed_m_s=.45,high_speed_m_s=.65,fast_transition_s=3.))

    def test_slow_motion_keeps_original_tracking_and_fast_settles_earlier(self):
        slow,sl=terminal_feedback_gains(self.base,self.terminal,1.5,.2)
        fast,fa=terminal_feedback_gains(self.base,self.terminal,1.5,.7)
        original,_=terminal_feedback_gains(self.base,dict(self.terminal,speed_scheduling=None),1.5)
        self.assertEqual(slow,original);self.assertEqual(sl['transition_s'],2.)
        self.assertEqual(fa['transition_s'],3.);self.assertLess(fast['position_gain'],slow['position_gain'])

    def test_speed_interpolation_is_smooth_at_both_bounds(self):
        for speed in (.45,.65):
            values=[terminal_feedback_gains(self.base,self.terminal,1.5,speed+d)[0]['position_gain'] for d in (-1e-5,0.,1e-5)]
            self.assertLess(max(values)-min(values),1e-9)
        _,mid=terminal_feedback_gains(self.base,self.terminal,1.5,.55)
        self.assertAlmostEqual(mid['transition_s'],2.5)

    def test_invalid_or_future_unavailable_speed_fails_closed(self):
        for speed in (-1.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):terminal_feedback_gains(self.base,self.terminal,1.,speed)

    def test_interception_gain_is_not_applied_to_alignment_hold_or_recovery(self):
        for phase in ('ALIGNING','USER_HOVER','PLANNING_HOVER','FAILED_ATTEMPT_RETURN'):
            gains,report=live.execution_feedback_gains(self.base,self.terminal,None,.3,phase)
            self.assertEqual(gains['position_gain'],1.5)
            self.assertEqual(gains['velocity_gain'],2.4)
            self.assertEqual(gains['acceleration_lead_s'],.12 if phase=='PLANNING_HOVER' else 0.)
            self.assertEqual(report['policy'],'stable_non_descent_tracking')
            self.assertIsNone(report['remaining_s'])
        expected=terminal_feedback_gains(self.base,self.terminal,3.,.3)[0]
        self.assertEqual(live.execution_feedback_gains(self.base,self.terminal,3.,.3,'TRACKING')[0],expected)

if __name__=='__main__':unittest.main()
