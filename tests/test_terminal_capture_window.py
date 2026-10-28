from pathlib import Path
import sys,json,math,unittest
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'work/receding_dependencies'),str(ROOT)]
import landing.receding_live as live

class TerminalCaptureWindowTests(unittest.TestCase):
    def test_native_mode_transition_waits_for_short_measured_free_flight(self):
        rows=[json.loads(l) for l in (ROOT/'work/constant79_control.jsonl').read_text().splitlines()]
        row=next(r for r in rows if abs(r['sim_t_s']-45.4)<1e-8)
        checks=self.conditions(row);self.assertTrue(all(checks.values()))
        gap=min(f['clearance_m'] for f in row['feet']);feet=[f for f in row['feet'] if f['clearance_m']<=gap+.001]
        closing=min(2*f['closing_speed_m_s']-f['closing_speed_upper_bound_m_s'] for f in feet)
        guard=live.native_capture_time_guard(gap,closing,.05)
        self.assertFalse(guard['passed'])
        self.assertFalse(live.terminal_capture_window(.14,.25,checks,.05,.02,True,native_time_guard=guard)['ready'])
        near=live.native_capture_time_guard(.001,closing,.05)
        self.assertTrue(near['passed'])
        self.assertTrue(live.terminal_capture_window(.06,.25,checks,.05,.02,True,native_time_guard=near)['ready'])
        checks['angular_speed']=False
        self.assertFalse(live.terminal_capture_window(.06,.25,checks,.05,.02,True,native_time_guard=near)['ready'])

    def test_native_time_guard_never_accepts_nonclosing_or_invalid_inputs(self):
        for speed in (0.,-.01):self.assertFalse(live.native_capture_time_guard(.001,speed,.05)['passed'])
        self.assertFalse(live.native_capture_time_guard(-.001,.04,.05)['passed'])
        for gap,speed,limit in ((float('nan'),.04,.05),(.001,float('inf'),.05),(.001,.04,0.)):
            with self.assertRaises(ValueError):live.native_capture_time_guard(gap,speed,limit)

    def conditions(self,row):
        gap=min(f['clearance_m'] for f in row['feet']);feet=[f for f in row['feet'] if f['clearance_m']<=gap+.001]
        obs=row['observation'];q=obs['motion']['pose']['orientation_xyzw'];omega=obs['motion']['angular_velocity_world_rad_s']
        return dict(fresh=True,armed_guided=obs['armed'] and obs['flight_mode']=='GUIDED',gap_0_to_1cm=0<=gap<=.01,
            relative_foot_speed=all(f['tangential_speed_upper_bound_m_s']<=.05 and .02<=2*f['closing_speed_m_s']-f['closing_speed_upper_bound_m_s'] and f['closing_speed_upper_bound_m_s']<=.12 for f in feet),
            attitude=2*math.acos(min(1.,abs(q[3])))<=math.pi/180,
            angular_speed=math.sqrt(sum(x*x for x in omega))<=math.radians(8),target_xy=True,footprint=all(min(f['whole_cylinder_footprint_margins_m'])>=.02 for f in row['feet']))

    def test_saved_noisy_capture_can_handover_when_all_original_conditions_hold(self):
        rows=[json.loads(l) for l in (ROOT/'work/noisy71_control.jsonl').read_text().splitlines()]
        ready=next(r for r in rows if abs(r['sim_t_s']-46.74)<1e-8)
        checks=self.conditions(ready);self.assertTrue(all(checks.values()))
        self.assertTrue(live.terminal_capture_window(.12,.25,checks,ready['whole_aircraft_deck_margin_m'],.02,True)['ready'])
        rejected=next(r for r in rows if abs(r['sim_t_s']-46.86)<1e-8)
        self.assertFalse(self.conditions(rejected)['relative_foot_speed'])
        self.assertFalse(live.terminal_capture_window(0.,.25,self.conditions(rejected),.05,.02,True)['ready'])

    def test_window_never_waives_any_capture_or_geometry_condition(self):
        checks={k:True for k in ('fresh','armed_guided','gap_0_to_1cm','relative_foot_speed','attitude','angular_speed','target_xy','footprint')}
        for key in checks:
            bad=dict(checks,**{key:False})
            self.assertFalse(live.terminal_capture_window(.1,.25,bad,.03,.02,True)['ready'])
        for remaining,margin,preview in ((.3,.03,True),(-.01,.03,True),(.1,.019,True),(.1,.03,False)):
            self.assertFalse(live.terminal_capture_window(remaining,.25,checks,margin,.02,preview)['ready'])
        self.assertFalse(live.terminal_capture_window(.1,.25,{},.03,.02,True)['ready'])

if __name__=='__main__':unittest.main()
