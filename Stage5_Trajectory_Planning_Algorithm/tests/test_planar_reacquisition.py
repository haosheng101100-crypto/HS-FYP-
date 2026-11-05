"""Scene-only maneuver trigger; no future command enters the landing policy."""
import unittest
import json
import math
from pathlib import Path
from landing.planar_motion import ReacquisitionManeuver,PlanarMotionCommand
from landing.planar_landing import PlanarLandingSupervisor
from landing.receding_landing import PlatformState,FlightState


class ReacquisitionTriggerTests(unittest.TestCase):
    def test_profile_preserves_original_gates_and_has_strict_landing_expectation(self):
        root=Path(__file__).resolve().parents[1]
        baseline=json.loads((root/'config/experiments/planar_landing_v1.json').read_text(encoding='utf-8'))
        cfg=json.loads((root/'config/experiments/planar_reacquisition_v1.json').read_text(encoding='utf-8'))
        for key in ('planner_limits','planar_landing','deck_half_extents','nominal_acceleration_reserve_m_s2'):
            self.assertEqual(cfg[key],baseline[key])
        self.assertEqual(cfg['moving_alignment']['minimum_leg_gap_m'],4.)
        self.assertTrue(all(v=='landing_after_reacquisition' for v in cfg['validation_expectations'].values()))
    def test_real_high_descent_triggers_once_and_keeps_new_target(self):
        for scenario in ('planar_reacquire_x','planar_reacquire_y','planar_reacquire_xy'):
            maneuver=ReacquisitionManeuver(scenario)
            original,event=maneuver.step(0.,3.,0.,'ALIGNING')
            self.assertIsNone(event)
            target,event=maneuver.step(.04,2.3,-.1,'TRACKING')
            self.assertIsNotNone(event)
            self.assertEqual(event['event'],'PLANAR_MANEUVER_TRIGGERED')
            self.assertLess(sum(a*b for a,b in zip(original,target)),0.)
            repeated,second=maneuver.step(.08,2.5,0.,'ALIGNING')
            self.assertEqual(repeated,target);self.assertIsNone(second)

    def test_no_trigger_during_follow_low_clearance_or_no_descent(self):
        for gap,vz,phase in ((2.3,-.1,'ALIGNING'),(1.,-.1,'TRACKING'),(2.3,0.,'TRACKING'),(3.,-.1,'TRACKING'),(2.3,-.1,'TERMINAL_TRACKING')):
            _,event=ReacquisitionManeuver('planar_reacquire_x').step(0.,gap,vz,phase)
            self.assertIsNone(event)

    def test_invalid_inputs_and_clocks_fail(self):
        with self.assertRaises(ValueError):ReacquisitionManeuver('unknown')
        m=ReacquisitionManeuver('planar_reacquire_x');m.step(1.,3.,0.,'ALIGNING')
        with self.assertRaises(ValueError):m.step(.9,2.3,-.1,'TRACKING')
        with self.assertRaises(ValueError):m.step(1.04,float('nan'),-.1,'TRACKING')

    def test_kinematic_scene_policy_sequence_reacquires_then_reopens_window(self):
        # Numerical contract check, not Gazebo or a contact-dynamics simulation.
        for scenario in ('planar_reacquire_x','planar_reacquire_y','planar_reacquire_xy'):
            maneuver=ReacquisitionManeuver(scenario);command=PlanarMotionCommand();policy=PlanarLandingSupervisor()
            position=[0.,0.,.8];descending=False;triggered=False;reacquired=False;resumed=False
            for i in range(700):
                t=i*.04;gap=2.3 if descending else 2.5;vz=-.08 if descending else 0.
                target,event=maneuver.step(t,gap,vz,'TRACKING' if descending else 'ALIGNING')
                triggered=triggered or event is not None
                velocity,acceleration=command.step(t,target)
                for axis in range(2):position[axis]+=velocity[axis]*.04
                p=PlatformState(t,tuple(position),(*velocity,0.),(*acceleration,0.))
                s=FlightState(t,(position[0],position[1],.8+.195+gap),(*velocity,vz),(0.,0.,0.))
                report=policy.update(t,s,p,gap,aligned=True,descending=descending)
                self.assertNotEqual(report['action'],'EXIT',report)
                if report['action']=='REACQUIRE':reacquired=True;descending=False
                elif report['action']=='DESCEND':
                    if reacquired:resumed=True;break
                    descending=True
            self.assertTrue(triggered and reacquired and resumed,scenario)

    def test_actual_worker_planner_retains_clearance_during_delayed_change_reaction(self):
        from dataclasses import replace
        import numpy as np
        from landing.receding_landing import BezierLandingPlanner,PlannerLimits
        from landing.contact_geometry import AirframeGeometry,evaluate_airframe
        from landing.swept_envelope import SweptEnvelope
        from landing.receding_live import braking_clearance_design
        root=Path(__file__).resolve().parents[1]
        cfg=json.loads((root/'config/experiments/planar_reacquisition_v1.json').read_text(encoding='utf-8'))
        path=root/'config/airframes/iris_fixed_gimbal_sim.json';profile=json.loads(path.read_text())
        limits=PlannerLimits(**cfg['planner_limits'])
        limits=replace(limits,acceleration=limits.acceleration-cfg['nominal_acceleration_reserve_m_s2'],entry_acceleration_capture_m_s2=limits.acceleration)
        planner=BezierLandingPlanner(AirframeGeometry.load(path),profile['nominal_com_in_body_m'],limits,SweptEnvelope.load(root/cfg['envelope_profile']))
        for velocity in ((.25,0.,0.),(0.,.25,0.),(.18,.18,0.)):
            p=PlatformState(0.,(3.,0.,.8),velocity,(0,0,0),half_extents=tuple(cfg['deck_half_extents']))
            s=FlightState(0.,(3.,planner.offset[1],.8+.195+cfg['moving_alignment']['minimum_leg_gap_m']+planner.offset[2]),velocity,(0,0,0))
            plan,report=planner.search(s,p,0.,cfg['moving_alignment']['initial_candidate_durations_s'],capture_design=cfg['planar_landing'])
            self.assertIsNotNone(plan,report)
            trigger=None
            for t in np.arange(0.,plan.duration,.04):
                gap=min(f.clearance_m for f in evaluate_airframe(planner.geometry,plan.motion(float(t)),p.predict(float(t)).deck()))
                if gap<=cfg['reacquisition_trial']['trigger_gap_m'] and plan.at(float(t))['velocity'][2]<-.02:trigger=float(t);break
            self.assertIsNotNone(trigger)
            for delay in (.4,.8,1.2):
                t=trigger+delay;self.assertLess(t,plan.duration)
                k=plan.at(t);state=FlightState(t,tuple(k['position']),tuple(k['velocity']),tuple(k['acceleration']))
                gap=min(f.clearance_m for f in evaluate_airframe(planner.geometry,plan.motion(t),p.predict(t).deck()))
                required=1.5+braking_clearance_design(state,upward_acceleration=.4,vertical_jerk=1.2)['required_clearance_m']
                self.assertGreater(gap-required,.2,(velocity,delay,gap,required))

if __name__=='__main__':unittest.main()
