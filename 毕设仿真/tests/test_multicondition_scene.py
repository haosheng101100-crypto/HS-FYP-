"""Real world-side trajectories, measured triggers and unchanged shaping."""
import json
import math
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
PATH=ROOT/'config/experiments/multicondition_matrix_v1.json'


def measured(**overrides):
    state=dict(sim_t_s=0.,clearance_m=4.,vertical_speed_m_s=0.,phase='ALIGNING',reacquisitions=0,activated_plan_count=1)
    state.update(overrides)
    return state


class MulticonditionSceneTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(PATH.is_file(), 'Frozen twenty-case matrix must exist')
        self.matrix=json.loads(PATH.read_text())
        self.assertTrue((ROOT/'tools/multicondition_scene.py').is_file(), 'World scene program must exist')
        global np, PlanarMotionCommand
        import numpy as np
        from landing.planar_motion import PlanarMotionCommand
        from tools.multicondition_scene import ScenarioProgram
        self.program=ScenarioProgram
        self.cases={c['case_id']:c for c in self.matrix['cases']}

    def test_twenty_unique_cases_do_not_expand_original_budget(self):
        self.assertEqual(set(self.cases),{f'M{i:02d}' for i in range(1,21)})
        self.assertEqual(len({c['scenario'] for c in self.cases.values()}),20)
        for c in self.cases.values():
            self.assertEqual(c['budget'],dict(total_task_s=40.,cumulative_wait_s=25.,max_reacquisitions=3))
            self.assertEqual(c['initial']['clearance_m'],4.)
            self.assertIn('formula',c['parameters'])

    def test_constant_velocities_are_composite_speeds_in_named_quadrants(self):
        for cid,want in [('M01',(.216506350946,.125)),('M02',(-.318198051534,.318198051534))]:
            got,events=self.program(self.cases[cid]).step(0.,measured())
            np.testing.assert_allclose(got,want,atol=1e-10)
            self.assertEqual(events,[])

    def test_curves_keep_turning_and_eight_has_two_crossings(self):
        for cid,sign in [('M03',-1),('M04',1)]:
            p=self.program(self.cases[cid]);v0,_=p.step(1.,measured());v1,_=p.step(2.,measured())
            self.assertEqual(int(np.sign((v0[0]*v1[1]-v0[1]*v1[0]))),sign)
        p=self.program(self.cases['M06'])
        v0,_=p.step(5.,measured());v1,_=p.step(15.,measured())
        self.assertLess(v0[1],0.);self.assertGreater(v1[1],0.)
        p=self.program(self.cases['M07']);pos=np.zeros(2);cross=[];previous=None
        for i in range(1001):
            t=i*.04;v,_=p.step(t,measured());pos+=np.array(v)*.04
            if previous is not None and previous[1]*pos[1]<0 and abs(pos[0]-.6)<.04:cross.append(t)
            previous=pos.copy()
        self.assertTrue(any(5.<t<7. for t in cross));self.assertTrue(any(17.<t<19. for t in cross))

    def test_height_events_need_measured_tracking_and_descent(self):
        for cid,h in [('M16',3.6),('M17',2.),('M18',.2)]:
            p=self.program(self.cases[cid])
            self.assertEqual(p.step(30.,measured(clearance_m=h))[1],[])
            _,events=p.step(30.1,measured(clearance_m=h,vertical_speed_m_s=-.1,phase='TRACKING'))
            self.assertEqual(len(events),1);self.assertFalse(events[0]['planner_knows_future_commands'])
            self.assertEqual(p.step(30.2,measured(clearance_m=h,vertical_speed_m_s=-.1,phase='TRACKING'))[1],[])

    def test_second_reversal_requires_realign_and_strictly_new_activated_plan(self):
        p=self.program(self.cases['M15'])
        first=measured(clearance_m=3.6,vertical_speed_m_s=-.1,phase='TRACKING',activated_plan_count=7)
        self.assertEqual(len(p.step(1.,first)[1]),1)
        self.assertEqual(p.step(2.,measured(phase='TRACKING',reacquisitions=1,activated_plan_count=8))[1],[])
        self.assertEqual(p.step(3.,measured(phase='ALIGNING',reacquisitions=1,activated_plan_count=7))[1],[])
        self.assertEqual(p.step(4.,measured(phase='TRACKING',reacquisitions=1,activated_plan_count=7))[1],[])
        self.assertEqual(len(p.step(5.,measured(phase='TRACKING',reacquisitions=1,activated_plan_count=8))[1]),1)

    def test_second_reversal_recognizes_native_planar_follow_phase(self):
        p=self.program(self.cases['M15'])
        initial=measured(sim_t_s=1.,clearance_m=3.6,vertical_speed_m_s=-.1,phase='TRACKING',activated_plan_count=7)
        self.assertEqual(len(p.step(1.,initial)[1]),1)
        self.assertEqual(p.step(2.,measured(sim_t_s=2.,phase='PLANAR_FOLLOW',reacquisitions=0,activated_plan_count=7))[1],[])
        self.assertEqual(p.step(3.,measured(sim_t_s=3.,phase='TRACKING',reacquisitions=1,activated_plan_count=8))[1],[])
        self.assertEqual(p.step(4.,measured(sim_t_s=4.,phase='PLANAR_FOLLOW',reacquisitions=1,activated_plan_count=7))[1],[])
        self.assertEqual(p.step(5.,measured(sim_t_s=5.,phase='TRACKING',reacquisitions=1,activated_plan_count=7))[1],[])
        target,events=p.step(6.,measured(sim_t_s=6.,phase='TRACKING',reacquisitions=1,activated_plan_count=8))
        self.assertEqual(len(events),1)
        self.assertEqual(events[0]['trigger_reason'],'realigned_with_new_activated_plan')
        self.assertEqual(target,(.25,0.))

    def test_restart_has_an_actual_shaped_stop_then_oblique_restart(self):
        p=self.program(self.cases['M11']);command=PlanarMotionCommand();stopped=False;restarted=False
        for i in range(351):
            t=i*.04;target,_=p.step(t,measured(sim_t_s=t));v,_=command.step(t,target)
            if 7.<=t<8. and math.hypot(*v)<.005:stopped=True
            if t>12. and v[0]<-.1 and v[1]>.1:restarted=True
        self.assertTrue(stopped);self.assertTrue(restarted)

    def test_native_contact_and_land_take_priority_over_late_event(self):
        for extra in [dict(phase='LAND'),dict(contact_detected=True),dict(contact_complete=True)]:
            p=self.program(self.cases['M18']);state=measured(clearance_m=.2,vertical_speed_m_s=-.1,phase='TRACKING');state.update(extra)
            self.assertEqual(p.step(1.,state)[1],[])

    def test_fixed_random_sequence_and_original_shaper_stay_bounded(self):
        for cid in self.cases:
            p=self.program(self.cases[cid]);q=self.program(self.cases[cid]);command=PlanarMotionCommand();previous_a=np.zeros(2);position=np.zeros(2)
            for i in range(1001):
                t=i*.04;state=measured(sim_t_s=t);v,ev=p.step(t,state)
                self.assertEqual((v,ev),q.step(t,state))
                shaped,a=command.step(t,v);position+=np.array(shaped)*.04
                self.assertLessEqual(math.hypot(*shaped),.8+1e-9);self.assertLessEqual(math.hypot(*a),.12+1e-9)
                self.assertLessEqual(math.dist(a,previous_a),.012+1e-9);previous_a=a
                self.assertLess(np.max(np.abs(position)),18.)


    def test_original_shaper_eight_revisits_crossing_with_distinct_tangents_within_budget(self):
        p=self.program(self.cases['M07']);command=PlanarMotionCommand();positions=[];velocities=[];pos=np.zeros(2)
        for i in range(1001):
            t=i*.04;target,_=p.step(t,measured(sim_t_s=t));v,_=command.step(t,target)
            pos+=np.array(v)*.04;positions.append(pos.copy());velocities.append(v)
        # Two passages at the transverse crossing must exist before 25s; a
        # periodic orbit revisiting itself in the same direction does not pass.
        crossings=[]
        for i in range(75,625):
            for j in range(i+100,625):
                if np.linalg.norm(positions[i]-positions[j])<.025:
                    a=np.array(velocities[i]);b=np.array(velocities[j])
                    if np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b))<.4:
                        crossings.append([i*.04,j*.04,float(np.linalg.norm(positions[i]-positions[j]))])
        self.assertTrue(crossings,'Original shaper must actually traverse figure-eight crossing before wait budget')

    def test_offline_reference_and_original_shaper_feasibility_report(self):
        report={}
        for cid,case in self.cases.items():
            p=self.program(case);command=PlanarMotionCommand();pos=np.zeros(2);extent=np.zeros(2);peak_a=0.;peak_ref_a=0.;last_ref=None;curvatures=[];previous_v=None
            for i in range(1001):
                t=i*.04;target,_=p.step(t,measured(sim_t_s=t));v,a=command.step(t,target)
                if last_ref is not None:peak_ref_a=max(peak_ref_a,math.dist(target,last_ref)/.04)
                last_ref=target;pos+=np.array(v)*.04;extent=np.maximum(extent,np.abs(pos));peak_a=max(peak_a,math.hypot(*a))
                if i>100 and previous_v is not None:
                    cross=previous_v[0]*v[1]-previous_v[1]*v[0]
                    curvatures.append(cross/.04/max(math.hypot(*v)**3,1e-10))
                previous_v=v
            report[cid]={'shaped_command_max_abs_xy_m':extent.tolist(),'shaped_command_peak_acceleration_m_s2':peak_a,'reference_peak_acceleration_m_s2':peak_ref_a,'shaped_command_curvature_min_max': [min(curvatures),max(curvatures)],'source':'original PlanarMotionCommand dt=.04; not physical validation'}
            if cid in ('M03','M04','M05','M06','M07','M08','M09','M10','M11','M12','M13'):self.assertLessEqual(peak_ref_a,.12+1e-7)
            if cid=='M06':self.assertLess(min(curvatures),-.1);self.assertGreater(max(curvatures),.1)
        destination=ROOT.parent/'scene_offline_feasibility.json'
        destination.write_text(json.dumps(report,indent=2)+'\n')

    def test_nonfinite_or_regressing_world_time_is_rejected(self):
        p=self.program(self.cases['M01']);p.step(1.,measured())
        for t in [1.,.5,float('nan')]:
            with self.assertRaises(ValueError):p.step(t,measured())

if __name__=='__main__':unittest.main()
