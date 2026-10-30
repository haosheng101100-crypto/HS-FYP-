from dataclasses import replace
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from landing.contact_geometry import Pose,RigidMotion,add,quaternion_from_rpy
from landing.reference_loader import load_verified_reference
from landing.execution import (AircraftObservation,ContactFrame,ContactHistory,EntryLimits,HandoverLimits,
    assess_entry,LandingExecutor,ContactHandover,ExecutionDecision,BodyReference,parse_deck_contacts)
from landing.mavlink_reference import LocalNedAlignment,dispatch
from landing.startup import VerticalLeadIn

ROOT=Path(__file__).resolve().parents[1]


class ExecutionChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.curve,cls.geometry,cls.deck,cls.profile=load_verified_reference(ROOT)
        cls.cfg=json.loads((ROOT/'config/experiments/execution_v1.json').read_text())
        cls.entry=EntryLimits(**cls.cfg['entry_limits']);cls.limits=HandoverLimits(**cls.cfg['handover_limits'])
        cls.legs=tuple(leg.name for leg in cls.geometry.legs)
        cls.rear=tuple(name for name in cls.legs if name.startswith('rear'))

    def observation(self,t,state=None,mode='GUIDED',armed=True):
        state=state or self.curve.at(0)
        return AircraftObservation(t,state['motion'],armed,mode,state['com_acceleration_world_m_s2'])

    def supported(self,t,armed=True,mode='LAND'):
        # Synthetic stationary supported pose, not integrated contact dynamics.
        p=self.curve.terminal_boundary.aircraft.pose.position_world_m
        motion=RigidMotion.stationary(Pose((p[0],p[1],.995),(0,0,0,1)))
        return AircraftObservation(t,motion,armed,mode,(0,0,0))

    def new_executor(self):return LandingExecutor(self.curve,self.geometry,self.deck,self.entry,self.limits)

    def contact_handover(self):
        hand=ContactHandover(self.geometry,self.deck,self.curve.terminal_boundary.aircraft,self.limits,16.4)
        hand.step(16.38,self.observation(16.38,self.curve.at(6.38)))
        decision=hand.step(16.4,self.observation(16.4,self.curve.at(6.4)),ContactFrame(16.395,self.rear))
        return hand,decision

    def test_entry_accepts_matching_body_origin_and_com_state(self):
        report=assess_entry(10.,self.observation(10.),self.curve.at(0),self.curve.com_offset_body_m,self.entry)
        self.assertTrue(report['ready']);self.assertEqual(report['actual_com_position_world_m'],tuple(self.curve.at(0)['com_position_world_m']))
        self.assertTrue(self.new_executor().step(10.,self.observation(10.)).reference)

    def test_vertical_lead_in_matches_main_com_derivatives_through_snap(self):
        lead=VerticalLeadIn(self.curve,1.5,.3)
        a=lead.at(lead.T);b=self.curve.at(0)
        for key in ['com_position_world_m','com_velocity_world_m_s','com_acceleration_world_m_s2','com_jerk_world_m_s3','com_snap_world_m_s4']:
            for x,y in zip(a[key],b[key]):self.assertAlmostEqual(x,y,places=12)
        self.assertEqual(a['motion'],b['motion'])
        self.assertEqual(lead.at(0)['com_velocity_world_m_s'],(0.,0.,0.))
        with self.assertRaises(ValueError):lead.at(1.51)

    def test_lead_in_then_measured_entry_gate_starts_main_reference(self):
        lead=VerticalLeadIn(self.curve,1.5,.3)
        executor=LandingExecutor(self.curve,self.geometry,self.deck,self.entry,self.limits,lead)
        first=executor.step(10.,self.observation(10.,lead.at(0)))
        self.assertEqual(first.phase,'LEAD_IN')
        for k in range(1,76):
            t=10.+k*.02
            final=executor.step(t,self.observation(t,lead.at(min(1.5,k*.02))))
        self.assertEqual(final.phase,'TRACK_REFERENCE');self.assertTrue(executor.approach_entry_report['ready'])
        self.assertAlmostEqual(executor.start_s,11.5)
        self.assertEqual(final.reference,BodyReference.from_state(self.curve.at(0)))

    def test_lead_in_end_mismatch_aborts_instead_of_extending_descent(self):
        lead=VerticalLeadIn(self.curve,1.5,.3)
        executor=LandingExecutor(self.curve,self.geometry,self.deck,self.entry,self.limits,lead)
        executor.step(10.,self.observation(10.,lead.at(0)))
        bad=self.observation(11.5,lead.at(0))
        decision=executor.step(11.5,bad)
        self.assertIsNone(decision.reference);self.assertTrue(decision.request_land)
        self.assertIn('lead_in_finished_without_matching_entry',decision.failures)

    def test_entry_does_not_silently_project_lateral_velocity_or_reset_pose(self):
        observation=self.observation(10.);motion=replace(observation.motion,velocity_at_origin_world_m_s=(0,.15,-.4))
        bad=replace(observation,motion=motion)
        executor=self.new_executor();decision=executor.step(10.,bad)
        self.assertEqual(decision.phase,'WAIT_ENTRY');self.assertIsNone(decision.reference)
        self.assertEqual(bad.motion.velocity_at_origin_world_m_s,(0,.15,-.4))
        bad=replace(observation,motion=replace(observation.motion,pose=Pose(add(observation.motion.pose.position_world_m,(.2,0,0)),observation.motion.pose.orientation_xyzw)))
        self.assertFalse(assess_entry(10.,bad,self.curve.at(0),self.curve.com_offset_body_m,self.entry)['ready'])

    def test_entry_rejects_attitude_rate_acceleration_stale_and_missing_inputs(self):
        good=self.observation(10.)
        bad_motion=replace(good.motion,pose=Pose(good.motion.pose.position_world_m,quaternion_from_rpy(0,0,.1)))
        cases=[replace(good,motion=bad_motion),replace(good,motion=replace(good.motion,angular_velocity_world_rad_s=(0,.2,0))),
            replace(good,com_acceleration_world_m_s2=(1,0,0)),replace(good,com_acceleration_world_m_s2=None),
            replace(good,sim_t_s=9.8),replace(good,sim_t_s=10.1),replace(good,armed=False),replace(good,flight_mode='LAND')]
        for case in cases:
            with self.subTest(case=case):self.assertFalse(assess_entry(10.,case,self.curve.at(0),self.curve.com_offset_body_m,self.entry)['ready'])

    def test_contact_history_is_causal_and_empty_frame_clears_support(self):
        history=ContactHistory();history.push(ContactFrame(1.,self.rear));history.push(ContactFrame(1.02));history.push(ContactFrame(1.04,self.legs))
        self.assertIsNone(history.at(.99,.06))
        self.assertEqual(history.at(1.01,.06).leg_names,self.rear)
        self.assertFalse(history.at(1.03,.06).active)
        self.assertEqual(history.at(1.04,.06).leg_names,tuple(sorted(self.legs)))
        self.assertIsNone(history.at(1.2,.06))
        with self.assertRaises(ValueError):history.push(ContactFrame(1.0))

    def test_first_collision_is_not_overwritten_between_slower_control_ticks(self):
        history=ContactHistory()
        left=(self.rear[0],)
        for frame in [ContactFrame(1.005,left),ContactFrame(1.006,self.rear),ContactFrame(1.01)]:history.push(frame)
        current,event=history.poll(1.,.06)
        self.assertIsNone(current);self.assertIsNone(event)
        current,event=history.poll(1.02,.06)
        self.assertFalse(current.active);self.assertEqual(event.sim_t_s,1.005);self.assertEqual(event.leg_names,left)
        hand=ContactHandover(self.geometry,self.deck,self.curve.terminal_boundary.aircraft,self.limits)
        hand.step(1.,self.observation(1.,self.curve.at(6.38)))
        decision=hand.step(1.02,self.observation(1.02,self.curve.at(6.4)),current,event)
        self.assertTrue(decision.stop_reference);self.assertTrue(decision.request_land)
        self.assertEqual(hand.first_contact['contact_sim_t_s'],1.005)
        self.assertEqual(hand.first_contact['legs'],left)
        self.assertIsNone(hand.stable_since)

    def test_unconsumed_collision_buffer_overflow_is_not_silently_accepted(self):
        history=ContactHistory(2);history.poll(1.,.06)
        for t in [1.01,1.02,1.03]:history.push(ContactFrame(t,self.rear))
        with self.assertRaisesRegex(ValueError,'overwritten'):history.poll(1.04,.06)

    def test_contact_before_entry_irreversibly_enters_land_handover(self):
        executor=self.new_executor()
        first=executor.step(10.,self.observation(10.),ContactFrame(9.99,self.rear))
        self.assertTrue(first.stop_reference);self.assertTrue(first.request_land)
        next_decision=executor.step(10.3,self.observation(10.3))
        self.assertNotEqual(next_decision.phase,'WAIT_ENTRY');self.assertTrue(next_decision.request_land)
        self.assertIsNone(executor.start_s);self.assertIsNone(next_decision.reference)

    def test_fresh_state_after_long_control_gap_does_not_resume_reference(self):
        executor=self.new_executor();executor.step(10.,self.observation(10.))
        decision=executor.step(10.5,self.observation(10.5,self.curve.at(.5)))
        self.assertTrue(decision.stop_reference);self.assertIn('tracking_update_gap',decision.failures)

    def test_waiting_before_entry_is_not_mistaken_for_a_tracking_gap(self):
        executor=self.new_executor()
        self.assertEqual(executor.step(9.,replace(self.observation(9.),armed=False)).phase,'WAIT_ENTRY')
        decision=executor.step(10.,self.observation(10.))
        self.assertEqual(decision.phase,'TRACK_REFERENCE');self.assertFalse(decision.failures)

    def test_goal_failure_does_not_cut_off_post_disarm_verification(self):
        executor=self.new_executor();executor.handover.fail('first_contact_leg_speed')
        executor.handover.phase='POST_DISARM'
        self.assertFalse(executor.experiment_can_finish(20.,False,16.9))
        executor.handover.phase='COMPLETE'
        self.assertTrue(executor.experiment_can_finish(20.04,False,16.9))
        self.assertFalse(executor.experiment_can_finish(20.04,True,16.9))

    def test_collision_parser_matches_full_deck_and_leg_paths(self):
        mapping={self.cfg['leg_collision_prefix']+name:name for name in self.legs}
        msg={'header':{'stamp':{'sec':1}},'contact':[
            {'collision1':{'name':self.cfg['deck_collision']},'collision2':{'name':self.cfg['leg_collision_prefix']+self.rear[0]}},
            {'collision1':{'name':'uav::body_collision'},'collision2':{'name':self.cfg['deck_collision']}},
            {'collision1':{'name':'ground'},'collision2':{'name':self.cfg['leg_collision_prefix']+self.legs[0]}}]}
        frame=parse_deck_contacts(msg,self.cfg['deck_collision'],mapping,self.cfg['uav_model_prefix'])
        self.assertEqual(frame.leg_names,(self.rear[0],));self.assertEqual(frame.unexpected_uav_collisions,('uav::body_collision',))
        self.assertFalse(parse_deck_contacts({'header':{'stamp':{}}},self.cfg['deck_collision'],mapping,self.cfg['uav_model_prefix']).active)

    def test_first_contact_immediately_removes_pva_and_does_not_force_disarm(self):
        hand,decision=self.contact_handover()
        self.assertTrue(decision.stop_reference);self.assertIsNone(decision.reference)
        self.assertTrue(decision.request_land);self.assertEqual(decision.pending_land_velocity_world_m_s,(0.,0.,0.))
        self.assertIsNotNone(hand.first_contact);self.assertLess(hand.first_contact['observation_sim_t_s'],hand.first_contact['contact_sim_t_s'])
        self.assertTrue(all(hand.first_contact['checks'].values()),hand.first_contact)
        native=hand.step(16.42,self.supported(16.42),ContactFrame(16.42,self.legs))
        self.assertIsNone(native.pending_land_velocity_world_m_s);self.assertFalse(native.request_land)

    def test_bounce_never_resumes_airborne_reference_and_resets_support_dwell(self):
        hand,_=self.contact_handover()
        for k in range(1,11):
            t=16.4+k*.02;hand.step(t,self.supported(t),ContactFrame(t,self.legs))
        t=16.62;decision=hand.step(t,self.supported(t),ContactFrame(t))
        self.assertTrue(decision.stop_reference);self.assertIsNone(decision.reference);self.assertIsNone(hand.stable_since)
        self.assertEqual(hand.phase,'LAND_SETTLE')

    def test_rear_pair_alone_does_not_satisfy_support_gate(self):
        hand,_=self.contact_handover()
        for k in range(1,31):
            t=16.4+k*.02;decision=hand.step(t,self.supported(t),ContactFrame(t,self.rear))
        self.assertEqual(decision.phase,'LAND_SETTLE');self.assertFalse(decision.complete)

    def test_normal_disarm_then_post_disarm_support_required(self):
        hand,_=self.contact_handover()
        for k in range(1,23):
            t=16.4+k*.02;decision=hand.step(t,self.supported(t),ContactFrame(t,self.legs))
        self.assertEqual(decision.phase,'WAIT_DISARM')
        for k in range(23,175):
            t=16.4+k*.02;decision=hand.step(t,self.supported(t,armed=False),ContactFrame(t,self.legs))
        self.assertTrue(decision.complete,decision)
        self.assertIsNone(decision.reference)

    def test_premature_disarm_and_post_disarm_support_loss_fail(self):
        hand,_=self.contact_handover()
        bad=hand.step(16.42,self.supported(16.42,armed=False),ContactFrame(16.42,self.legs))
        self.assertFalse(bad.complete);self.assertIn('disarmed_before_support_gate',bad.failures)
        hand,_=self.contact_handover()
        for k in range(1,23):
            t=16.4+k*.02;hand.step(t,self.supported(t),ContactFrame(t,self.legs))
        hand.step(16.86,self.supported(16.86,armed=False),ContactFrame(16.86,self.legs))
        bad=hand.step(16.88,self.supported(16.88,armed=False),ContactFrame(16.88))
        self.assertIn('post_disarm_support_lost',bad.failures);self.assertFalse(bad.complete)

    def test_reference_expiry_has_no_below_deck_continuation(self):
        executor=self.new_executor();executor.step(10.,self.observation(10.))
        decision=executor.step(16.41,self.observation(16.41,self.curve.at(6.4)))
        self.assertIsNone(decision.reference);self.assertTrue(decision.request_land)
        self.assertIn('reference_ended_without_contact',decision.failures)
        later=executor.step(16.45,self.observation(16.45,self.curve.at(6.4)))
        self.assertTrue(later.stop_reference);self.assertIsNone(later.reference)

    def test_missing_land_ack_and_stale_tracking_are_failures(self):
        hand,_=self.contact_handover()
        result=hand.step(18.5,self.observation(18.5,self.curve.at(6.4)))
        self.assertIn('land_mode_ack_timeout',result.failures);self.assertTrue(result.request_land)
        executor=self.new_executor();executor.step(10.,self.observation(10.))
        result=executor.step(10.2,self.observation(10.01))
        self.assertTrue(result.stop_reference);self.assertIn('tracking_observation_stale',result.failures)

    def test_bad_or_early_collision_stops_tracking_without_success(self):
        executor=self.new_executor();executor.step(10.,self.observation(10.))
        decision=executor.step(10.02,self.observation(10.02),ContactFrame(10.015,(),('uav::body_collision',)))
        self.assertTrue(decision.stop_reference);self.assertIsNone(decision.reference)
        self.assertIn('first_contact_known_legs_only',decision.failures)
        self.assertIn('first_contact_contact_time',decision.failures)

    def test_body_reference_mapping_and_contact_dispatch_clears_acceleration(self):
        calls=[];lands=[]
        transport=SimpleNamespace(mav=SimpleNamespace(set_position_target_local_ned_send=lambda *a:calls.append(a)))
        alignment=LocalNedAlignment((10,20,30))
        ref=BodyReference((2,3,4),(1,2,-.08),(-.34,0,0),0.)
        dispatch(ExecutionDecision('TRACK_REFERENCE',False,ref),transport,lambda:lands.append(True),alignment,1.)
        self.assertEqual(calls[0][4],2048)
        self.assertEqual(calls[0][5:8],(13,22,26))  # no old +0.195 offset
        self.assertEqual(calls[0][8:11],(2,1,.08));self.assertEqual(calls[0][11:14],(0,-.34,0))
        _,contact=self.contact_handover();dispatch(contact,transport,lambda:lands.append(True),alignment,2.)
        self.assertEqual(len(lands),1);self.assertEqual(calls[-1][4],2503)
        self.assertEqual(calls[-1][11:14],(0.,0.,0.))
        hand,_=self.contact_handover();native=hand.step(16.42,self.supported(16.42),ContactFrame(16.42,self.legs))
        count=len(calls);dispatch(native,transport,lambda:lands.append(True),alignment,2.02)
        self.assertEqual(len(calls),count)
        with self.assertRaises(ValueError):dispatch(replace(contact,reference=ref),transport,lambda:None,alignment,3.)


if __name__=='__main__':unittest.main()
