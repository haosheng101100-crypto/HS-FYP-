import unittest, math
from pathlib import Path
from landing.contact_geometry import Pose,RigidMotion,Deck,add,scale,sub,norm,evaluate_airframe
from landing.reference_loader import load_verified_reference
from landing.moving_platform import plan_interception,translate_deck,MovingContactHandover,MovingLandingExecutor
from landing.execution import AircraftObservation,HandoverLimits,EntryLimits
import json

ROOT=Path(__file__).resolve().parents[1]

class MovingPlatformTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _,cls.geometry,_,cls.profile=load_verified_reference(ROOT)
        cls.deck=Deck(RigidMotion(Pose((3.6,0,.8),(0,0,0,1)),(.3,0,0),(0,0,0)),(1,.7))
        cls.curve,cls.ideal,cls.report=plan_interception(ROOT,cls.geometry,cls.profile,cls.deck)

    def test_intercept_prediction_and_relative_velocity(self):
        c=self.curve;d=translate_deck(self.deck,1.5+c.T);s=c.at(c.T)
        feet=evaluate_airframe(self.geometry,s['motion'],d)
        active=[f for f in feet if abs(f.clearance_m)<1e-8]
        self.assertEqual(len(active),2)
        self.assertAlmostEqual(sum(f.point_world_m[0] for f in active)/2,d.motion.pose.position_world_m[0],places=9)
        for f in active:
            self.assertLess(f.tangential_speed_upper_bound_m_s,1e-7)
            self.assertAlmostEqual(f.closing_speed_m_s,.08,places=8)
        self.assertAlmostEqual(s['com_velocity_world_m_s'][0],.3,places=10)
        self.assertAlmostEqual(math.degrees(s['nose_up_rad']),2,places=9)
        self.assertTrue(self.report['checks']['passed'])

    def test_initial_world_hover_and_acceleration_then_braking(self):
        c=self.curve;s=c.at(0)
        self.assertAlmostEqual(s['com_velocity_world_m_s'][0],0)
        samples=[c.at(c.T*i/100) for i in range(101)]
        self.assertGreater(max(x['com_velocity_world_m_s'][0] for x in samples),.6)
        self.assertGreater(max(x['com_acceleration_world_m_s2'][0] for x in samples),.1)
        self.assertLess(min(x['com_acceleration_world_m_s2'][0] for x in samples),-.2)
        self.assertLess(self.report['checks']['relative_low_speed_lead_s'],.7)
        with self.assertRaises(ValueError):c.at(c.T+.01)

    def test_pending_land_matches_vehicle_and_support_uses_relative_speed(self):
        limits=HandoverLimits(**json.loads((ROOT/'config/experiments/execution_v1.json').read_text())['handover_limits'])
        terminal=translate_deck(self.deck,1.5+self.curve.T)
        h=MovingContactHandover(self.geometry,self.curve.terminal_boundary,terminal,limits,lambda t:terminal)
        h.begin(1.)
        end=self.curve.terminal_boundary.aircraft
        o=AircraftObservation(1.,RigidMotion(end.pose,(.3,0,0),(0,0,0)),True,'GUIDED')
        decision=h.step(1.,o)
        self.assertEqual(decision.pending_land_velocity_world_m_s,(.3,0,0))
        self.assertTrue(h.last_support_checks['speed'])
        stopped=AircraftObservation(1.04,RigidMotion(o.motion.pose,(0,0,0),(0,0,0)),True,'LAND')
        h.step(1.04,stopped)
        self.assertFalse(h.last_support_checks['speed'])

    def executor_at_terminal(self,gap):
        cfg=json.loads((ROOT/'config/experiments/execution_v1.json').read_text());limits=HandoverLimits(**cfg['handover_limits'])
        deck=translate_deck(self.deck,1.5+self.curve.T)
        static=Deck(RigidMotion.stationary(deck.motion.pose),deck.half_extents_m)
        ex=MovingLandingExecutor(self.curve,self.geometry,static,EntryLimits(**cfg['entry_limits']),limits)
        ex.handover=MovingContactHandover(self.geometry,self.curve.terminal_boundary,deck,limits,lambda t:deck)
        ex.start_s=1.;t=1.+self.curve.T;ex.last_t=t-.04
        end=self.curve.terminal_boundary.aircraft
        obs=AircraftObservation(t,RigidMotion(Pose(add(end.pose.position_world_m,(0,0,gap)),end.pose.orientation_xyzw),end.velocity_at_origin_world_m_s,(0,0,0)),True,'GUIDED')
        return ex,t,obs

    def test_guarded_terminal_gap_finishes_in_land_without_extrapolation(self):
        ex,t,o=self.executor_at_terminal(.005);d=ex.step(t,o)
        self.assertTrue(d.request_land);self.assertTrue(d.stop_reference);self.assertIsNone(d.reference)
        self.assertFalse(d.failures);self.assertIsNotNone(ex.native_contact_completion)
        from dataclasses import replace
        d=ex.step(t+.3,replace(o,sim_t_s=t+.3,flight_mode='LAND'))
        self.assertIn('native_contact_completion_timeout',d.failures)

    def test_large_terminal_gap_retains_expiry_failure(self):
        ex,t,o=self.executor_at_terminal(.02);d=ex.step(t,o)
        self.assertIn('reference_ended_without_contact',d.failures)
        self.assertIsNone(ex.native_contact_completion)

if __name__=='__main__':unittest.main()
