"""Execute actual backend policy methods without importing Gazebo or arming."""
import ast
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from dataclasses import replace,asdict
from landing.planar_landing import PlanarLandingSupervisor
from landing.receding_live import DeckAlignmentReference,FilePlannerClient,AsyncLandingReference,pack_plan,atomic_json
from landing.receding_landing import FlightState,PlatformState
from landing.execution import BodyReference

ROOT=Path(__file__).resolve().parents[1]


def backend_methods():
    tree=ast.parse((ROOT/'simulation/operator_package/operator_backend.py').read_text(encoding='utf-8-sig'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('update_planar_policy','start_planar_follow')]
    namespace=dict(FlightState=FlightState,DeckAlignmentReference=DeckAlignmentReference,asdict=asdict)
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods,type_ignores=[])),'backend policy','exec'),namespace)
    return namespace


class PlanarRuntimeTests(unittest.TestCase):
    def test_high_change_abandons_old_worker_and_preserves_current_pva(self):
        methods=backend_methods()
        self.assertIn('update_planar_policy',methods)
        self.assertIn('start_planar_follow',methods)
        with tempfile.TemporaryDirectory() as temp:
            client=FilePlannerClient(temp)
            command=BodyReference((1,0,3),(.2,.1,-.1),(.05,.03,.1),0.)
            old=AsyncLandingReference(client,(0,0,0),command)
            platform=PlatformState(0.,(1,0,.8),(.2,0,0),(0,.4,0))
            state=FlightState(0.,command.position_world_m,command.velocity_world_m_s,command.acceleration_world_m_s2)
            old.request=client.submit(state,platform,(4.,),None)
            fake=SimpleNamespace(planar_supervisor=PlanarLandingSupervisor(),recovery=None,
                executor=SimpleNamespace(handover=SimpleNamespace(started=None)),alignment_reference=None,
                scene_gap=2.,curve=SimpleNamespace(com_offset_body_m=(0,0,0)),reference_session=old,
                last_sent_reference=command,hold_reference=command,terminal_prediction=None,
                live_cfg={'moving_alignment':{'minimum_leg_gap_m':1.5,'dwell_s':.6}})
            fake.make_reference_session=lambda ref:AsyncLandingReference(client,(0,0,0),ref)
            fake.start_planar_follow=lambda *args:methods['start_planar_follow'](fake,*args)
            fake.start_recovery=lambda *args:self.fail('High clearance should reacquire before returning')
            obs=SimpleNamespace(sim_t_s=0.,motion=SimpleNamespace(pose=SimpleNamespace(position_world_m=command.position_world_m,rotate=lambda v:v),velocity_at_origin_world_m_s=command.velocity_world_m_s,angular_velocity_world_rad_s=(0,0,0)),com_acceleration_world_m_s2=command.acceleration_world_m_s2)
            report=methods['update_planar_policy'](fake,0.,obs,platform,False)
            self.assertEqual(report['action'],'REACQUIRE')
            self.assertIsNone(client.outstanding)
            self.assertIsNot(fake.reference_session,old)
            self.assertEqual(fake.alignment_reference.step(0.,platform,(0,0,0)),command)

    def test_real_contact_does_not_restart_alignment(self):
        methods=backend_methods();self.assertIn('update_planar_policy',methods)
        fake=SimpleNamespace(planar_supervisor=PlanarLandingSupervisor(),recovery=None,
            executor=SimpleNamespace(handover=SimpleNamespace(started=None)),alignment_reference=None,
            curve=SimpleNamespace(com_offset_body_m=(0,0,0)),scene_gap=0.,last_sent_reference=None)
        fake.start_planar_follow=lambda *args:self.fail('Contact must take precedence')
        obs=SimpleNamespace(sim_t_s=0.,motion=SimpleNamespace(pose=SimpleNamespace(position_world_m=(0,0,1),rotate=lambda v:v),velocity_at_origin_world_m_s=(0,0,0),angular_velocity_world_rad_s=(0,0,0)),com_acceleration_world_m_s2=(0,0,0))
        report=methods['update_planar_policy'](fake,0.,obs,PlatformState(0,(0,0,.8),(0,0,0),(1,0,0)),True)
        self.assertEqual(report['action'],'CONTACT')


if __name__=='__main__':unittest.main()
