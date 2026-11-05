"""Regression of M18 command discontinuity, not an escape guarantee."""
import ast,unittest
from pathlib import Path
from types import SimpleNamespace
from dataclasses import asdict
import numpy as np
from landing.receding_live import RecoveryReference
from landing.execution import AircraftObservation,BodyReference
from landing.contact_geometry import Pose,RigidMotion
ROOT=Path(__file__).resolve().parents[1]

class RecoveryHandoffTests(unittest.TestCase):
 def obs(self):
  return AircraftObservation(55.08,RigidMotion(Pose((5.3058,2.3109,1.02565),(0,0,0,1)),(.15826,.17574,-.146),(0,0,0)),True,'GUIDED',(-.2,0,.27))
 def reference(self):return BodyReference((5.28319,2.29747,1.057299),(.130175,.176682,-.156924),(-.199461,.001006,.272551),0.)
 def test_same_tick_handoff_preserves_actual_sent_pva(self):
  obs=self.obs();initial=self.reference();r=RecoveryReference(obs,initial_reference=initial,start_s=55.08)
  ref,done=r.step(55.08,obs)
  self.assertEqual(ref,initial);self.assertFalse(done)
 def test_next_tick_keeps_existing_upward_braking_and_position_continuity(self):
  obs=self.obs();initial=self.reference();r=RecoveryReference(obs,initial_reference=initial,start_s=55.08);r.step(55.08,obs)
  ref,_=r.step(55.12,obs);a=np.array(ref.acceleration_world_m_s2);p=np.array(initial.position_world_m);v=np.array(initial.velocity_world_m_s);prior=np.array(initial.acceleration_world_m_s2)
  self.assertGreater(a[2],.27);self.assertLessEqual(np.linalg.norm(a),1.5+1e-9);self.assertLessEqual(np.linalg.norm((a-prior)/.04),3.+1e-8)
  np.testing.assert_allclose(ref.position_world_m,p+v*.04+.5*prior*.04**2,atol=1e-9)
 def test_native_backend_passes_sent_reference_and_tick_to_recovery(self):
  tree=ast.parse((ROOT/'simulation/operator_package/operator_multicondition_backend.py').read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef));method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='start_recovery');ns={'RecoveryReference':RecoveryReference};exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'actual recovery backend','exec'),ns)
  f=SimpleNamespace(last_sent_reference=self.reference(),hold_reference=BodyReference((0,0,4),(0,0,0),(0,0,0),0),reference_session=SimpleNamespace(cancel=lambda reason:None),result={},event=lambda msg:None)
  ns['start_recovery'](f,55.08,self.obs(),'REGRESSION');ref,_=f.recovery.step(55.08,self.obs());self.assertEqual(ref,self.reference())
 def test_recovery_does_not_reseed_from_each_new_measurement(self):
  obs=self.obs();r=RecoveryReference(obs,initial_reference=self.reference(),start_s=55.08);r.step(55.08,obs);a,_=r.step(55.12,obs);b,_=r.step(55.16,self.obs());self.assertLess(np.linalg.norm(np.array(b.position_world_m)-a.position_world_m),.04)
