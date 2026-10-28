"""Exercise recovery without importing Gazebo or creating a MAVLink link."""
import ast,unittest
from pathlib import Path

class RecoveryTests(unittest.TestCase):
    def method(self):
        source=(Path(__file__).resolve().parents[1]/'tools/moving_approach_demo.py').read_text(encoding='utf-8')
        tree=ast.parse(source)
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LiveApproach')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='emergency')
        scope={};exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'bridge_recovery','exec'),scope)
        return scope['emergency']
    def test_interrupt_restores_bounded_land_wait(self):
        class Fake:
            running=False;enabled=True;external_failures=[];result={};is_armed=True;requested=False
            def event(self,text):pass
            def armed(self):return self.is_armed
            def request_land(self):self.requested=True
            def wait(self,predicate,timeout,label):
                assert self.running and self.requested and timeout==180
                self.is_armed=False;assert predicate()
        f=Fake();self.method()(f,'interrupted')
        self.assertFalse(f.enabled);self.assertTrue(f.result['normal_disarm_observed']);self.assertFalse(f.result['completed'])
    def test_failed_recovery_stays_failed(self):
        class Fake:
            running=False;enabled=True;external_failures=[];result={}
            def event(self,text):pass
            def armed(self):return True
            def request_land(self):pass
            def wait(self,*args):raise RuntimeError('no ack')
        f=Fake();self.method()(f,'failure')
        self.assertFalse(f.result['normal_disarm_observed']);self.assertIn('recovery: no ack',f.external_failures)
