import unittest, importlib.util
from pathlib import Path
P=Path(__file__).resolve().parents[1]/'search_benchmark.py'
class PaperBenchmarkTests(unittest.TestCase):
    def test_runner_exists(self):
        self.assertTrue(P.exists(), 'paper benchmark has not been implemented')
    def load(self):
        s=importlib.util.spec_from_file_location('paper_benchmark',P);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
    def test_comparison_rejects_false_audit_and_nonfinite_score(self):
        m=self.load();a=dict(available=True,duration=8.,score=4.,audit_passed=True)
        self.assertTrue(m.equivalent(a,a))
        self.assertFalse(m.equivalent(a,dict(a,audit_passed=False)))
        self.assertFalse(m.equivalent(a,dict(a,score=float('nan'))))
        self.assertFalse(m.equivalent(a,dict(a,duration=10.)))
    def test_no_solution_is_not_successful_landing(self):
        m=self.load();a=dict(available=False,duration=None,score=None,audit_passed=None)
        self.assertTrue(m.equivalent(a,a))
        self.assertFalse(m.equivalent(a,dict(a,available=True)))
    def test_deduplicates_requests_but_keeps_changed_input(self):
        m=self.load();self.assertEqual(m.request_hash({'x':1,'y':2}),m.request_hash({'y':2,'x':1}))
        self.assertNotEqual(m.request_hash({'x':1}),m.request_hash({'x':2}))
if __name__=='__main__': unittest.main()
