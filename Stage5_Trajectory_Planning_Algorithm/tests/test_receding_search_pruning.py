"""Search scheduling: preserve the finite-candidate optimum and safety filtering."""
import math
from pathlib import Path
import sys,unittest
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'work/receding_dependencies'),str(ROOT)]
from landing.receding_landing import BezierLandingPlanner

class SearchPruningTests(unittest.TestCase):
    def planner(self,costs):
        p=object.__new__(BezierLandingPlanner);p.calls=[]
        def solve(state,platform,now,duration,uncertainty,tracking_envelope):
            p.calls.append(duration);cost=costs[duration]
            plan=None if cost is None else SimpleNamespace(cost=cost,duration=duration,arrival=now+duration)
            return plan,dict(duration_s=duration,status='infeasible' if plan is None else 'ACCEPTED')
        p.solve=solve;return p

    def test_dominated_candidate_needs_no_solve_and_is_not_called_infeasible(self):
        p=self.planner({8:1.05,10:.33,12:.07})
        plan,report=p.search(None,None,0.,(8,10,12))
        self.assertEqual(plan.duration,8);self.assertEqual(p.calls,[8,10])
        skipped=report['candidates'][-1]
        self.assertEqual(skipped['status'],'SCORE_BOUND_PRUNED')
        self.assertEqual(skipped['score_lower_bound'],6.)
        self.assertEqual(report['solved_candidates'],2)

    def test_exhaustive_mode_has_same_choice_and_score(self):
        costs={4:1.5,5:.46,6:.18}
        p=self.planner(costs);q=self.planner(costs)
        a,ar=p.search(None,None,0.,costs)
        b,br=q.search(None,None,0.,costs,prune_score_dominated=False)
        self.assertEqual(a.duration,b.duration);self.assertEqual(ar['selected_score'],br['selected_score'])
        self.assertEqual(q.calls,[4,5,6])

    def test_contact_time_penalty_is_in_bound_and_equality_is_solved(self):
        p=self.planner({2:0.,3:0.,4:0.})
        plan,report=p.search(None,None,10.,(2,3,4),previous_arrival=14.)
        self.assertEqual(plan.duration,4.)
        p=self.planner({2:.5,3:0.})
        plan,_=p.search(None,None,0.,(2,3))
        self.assertEqual(p.calls,[2,3]);self.assertEqual(plan.duration,2.)

    def test_no_incumbent_solves_every_candidate_and_forwards_feedback(self):
        p=self.planner({2:None,4:None,6:None})
        plan,report=p.search(None,None,0.,(6,2,4,2))
        self.assertIsNone(plan);self.assertEqual(p.calls,[2,4,6])
        self.assertEqual(report['solved_candidates'],3)

    def test_invalid_duration_cannot_be_hidden_by_pruning(self):
        p=self.planner({2:0.,20:0.})
        with self.assertRaises(ValueError):p.search(None,None,0.,(2,20))

    def test_nan_solver_cost_never_becomes_an_incumbent(self):
        p=self.planner({2:math.nan,3:.2,9:0.})
        plan,report=p.search(None,None,0.,(2,3,9))
        self.assertEqual(plan.duration,3.)
        self.assertEqual(report['candidates'][0]['status'],'NONFINITE_SCORE_REJECTED')

    def test_random_candidate_sets_match_exhaustive_oracle(self):
        import random
        rng=random.Random(2910)
        for _ in range(150):
            costs={d:(None if rng.random()<.3 else rng.random()*15.) for d in (1.,2.,3.,5.,8.,12.)}
            anchor=rng.choice((None,3.,7.,12.))
            a=self.planner(costs);b=self.planner(costs)
            pa,ra=a.search(None,None,0.,costs,anchor)
            pb,rb=b.search(None,None,0.,costs,anchor,prune_score_dominated=False)
            self.assertEqual(pa.duration if pa else None,pb.duration if pb else None)
            self.assertEqual(ra['selected_score'],rb['selected_score'])

if __name__=='__main__':unittest.main()
