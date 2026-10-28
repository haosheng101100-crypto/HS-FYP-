from pathlib import Path
import sys,json,unittest
from dataclasses import replace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'work/receding_dependencies'))
import numpy as np
from landing.receding_landing import FlightState,PlatformState,PlannerLimits,BezierLandingPlanner
from landing.contact_geometry import AirframeGeometry
from landing.swept_envelope import SweptEnvelope

class DirectionalLandingTests(unittest.TestCase):
    def setUp(self):
        self.env=SweptEnvelope.load(ROOT/'config/airframes/iris_swept_envelope_v1.json')
        self.cfg=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())
        profile=json.loads((ROOT/'config/airframes/iris_fixed_gimbal_sim.json').read_text())
        self.offset=profile['nominal_com_in_body_m']
        self.geometry=AirframeGeometry.load(ROOT/'config/airframes/iris_fixed_gimbal_sim.json')

    def test_directional_mode_recovers_frozen_short_turn_candidate(self):
        old=ROOT/'outputs/uncertainty_landing_20261024/batch_066/cycle_002/runtime/receding_planner/response_000016.json'
        data=json.loads(old.read_text());req=data['request']
        self.assertEqual(data['report']['candidates'][0]['status'],'infeasible')
        limits=PlannerLimits(**self.cfg['planner_limits'])
        limits=replace(limits,acceleration=1.2,entry_acceleration_capture_m_s2=1.5,directional_corridor=True)
        planner=BezierLandingPlanner(self.geometry,self.offset,limits,self.env)
        from landing.prediction_uncertainty import ForecastTube
        plan,report=planner.solve(FlightState(**req['state']),PlatformState(**req['platform']),req['now'],req['durations'][0],ForecastTube(**req['uncertainty']))
        self.assertIsNotNone(plan,report)
        self.assertTrue(plan.audit['passed'])
        self.assertEqual(plan.audit['corridor_model'],'rotating_rectangle_support_bound')
        platform=PlatformState(**plan.audit['platform_prediction_input'])
        for t in np.linspace(0,plan.duration,401):
            # All of this short horizon is inside the declared full-coverage phase.
            self.assertGreaterEqual(self.env.clearance(plan.motion(float(t)),platform.predict(float(t)).deck()),limits.margin-2e-5)
        tampered=replace(plan,points=plan.points.copy())
        tampered.points[1,3,0]+=.08
        self.assertFalse(planner.audit(tampered,FlightState(**req['state']),platform,0.)['passed'])

    def test_heading_extremum_is_covered_not_only_segment_endpoints(self):
        p=PlatformState(0,(0,0,.8),(0,0,0),(0,0,0),yaw=.1,yaw_rate=.1,yaw_acceleration=-.2,half_extents=(.34,.4082))
        facets=self.env.corridor_facets(p,1.,self.offset,.03)
        for f in facets:
            u=np.array(f['axis']);b=f['heading_chord']
            for t in np.linspace(0,1,251):
                yaw=p.predict(float(t)).yaw;local=f['local_axis']
                actual=np.array((np.cos(yaw)*local[0]-np.sin(yaw)*local[1],np.sin(yaw)*local[0]+np.cos(yaw)*local[1],0))
                self.assertLessEqual(np.linalg.norm(actual-u),b+1e-12)
                self.assertLessEqual(self.env.support(actual,self.offset),f['support_m']+b*f['body_radius_m']+1e-12)

if __name__=='__main__':unittest.main()
