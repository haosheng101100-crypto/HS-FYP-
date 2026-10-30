"""Postflight response measurement, never a guaranteed arrest capability model."""
from pathlib import Path
import json,sys,math,hashlib,bisect
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R),str(R/'tools'),str(R/'work/plot_dependencies')]
from analyze_moving_rapid import raw_data
from landing.contact_geometry import Pose,RigidMotion,Deck,evaluate_airframe
from landing.reference_loader import load_verified_reference

def analyze(run):
 res=run/'results';s=json.loads((res/'execution_status.json').read_text())
 assert s.get('abort_return_completed') and not s['external_failures']
 decision=s['unified_event_decision'] if 'unified_event_decision' in s else s['policy_decision']
 epoch=decision['decision_sim_t_s'];end=s['return_reference']['start_sim_t_s']
 raw,_=raw_data(run);car=raw['car_pose'];times=[t for t,p in car];_,g,_,_=load_verified_reference(R)
 measured=[]
 for t,p in raw['uav_pose']:
  if not epoch<=t<=end:continue
  i=bisect.bisect_right(times,t)-1;ct,cp=car[i]
  if ct<t:
   nt,np=car[i+1];assert nt-ct<.06;f=(t-ct)/(nt-ct)
   cp=Pose(tuple(a+(b-a)*f for a,b in zip(cp.position_world_m,np.position_world_m)),cp.orientation_xyzw)
  d=Deck(RigidMotion.stationary(Pose(cp.transform((0,0,.8)),cp.orientation_xyzw)),(1.,.7))
  gap=min(f.clearance_m for f in evaluate_airframe(g,RigidMotion.stationary(p),d))
  measured.append((t,p.position_world_m[2],gap))
 assert len(measured)>50
 low=min(measured,key=lambda r:r[2]);bodylow=min(measured,key=lambda r:r[1])
 metrics=decision.get('metrics',decision)
 initial_gap=metrics.get('lowest_foot_gap_m');down=metrics.get('downward_speed_m_s')
 observed_loss=max(0.,initial_gap-low[2]);prediction=.2*down+down*down/(2*.8)
 checks={'complete_raw_abort_window':measured[-1][0]-measured[0][0]>2.,
         'nominal_design_loss_covers_observed_foot_loss':observed_loss<=prediction+.005,
         'actual_retained_gap_above_0_3_m':low[2]>=.3,
         'arrest_loss_not_claimed_as_identified_guarantee':True}
 return {'passed':all(checks.values()),'checks':checks,'decision_sim_t_s':epoch,
         'initial_downward_speed_m_s':down,'initial_foot_gap_m':initial_gap,
         'minimum_actual_foot_gap_m':low[2],'observed_foot_gap_loss_m':observed_loss,
         'decision_to_minimum_foot_gap_s':low[0]-epoch,'decision_to_minimum_body_height_s':bodylow[0]-epoch,
         'declared_design_loss_prediction_m':prediction,'prediction_minus_observed_loss_m':prediction-observed_loss,
         'scope':'Observed response of this exact frozen controller/configuration and initial state; no lower-bound acceleration guarantee, interpolation or hardware claim',
         'raw_samples':len(measured),'raw_log_sha256':hashlib.sha256((res/'model_states.jsonl').read_bytes()).hexdigest()}

if __name__=='__main__':
 run=Path(sys.argv[1]);out=run/'response_analysis';out.mkdir(exist_ok=True);target=out/'verification.json';assert not target.exists()
 result=analyze(run);target.write_text(json.dumps(result,indent=2));print(json.dumps(result))
