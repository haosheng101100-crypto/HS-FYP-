"""Unarmed computation-time check before submitting a new selector to physics."""
from pathlib import Path
import sys,time,json
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R))
from landing.contact_geometry import Pose,RigidMotion,Deck
from landing.execution import AircraftObservation
from landing.reference_loader import load_verified_reference
from landing.braking_candidates import state_selected_braking_plan
_,g,_,profile=load_verified_reference(R)
o=AircraftObservation(10.,RigidMotion(Pose((45,0,2.995),(0,0,0,1)),(5,0,-.63),(0,0,0)),True,'GUIDED',(0,0,0))
d=Deck(RigidMotion(Pose((45.03,0,.8),(0,0,0,1)),(4.95,0,0),(0,0,0)),(1,.7))
times=[];durations=[]
for _ in range(8):
 at=time.perf_counter();curve,_,plan=state_selected_braking_plan(g,profile,o,d,(-2.5,0,0));times.append(time.perf_counter()-at);durations.append(curve.T)
result={'maximum_wall_s':max(times),'wall_times_s':times,'selected_durations_s':durations,'scope':'Unarmed current-host computation check, not a real-time scheduling guarantee'}
result['selected_reference_shape']=plan['candidate_search'].get('selected_braking_shape')
result['maximum_nominal_pitch_rate_deg_s']=plan['candidate_search'].get('maximum_nominal_pitch_rate_deg_s')
print(json.dumps(result))
