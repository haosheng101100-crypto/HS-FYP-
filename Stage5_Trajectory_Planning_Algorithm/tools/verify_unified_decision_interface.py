"""Recompute the common decision at archived measured events; no new flight."""
from pathlib import Path
import json, sys, hashlib
R=Path(__file__).resolve().parents[1]
if str(R) not in sys.path:sys.path.append(str(R))
from landing.contact_geometry import Pose,RigidMotion,Deck
from landing.execution import AircraftObservation
from landing.reference_loader import load_verified_reference
from landing.descent_decision import assess_interrupted_descent

def motion(d):
    return RigidMotion(Pose(**d['pose']),d['velocity_at_origin_world_m_s'],d['angular_velocity_world_rad_s'])

def observation(d):
    return AircraftObservation(d['sim_t_s'],motion(d['motion']),d['armed'],d['flight_mode'],d.get('com_acceleration_world_m_s2'))

def deck(d):
    return Deck(motion(d['motion']),d['half_extents_m'])

if __name__=='__main__':
    _,g,_,_=load_verified_reference(R)
    out=R/'outputs/model_audit/step28/interface_replay.json'
    assert not out.exists()
    results=[]
    for case,step,index in [('braking',26,'005'),('acceleration',27,'003')]:
        source=R/f'outputs/model_audit/step{step}/run_{index}/results'
        status=json.loads((source/'execution_status.json').read_text())
        if case=='braking':
            at=status['measured_braking_detection']['sim_t_s']
            row=next(json.loads(l) for l in (source/'execution_feedback.jsonl').read_text().splitlines() if json.loads(l)['sim_t_s']==at)
            o=observation(row['observation']);d=deck(row['measured_deck']);a=row['shared_acceleration_m_s2'];candidate=status['direct_braking_plan']
        else:
            o=observation(status['decision_observation']);d=deck(status['decision_deck']);a=status['policy_decision']['measured_acceleration_m_s2'];at=status['policy_decision']['decision_sim_t_s'];candidate=None
        decision=assess_interrupted_descent(g,o,d,a,at,True,candidate)
        expected='EXECUTE_BRAKING_DESCENT_CANDIDATE' if case=='braking' else 'ABORT_DESCENT_VALIDATE_RETURN_CORRIDOR'
        results.append({'case':case,'source':str(source.relative_to(R)),'status_sha256':hashlib.sha256((source/'execution_status.json').read_bytes()).hexdigest(),
                        'passed':decision['action']==expected,'decision':decision})
    checks={'both_archived_measured_events_recomputed':len(results)==2,
            'same_router_selects_different_actions':all(r['passed'] for r in results),
            'no_new_physics_or_safe_region_claim':True}
    result={'passed':all(checks.values()),'checks':checks,'cases':results,'new_physical_runs':0}
    out.write_text(json.dumps(result,indent=2))
    print(json.dumps({'passed':result['passed'],'actions':[{k:r[k] for k in ['case','passed']}|{'action':r['decision']['action'],'checks':r['decision']['checks']} for r in results]}))
