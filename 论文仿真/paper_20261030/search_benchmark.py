"""Paired replay of recorded planner requests. Does not operate a vehicle."""
import argparse, hashlib, json, math, platform, sys, time
from pathlib import Path
from dataclasses import replace

def request_hash(request):
    return hashlib.sha256(json.dumps(request,sort_keys=True,allow_nan=False).encode()).hexdigest()

def equivalent(a,b):
    if a['available'] != b['available']: return False
    if not a['available']: return True
    return (a['audit_passed'] is True and b['audit_passed'] is True
        and a['duration']==b['duration']
        and all(math.isfinite(x) for x in (a['score'],b['score']))
        and math.isclose(a['score'],b['score'],rel_tol=1e-4,abs_tol=1e-6))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--inputs',type=Path,required=True,help='Read-only directory containing response_*.json with request objects')
    ap.add_argument('--config',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True,help='New directory only')
    ap.add_argument('--repeats',type=int,default=3)
    ap.add_argument('--limit',type=int,default=0,help='0 means all unique requests')
    ap.add_argument('--expected-planner-sha',required=True)
    a=ap.parse_args()
    if a.repeats<1 or a.limit<0: ap.error('repeats >= 1; limit >= 0')
    root=Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(root))
    from landing.receding_landing import FlightState,PlatformState,PlannerLimits,BezierLandingPlanner
    from landing.contact_geometry import AirframeGeometry
    from landing.swept_envelope import SweptEnvelope
    from landing.prediction_uncertainty import ForecastTube
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    planner_path=root/'landing/receding_landing.py'
    if sha(planner_path)!=a.expected_planner_sha: raise ValueError('Planner hash mismatch; do not silently mix algorithm versions')
    cfg=json.loads(a.config.read_text(encoding='utf-8'))
    profile_path=root/'config/airframes/iris_fixed_gimbal_sim.json'
    profile=json.loads(profile_path.read_text(encoding='utf-8'))
    envelope=root/cfg['envelope_profile']
    requests={}
    for p in sorted(a.inputs.rglob('response_*.json')):
        d=json.loads(p.read_text(encoding='utf-8'));req=d['request'];key=request_hash(req)
        requests.setdefault(key,dict(request=req,sources=[]))['sources'].append(dict(path=str(p.resolve()),sha256=sha(p)))
    if not requests: raise ValueError('No recorded requests found')
    selected=list(requests.items())[:a.limit or None]
    a.output.mkdir(parents=True,exist_ok=False)
    def build():
        limits=PlannerLimits(**cfg['planner_limits'])
        reserve=cfg['nominal_acceleration_reserve_m_s2']
        limits=replace(limits,acceleration=limits.acceleration-reserve,entry_acceleration_capture_m_s2=limits.acceleration)
        return BezierLandingPlanner(AirframeGeometry.load(profile_path),profile['nominal_com_in_body_m'],limits,SweptEnvelope.load(envelope))
    def solve(p,req,pruned):
        tube=ForecastTube(**req['uncertainty']) if req.get('uncertainty') else None
        return p.search(FlightState(**req['state']),PlatformState(**req['platform']),req['now'],req['durations'],req['previous_arrival'],tube,tracking_envelope=req.get('tracking_envelope'),prune_score_dominated=pruned)
    metadata=dict(scope='offline paired request replay; no flight-success or deadline-rate claim',python=sys.version,platform=platform.platform(),planner_sha256=sha(planner_path),config_sha256=sha(a.config),geometry_sha256=sha(profile_path),envelope_sha256=sha(envelope),repeats=a.repeats,requests=selected)
    metadata['algorithm_sources_sha256']={str(p.relative_to(root)):sha(p) for p in sorted((root/'landing').glob('*.py'))}
    (a.output/'manifest.json').write_text(json.dumps(metadata,indent=2,allow_nan=False),encoding='utf-8')
    records=[]
    with (a.output/'pairs.jsonl').open('x',encoding='utf-8') as out:
        for index,(key,item) in enumerate(selected):
            for repeat in range(a.repeats):
                modes=[False,True] if (index+repeat)%2==0 else [True,False]
                runs={}
                for pruned in modes:
                    p=build();solve(p,item['request'],pruned) # Equal one-request warmup outside timing.
                    start=time.perf_counter();plan,report=solve(p,item['request'],pruned);elapsed=time.perf_counter()-start
                    runs['pruned' if pruned else 'exhaustive']=dict(available=plan is not None,duration=plan.duration if plan else None,score=report['selected_score'],audit_passed=bool(plan.audit['passed']) if plan else None,elapsed_wall_s=elapsed,solve_calls=report.get('solved_candidates'),pruned_candidates=report.get('score_bound_pruned_candidates'),candidate_report=report)
                row=dict(request_sha256=key,repeat=repeat,order=modes,equivalent=equivalent(runs['exhaustive'],runs['pruned']),**runs)
                out.write(json.dumps(row,allow_nan=False)+'\n');out.flush();records.append(row)
                print(key[:12],repeat,row['equivalent'],flush=True)
    summary=dict(pairs=len(records),unique_requests=len(selected),all_equivalent=all(r['equivalent'] for r in records),scope='Repeated request timings are not independent flight trials')
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    if not summary['all_equivalent']: raise SystemExit(2)
if __name__=='__main__': main()
