"""Raw sampled COM bounds during optimized flight and the entire guided return.

Uses the same finite differences and original limits as the contact auditor.
Native takeoff, impact and native ground LAND are outside this flight interval.
Planar extension: include initial PLANAR_FOLLOW; original bounds/math retained.
"""
from pathlib import Path
import argparse,gzip,hashlib,json,sys
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'work/receding_dependencies'),str(ROOT)]
import numpy as np
from landing.contact_geometry import Pose

def motion_peaks(times,points,speed_limit,acceleration_limit):
    t=np.asarray(times,dtype=float);p=np.asarray(points,dtype=float)
    if t.ndim!=1 or len(t)<4 or p.shape!=(len(t),3) or not np.isfinite(t).all() or not np.isfinite(p).all():raise ValueError('Finite raw COM pose sequence required')
    dt=np.diff(t)
    if (dt<=0).any() or dt.max()>.02500001:raise ValueError('Raw pose gap or non-increasing timestamp')
    velocity=np.diff(p,axis=0)/dt[:,None];vt=(t[:-1]+t[1:])/2
    acceleration=np.diff(velocity,axis=0)/np.diff(vt)[:,None];at=(vt[:-1]+vt[1:])/2
    speeds=np.linalg.norm(velocity,axis=1);accelerations=np.linalg.norm(acceleration,axis=1)
    checks=dict(actual_speed=bool(speeds.max()<=speed_limit),actual_acceleration=bool(accelerations.max()<=acceleration_limit))
    return dict(passed=all(checks.values()),checks=checks,samples=len(t),start_s=float(t[0]),end_s=float(t[-1]),max_raw_gap_s=float(dt.max()),
        peak_speed_m_s=float(speeds.max()),peak_acceleration_m_s2=float(accelerations.max()),acceleration_peak_t_s=float(at[accelerations.argmax()]),
        speed_limit_m_s=speed_limit,acceleration_limit_m_s2=acceleration_limit)

def audit(run):
    run=Path(run);r=run/'results'
    control=r/'receding_control.jsonl';rows=[json.loads(l) for l in control.read_text().splitlines()]
    cfgpath=run/'config/experiments/receding_live_v1.json';cfg=json.loads(cfgpath.read_text())
    ex=json.loads((run/'config/experiments/execution_v1.json').read_text());profile=run/ex['airframe_profile']
    offset=json.loads(profile.read_text())['nominal_com_in_body_m']
    active=[v['sim_t_s'] for v in rows if v.get('planner_phase') in ('ALIGNING','TRACKING','TERMINAL_TRACKING','PLANAR_FOLLOW')]
    assert active,'No optimized/preparation flight exercised'
    start=min(active);end=rows[-1]['sim_t_s'];status=json.loads((r/'execution_status.json').read_text())
    if status['completed']:end=json.loads((r/'independent_contact_audit.json').read_text())['raw_contact_s']
    seq={};h=hashlib.sha256();raw=r/'model_states.jsonl'
    stream=raw.open('rb') if raw.exists() else gzip.open(raw.with_suffix('.jsonl.gz'),'rb')
    with stream:
        for line in stream:
            h.update(line);v=json.loads(line)
            if v['kind']!='uav_pose':continue
            for p in v['message'].get('pose',[]):
                if p['name']!='uav':continue
                stamp=p['header']['stamp'];t=float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9
                if start<=t<end:
                    pose=Pose(tuple(p.get('position',{}).get(k,0.) for k in 'xyz'),tuple(p.get('orientation',{}).get(k,0.) for k in 'xyzw'))
                    point=pose.transform(offset)
                    if t in seq:assert seq[t]==point,'Conflicting raw poses'
                    seq[t]=point
    times=sorted(seq);report=motion_peaks(times,[seq[t] for t in times],cfg['planner_limits']['speed'],cfg['planner_limits']['acceleration'])
    report.update(scenario=status['scenario'],guided_recovery_included=not status['completed'],declared_interval_s=[start,end],
        scope='Sampled raw COM velocity/acceleration, from first PLANAR_FOLLOW/ALIGNING/TRACKING through contact or guided return end. Native takeoff, impact and native ground LAND excluded; no continuous bound guarantee.',
        raw_uncompressed_sha256=h.hexdigest(),source_sha256={str(f.relative_to(run)):hashlib.sha256(f.read_bytes()).hexdigest() for f in (control,cfgpath,profile)},
        auditor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    assert report['raw_uncompressed_sha256']==json.loads((r/'raw_compression.json').read_text())['sha256_uncompressed']
    out=r/'independent_guided_motion_audit.json'
    if out.exists():assert json.loads(out.read_text())==report,'Existing raw motion report differs'
    else:out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args()
    print(json.dumps(audit(args.run),indent=2))
