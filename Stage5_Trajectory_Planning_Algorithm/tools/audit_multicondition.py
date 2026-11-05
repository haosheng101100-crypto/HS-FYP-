"""Independent raw coverage and unchanged physical/safety auditors for M01--M20.

Evidence thresholds below describe excitation, never relax flight/contact limits.
Raw full-rate physical poses (including gzip) determine coverage; scene commands
alone cannot certify that a perturbation actually happened.
"""
from pathlib import Path
import argparse,gzip,json,math,sys,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

IDS={f'M{i:02d}' for i in range(1,21)}

def raw_platform(run):
 path=Path(run)/'results/model_states.jsonl';source=path.open('rb') if path.exists() else gzip.open(path.with_suffix('.jsonl.gz'),'rb')
 poses={};digest=hashlib.sha256()
 with source:
  for line in source:
   digest.update(line);row=json.loads(line)
   if row['kind']!='car_pose':continue
   for p in row['message'].get('pose',[]):
    if p.get('name')!='landing_vehicle':continue
    stamp=p['header']['stamp'];t=float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9
    value=(tuple(p.get('position',{}).get(k,0.) for k in 'xyz'),tuple(p.get('orientation',{}).get(k,0.) for k in 'xyzw'))
    if t in poses and poses[t]!=value:raise ValueError('Conflicting full-rate raw platform poses')
    poses[t]=value
 t=np.asarray(sorted(poses));p=np.asarray([poses[x][0] for x in t]);q=np.asarray([poses[x][1] for x in t])
 return t,p,q,digest.hexdigest()

def kinematics(t,p):
 t=np.asarray(t,float);p=np.asarray(p,float)
 if len(t)<30 or p.shape!=(len(t),3) or not np.isfinite(t).all() or not np.isfinite(p).all() or np.min(np.diff(t))<=0 or np.max(np.diff(t))>.06000001:raise ValueError('Missing finite, ordered, full-rate raw platform trajectory')
 # Central 0.20s displacement smooths numerical pose differentiation without
 # taking information from scene targets. Coverage replay is offline.
 idx=np.arange(5,len(t)-5,5);v=(p[idx+5,:2]-p[idx-5,:2])/(t[idx+5]-t[idx-5])[:,None]
 tv=t[idx];a=np.gradient(v,tv,axis=0);speed=np.linalg.norm(v,axis=1)
 cross=v[:,0]*a[:,1]-v[:,1]*a[:,0];curvature=cross/np.maximum(speed,1e-5)**3
 return tv,v,speed,curvature

def crossing(t,p,v):
 # A crossing needs two traversals of the same area separated by at least 2s,
 # with distinct tangents. Nearby consecutive samples cannot manufacture it.
 for i in range(0,len(t),3):
  eligible=np.where((t>t[i]+2)&(np.linalg.norm(p-p[i],axis=1)<.12))[0]
  for j in eligible:
   ni=np.linalg.norm(v[i]);nj=np.linalg.norm(v[j])
   if ni>.04 and nj>.04 and abs(float(np.dot(v[i],v[j])/(ni*nj)))<.90:return True
 return False

def trajectory_evidence(t,p,q,case,end_s):
 cid=case['case_id']
 if cid not in IDS:raise ValueError('Unknown multicondition case')
 mask=np.asarray(t)<=end_s;t=np.asarray(t)[mask];p=np.asarray(p)[mask];q=np.asarray(q)[mask]
 tv,v,speed,k=kinematics(t,p)
 moving=speed>.04;heading=np.unwrap(np.arctan2(v[:,1],v[:,0]));moving_head=heading[moving]
 axes=(0,) if cid in {'M15','M16'} else (0,1)
 checks={'declared_axes_motion':bool(np.all(np.ptp(p[:,axes],axis=0)>.01)),
         'fixed_height':bool(np.ptp(p[:,2])<=.005),
         'fixed_deck_orientation':bool(q.shape==(len(t),4) and np.isfinite(q).all() and np.max(2*np.arccos(np.clip(np.abs(np.sum(q*q[0],axis=1)),-1,1)))<=.005)}
 late=(tv>=end_s-1)&moving
 turns=k[moving];span=float(np.ptp(moving_head)) if len(moving_head) else 0.
 if cid in {'M03','M04','M05','M06','M07'}:
  checks['continuously_turns_before_contact']=bool(np.count_nonzero(late)>=4 and np.median(np.abs(k[late]))>.015 and span>.10)
  final=(t[1:]>=end_s-.20)&(t[1:]<=end_s)
  raw_speed=np.linalg.norm(np.diff(p[:,:2],axis=0)/np.diff(t)[:,None],axis=1)
  checks['curve_still_physically_moving_at_contact']=bool(np.count_nonzero(final)>=3 and t[-1]>=end_s-.06 and np.all(raw_speed[final]>.04))
 if cid in {'M03','M04'}:
  sign=-1 if cid=='M03' else 1
  checks['declared_turn_direction']=bool(len(turns)>4 and np.mean(sign*turns>.015)>.75)
  radius=2.5 if cid=='M03' else 3.
  # Fit actual circle center; startup is excluded using moving samples.
  xy=p[np.searchsorted(t,tv[moving]),:2];A=np.column_stack((2*xy,np.ones(len(xy))))
  z=np.sum(xy*xy,axis=1);fit=np.linalg.lstsq(A,z,rcond=None)[0];radii=np.linalg.norm(xy-fit[:2],axis=1)
  checks['declared_circle_geometry']=bool(len(xy)>5 and abs(np.median(radii)-radius)<max(.10,.12*radius) and np.std(radii)<max(.10,.12*radius))
 if cid=='M05':
  checks['variable_curvature']=bool(len(turns)>4 and np.percentile(np.abs(turns),80)-np.percentile(np.abs(turns),20)>.025)
  # Axis-aligned ellipse fit to physical samples after startup; center may
  # shift under the command shaper, but semiaxes and changing curvature remain.
  xy=p[np.searchsorted(t,tv[(tv>tv[0]+3)&moving]),:2]
  A=np.column_stack((xy[:,0]**2,xy[:,1]**2,xy[:,0],xy[:,1],np.ones(len(xy))))
  if len(xy)>=20:
   _,_,V=np.linalg.svd(A,full_matrices=False);aa,bb,cc,dd,ee=V[-1]
   if aa<0:aa,bb,cc,dd,ee=-aa,-bb,-cc,-dd,-ee
   scale=cc*cc/(4*aa)+dd*dd/(4*bb)-ee if aa>0 and bb>0 else -1
   axes=np.sqrt(scale/np.array([aa,bb])) if scale>0 else np.array([math.nan,math.nan])
   checks['actual_ellipse_shape']=bool(np.isfinite(axes).all() and np.all(np.abs(axes-np.array([3.,1.5]))<np.array([.5,.3])))
  else:checks['actual_ellipse_shape']=False
 if cid=='M06':
  checks['curvature_sign_change']=bool(np.count_nonzero(turns>.015)>=4 and np.count_nonzero(turns<-.015)>=4)
  checks['sustained_forward_motion']=bool(np.mean(v[moving,0]>.04)>.9)
 if cid=='M07':checks['actual_crossing']=crossing(tv,p[np.searchsorted(t,tv),:2],v)
 if cid in {'M01','M02'}:
  angle=math.radians(30 if cid=='M01' else 135);desired=np.array([math.cos(angle),math.sin(angle)])
  checks['declared_speed_exercised']=bool(len(speed)>5 and np.percentile(speed,75)>.8*(.25 if cid=='M01' else .45))
  checks['declared_nonaxial_heading']=bool(np.count_nonzero(moving)>4 and np.dot(np.median(v[moving],axis=0),desired)/max(np.linalg.norm(np.median(v[moving],axis=0)),1e-8)>.98)
 if cid=='M08':checks['actual_acceleration']=bool(np.max(speed)>.38 and np.min(speed)<.16)
 if cid=='M09':checks['actual_speed_and_heading_change']=bool(np.ptp(speed)>.10 and span>.15)
 if cid=='M10':checks['actual_stop']=bool(np.max(speed)>.20 and np.count_nonzero((tv>tv[-1]-2)&(speed<.03))>=5)
 if cid=='M11':
  stopped=np.where(speed<.03)[0];checks['actual_restart']=bool(len(stopped) and any(np.any(speed[j+1:]>.10) for j in stopped))
 if cid in {'M12','M13'}:checks['actual_large_turn']=bool(span>=math.radians(70 if cid=='M12' else 110))
 return dict(passed=all(checks.values()),checks=checks,samples=len(t),heading_span_rad=span,peak_speed_m_s=float(max(speed)),height_range_m=float(np.ptp(p[:,2])),curvature_range_m_inv=[float(min(k)),float(max(k))],evidence_thresholds={'motion_m':.01,'fixed_height_range_m':.005,'fixed_orientation_rad':.005,'curvature_m_inv':.015,'crossing_distance_m':.12})

def recovery_evidence(events,rows,end_s,landed):
 changes=[e for e in events if e['event']=='PLANAR_REACQUIRE'];chains=[]
 for i,e in enumerate(changes):
  stop=changes[i+1]['time'] if i+1<len(changes) else end_s
  control=any(abs(r['sim_t_s']-e['time'])<1e-8 and (r.get('planar_supervision') or {}).get('action')=='REACQUIRE' for r in rows)
  aligned=[x['time'] for x in events if x['event']=='ALIGNMENT_COMPLETE' and e['time']<x['time']<stop]
  plans=[x['time'] for x in events if x['event']=='PLAN_ACTIVATED' and aligned and aligned[0]<x['time']<stop]
  # Final safe exit may exhaust a recovery; a landing may not hide it.
  required=landed or i+1<len(changes)
  chains.append(dict(time=e['time'],actual_reacquire_control=control,alignment_then_new_plan=bool(plans),passed=bool(control and (not required or plans))))
 return dict(passed=all(x['passed'] for x in chains),reacquisitions=len(changes),chains=chains)

def classify(safety,coverage,landed,cid):
 if not safety:return 'control_physical_audit_failure'
 if not coverage:return 'coverage_failure'
 return 'landing' if landed else 'safe_exit'

def budget_evidence(events,rows,cfg,case,end_s):
 design=cfg['planar_landing'];declared=case['budget']
 starts=[e['time'] for e in events if e['event']=='PLANAR_INITIAL_FOLLOW']
 waits=[(r.get('planar_supervision') or {}).get('wait_elapsed_s',0.) for r in rows]
 changes=[e for e in events if e['event']=='PLANAR_REACQUIRE']
 # Exit/return may finish later; policy must already have stopped accepting
 # descent by its original task budget. Replay independently checks this.
 exits=[r for r in rows if (r.get('planar_supervision') or {}).get('action')=='EXIT']
 first_exit=exits[0] if exits else None
 stop=first_exit['sim_t_s'] if first_exit else end_s
 active=[r for r in rows if not first_exit or r['sim_t_s']<stop]
 previous=active[-1] if active else None
 reason=(first_exit.get('planar_supervision') or {}).get('reason') if first_exit else None
 waiting_ok=bool(waits) and all(math.isfinite((r.get('planar_supervision') or {}).get('wait_elapsed_s',0.)) and 0<=(r.get('planar_supervision') or {}).get('wait_elapsed_s',0.)<=25.+1e-8 for r in active)
 if first_exit:
  final_wait=(first_exit.get('planar_supervision') or {}).get('wait_elapsed_s',0.)
  # The unmodified supervisor rejects strictly AFTER exceeding a limit; the
  # first sampled crossing is evidence of timely exit, never extra permission.
  crossing=bool(previous and 0<stop-previous['sim_t_s']<=.06000001)
  waiting_ok &= math.isfinite(final_wait) and (0<=final_wait<=25.+1e-8 or reason=='PLANAR_WAIT_TIMEOUT' and crossing and final_wait<=25.+stop-previous['sim_t_s']+1e-8)
 else:crossing=False
 task_ok=bool(starts) and 0<=stop-starts[0]<=40.+1e-8
 if first_exit and not task_ok:
  task_ok=bool(starts and reason=='PLANAR_TASK_TIMEOUT' and crossing and 0<=previous['sim_t_s']-starts[0]<=40.+1e-8)
 checks=dict(original_budgets_preserved=declared=={'total_task_s':40.,'cumulative_wait_s':25.,'max_reacquisitions':3}
             and design['maximum_task_s']==40. and design['maximum_wait_s']==25. and design['maximum_reacquisitions']==3,
             single_initial_follow=len(starts)==1,
             reacquisition_count=len(changes)<=3,
             cumulative_wait=bool(waiting_ok),
             task_expiry=bool(task_ok))
 return dict(passed=all(checks.values()),checks=checks,task_stop_s=stop)

def audit(run,case):
 from tools.audit_planar_reacquisition import run_auditor
 run=Path(run).resolve();res=run/'results'
 status=json.loads((res/'execution_status.json').read_text());cfg=json.loads((run/'config/experiments/receding_live_v1.json').read_text())
 if case['case_id'] not in IDS or case['scenario']!='multi_'+case['case_id'].lower():raise ValueError('Unknown case identity')
 frozen=json.loads((ROOT/'config/experiments/multicondition_matrix_v1.json').read_text())
 if case!=next(c for c in frozen['cases'] if c['case_id']==case['case_id']):raise ValueError('Case differs from frozen source matrix')
 if status['scenario']!=case['scenario'] or json.loads((res/'world_case.json').read_text())!=case:raise ValueError('Run world case differs from frozen declaration')
 if cfg.get('multicondition_case')!={'case_id':case['case_id'],'scenario':case['scenario']}:raise ValueError('Missing minimal case identity or future commands leaked into worker config')
 if status.get('armed_at_save') is not False:raise ValueError('Run not disarmed/finalized; cannot certify interrupted evidence')
 rows=[json.loads(l) for l in (res/'receding_control.jsonl').read_text().splitlines()]
 record=res/'receding_session.json';session=json.loads(record.read_text()) if record.exists() else json.loads((res/'receding_attempts.json').read_text())[-1]
 events=session['events'];scene=json.loads((res/'scene_maneuvers.json').read_text())
 specs=[('policy','audit_planar_policy.py','independent_planar_policy_audit.json'),('native','audit_native_landing.py','native_landing_audit.json')]
 if status['completed']:specs.append(('contact','audit_receding_physics.py','independent_contact_audit.json'))
 specs.extend([('motion','audit_planar_guided_motion.py','independent_guided_motion_audit.json'),('platform','audit_planar_physics.py','independent_planar_physics_audit.json')])
 codes={};reports={}
 for label,script,out in specs:
  codes[label]=run_auditor(ROOT/'tools'/script,run,out)
  if (res/out).exists():reports[label]=json.loads((res/out).read_text())
 history=history_evidence(run,rows=rows,events=events,status=status)
 contact=reports.get('contact',{}).get('raw_contact_s')
 landed=status['completed'] is True and status.get('final_mode')=='LAND' and isinstance(contact,(float,int)) and math.isfinite(contact)
 # Coverage ends at physical contact, or at the policy's actual exit. Recovery
 # after abort must never add a late maneuver to a descent coverage claim.
 end=contact if landed else next((r['sim_t_s'] for r in rows if r.get('stage')=='RECOVERING'),rows[-1]['sim_t_s'])
 t,p,q,digest=raw_platform(run)
 if digest!=json.loads((res/'raw_compression.json').read_text())['sha256_uncompressed']:raise ValueError('Raw trajectory digest mismatch')
 from tools.audit_planar_physics import planar_interval_start
 start=planar_interval_start(rows,events,status['scenario'])
 select=t>=start;t=t[select];p=p[select];q=q[select]
 trajectory=trajectory_evidence(t,p,q,case,end)
 scene_report=event_evidence(case,scene,rows,events,t,p,end)
 recovery=recovery_evidence(events,rows,end,landed)
 budgets=budget_evidence(events,rows,cfg,case,end)
 safety=bool(all(codes[k]==0 and reports.get(k,{}).get('passed') is True for k,_,_ in specs) and history['passed'] and budgets['passed'])
 coverage=bool(trajectory['passed'] and scene_report['passed'] and recovery['passed'])
 objective=case['expected']['landing_objective'];classification=classify(safety,coverage,landed,case['case_id'])
 return dict(auditor_version='multicondition_audit_v1',case_id=case['case_id'],scenario=case['scenario'],passed=bool(safety and coverage and (landed or not objective)),
             classification=classification,physics=dict(passed=all(reports.get(k,{}).get('passed') is True for k in ('motion','platform'))),
             safety=dict(passed=safety,budgets=budgets,original_audit_exit_codes=codes,original_reports=reports,request_history=history),
             coverage=dict(passed=coverage,trajectory=trajectory,scene_events=scene_report,recovery=recovery),
             landing_objective=dict(required=objective,landed=bool(landed),achieved=bool(landed and safety and coverage),safe_exit_is_landing=False),
             actual_initial_evidence=dict(first_platform_raw_time_s=float(t[0]),first_platform_raw_position_m=p[0].tolist(),first_control_body_pose=rows[0].get('observation',{}).get('motion',{}).get('pose'),first_allowed_descent=next((dict(time_s=r['sim_t_s'],clearance_m=r.get('measured_leg_gap_m',min((f['clearance_m'] for f in r.get('feet',[])),default=None))) for r in rows if r.get('planner_phase')=='TRACKING'),None)),
             raw_uncompressed_sha256=digest,scope='M01-M13 landing objectives; M14-M20 safety exploration. No controller/contact/speed/acceleration/return threshold is relaxed.')

def main():
 parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);parser.add_argument('--case',required=True,type=Path);args=parser.parse_args()
 try:report=audit(args.run,json.loads(args.case.read_text()))
 except (OSError,ValueError,KeyError,IndexError,AssertionError,StopIteration,TypeError) as exc:
  report=dict(passed=False,classification='environment_evidence',physics={'passed':False},safety={'passed':False},coverage={'passed':False},landing_objective={'achieved':False},error=type(exc).__name__+': '+str(exc))
 out=args.run/'results/multicondition_audit.json';out.parent.mkdir(parents=True,exist_ok=True)
 # Keep prior results immutable while allowing a byte-equivalent replay.
 if out.exists() and json.loads(out.read_text())!=report:raise ValueError('Existing multicondition audit differs; preserve original and use separate replay')
 if not out.exists():out.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps(report,ensure_ascii=False,indent=2));return 0 if report['passed'] else 1


def event_evidence(case,scene,rows,events,t,p,end_s):
 cid=case['case_id'];trigger=case['trigger'];required=case['coverage']['required_events']
 original_scene=scene;scene=[e for e in scene if e.get('time',math.inf)<end_s]
 checks={'known_events_only':all(e.get('event')=='MULTICONDITION_MANEUVER_TRIGGERED' for e in original_scene),
         'required_event_count':len(scene)>=required,
         'sequence_and_identity':all(e.get('index')==i+1 and e.get('case_id')==cid and e.get('scenario')==case['scenario'] and e.get('planner_knows_future_commands') is False for i,e in enumerate(scene)),
         'strict_event_order':all(scene[i]['time']<scene[i+1]['time'] for i in range(len(scene)-1))}
 # Require exact count for measured cases; timed events after contact are not
 # required, but scheduled changes that were due before contact may not vanish.
 if trigger['type']=='measured_descent':checks['required_event_count']=len(scene)==required
 if cid in {'M14','M19','M20'}:
  times=case['parameters']['switch_times_s'];scheduled=[x for x in times if x>0]
  start=scene[0]['time']-scene[0]['elapsed'] if scene else next((e['time'] for e in events if e.get('event')=='PLANAR_INITIAL_FOLLOW'),end_s)
  due=[x for x in scheduled if start+x<end_s-.1]
  checks['scheduled_changes_present']=len(scene)>=len(due)
 details=[];t=np.asarray(t);p=np.asarray(p)
 dt=np.diff(t);v=np.diff(p[:,:2],axis=0)/dt[:,None];vt=(t[1:]+t[:-1])/2
 for i,e in enumerate(scene):
  stamp=e.get('time',math.nan);stop=min(end_s,scene[i+1]['time']) if i+1<len(scene) else end_s
  matching=[r for r in rows if abs(r['sim_t_s']-stamp)<1e-8]
  measured=False
  if len(matching)==1:
   row=matching[0];gap=row.get('measured_leg_gap_m')
   if gap is None and row.get('feet'):gap=min(f['clearance_m'] for f in row['feet'])
   vz=row.get('observation',{}).get('motion',{}).get('velocity_at_origin_world_m_s',[math.nan]*3)[2]
   measured=bool(isinstance(gap,(float,int)) and math.isclose(gap,e.get('measured_leg_gap_m',math.nan),abs_tol=1e-8) and math.isclose(vz,e.get('measured_vertical_speed_m_s',math.nan),abs_tol=1e-8) and row.get('planner_phase')==e.get('planner_phase'))
   if trigger['type']=='measured_descent':
    # Only first M15 trigger is clearance gated; second is recovery gated.
    if i==0:measured &= bool(trigger['clearance_range_m'][0]<=gap<=trigger['clearance_range_m'][1])
    measured &= bool(row.get('planner_phase')==trigger['phase'] and (cid=='M15' and i==1 or vz<trigger['vertical_speed_less_than_m_s']))
  before=v[(t[:-1]>=stamp-.4)&(t[1:]<stamp-.05)]
  after_mask=(t[:-1]>stamp+.05)&(t[1:]<stop)
  after=v[after_mask]
  response=False;response_s=None
  if len(before)>=3 and len(after)>=3:
   baseline=np.median(before,axis=0);target=np.asarray(e.get('target_xy',[math.nan,math.nan]),float)
   if cid=='M11':
    params=case['parameters']
    angle=math.radians(params['final_angle_deg'])
    target=np.array([0.,0.]) if i==0 else params['speed_m_s']*np.array([math.cos(angle),math.sin(angle)])
   change=target-baseline
   if change.shape==(2,) and np.isfinite(change).all() and np.linalg.norm(change)>.01:
    projected=(after-baseline)@change/np.linalg.norm(change)
    valid=projected>.01
    if cid in {'M15','M16','M17'}:
     valid &= after@baseline < -.5*np.linalg.norm(baseline)*np.linalg.norm(after,axis=1)
     valid &= np.linalg.norm(after,axis=1)>.04
    sustained=np.convolve(valid.astype(int),np.ones(3,dtype=int),mode='valid')
    if len(sustained) and max(sustained)>=3:
     response=True;response_s=float(vt[after_mask][np.where(sustained>=3)[0][0]])
  # Every reported change must be before actual contact, and physically
  # evidenced. No report can claim coverage from post-contact platform motion.
  details.append(dict(index=i+1,time=stamp,measured_trigger_matches_control=bool(measured),actual_precontact_response=bool(response),response_time_s=response_s,precontact=bool(stamp<end_s)))
 checks['all_triggers_match_physical_control']=all(d['measured_trigger_matches_control'] and d['precontact'] for d in details)
 # Continuous restart has explicit events even with required_events zero.
 require_response=bool(case['coverage'].get('require_precontact_actual_motion_response') or cid in {'M11','M14','M15','M16','M17','M18','M19','M20'})
 checks['actual_physical_responses']=all(d['actual_precontact_response'] for d in details) if require_response else True
 if cid=='M15':
  good=False
  if len(scene)==2:
   a,b=scene;changes=[e['time'] for e in events if e['event']=='PLANAR_REACQUIRE' and a['time']<e['time']<b['time']]
   alignment=[e['time'] for e in events if e['event']=='ALIGNMENT_COMPLETE' and changes and changes[-1]<e['time']<b['time']]
   # World scene runs after this tick's controller update. A new plan
   # logged at the same sim timestamp is causal only when its already-
   # activated sequence count also matches the world-side measured count.
   plans=[e['time'] for e in events if e['event']=='PLAN_ACTIVATED' and alignment and alignment[-1]<e['time']<=b['time']]
   counts_match=all(sum(e['event']=='PLAN_ACTIVATED' and e['time']<=sample['time'] for e in events)==sample.get('activated_plan_count') for sample in (a,b))
   real_align=any(a['time']<r['sim_t_s']<b['time'] and r.get('planner_phase')=='PLANAR_FOLLOW' and r.get('stage')=='ALIGNING' for r in rows)
   good=bool(plans and counts_match and real_align and b.get('reacquisitions',0)>a.get('reacquisitions',0) and b.get('activated_plan_count',0)>a.get('activated_plan_count',0))
  checks['second_trigger_after_real_recovery_and_new_plan']=good
 return dict(passed=all(checks.values()),checks=checks,events=details,post_objective_events_retained=[e for e in original_scene if e.get('time',math.inf)>=end_s],response_velocity_delta_threshold_m_s=.01,response_sustained_raw_intervals=3)

def history_evidence(run,*,rows=None,events=None,status=None):
 from tools.run_repeatability_group_a import history_check
 run=Path(run);folder=run/'runtime/receding_planner'
 traces=list(folder.glob('request*.json'))+list(folder.glob('response*.json'))
 if not traces:
  res=run/'results'
  if rows is None:rows=[json.loads(l) for l in (res/'receding_control.jsonl').read_text().splitlines()]
  if status is None:status=json.loads((res/'execution_status.json').read_text())
  if events is None:
   path=res/'receding_session.json';record=json.loads(path.read_text()) if path.exists() else json.loads((res/'receding_attempts.json').read_text())[-1];events=record['events']
  checks=dict(explicit_zero_submissions=type(status.get('planner_request_sequence_max')) is int and status['planner_request_sequence_max']==0 and bool(rows) and all(type(r.get('planner_request_sequence')) is int and r['planner_request_sequence']==0 for r in rows),
    disarmed_nonlanding_exit=status.get('completed') is False and status.get('armed_at_save') is False,
    actual_follow_and_exit=any(r.get('planner_phase')=='PLANAR_FOLLOW' for r in rows) and any((r.get('planar_supervision') or {}).get('action')=='EXIT' for r in rows),
    no_planning_or_activation=not any(r.get('planner_phase') in ('PLANNING_HOVER','TRACKING','TERMINAL_TRACKING') for r in rows) and any(e.get('event')=='PLANAR_INITIAL_FOLLOW' for e in events) and not any(e.get('event') in ('ALIGNMENT_COMPLETE','PLAN_ACTIVATED') for e in events),
    planner_directory_present=folder.is_dir(),no_request_response_or_mailbox_files=not traces)
  return dict(passed=all(checks.values()),requests=0,responses=0,planner_exercised=False,zero_submission_evidence=checks,scope='No planning request was submitted during a measured no-window safe exit; this does not demonstrate planner exercise.')
 report=history_check(run);matches=[]
 for response in sorted(folder.glob('response_*.json')):
  saved=json.loads(response.read_text());seq=saved['sequence'];request=folder/f'request_{seq:06d}.json'
  matches.append(response.name==f'response_{seq:06d}.json' and request.exists() and saved.get('request')==json.loads(request.read_text()))
 report['embedded_response_requests_match_originals']=bool(matches) and all(matches)
 report['passed'] &= report['embedded_response_requests_match_originals'];report['planner_exercised']=True
 return report

if __name__=='__main__':sys.exit(main())
