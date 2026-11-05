"""Declared A-group physical repetitions; existing controller/profile unchanged.

prepare writes a new machine-readable matrix and case-specific launch metadata.
run requires a code commit receipt and executes one fresh simulator at a time.
Any failed case stops expansion; all attempted records and the denominator remain.
"""
from pathlib import Path
import argparse,datetime,gzip,hashlib,json,os,shutil,subprocess,sys,time,traceback

def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def save(p,d):
 p=Path(p);t=p.with_suffix('.tmp');t.write_text(json.dumps(d,ensure_ascii=False,indent=2));t.replace(p)
def stamp():return datetime.datetime.now(datetime.timezone.utc).isoformat()
def classify(status,audit,exit_code,integrity_ok,history_ok):
 if not integrity_ok or not history_ok or audit is None:return 'environment_or_insufficient_evidence'
 if audit.get('passed') and exit_code==0 and status.get('completed') and not status.get('armed_at_save') and status.get('final_mode')=='LAND':return 'complete_function_success'
 checks=audit.get('checks',{})
 if not status.get('completed') and checks.get('original_audits_passed') and status.get('final_mode')=='LAND' and not status.get('armed_at_save'):return 'correct_safe_exit'
 if checks.get('original_audits_passed') and (not checks.get('one_measured_trigger') or not checks.get('reacquisition_present')):return 'coverage_failure'
 return 'algorithm_control_or_physical_failure'
def prepare(root,output,base):
 assert not output.exists(),'Choose a new output; preserve previous attempts'
 output.mkdir(parents=True);(output/'profiles').mkdir()
 profile=json.loads((root/'config/experiments/planar_reacquisition_v1.json').read_text())
 matrix=[]
 for number in range(1,11):
  for axis in ('x','y','xy'):
   scenario='planar_reacquire_'+axis;case=f'A_{axis.upper()}_{number:02d}'
   cfg=dict(profile,validation_scenarios=[scenario]);path=output/'profiles'/f'{axis}.json'
   if not path.exists():save(path,cfg)
   matrix.append(dict(case_id=case,group='A',repeat=number,scenario=scenario,profile=str(path.resolve()),profile_sha256=sha(path),expected='landing_after_reacquisition',initial_leg_gap_m=4.0,trigger=profile['reacquisition_trial'],random_seed=None,synthetic_observation_stress_enabled=False,auditor='tools/audit_planar_reacquisition.py',session=str((output/'cases'/case/'session_001').resolve())))
 save(output/'matrix.json',dict(planned_total=92,group='A',planned_group=30,remaining_B_to_F=62,baseline='0b49cf9b906be13e3d41e3e1f7edd97fab52f662',control_configuration_source='config/experiments/planar_reacquisition_v1.json',baseline_profile_sha256=sha(root/'config/experiments/planar_reacquisition_v1.json'),model_source=str(base/'reference'),cases=matrix))
 save(output/'matrix_hash.json',dict(sha256=sha(output/'matrix.json')))
 print('Prepared and hashed 30 cases; B-F remain pending',flush=True)
def history_check(run):
 folder=run/'runtime/receding_planner';responses=[];missing=[]
 for p in sorted(folder.glob('response_*.json')):
  d=json.loads(p.read_text());seq=d['sequence'];responses.append(seq)
  req=folder/f'request_{seq:06d}.json'
  if not req.exists() or json.loads(req.read_text()).get('sequence')!=seq:missing.append(seq)
 requests=sorted(folder.glob('request_*.json'));latest=folder/'request.json'
 sequences=[json.loads(p.read_text())['sequence'] for p in requests]
 last=json.loads(latest.read_text()) if latest.exists() else {}
 last_seq=last.get('sequence',0)
 contiguous=isinstance(last_seq,int) and last_seq>0 and len(requests)==last_seq and all(seq==i for i,seq in enumerate(sequences,1))
 filenames_match=all(p.name==f'request_{seq:06d}.json' for p,seq in zip(requests,sequences))
 last_file=folder/f'request_{last_seq:06d}.json'
 latest_ok=latest.exists() and last_file.exists() and json.loads(last_file.read_text())==last
 return dict(passed=bool(requests) and bool(responses) and not missing and contiguous and filenames_match and latest_ok,requests=len(requests),responses=len(responses),request_sequences=sequences,response_sequences=responses,missing_request_sequences=missing,contiguous_original_requests=contiguous,archive_filenames_match=filenames_match,latest_mailbox_archived=latest_ok,files_sha256={p.name:sha(p) for p in [*requests,*sorted(folder.glob('response_*.json'))]})
def run(root,output,base,package,source,frozen):
 if (output/'result_index.json').exists() or ((output/'cases').exists() and any((output/'cases').iterdir())):
  raise FileExistsError('Existing experiment: retain its index and choose a new output')
 receipt=json.loads((output/'code_receipt.json').read_text());assert len(receipt['run_commit'])==40
 matrix=json.loads((output/'matrix.json').read_text());assert sha(output/'matrix.json')==json.loads((output/'matrix_hash.json').read_text())['sha256']
 for location,files in receipt['runtime_asset_sha256'].items():
  assert all(sha(Path(location)/n)==h for n,h in files.items()),location
 source_map=json.loads((package/'source_manifest.json').read_text())['source_sha256']
 assert all(sha(package/n)==h for n,h in source_map.items())
 results=[];save(output/'result_index.json',dict(planned_total=92,planned_A=30,executed_A=0,unexecuted_A=30,pending_B_to_F=62,stage='RUNNING',results=[]))
 for case in matrix['cases']:
  case_dir=Path(case['session']).parent;assert not case_dir.exists();case_dir.mkdir(parents=True)
  started=stamp()
  row=dict(case_id=case['case_id'],scenario=case['scenario'],repeat=case['repeat'],category='in_progress',started_utc=started,run_commit=receipt['run_commit'])
  results.append(row)
  save(case_dir/'result.json',row)
  save(output/'result_index.json',dict(planned_total=92,planned_A=30,executed_A=len(results),unexecuted_A=30-len(results),pending_B_to_F=62,stage='RUNNING',results=results))
  try:
   cfg=json.loads(Path(case['profile']).read_text());assert sha(case['profile'])==case['profile_sha256']
   # Root controller/auditor deployment hashes are checked before every launch.
   for n,h in receipt['source_sha256'].items():assert sha(root/n)==h,n
   for location,files in receipt['runtime_asset_sha256'].items():
    assert all(sha(Path(location)/n)==h for n,h in files.items()),location
   checks_map={}
   for n,h in source_map.items():
    if n.startswith(('landing/','tools/','config/airframes/')):checks_map[n]=h
    elif n in ('interactive_base.py','operator_backend.py','operator_clock.py','operator_legacy.py'):checks_map['tools/'+n]=h
   save(case_dir/'declared_protocol.json',dict(source_commit=receipt['run_commit'],control_configuration=cfg,source_sha256=checks_map,case=case,thresholds_relaxed=False))
   env=dict(os.environ,HS_SIM_BASE=str(base),HS_OPERATOR_SESSION=case['session'],HS_OPERATOR_PACKAGE=str(package),HS_LANDING_PROFILE=case['profile'],HS_SIM_SOURCE=str(source),HS_FROZEN_PACKAGE=str(frozen))
   print(json.dumps(dict(event='CASE_START',case_id=case['case_id'],time=stamp()),ensure_ascii=False),flush=True)
   with (case_dir/'supervisor.log').open('x') as log:
    proc=subprocess.Popen([sys.executable,str(package/'operator_supervisor.py')],cwd=package,env=env,stdout=log,stderr=subprocess.STDOUT)
    while proc.poll() is None:
     status_file=Path(case['session'])/'status.json'
     try:ui=json.loads(status_file.read_text())
     except (FileNotFoundError,json.JSONDecodeError):ui={}
     # Close native initialization errors; never destroy an armed simulation.
     if ui.get('stage')=='ERROR' and not ui.get('armed',False):(Path(case['session'])/'close.request').touch()
     time.sleep(1)
   cycle=Path(case['session'])/'cycle_001';res=cycle/'results';status_path=res/'execution_status.json'
   status=json.loads(status_path.read_text()) if status_path.exists() else {}
   integrity=status_path.exists() and (res/'cycle_closed.json').exists() and all((cycle/n).exists() and sha(cycle/n)==h for n,h in checks_map.items())
   integrity=bool(integrity and json.loads((cycle/'config/experiments/receding_live_v1.json').read_text())==cfg)
   raw=res/'model_states.jsonl';packed=res/'model_states.jsonl.gz'
   if raw.exists():
    original=sha(raw);size=raw.stat().st_size
    with raw.open('rb') as src,gzip.open(packed,'wb',compresslevel=1) as dst:shutil.copyfileobj(src,dst)
    h=hashlib.sha256()
    with gzip.open(packed,'rb') as src:
     for b in iter(lambda:src.read(1048576),b''):h.update(b)
    assert h.hexdigest()==original
    save(res/'raw_compression.json',dict(sha256_uncompressed=original,bytes_uncompressed=size,bytes_compressed=packed.stat().st_size,verified_lossless=True));raw.unlink()
   audit=None;audit_code=None;history=dict(passed=False)
   if status_path.exists():
    with (res/'strict_reacquisition_audit.log').open('x') as log:
     p=subprocess.run([sys.executable,str(root/'tools/audit_planar_reacquisition.py'),str(cycle)],cwd=root,stdout=log,stderr=subprocess.STDOUT)
    audit_code=p.returncode;f=res/'independent_planar_reacquisition_audit.json'
    if f.exists():audit=json.loads(f.read_text())
    history=history_check(cycle);save(res/'request_history_audit.json',history)
   category=classify(status,audit,audit_code,integrity,history['passed'])
   row.update(dict(case_id=case['case_id'],scenario=case['scenario'],repeat=case['repeat'],category=category,started_utc=started,finished_utc=stamp(),supervisor_exit_code=proc.returncode,audit_exit_code=audit_code,source_configuration_integrity=integrity,request_history_ok=history['passed'],audit=audit,execution_completed=status.get('completed'),armed_at_save=status.get('armed_at_save'),final_mode=status.get('final_mode'),run=str(cycle),run_commit=receipt['run_commit']))
   if proc.returncode and row['category']=='complete_function_success':row['category']='environment_or_insufficient_evidence'
  except Exception as exc:
   row.update(category='environment_or_insufficient_evidence',finished_utc=stamp(),evidence_error=repr(exc))
   (case_dir/'evidence_error.log').write_text(traceback.format_exc())
  save(case_dir/'result.json',row)
  counts={c:sum(x['category']==c for x in results) for c in ['complete_function_success','correct_safe_exit','coverage_failure','algorithm_control_or_physical_failure','environment_or_insufficient_evidence']}
  stage='STOPPED_FOR_INVESTIGATION' if row['category']!='complete_function_success' else ('COMPLETE' if len(results)==30 else 'RUNNING')
  save(output/'result_index.json',dict(planned_total=92,planned_A=30,executed_A=len(results),unexecuted_A=30-len(results),pending_B_to_F=62,stage=stage,counts=counts,results=results))
  print(json.dumps(dict(event='CASE_END',case_id=case['case_id'],category=row['category'],executed=len(results),planned=30,time=stamp()),ensure_ascii=False),flush=True)
  if stage=='STOPPED_FOR_INVESTIGATION':return 2
 return 0
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','run']);p.add_argument('--output',type=Path,required=True);p.add_argument('--base',type=Path,required=True);p.add_argument('--package',type=Path);p.add_argument('--source',type=Path);p.add_argument('--frozen',type=Path);a=p.parse_args();root=Path(__file__).resolve().parents[1]
 if a.action=='prepare':prepare(root,a.output.resolve(),a.base.resolve())
 else:sys.exit(run(root,a.output.resolve(),a.base.resolve(),a.package.resolve(),a.source.resolve(),a.frozen.resolve()))
