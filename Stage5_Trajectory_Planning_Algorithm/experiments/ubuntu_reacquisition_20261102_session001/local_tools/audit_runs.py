from pathlib import Path
import json,subprocess,hashlib,gzip,sys
w=Path('/workspace');p=w/'work/reacquisition_20261102';r=p/'repository/毕设仿真';o=w/'outputs/重新跟随专项_20261102';decl=json.loads((o/'declared_protocol.json').read_text());summary=[]
for run in sorted((o/'session_001').glob('cycle_*')):
 res=run/'results'
 if not (res/'cycle_closed.json').exists():continue
 if not (res/'execution_status.json').exists():
  summary.append({'cycle':run.name,'passed':False,'reason':'Initialization incomplete'});continue
 status=json.loads((res/'execution_status.json').read_text());checks={n:hashlib.sha256((run/n).read_bytes()).hexdigest()==h for n,h in decl['source_sha256'].items()};assert json.loads((run/'config/experiments/receding_live_v1.json').read_text())==decl['control_configuration']
 raw=res/'model_states.jsonl';packed=raw.with_suffix('.jsonl.gz')
 if raw.exists():
  assert not packed.exists();h=hashlib.sha256();size=0
  with raw.open('rb') as src,gzip.open(packed,'wb',compresslevel=1) as dst:
   for b in iter(lambda:src.read(1048576),b''):h.update(b);size+=len(b);dst.write(b)
  h2=hashlib.sha256()
  with gzip.open(packed,'rb') as src:
   for b in iter(lambda:src.read(1048576),b''):h2.update(b)
  assert h.digest()==h2.digest();(res/'raw_compression.json').write_text(json.dumps({'sha256_uncompressed':h.hexdigest(),'bytes_uncompressed':size,'bytes_compressed':packed.stat().st_size,'verified_lossless':True},indent=2));raw.unlink()
 with (res/'strict_reacquisition_audit.log').open('w') as log:
  audit=subprocess.run([sys.executable,str(r/'tools/audit_planar_reacquisition.py'),str(run)],stdout=log,stderr=subprocess.STDOUT)
 f=res/'independent_planar_reacquisition_audit.json';report=json.loads(f.read_text()) if f.exists() else {'passed':False,'reason':'Strict auditor did not produce report'}
 motion=res/'independent_guided_motion_audit.json';motion=json.loads(motion.read_text()) if motion.exists() else {}
 row={'cycle':run.name,'scenario':status['scenario'],'expected':'landing_after_reacquisition','passed':report['passed'] and all(checks.values()) and audit.returncode==0,'audit_exit_code':audit.returncode,'source_checks':checks,'strict_audit':report,'execution_status':status,'peak_speed_m_s':motion.get('peak_speed_m_s'),'peak_acceleration_m_s2':motion.get('peak_acceleration_m_s2')};summary.append(row)
 (o/'new_run_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));print(json.dumps({'scenario':row['scenario'],'passed':row['passed'],'checks':report.get('checks')},ensure_ascii=False),flush=True)
(o/'new_run_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
