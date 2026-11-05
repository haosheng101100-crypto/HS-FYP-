from pathlib import Path
import datetime,json,subprocess,shutil
w=Path('/workspace');p=w/'work/reacquisition_20261102';r=p/'repository/毕设仿真';d=p/'deployment';o=w/'outputs/重新跟随专项_20261102'
def stamp():return datetime.datetime.now(datetime.timezone.utc).isoformat()
state={'source_commit':'ca86ce7cf8c8ee1a392a23867296718b7583d59e','feature_commit':'58065a94335388d6b698f9463cf30185fc4d3cfa','stage':'RUNNING','started_utc':stamp(),'scope':'毕设仿真 only'}
def save():(o/'run_receipt.json').write_text(json.dumps(state,ensure_ascii=False,indent=2))
save();shutil.copy2(p/'unit_tests.log',o/'unit_tests.log')
with (p/'supervisor.log').open('x') as log:
 q=subprocess.run(['/opt/hs-venv/bin/python','-u',str(r/'tools/run_planar_simulation.py'),'--profile','planar_reacquisition_v1','--base',str(d),'--session',str(o/'session_001'),'--source',str(d/'reference'),'--frozen',str(d/'baseline'),'--package',str(d/'operator')],stdout=log,stderr=subprocess.STDOUT)
state.update(stage='AUDITING',supervisor_exit_code=q.returncode,physical_finished_utc=stamp());save()
with (p/'audits.log').open('x') as log:
 a=subprocess.run(['/opt/hs-venv/bin/python','-u',str(p/'audit_runs.py')],stdout=log,stderr=subprocess.STDOUT)
state.update(stage='AUDITS_ENDED',audit_driver_exit_code=a.returncode,finished_utc=stamp());save()
subprocess.run(['chown','-R','1000:1000',str(o),str(p)],check=True);print(json.dumps(state),flush=True)
