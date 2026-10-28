"""Freeze a new independent physical trial version, never overwrite guest sources."""
from pathlib import Path
import argparse,json,hashlib
p=argparse.ArgumentParser();p.add_argument('--previous',type=int,required=True);p.add_argument('--version',type=int,required=True)
a=p.parse_args();base=Path(__file__).parent;root=base.parent;dest=base/'operator_receding_v1'
old=f'operator_receding_v{a.previous}';new=f'operator_receding_v{a.version}'
oldsession=f'receding_session_{a.previous:03d}';session=f'receding_session_{a.version:03d}'
sources=[('operator_backend.py',base/'operator_receding_backend.py')]+[(n,root/n) for n in ['tools/receding_planner_worker.py','landing/receding_live.py','landing/receding_landing.py','landing/swept_envelope.py','landing/platform_prediction.py','landing/platform_filter.py','landing/prediction_uncertainty.py','config/airframes/iris_swept_envelope_v1.json','config/experiments/receding_live_v1.json']]
for name,source in sources:
 (dest/name).parent.mkdir(parents=True,exist_ok=True)
 (dest/name).write_bytes(source.read_bytes())
f=dest/'operator_supervisor.py';s=f.read_text(encoding='utf-8-sig');assert old in s and oldsession in s
s=s.replace(old,new).replace(oldsession,session)
if "'landing/platform_prediction.py'" not in s:s=s.replace("'landing/swept_envelope.py'","'landing/swept_envelope.py','landing/platform_prediction.py'")
if "'landing/prediction_uncertainty.py'" not in s:s=s.replace("'landing/platform_prediction.py'","'landing/platform_prediction.py','landing/prediction_uncertainty.py'")
if "'landing/platform_filter.py'" not in s:s=s.replace("'landing/platform_prediction.py'","'landing/platform_prediction.py','landing/platform_filter.py'")
f.write_text(s,encoding='utf-8')
m=dest/'source_manifest.json';d=json.loads(m.read_text());d['source_sha256']={f.relative_to(dest).as_posix():hashlib.sha256(f.read_bytes()).hexdigest() for f in dest.rglob('*') if f.is_file() and '__pycache__' not in f.parts and f.name!='source_manifest.json'};m.write_text(json.dumps(d,indent=2))
launch=f'''set -eu
python3 - <<'PY'
from pathlib import Path
import subprocess,json,hashlib
b=Path('/home/haosheng/hs_ardupilot_stage1');pkg=b/'paper_experiments/{new}'
prior=b/'interactive_20261023/{oldsession}'
d=json.loads((prior/'status.json').read_text())
if d['armed']:
 review=json.loads((prior/'interruption_review.json').read_text())
 assert review['cause']=='host_restart' and review['all_previous_owned_processes_absent']
 assert review['status_sha256']==hashlib.sha256((prior/'status.json').read_bytes()).hexdigest()
 assert review['current_guest_boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip()
 for proc in Path('/proc').iterdir():
  if not proc.name.isdigit():continue
  try:
   if not (proc/'cmdline').read_bytes():continue
   cwd=(proc/'cwd').resolve()
   assert cwd!=prior and prior not in cwd.parents,'Interrupted simulation unexpectedly running again'
  except (FileNotFoundError,ProcessLookupError,PermissionError):pass
for cycle in prior.glob('cycle_*'):
 if (cycle/'results/cycle_closed.json').exists():continue
 if not (cycle/'results/interruption_record.json').exists():raise RuntimeError('Previous simulation is still running or has not been safely reviewed; no new flight started')
 for row in json.loads((cycle/'results/managed_processes.json').read_text()):
  proc=Path('/proc')/str(row['pid'])
  try:
   if (proc/'cmdline').read_bytes():
    cwd=(proc/'cwd').resolve();assert cwd!=cycle and cycle not in cwd.parents
  except FileNotFoundError:pass
(prior/'close.request').write_text('Next independent algorithm trial')
deps=pkg/'linux_deps'
if not deps.exists():deps.symlink_to(b/'paper_experiments/operator_receding_v1/linux_deps',target_is_directory=True)
manifest=json.loads((pkg/'source_manifest.json').read_text())
for name,digest in manifest['source_sha256'].items():assert hashlib.sha256((pkg/name).read_bytes()).hexdigest()==digest,name
assert not (b/'interactive_20261023/{session}').exists()
with (pkg/'launch.log').open('x') as f:
 proc=subprocess.Popen(['python3','-u',str(pkg/'operator_supervisor.py')],cwd=pkg,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
print('Physical algorithm trial',proc.pid,'{session}')
PY
'''
(base/f'launch_receding_operator_v{a.version}.sh').write_text(launch,encoding='utf-8')
progress=(base/f'progress_receding_v{a.previous}.sh').read_text().replace(oldsession,session)
(base/f'progress_receding_v{a.version}.sh').write_text(progress,encoding='utf-8')
