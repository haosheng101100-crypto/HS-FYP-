"""Independent owned process watcher; observe /proc before flight preparation."""
from pathlib import Path
import time,json,os,datetime,platform
r=Path(__file__).resolve().parents[1];start=time.monotonic()
assert r.name.startswith('unified_decision_') and r.parent.name=='reproduction_20261022'
while time.monotonic()-start<240:
 try:
  managed=json.loads((r/'results/managed_processes.json').read_text())
  wanted={'gazebo','arducopter','execution','scenario_driver','model_states'}
  selected=[p for p in managed['processes'] if p['name'] in wanted]
  if len(selected)!=5:time.sleep(.1);continue
  inspected=[]
  for p in selected:
   proc=Path('/proc')/str(p['pid']);cwd=(proc/'cwd').resolve();cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode()
   assert cmd and (cwd==r or r in cwd.parents) and os.getpgid(p['pid'])==p['pgid']
   inspected.append({**p,'actual_proc_cwd':str(cwd),'actual_proc_cmdline':cmd,'proc_was_live':True})
  now=datetime.datetime.now(datetime.timezone.utc).isoformat();boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
  guest={'guest_capture_utc':now,'hostname':platform.node(),'virtualisation_vendor':Path('/sys/class/dmi/id/sys_vendor').read_text().strip(),
   'boot_id':boot,'interfaces':{p.parent.name:p.read_text().strip() for p in Path('/sys/class/net').glob('*/address')},'execution_root':str(r.parent.parent)}
  with (r/'results/guest_execution_provenance.json').open('x') as f:json.dump(guest,f,indent=2)
  proof={'capture_utc':now,'boot_id':boot,'processes':inspected}
  with (r/'results/live_process_inspection.json').open('x') as f:json.dump(proof,f,indent=2)
  guest['owned_processes']=managed
  with (r/'results/guest_session_provenance_live.json').open('x') as f:json.dump(guest,f,indent=2)
  print(json.dumps({'actual_live_processes_captured':5,'capture_utc':now}));break
 except (FileNotFoundError,ProcessLookupError):time.sleep(.1)
else:raise RuntimeError('No complete live process proof before preparation deadline')
