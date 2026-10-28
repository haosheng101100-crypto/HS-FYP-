"""Link the run to the user's local VMware guest and captured live processes."""
from pathlib import Path
import json,sys,datetime,hashlib
run=Path(sys.argv[1]);out=run.parent
host=json.loads((out/'local_execution_provenance.json').read_text())
index=run.name.rsplit('_',1)[1]
live_path=out/('guest_session_provenance.json' if index=='001' else f'guest_session_provenance_{index}.json')
active=out/f'guest_session_provenance_active_{index}.json'
status=json.loads((run/'results/execution_status.json').read_text())
if status.get('state_selected_braking_trial') and not active.exists():
 result={'passed':False,'checks':{'independent_during_execution_provenance_available':False},'reason':'No during-execution capture was obtained. A post-completion managed-process list is retained separately and cannot prove actual live processes.'}
 target=run/'landing_analysis/local_execution_verification.json';target.parent.mkdir(exist_ok=True);assert not target.exists();target.write_text(json.dumps(result,indent=2));print(json.dumps(result));sys.exit(0)
live=json.loads((active if active.exists() else live_path).read_text())
guest=json.loads((run/'results/guest_execution_provenance.json').read_text())
launch=json.loads((run/'results/launch_summary.json').read_text())
manifest=json.loads((run/'results/attempt_manifest.json').read_text())
post=json.loads((run/'results/postflight_checks.json').read_text())
created=manifest['created_unix'];begin=datetime.datetime.fromisoformat(host['host_capture_utc']).timestamp();capture=datetime.datetime.fromisoformat(live['guest_capture_utc']).timestamp()
owned={p['name']:p['pid'] for p in launch['processes']};seen={p['name']:p['pid'] for p in live['owned_processes']['processes']}
checks={'local_vm_was_running':host['local_vmx_absolute_path'].replace('/','\\') in host['local_vmrun_inventory'],
 'guest_mac_matches_local_vm':host['vmx_guest_mac'] in guest['interfaces'].values(),
 'guest_boot_matches_live_capture':guest['boot_id']==live['boot_id']==host['guest_boot_id'],
 'guest_is_vmware':guest['virtualisation_vendor']=='VMware, Inc.',
 'actual_physics_and_fcu_pids_match_live_capture':all(owned[name]==seen[name] for name in ['gazebo','arducopter','model_states','execution']),
 'created_after_local_provenance_before_live_capture':begin-.5<=created<=capture+1.,
 'run_path_matches_captured_guest':str(manifest['run']).startswith(guest['execution_root']+'/reproduction_20261022/'+('unified_decision_' if json.loads((run/'results/execution_status.json').read_text()).get('unified_decision_trial') else 'acceleration_decision_' if json.loads((run/'results/execution_status.json').read_text()).get('accelerating_decision_trial') else 'interrupted_descent_' if json.loads((run/'results/execution_status.json').read_text()).get('external_scenario_driver') else 'centred_braking_')),
 'raw_log_hash_verified':hashlib.sha256((run/'results/model_states.jsonl').read_bytes()).hexdigest()==post['raw_sha256']}
result={'passed':all(checks.values()),'checks':checks,'local_vmx':host['local_vmx_absolute_path'],'guest_ip':host['guest_ssh_host_only_ip'],
 'run_created_utc':datetime.datetime.fromtimestamp(created,datetime.timezone.utc).isoformat(),
 'live_processes_observed':seen,'raw_log_sha256':post['raw_sha256']}
inspection=run/'results/live_process_inspection.json'
if status.get('state_selected_braking_trial'):checks['independent_during_execution_proc_inspection_required']=inspection.exists()
result['passed']=all(checks.values())
if inspection.exists():
 proof=json.loads(inspection.read_text())
 checks['independent_proc_inspection_matches_live_owned_processes']=proof['boot_id']==guest['boot_id'] and len(proof['processes'])==5 and all(p['proc_was_live'] and p['pid']==owned[p['name']] and (p['actual_proc_cwd']==manifest['run'] or p['actual_proc_cwd'].startswith(manifest['run']+'/')) for p in proof['processes'])
 result['passed']=all(checks.values())
analysis_dir='decision_analysis' if json.loads((run/'results/execution_status.json').read_text()).get('accelerating_decision_trial') else 'landing_analysis'
target=run/analysis_dir/'local_execution_verification.json';target.parent.mkdir(exist_ok=True);assert not target.exists();target.write_text(json.dumps(result,indent=2));print(json.dumps(result))
