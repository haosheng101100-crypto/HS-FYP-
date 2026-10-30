from pathlib import Path
import sys,json,gzip,shutil,hashlib
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R))
from landing.outcome_audit import classify_disarm
run=Path(sys.argv[1]);res=run/'results';out=run/'unified_analysis';out.mkdir(exist_ok=True)
path=res/'model_states.jsonl'
if not path.exists():
 with gzip.open(res/'model_states.jsonl.gz','rb') as a,path.open('xb') as b:shutil.copyfileobj(a,b)
s=json.loads((res/'execution_status.json').read_text());post=json.loads((res/'postflight_checks.json').read_text())
df=[json.loads(l) for l in (res/'dataflash_final/control_messages.jsonl').read_text().splitlines()]
stop=classify_disarm(s['armed_at_save'],s['events']+[r.get('Message','') for r in df if r.get('mavpackettype')=='MSG'],s['final_mode'],[r for r in df if r.get('mavpackettype')=='ARM'])
rows=[json.loads(l) for l in (res/'execution_feedback.jsonl').read_text().splitlines()]
gaps={};last={};counts={};contacts=[]
with path.open() as f:
 for l in f:
  row=json.loads(l);k=row['kind'];t=row['received_monotonic_s'];counts[k]=counts.get(k,0)+1
  if k in last:gaps[k]=max(gaps.get(k,0),t-last[k])
  last[k]=t
  if k=='deck_contact' and row['message'].get('contact'):contacts.append(row)
checks={'raw_hash_matches_postflight':hashlib.sha256(path.read_bytes()).hexdigest()==post['raw_sha256'],
 'failed_not_counted_as_success':not s['completed'],
 'never_entered_event_descent':bool(rows) and all(r['stage']=='HOVER' for r in rows) and not (res/'scenario_brake_event.json').exists(),
 'no_platform_contact':not contacts,
 'normal_native_disarm':stop['normal_disarm_observed'] and not stop['forced_disarm_detected'] and not stop['crash_detected'],
 'owned_processes_cleaned':not post['remaining_owned_processes']}
result={'passed':False,'outcome':'PREPARATION_RECEPTION_TIMEOUT; NOT_A_LANDING_TRIAL_SUCCESS',
        'failure_record_and_cleanup_checks_passed':all(checks.values()),'checks':checks,
        'independent_disarm_classification':stop,'external_failures':s['external_failures'],
        'raw_receiver_maximum_wall_gaps_s':gaps,'raw_message_counts':counts,
        'diagnostic_scope':'Recorder reception gaps do not alone identify whether the controller callback, simulator or scheduling caused timeout; original 0.5 s guard retained.',
        'source_log_sha256':post['raw_sha256']}
target=out/'verification.json';assert not target.exists();target.write_text(json.dumps(result,indent=2));print(json.dumps(result))
