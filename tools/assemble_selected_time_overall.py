from pathlib import Path
import sys,json,hashlib
run=Path(sys.argv[1]);checks={}
for folder in ['', 'search_analysis']:
 p=run/folder/('verification_v2.json' if (run/folder/'verification_v2.json').exists() else 'verification.json');checks['flight_contact_sources_and_decision' if not folder else 'candidate_search_budget_and_feedback']=json.loads(p.read_text())['passed']
result={'passed':all(checks.values()),'checks':checks,
 'source_log_sha256':json.loads((run/'results/postflight_checks.json').read_text())['raw_sha256'],
 'physical_contact_acceptance_and_computation_budget_reported_separately':True,
 'evidence_sha256':{str(p.relative_to(run)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [run/('verification_v2.json' if (run/'verification_v2.json').exists() else 'verification.json'),run/'search_analysis'/('verification_v2.json' if (run/'search_analysis/verification_v2.json').exists() else 'verification.json')]}}
p=run/(sys.argv[2] if len(sys.argv)>2 else 'overall_verification.json');assert p.parent==run and not p.exists();p.write_text(json.dumps(result,indent=2));print(json.dumps(result))
