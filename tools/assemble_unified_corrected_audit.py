"""Keep the initial failed audit and explicitly join supplementary source proof."""
from pathlib import Path
import sys,json,hashlib
run=Path(sys.argv[1]);old=run/'verification.json';target=run/'verification_corrected.json'
assert old.exists() and not target.exists()
initial=json.loads(old.read_text());corrected=run/'unified_analysis/verification_corrected.json'
c=json.loads(corrected.read_text());assert c['passed']
checks=dict(initial['checks']);checks['unified_analysis']=c['passed']
result={'passed':all(checks.values()),'case':initial['case'],'checks':checks,
 'source_log_sha256':initial['source_log_sha256'],'original_verification_preserved':True,
 'correction_reason':'Initial common-interface source audit compared outputs reference artifacts against a runtime source manifest covering scripts/tools/landing/config/models/worlds/tests only. Seven outputs artifacts now independently hashed in the actual guest run. Aircraft code, control parameters and physical logs unchanged.',
 'supporting_sha256':{str(p.relative_to(run)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [old,corrected,run/'unified_analysis/supplemental_package_reference_hashes.json']}}
target.write_text(json.dumps(result,indent=2));print(json.dumps(result))
