"""Verify local delivery matches the immutable native/raw/control evidence."""
from pathlib import Path
import hashlib,json,gzip
ROOT=Path(__file__).resolve().parents[1];base=ROOT/'outputs/receding_physical_20261023'
report=json.loads((base/'final_validation/verification.json').read_text());assert report['passed']
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for data in iter(lambda:f.read(1048576),b''):h.update(data)
    return h.hexdigest()
checked=[]
for row in report['cases']:
    run=base/f"batch_{row['session']:03d}"/f"cycle_{row['cycle']:03d}";r=run/'results'
    for name,sha in report['control_source_sha256'].items():assert digest(run/name)==sha
    for name in [row['physics_audit'],'native_landing_audit.json','independent_command_audit.json']:assert json.loads((r/name).read_text())['passed']
    raw=r/'model_states.jsonl.gz';h=hashlib.sha256()
    with gzip.open(raw,'rb') as f:
        for data in iter(lambda:f.read(1048576),b''):h.update(data)
    sha=h.hexdigest();assert sha==json.loads((r/'raw_compression.json').read_text())['sha256_uncompressed']
    audit=json.loads((r/row['physics_audit']).read_text())
    assert sha==(audit['source_sha256']['model_states.jsonl'] if row['outcome']=='deck_landing' else audit['raw_source_sha256'])
    if row['outcome']=='deck_landing':
        for name,expected in audit['source_sha256'].items():
            if name!='model_states.jsonl':assert digest(r/name)==expected
    native=json.loads((r/'native_landing_audit.json').read_text())
    for name,expected in native['source_sha256'].items():assert digest(run/'runtime/logs'/name)==expected
    checked.append(dict(session=row['session'],cycle=row['cycle'],scenario=row['scenario'],verified=True))
interrupted=base/'batch_031/cycle_005/results';record=json.loads((interrupted/'interruption_record.json').read_text())
for name,expected in record['source_sha256'].items():assert digest(interrupted/name)==expected
out=base/'final_validation/local_archive_verification.json';assert not out.exists()
out.write_text(json.dumps(dict(passed=True,cases=checked,interrupted_evidence_preserved=True,scope='Bytewise local source, lossless raw log and native DataFlash verification; experimental conclusions remain scoped by verification.json.'),indent=2))
print('Local archive verified: control source, raw log hashes, native DataFlash, all selected audits and interrupted evidence.')
