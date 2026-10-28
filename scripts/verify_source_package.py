"""Verify every uploaded file byte-for-byte against the curated package manifest."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1]
manifest=json.loads((root/'handoff/source_package_manifest.json').read_text(encoding='utf-8'))
for name,digest in manifest['files_sha256'].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
print('Verified',len(manifest['files_sha256']),'source/package files')
