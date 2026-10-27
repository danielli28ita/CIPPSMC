#!/usr/bin/env python3
"""Verify all immutable release files using only the Python standard library."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
manifest = json.loads((ROOT/'RELEASE_MANIFEST.json').read_text())
for name, info in manifest['files'].items():
    path = (ROOT/name).resolve()
    if ROOT not in path.parents:
        raise SystemExit('Unsafe path in manifest: '+name)
    if not path.is_file() or path.stat().st_size != info['bytes']:
        raise SystemExit('Missing or wrong size: '+name)
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    if h.hexdigest() != info['sha256']:
        raise SystemExit('Checksum mismatch: '+name)
samples = list((ROOT/'data').rglob('*.ulg'))
if len(samples) != 1 or samples[0].name != 'log_19_2026-10-19-10-01-54.ulg':
    raise SystemExit('Unexpected sample data')
print('PASS: '+str(len(manifest['files']))+' release files match SHA-256; only requested log_19 is bundled')
