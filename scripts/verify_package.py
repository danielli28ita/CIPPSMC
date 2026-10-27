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
samples = {path.name: path for path in (ROOT/'data').rglob('*.ulg')}
expected = {
    'log_19_2026-10-19-10-01-54.ulg': '34d4087e9f9f53c8364c87fd2501edbf2f88131b98b14af4910c027afb3eea7e',
    'log_2.ulg': 'd96751a3576ee280d0c288d8a53f4f393f5006c9c6b5c32192b82d8dcbfde632',
}
found = {}
for name, path in samples.items():
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    found[name] = digest
if found != expected:
    raise SystemExit('Unexpected flight logs: '+str(sorted(found)))
print('PASS: '+str(len(manifest['files']))+' release files match SHA-256; Flight A and Flight B logs match')
