#!/usr/bin/env python3
"""Apply and verify the ET-BPPSMC overlay on a clean PX4 v1.15.4 tree."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('px4', type=Path)
    p.add_argument('--check-only', action='store_true')
    a = p.parse_args()
    root = a.px4.resolve()
    package = Path(__file__).resolve().parents[1]
    overlay = package / 'firmware'
    meta = json.loads((overlay / 'BASELINE.json').read_text())
    def git(*args):
        return subprocess.check_output(['git', '-C', str(root), *args])
    def digest(b):
        return hashlib.sha256(b).hexdigest()
    if git('rev-parse', 'HEAD').decode().strip() != meta['baseline_commit']:
        raise SystemExit('Expected exact v1.15.4 baseline commit.')
    if git('status', '--porcelain', '--untracked-files=all').strip():
        raise SystemExit('Refuse dirty checkout; use a separate clean PX4 clone.')
    for rel, info in meta['files'].items():
        payload = overlay / 'changed' / rel
        if digest(payload.read_bytes()) != info['sha256']:
            raise SystemExit('Snapshot checksum mismatch: ' + rel)
        target = root / rel
        if info['baseline_sha256'] is None:
            if target.exists():
                raise SystemExit('New-file collision: ' + rel)
        elif not target.is_file() or digest(target.read_bytes()) != info['baseline_sha256']:
            raise SystemExit('Baseline checksum mismatch: ' + rel)
    patch = str(overlay / 'against-v1.15.4.patch')
    git('apply', '--check', patch)
    if a.check_only:
        print('ET-BPPSMC overlay checks passed; no files changed.')
        return
    git('apply', patch)
    for rel, info in meta['files'].items():
        if digest((root / rel).read_bytes()) != info['sha256']:
            raise SystemExit('Restored file mismatch: ' + rel)
    print('ET-BPPSMC restored and verified: 9 files, BASELINE_CONTROLLER=0.')


if __name__ == '__main__':
    main()
