#!/usr/bin/env python3
"""Regenerate integrity manifest and deterministic tar.gz, excluding all workspaces."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[1]


def excluded(path):
    parts = path.relative_to(ROOT).parts
    return parts[0] in ('workspace', 'reports') or '__pycache__' in parts or '.git' in parts or path.suffix == '.pyc'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=ROOT.parent/'releases')
    args = parser.parse_args()
    out = args.out.resolve()
    if out == ROOT or ROOT in out.parents:
        parser.error('Archive output must be outside the package directory')
    files = []
    for path in ROOT.rglob('*'):
        if excluded(path):
            continue
        if path.is_symlink():
            raise SystemExit('Unexpected symlink in release: '+str(path))
        if path.is_file() and path.name != 'RELEASE_MANIFEST.json':
            files.append(path)
    files.sort()
    samples = [p for p in files if p.suffix == '.ulg']
    if len(samples) != 1 or samples[0].name != 'log_19_2026-10-19-10-01-54.ulg':
        raise SystemExit('Release must contain exactly the requested log_19')
    manifest = {'version': '1.0.0', 'date': '2026-10-26',
                'files': {p.relative_to(ROOT).as_posix(): {'sha256': sha(p), 'bytes': p.stat().st_size} for p in files}}
    mp = ROOT/'RELEASE_MANIFEST.json'
    mp.write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+'\n')
    files.append(mp)
    out.mkdir(parents=True, exist_ok=True)
    archive = out/(ROOT.name+'.tar.gz')
    with archive.open('wb') as stream:
        with gzip.GzipFile(fileobj=stream, filename='', mode='wb', mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode='w') as tar:
                for path in sorted(files):
                    info = tar.gettarinfo(str(path), arcname=ROOT.name+'/'+path.relative_to(ROOT).as_posix())
                    info.uid = info.gid = 0
                    info.uname = info.gname = ''
                    info.mtime = 0
                    with path.open('rb') as source:
                        tar.addfile(info, source)
    checksum = sha(archive)
    (out/(archive.name+'.sha256')).write_text(checksum+'  '+archive.name+'\n')
    print(json.dumps({'archive': str(archive), 'bytes': archive.stat().st_size,
                      'sha256': checksum, 'release_files': len(files), 'bundled_ulog_count': len(samples)}, indent=2))


if __name__ == '__main__':
    main()
