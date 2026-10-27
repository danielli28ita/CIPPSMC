#!/usr/bin/env python3
"""Prepare, build and check this release in its own workspace only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
META = json.loads((ROOT/'firmware/BASELINE.json').read_text())
WORK = ROOT/'workspace'
SOURCE = WORK/'PX4-Autopilot'
TARGETS = {'real': 'px4_fmu-v6c_onboard', 'sitl': 'px4_sitl_default'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path), *args], text=True).strip()


def clone(url, target, mirror=None):
    if target.exists():
        if not (target/'.git').exists():
            raise RuntimeError('Path collision, refusing to overwrite: '+str(target))
        return False
    target.parent.mkdir(parents=True, exist_ok=True)
    command = ['git', '-c', 'core.autocrlf=false', 'clone', '--no-checkout']
    if mirror:
        command += ['--shared', str(mirror)]
    else:
        command += [url]
    subprocess.run(command+[str(target)], check=True)
    return True


def verify_source():
    if git(SOURCE, 'rev-parse', 'HEAD') != META['baseline_commit']:
        raise RuntimeError('PX4 base commit mismatch')
    for sub in META['submodules']:
        if git(SOURCE/sub['path'], 'rev-parse', 'HEAD') != sub['commit']:
            raise RuntimeError('Dependency mismatch: '+sub['path'])
    for name, entry in META['files'].items():
        path = SOURCE/name
        if not path.is_file() or digest(path) != entry['sha256']:
            raise RuntimeError('Source payload mismatch: '+name)
    print('PASS: baseline, '+str(len(META['submodules']))+' dependencies and '+str(len(META['files']))+' source payloads', flush=True)


def prepare(mirror):
    WORK.mkdir(exist_ok=True)
    marker = WORK/'source_preparation.json'
    if marker.exists():
        verify_source()
        return
    if mirror:
        mirror = mirror.resolve()
        if git(mirror, 'rev-parse', 'HEAD') != META['baseline_commit']:
            raise RuntimeError('Mirror baseline does not match release')
    newly_cloned = clone(META['upstream_url'], SOURCE, mirror)
    if not newly_cloned and git(SOURCE, 'status', '--porcelain', '--ignore-submodules=all'):
        raise RuntimeError('Refuse dirty/incomplete source preparation; retain it for diagnosis')
    subprocess.run(['git', '-C', str(SOURCE), 'checkout', '--detach', META['baseline_commit']], check=True)
    for sub in sorted(META['submodules'], key=lambda x: x['path'].count('/')):
        target = SOURCE/sub['path']
        # git checkout creates empty gitlink directories; remove only an empty one.
        if target.is_dir() and not any(target.iterdir()):
            target.rmdir()
        clone(sub['url'], target, mirror/sub['path'] if mirror else None)
        subprocess.run(['git', '-C', str(target), 'checkout', '--detach', sub['commit']], check=True)
        parent = SOURCE/sub['parent']
        subprocess.run(['git', '-C', str(parent), 'config', 'submodule.'+sub['name']+'.url', sub['url']], check=True)
        subprocess.run(['git', '-C', str(parent), 'config', 'submodule.'+sub['name']+'.active', 'true'], check=True)
    # Validate every baseline before applying any release file.
    for name, entry in META['files'].items():
        payload, destination = ROOT/'firmware/overlay'/name, SOURCE/name
        if digest(payload) != entry['sha256']:
            raise RuntimeError('Damaged release payload: '+name)
        old = entry['baseline_sha256']
        if old is None:
            if destination.exists():
                raise RuntimeError('New-file collision: '+name)
        elif not destination.is_file() or digest(destination) != old:
            raise RuntimeError('Baseline file mismatch: '+name)
    for name in META['files']:
        destination = SOURCE/name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/'firmware/overlay'/name, destination)
    verify_source()
    marker.write_text(json.dumps({'baseline': META['baseline_commit'], 'local_git_mirror_used': bool(mirror),
                                  'note': 'Only committed mirror objects are used; release payloads determine modified files.'}, indent=2)+'\n')


def build(which, jobs):
    verify_source()
    target = TARGETS[which]
    env = os.environ.copy()
    env['PATH'] = str(Path(sys.executable).parent)+os.pathsep+env.get('PATH', '')
    env['CCACHE_DIR'] = str(WORK/'ccache')
    env.pop('PYTHONPATH', None)
    log = WORK/(target+'.build.log')
    print('Building '+target+'; log: '+str(log), flush=True)
    with log.open('w') as stream:
        subprocess.run(['make', target, '-j'+str(jobs)], cwd=SOURCE, env=env,
                       stdout=stream, stderr=subprocess.STDOUT, check=True)
    output = SOURCE/'build'/target
    receipt = {'target': target, 'baseline': META['baseline_commit'], 'overlay_files': META['files'],
               'compiler': subprocess.check_output(['arm-none-eabi-gcc' if which == 'real' else 'c++', '--version'], text=True).splitlines()[0],
               'python': sys.version.split()[0], 'artifacts': {}, 'exact_identity_with_sample_firmware_verified': False}
    if which == 'real':
        destination = WORK/'artifacts'/target
        destination.mkdir(parents=True, exist_ok=True)
        board = (output/'boardconfig').read_text()
        if 'CONFIG_MODULES_FIGURE8_TRAJECTORY=y' not in board or 'CONFIG_MODULES_UXRCE_DDS_CLIENT=y' in board:
            raise RuntimeError('Unexpected onboard module configuration')
        for name in (target+'.px4', target+'.elf', 'parameters.xml', 'airframes.xml'):
            shutil.copy2(output/name, destination/name)
            receipt['artifacts'][name] = digest(destination/name)
    else:
        receipt['artifacts']['bin/px4'] = digest(output/'bin/px4')
    (WORK/(target+'.build.json')).write_text(json.dumps(receipt, indent=2)+'\n')
    check_binary(which)
    print('PASS: build '+target, flush=True)


def check_binary(which):
    target = TARGETS[which]
    output = SOURCE/'build'/target
    binary = output/(target+'.elf') if which == 'real' else output/'bin/px4'
    nm = 'arm-none-eabi-nm' if which == 'real' else 'nm'
    symbols = subprocess.check_output([nm, '-C', str(binary)], text=True)
    required = ['PositionControl::_applyBppsmcOverride(', 'figure8_trajectory_main']
    if not all(item in symbols for item in required):
        raise RuntimeError('Compiled controller/module symbols missing: '+str(binary))
    objdump = 'arm-none-eabi-objdump' if which == 'real' else 'objdump'
    disassembly = subprocess.check_output([objdump, '-d', '-C', str(binary)], text=True)
    # Verify the actual call in update(), rather than accepting a merely linked function.
    start = disassembly.index('<PositionControl::update(float)>:')
    section = disassembly[start:].split('\n\n', 1)[0]
    if '_applyBppsmcOverride' not in section:
        raise RuntimeError('PositionControl::update() has no compiled BPPSMC call')
    print('PASS: compiled BPPSMC call and onboard trajectory module in '+target, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='action', required=True)
    prep = subs.add_parser('prepare')
    prep.add_argument('--mirror-source', type=Path, help='Optional local Git object mirror; working-tree edits are ignored')
    build_parser = subs.add_parser('build')
    build_parser.add_argument('target', choices=['real', 'sitl', 'all'])
    build_parser.add_argument('--jobs', type=int, default=2)
    check = subs.add_parser('check')
    check.add_argument('--binary', choices=['real', 'sitl'])
    args = parser.parse_args()
    if args.action == 'prepare':
        prepare(args.mirror_source)
    elif args.action == 'build':
        if args.jobs < 1:
            parser.error('--jobs must be positive')
        for which in (['real', 'sitl'] if args.target == 'all' else [args.target]):
            build(which, args.jobs)
    else:
        verify_source()
        if args.binary:
            check_binary(args.binary)


if __name__ == '__main__':
    main()
