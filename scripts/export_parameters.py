#!/usr/bin/env python3
"""Export parameters only from the single bundled log_19."""
import hashlib
import json
from pathlib import Path
from pyulog import ULog

ROOT = Path(__file__).resolve().parents[1]
SAMPLE = ROOT/'data/log_19_2026-10-19-10-01-54.ulg'
EXPECTED = '34d4087e9f9f53c8364c87fd2501edbf2f88131b98b14af4910c027afb3eea7e'


def main():
    if hashlib.sha256(SAMPLE.read_bytes()).hexdigest() != EXPECTED:
        raise SystemExit('Sample checksum mismatch')
    log = ULog(str(SAMPLE))
    params = log.initial_parameters
    destination = ROOT/'params'
    destination.mkdir(exist_ok=True)
    (destination/'all_parameters.json').write_text(json.dumps(params, indent=2, sort_keys=True)+'\n')
    profile = {k: v for k, v in params.items() if k.startswith(('BPP_', 'F8_')) or k in ('MPC_CTRL_METHOD', 'MPC_THR_HOVER')}
    (destination/'flight_profile.json').write_text(json.dumps(profile, indent=2, sort_keys=True)+'\n')
    for name, values in [('all_parameters_REFERENCE_ONLY.params', params), ('flight_profile.params', profile)]:
        lines = ['# Extracted solely from log_19; not a universal vehicle setup.\n',
                 '# Full calibration/system backup is reference only; see docs/PARAMETERS.md.\n']
        for k, v in sorted(values.items()):
            kind = 6 if isinstance(v, int) else 9
            value = str(v) if kind == 6 else format(v, '.17g')
            lines.append(f'1\t1\t{k}\t{value}\t{kind}\n')
        (destination/name).write_text(''.join(lines))
    record = {'sample': SAMPLE.name, 'sha256': EXPECTED, 'parameter_count': len(params),
              'profile_parameter_count': len(profile), 'changed_parameters': log.changed_parameters,
              'provenance': 'Decoded ULog initial parameter values; float32 values retained exactly in JSON/17-digit text.',
              'firmware_metadata': log.msg_info_dict,
              'exact_rebuilt_binary_identity_with_sample': False}
    (ROOT/'data/PROVENANCE.json').write_text(json.dumps(record, indent=2, ensure_ascii=False)+'\n')
    print('PASS: exported '+str(len(params))+' initial parameters and '+str(len(profile))+' task/controller profile parameters from log_19 only')


if __name__ == '__main__':
    main()
