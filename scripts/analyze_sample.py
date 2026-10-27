#!/usr/bin/env python3
"""Analyze the one bundled sample and optionally check archived expected metrics."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SAMPLE = ROOT/'data/log_1.ulg'


def metrics(summary):
    s = summary['stages']['5']
    return {'duration_s': summary['duration_s'],
            'figure8_3d_rmse_m': s['position_3d_error']['rms'],
            'figure8_3d_max_m': s['position_3d_error']['max'],
            'figure8_horizontal_rmse_m': s['horizontal_error']['rms'],
            **{f'figure8_{axis}_bias_m': s['error_xyz'][i]['mean'] for i, axis in enumerate('xyz')},
            **{f'figure8_{axis}_rmse_m': s['error_xyz'][i]['rms'] for i, axis in enumerate('xyz')},
            **{f'lap_{lap}_3d_rmse_m': value['error_3d']['rms'] for lap, value in summary['laps'].items()},
            'event_count': len(summary['events']), 'dropout_records': len(summary['dropouts'])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=ROOT/'reports/log_1')
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    provenance = json.loads((ROOT/'data/PROVENANCE.json').read_text())
    if hashlib.sha256(SAMPLE.read_bytes()).hexdigest() != provenance['sha256']:
        raise SystemExit('Sample checksum mismatch')
    out = args.out.resolve()
    subprocess.run([sys.executable, str(ROOT/'analysis/analyze_tracking_quality.py'), str(SAMPLE), '--out', str(out)], check=True)
    subprocess.run([sys.executable, str(ROOT/'analysis/analyze_sliding_terms.py'), str(SAMPLE), '--out', str(out)], check=True)
    summary = json.loads((out/'summary.json').read_text())
    values = metrics(summary)
    (out/'metrics.json').write_text(json.dumps(values, indent=2)+'\n')
    if args.verify:
        expected = json.loads((ROOT/'data/expected_metrics.json').read_text())
        for key, value in expected['metrics'].items():
            if not math.isclose(values[key], value, rel_tol=expected['rtol'], abs_tol=expected['atol']):
                raise SystemExit(f'Metric mismatch: {key}, got {values[key]}, expected {value}')
        print('PASS: '+str(len(values))+' sample metrics reproduced within declared tolerances')
    print('Report: '+str(out))


if __name__ == '__main__':
    main()
