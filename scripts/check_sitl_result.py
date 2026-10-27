#!/usr/bin/env python3
"""Verify lifecycle/profile in a generated local SITL ULog, reporting precision separately."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from pyulog import ULog

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    logs = sorted(args.run.rglob('*.ulg'), key=lambda p: p.stat().st_mtime)
    if not logs:
        raise SystemExit('No simulator ULog found')
    log = ULog(str(logs[-1]))
    f = log.get_dataset('figure8_status').data
    b = log.get_dataset('bppsmc_debug').data
    status = log.get_dataset('vehicle_status').data
    mask = f['stage'] == 5
    if not np.any(mask):
        raise SystemExit('No figure-eight phase')
    start, end = f['timestamp'][mask][0], f['timestamp'][mask][-1]
    params = dict(log.initial_parameters)
    for timestamp, key, value in log.changed_parameters:
        if timestamp < start:
            params[key] = value
    # Modules constructed after logger startup can omit unchanged defaults from ULog.
    xml = ROOT/'workspace/PX4-Autopilot/build/px4_sitl_default/parameters.xml'
    defaults = {p.attrib['name']: float(p.attrib['default']) for p in ET.parse(xml).getroot().iter('parameter')}
    profile = json.loads((ROOT/'params/flight_profile.json').read_text())
    fallback = []
    mismatches = {}
    for key, value in profile.items():
        if key == 'MPC_THR_HOVER':
            continue
        if key not in params:
            fallback.append(key)
        actual = params.get(key, defaults.get(key, np.nan))
        if not np.isclose(value, actual, rtol=1e-6, atol=1e-8):
            mismatches[key] = [value, actual]
    messages = ' '.join(m.message for m in log.logged_messages)
    active = (status['timestamp'] >= start) & (status['timestamp'] <= end)
    complete = 'Figure8 complete: landed and disarmed' in messages
    passed = complete and not mismatches and len(b['timestamp']) > 0 and not np.any(status['failsafe'][active])
    report = {'passed_lifecycle_and_profile': bool(passed), 'parameter_mismatches': mismatches,
              'parameter_values_from_compiled_defaults_if_missing_in_ulog': fallback,
              'sim_hover_parameter': params.get('MPC_THR_HOVER'),
              'sim_airframe': params.get('SYS_AUTOSTART'), 'sim_ev_ctrl': params.get('EKF2_EV_CTRL'),
              'sim_gps_ctrl': params.get('EKF2_GPS_CTRL'),
              'figure8_error_rmse_m': float(np.sqrt(np.mean(f['tracking_error'][mask]**2))),
              'figure8_error_max_m': float(np.max(f['tracking_error'][mask])),
              'laps': np.unique(f['lap'][mask]).tolist(), 'bppsmc_debug_samples': len(b['timestamp']),
              'tracking_precision_criteria': {'figure8_rmse_m_below': .15, 'figure8_max_m_below': .30},
              'tracking_precision_acceptance_test_passed': bool(np.sqrt(np.mean(f['tracking_error'][mask]**2)) < .15 and np.max(f['tracking_error'][mask]) < .30),
              'note': 'Simulation engineering checks with explicit X500-only hover mapping; not a claim to reproduce real flight errors. Generated simulator ULog is kept in workspace, not bundled as another sample.'}
    out = args.out or args.run/'verification.json'
    out.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if not passed:
        raise SystemExit('SITL verification failed')


if __name__ == '__main__':
    main()
