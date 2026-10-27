"""Read-only tracking diagnostics; timestamps are seconds since ULog start.

Uses linear interpolation for continuous logged signals and previous-sample
selection for mission stages. Metrics describe EKF estimates, not ground truth.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    u = ULog(str(args.log))
    d = lambda name, instance=0: u.get_dataset(name, instance).data
    t = lambda data: (data['timestamp'].astype(float)-u.start_timestamp)*1e-6
    interp = lambda data, key, times: np.interp(times, t(data), data[key])
    def stats(x):
        x = np.asarray(x, float)
        x = x[np.isfinite(x)]
        if not len(x):
            return None
        return dict(zip(('mean', 'std', 'rms', 'min', 'p05', 'median', 'p95', 'max'),
                        map(float, [np.mean(x), np.std(x), np.sqrt(np.mean(x*x)),
                                    *np.percentile(x, [0, 5, 50, 95, 100])])) )
    p, r, f, b = map(d, ['vehicle_local_position', 'trajectory_setpoint', 'figure8_status', 'bppsmc_debug'])
    tp = t(p)
    stage_at = lambda times: f['stage'][np.clip(np.searchsorted(t(f), times, side='right')-1, 0, len(t(f))-1)]
    stage = stage_at(tp)
    pos = np.column_stack([p[k] for k in ('x', 'y', 'z')])
    ref = np.column_stack([interp(r, f'position[{i}]', tp) for i in range(3)])
    vel = np.column_stack([p[k] for k in ('vx', 'vy', 'vz')])
    vr = np.column_stack([interp(r, f'velocity[{i}]', tp) for i in range(3)])
    err = pos-ref
    summary = {'log': str(args.log.resolve()), 'sha256': hashlib.sha256(args.log.read_bytes()).hexdigest(),
               'duration_s': (u.last_timestamp-u.start_timestamp)*1e-6,
               'parameters': {k: v for k, v in u.initial_parameters.items() if k.startswith(('BPP_', 'F8_', 'MPC_', 'HTE_', 'MC_', 'EKF2_EV'))},
               'changed_parameters': u.changed_parameters,
               'dropouts': [{'time_s': (x.timestamp-u.start_timestamp)*1e-6, 'duration_ms': x.duration} for x in u.dropouts],
               'messages': [{'time_s': (m.timestamp-u.start_timestamp)*1e-6, 'text': m.message} for m in u.logged_messages],
               'stages': {}}
    for s in np.unique(stage):
        mask = stage == s
        ft = f['stage'] == s
        summary['stages'][str(s)] = {'first_s': float(tp[mask][0]), 'last_s': float(tp[mask][-1]),
            'error_xyz': [stats(err[mask, i]) for i in range(3)],
            'horizontal_error': stats(np.linalg.norm(err[mask, :2], axis=1)),
            'position_3d_error': stats(np.linalg.norm(err[mask], axis=1)),
            'task_tracking_error': stats(f['tracking_error'][ft]),
            'velocity_error_xyz': [stats((vel-vr)[mask, i]) for i in range(3)]}
    flight = stage == 5
    summary['figure8_bias_energy_fraction'] = float(np.sum(np.mean(err[flight], axis=0)**2)/np.mean(np.sum(err[flight]**2, axis=1)))
    summary['figure8_demeaned_rmse_m'] = float(np.sqrt(np.mean(np.sum((err[flight]-np.mean(err[flight], axis=0))**2, axis=1))))
    summary['reference_figure8'] = {'peak_speed_m_s': float(np.max(np.linalg.norm(vr[flight], axis=1))),
        'peak_acceleration_m_s2': float(np.max(np.linalg.norm(np.column_stack([interp(r, f'acceleration[{i}]', tp) for i in range(3)])[flight], axis=1)))}
    tb, sb = t(b), stage_at(t(b))
    events = []
    for i in range(3):
        for j in np.flatnonzero(np.diff(b[f'trigger_count[{i}]'].astype(np.int64)) > 0)+1:
            if j+1 >= len(tb):
                continue
            events.append({'axis': i, 'time_s': float(tb[j]), 'stage': int(sb[j]),
                           'e0': float(b[f'e0[{i}]'][j]),
                           'acceleration_before': float(b[f'acceleration_sp[{i}]'][j]),
                           'acceleration_after': float(b[f'acceleration_sp[{i}]'][j+1]),
                           'next_sample_dt_s': float(tb[j+1]-tb[j])})
    summary['events'] = events
    summary['debug_figure8'] = [{'acceleration': stats(b[f'acceleration_sp[{i}]'][sb == 5]),
        'saturated_fraction': float(np.mean(np.abs(b[f'acceleration_sp[{i}]'][sb == 5]) >= u.initial_parameters['BPP_ACC_XY' if i < 2 else 'BPP_ACC_Z']-.001)),
        'rho': stats(b[f'rho_center[{i}]'][sb == 5]),
        'surface': stats(b[f'sliding_surface[{i}]'][sb == 5])} for i in range(3)]
    summary['laps'] = {}
    lap = f['lap'][np.clip(np.searchsorted(t(f), tp, side='right')-1, 0, len(t(f))-1)]
    for l in np.unique(lap[flight]):
        mask = flight & (lap == l)
        summary['laps'][str(l)] = {'error_xyz': [stats(err[mask, i]) for i in range(3)],
                                  'error_3d': stats(np.linalg.norm(err[mask], axis=1))}
    a, asp = d('vehicle_attitude'), d('vehicle_attitude_setpoint')
    def euler(data, prefix):
        w, x, y, z = [data[f'{prefix}[{i}]'].astype(float) for i in range(4)]
        return [np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
                np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))]
    ea, es = euler(a, 'q'), euler(asp, 'q_d')
    summary['attitude_figure8'] = []
    for i in range(3):
        actual = np.interp(t(asp), t(a), np.unwrap(ea[i]))
        dif = np.arctan2(np.sin(actual-es[i]), np.cos(actual-es[i]))
        m = stage_at(t(asp)) == 5
        summary['attitude_figure8'].append({'error_deg': stats(np.degrees(dif[m])),
                                          'setpoint_deg': stats(np.degrees(es[i][m]))})
    rates, rsp = d('vehicle_angular_velocity'), d('vehicle_rates_setpoint')
    summary['rates_figure8'] = []
    for i, key in enumerate(('roll', 'pitch', 'yaw')):
        m = stage_at(t(rates)) == 5
        summary['rates_figure8'].append(stats(np.degrees((rates[f'xyz[{i}]']-interp(rsp, key, t(rates)))[m])))
    sp = d('vehicle_local_position_setpoint')
    sm = stage_at(t(sp)) == 5
    az, th = sp['acceleration[2]'], -sp['thrust[2]']
    inferred = th/(1-az/9.80665)
    summary['hover'] = {'estimate_topic_logged': any(x.name == 'hover_thrust_estimate' for x in u.data_list),
        'mapping_figure8': stats(inferred[sm & (np.abs(az) < 8)]),
        'vertical_thrust_figure8': stats(th[sm]), 'acceleration_z_figure8': stats(az[sm])}
    summary['hold_windows'] = {}
    for label, stage_id in [('initial_hover', 4), ('return_hold', 6)]:
        stage_times = t(f)[f['stage'] == stage_id]
        if not len(stage_times):
            continue
        lo, hi = float(stage_times[0]+.5), float(stage_times[-1]-.5)
        m = (tp > lo) & (tp < hi) & (stage == stage_id)
        ms = (t(sp) > lo) & (t(sp) < hi)
        summary['hold_windows'][label] = {'window_s': [lo, hi],
            'position_error_xyz': [stats(err[m, i]) for i in range(3)],
            'velocity_xyz': [stats(vel[m, i]) for i in range(3)],
            'position_slope_xyz_m_s': [float(np.polyfit(tp[m], pos[m, i], 1)[0]) for i in range(3)],
            'vertical_thrust': stats(th[ms]), 'acceleration_z': stats(az[ms])}
    summary['lags'] = {}
    # Diagnostic alignment shift, not an identified sensor or control delay.
    for i in range(2):
        times = tp[flight][10:-10]
        shifts = np.arange(-.5, .501, .01)
        errors = [np.mean((np.interp(times+shift, tp, pos[:, i])-np.interp(times, t(r), r[f'position[{i}]']))**2) for shift in shifts]
        best = int(np.argmin(errors))
        summary['lags'][str(i)] = {'best_shift_s': float(shifts[best]), 'aligned_rmse': float(np.sqrt(errors[best]))}
    sel = d('estimator_selector_status')
    summary['estimator'] = {'primary_instances': list(map(int, np.unique(sel['primary_instance']))),
        'reset_counters': {k: list(map(int, np.unique(p[k]))) for k in ('xy_reset_counter', 'z_reset_counter', 'heading_reset_counter')}}
    summary['estimator']['instances'] = {}
    for inst in np.unique(sel['primary_instance']):
        est = d('estimator_status', int(inst))
        summary['estimator']['instances'][str(inst)] = {k: list(map(int, np.unique(est[k]))) for k in ('filter_fault_flags', 'innovation_check_flags')}
    names = {(x.name, x.multi_id) for x in u.data_list}
    if ('system_power', 0) in names:
        power = d('system_power')
        low = power['voltage5v_v'] < 4.8
        summary['power_rail'] = {'voltage5v_full_log': stats(power['voltage5v_v']),
            'below_4_8_sample_times_s': t(power)[low].tolist(),
            'below_4_5_samples': int(np.sum(power['voltage5v_v'] < 4.5)),
            'figure8': stats(power['voltage5v_v'][stage_at(t(power)) == 5])}
    if all(('estimator_local_position', i) in names for i in (0, 1)):
        est0, est1 = d('estimator_local_position', 0), d('estimator_local_position', 1)
        summary['estimator']['instance_1_minus_0_position_figure8'] = [
            stats((interp(est1, key, tp)-interp(est0, key, tp))[flight]) for key in ('x', 'y', 'z')]
    for topic, keys in [('actuator_motors', [f'control[{i}]' for i in range(4)]),
                        ('control_allocator_status', ['unallocated_thrust[0]', 'unallocated_thrust[1]', 'unallocated_thrust[2]']),
                        ('battery_status', ['voltage_v', 'connected'])]:
        data = d(topic)
        summary[topic] = {k: stats(data[k][stage_at(t(data)) == 5]) for k in keys if k in data}
    (args.out/'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False)+'\n')
    with (args.out/'tracking.csv').open('w') as out:
        writer = csv.writer(out)
        writer.writerow(['time_s', 'stage', 'x', 'y', 'z', 'ref_x', 'ref_y', 'ref_z', 'error_x', 'error_y', 'error_z'])
        writer.writerows(zip(tp, stage, *pos.T, *ref.T, *err.T))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(5, 1, figsize=(13, 13), sharex=True)
    for i, label in enumerate(('North', 'East', 'Down')):
        axes[i].plot(tp, pos[:, i], label=label+' estimate')
        axes[i].plot(t(r), r[f'position[{i}]'], '--', label='reference')
        axes[i].set_ylabel('Position [m]')
    for i, label in enumerate(('X', 'Y', 'Z')):
        axes[3].plot(tp, err[:, i], label=label)
        axes[4].plot(tb, b[f'acceleration_sp[{i}]'], label=label)
    axes[3].set_ylabel('Error [m]')
    axes[4].set_ylabel('Acceleration SP [m/s²]')
    axes[4].set_xlabel('Seconds since log start')
    for ax in axes:
        ax.legend(loc='upper right'); ax.grid(alpha=.25)
        ax.axvspan(tp[flight][0], tp[flight][-1], color='green', alpha=.06)
    fig.tight_layout()
    for ext in ('png', 'pdf'):
        fig.savefig(args.out/f'tracking.{ext}', dpi=160)
    fig2, ax = plt.subplots(figsize=(9, 5))
    ax.plot(ref[flight, 0], ref[flight, 1], '--', label='reference')
    ax.plot(pos[flight, 0], pos[flight, 1], label='estimate')
    ax.axis('equal'); ax.grid(alpha=.25); ax.legend()
    ax.set(xlabel='North [m]', ylabel='East [m]')
    fig2.tight_layout(); fig2.savefig(args.out/'trajectory.png', dpi=160)
    print(args.out/'summary.json')


if __name__ == '__main__':
    main()
