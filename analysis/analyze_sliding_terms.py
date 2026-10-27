"""Decompose logged BPPSMC and compare gains at frozen logged states.

Only valid for a figure-eight window without reanchors and with zero PPF center.
This is parameter sensitivity, not closed-loop simulation or flight validation.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def stats(values):
    x = np.asarray(values, float)
    x = x[np.isfinite(x)]
    return {'mean': float(np.mean(x)), 'rms': float(np.sqrt(np.mean(x*x))),
            'abs_p95': float(np.percentile(np.abs(x), 95)),
            'min': float(np.min(x)), 'max': float(np.max(x))}


def local_gains(params, suffix):
    ks, k0, k1, phi = [params[f'BPP_{k}_{suffix}'] for k in ('KS', 'K0', 'K1', 'PHI')]
    return {'kp': ks*(k0+k1/phi), 'kd': ks+k0+k1/phi}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--configs', type=Path, help='JSON mapping candidate names to parameter overrides; baseline is always included')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    log = ULog(str(args.log))
    params = log.initial_parameters
    data = lambda name: log.get_dataset(name).data
    time = lambda d: (d['timestamp'].astype(float)-log.start_timestamp)*1e-6
    debug, pos, ref, task = map(data, ['bppsmc_debug', 'vehicle_local_position', 'trajectory_setpoint', 'figure8_status'])
    all_times = time(debug)
    stages = task['stage'][np.clip(np.searchsorted(time(task), all_times, side='right')-1, 0, len(time(task))-1)]
    mask = stages == 5
    t = all_times[mask]
    interp = lambda d, key: np.interp(t, time(d), d[key])
    configs = {'baseline': {}}
    if args.configs:
        configs = {**json.loads(args.configs.read_text()), 'baseline': {}}
    result = {'log_sha256': hashlib.sha256(args.log.read_bytes()).hexdigest(),
              'window_s': [float(t[0]), float(t[-1])], 'samples': len(t),
              'method': 'Position/reference interpolated; velocity error inferred from logged S and baseline lambda. Frozen-state changes do not predict flight trajectories.',
              'source_sha256': hashlib.sha256((Path(__file__).resolve().parents[1]/'firmware/overlay/src/modules/mc_pos_control/PositionControl/PositionControl.cpp').read_bytes()).hexdigest(),
              'axes': [], 'configs': {}}
    evaluations = {name: [] for name in configs}
    term_series = []
    for i, key in enumerate(('x', 'y', 'z')):
        suffix = 'XY' if i < 2 else 'Z'
        rho = debug[f'rho_center[{i}]'][mask]
        counts = debug[f'trigger_count[{i}]'][mask]
        if np.any(counts != 0) or np.any(rho != 0):
            raise ValueError('This diagnostic requires zero centers and no accepted events')
        e = interp(pos, key)-interp(ref, f'position[{i}]')
        band = np.maximum(params['BPP_DELTA']*debug[f'e0[{i}]'][mask], params[f'BPP_BAND_{suffix}'])
        z = np.clip((e+band)/(2*band), 1e-6, 1-1e-6)
        eps = np.log(z/(1-z))
        jac = 1/(2*band*z*(1-z))
        tau = (debug['timestamp'][mask].astype(float)-debug[f't0[{i}]'][mask])*1e-6
        ts = np.maximum(0, tau-params['BPP_T_PRE'])
        denom = params['BPP_EPS_SLIDE']*params['BPP_T_SLIDE']**2+ts**2
        eta = ts/denom
        eta_dot = (params['BPP_EPS_SLIDE']*params['BPP_T_SLIDE']**2-ts**2)/denom**2*(tau > params['BPP_T_PRE'])
        gamma = tau/(params['BPP_EPS_PRE']*params['BPP_T_PRE']**2+tau**2)
        surface = debug[f'sliding_surface[{i}]'][mask].astype(float)
        eps_dot = surface-(params[f'BPP_KS_{suffix}']+eta)*eps
        e2 = eps_dot/jac
        jac_dot = -jac*jac*e2*(1-2*z)
        aff = interp(ref, f'acceleration[{i}]')
        baseline_terms = None
        for name, overrides in configs.items():
            q = {**params, **overrides}
            lam = q[f'BPP_KS_{suffix}']+eta
            kappa = q[f'BPP_K0_{suffix}']+gamma
            s = eps_dot+lam*eps
            terms = {'feedforward': aff, 'jacobian': -jac_dot*e2/jac,
                     'surface_derivative': -lam*eps_dot/jac, 'lambda_derivative': -eta_dot*eps/jac,
                     'linear_reaching': -kappa*s/jac,
                     'switching': -q[f'BPP_K1_{suffix}']*np.tanh(s/q[f'BPP_PHI_{suffix}'])/jac}
            acc = np.clip(sum(terms.values()), -q[f'BPP_ACC_{suffix}'], q[f'BPP_ACC_{suffix}'])
            evaluations[name].append(acc)
            if name == 'baseline':
                baseline_terms = terms
        term_series.append(baseline_terms)
        result['axes'].append({'axis': key, 'surface': stats(surface),
            'abs_surface_over_phi': stats(np.abs(surface/params[f'BPP_PHI_{suffix}'])),
            'outside_phi_fraction': float(np.mean(np.abs(surface)>params[f'BPP_PHI_{suffix}'])),
            'baseline_acceleration_reconstruction_residual': stats(evaluations['baseline'][i]-debug[f'acceleration_sp[{i}]'][mask]),
            'inferred_velocity_error_minus_interpolated': stats(e2-(interp(pos, 'v'+key)-interp(ref, f'velocity[{i}]'))),
            'terms_m_s2': {k: stats(v) for k, v in baseline_terms.items()},
            'lambda': stats(params[f'BPP_KS_{suffix}']+eta), 'kappa': stats(params[f'BPP_K0_{suffix}']+gamma)})
    for name, overrides in configs.items():
        q = {**params, **overrides}
        result['configs'][name] = {'overrides': overrides,
            'local_gains': {suffix: local_gains(q, suffix) for suffix in ('XY', 'Z')},
            'delta_acceleration_xyz_m_s2': [stats(evaluations[name][i]-evaluations['baseline'][i]) for i in range(3)]}
    (args.out/'sliding_terms.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(t, debug[f'acceleration_sp[{i}]'][mask], label='logged total', color='black', alpha=.7)
        for name in ('linear_reaching', 'switching', 'surface_derivative'):
            ax.plot(t, term_series[i][name], label=name)
        ax.set_ylabel(f'{"XYZ"[i]} [m/s²]'); ax.grid(alpha=.25); ax.legend(loc='upper right')
    axes[-1].set_xlabel('Seconds since ULog start')
    fig.tight_layout(); fig.savefig(args.out/'sliding_terms.png', dpi=160)
    print(args.out/'sliding_terms.json')


if __name__ == '__main__':
    main()
