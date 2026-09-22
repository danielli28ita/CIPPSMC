"""Extended BPPSMC metrics: PPF tube entry time (convergence) and control energy."""

from __future__ import annotations

import numpy as np

# Match PositionControl.cpp ET-BPPSMC parameters
BPPSMC_A_MAX = (0.72, 0.72, 0.88)
BPPSMC_DELTA = 1.0
BPPSMC_DELTA_AXES = (1.0, 1.0, 1.0)


def infer_ppf_dir(trigger_count: np.ndarray, e1: np.ndarray) -> np.ndarray:
    """Per-sample PPF direction sign; resets at each event-trigger step (firmware _dir)."""
    tc = np.asarray(trigger_count, dtype=np.int64)
    e1 = np.asarray(e1, dtype=np.float64)
    n = len(e1)
    dir_out = np.ones(n, dtype=np.float64)
    if n == 0:
        return dir_out

    changes = np.where(np.diff(tc, prepend=int(tc[0])) > 0)[0]
    segment_starts = [0] + changes.tolist()
    for k, start in enumerate(segment_starts):
        stop = segment_starts[k + 1] if k + 1 < len(segment_starts) else n
        sign = 1.0 if e1[start] >= 0.0 else -1.0
        dir_out[start:stop] = sign
    return dir_out


def recompute_rho_ppf(
    t_us: np.ndarray,
    t0_us: np.ndarray,
    e0: np.ndarray,
    dir_sign: np.ndarray,
    a_max: float,
) -> np.ndarray:
    """Analytical PPF tube center rho = dir * e0 * cos^2(pi*tau/(2*Ts)), tau=(t-t0)."""
    t_us = np.asarray(t_us, dtype=np.float64)
    t0_us = np.asarray(t0_us, dtype=np.float64)
    e0 = np.asarray(e0, dtype=np.float64)
    dir_sign = np.asarray(dir_sign, dtype=np.float64)
    tau = (t_us - t0_us) * 1e-6
    e0_pos = np.maximum(e0, 0.0)
    ts = np.pi * np.sqrt(e0_pos / (2.0 * a_max))
    ts = np.where(ts > 0.0, ts, np.nan)
    phase = np.pi * tau / (2.0 * ts)
    rho_norm = np.cos(phase) ** 2
    active = (tau >= 0.0) & (tau < ts) & (e0_pos > 0.0) & np.isfinite(ts)
    rho = np.where(active, dir_sign * e0_pos * rho_norm, 0.0)
    return rho


def ppf_bounds_for_plot_error(
    rho: np.ndarray,
    delta: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Map tube bounds on e1=pos-sp to plotted error ex=sp-pos (= -e1)."""
    rho = np.asarray(rho, dtype=np.float64)
    upper = -rho + delta
    lower = -rho - delta
    bad = ~np.isfinite(rho)
    upper[bad] = np.nan
    lower[bad] = np.nan
    return lower, upper


def _first_entry_time(
    t: np.ndarray,
    in_tube: np.ndarray,
) -> float:
    """First time [s] where in_tube is True; nan if never."""
    t = np.asarray(t, dtype=np.float64)
    ok = np.asarray(in_tube, dtype=bool) & np.isfinite(t)
    if not np.any(ok):
        return float("nan")
    return float(t[np.argmax(ok)])


def ppf_tube_mask(
    ex: np.ndarray,
    ey: np.ndarray,
    ez: np.ndarray,
    rho_x: np.ndarray,
    rho_y: np.ndarray,
    rho_z: np.ndarray,
    delta: float = BPPSMC_DELTA,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    In-tube on controller error e1 = pos - sp = -ex_plot.
    Tube: |e1 - rho_center| <= delta  <=>  |(-ex) - rho| <= delta.
    """
    e1x, e1y, e1z = -ex, -ey, -ez
    finite = np.isfinite(rho_x) & np.isfinite(rho_y) & np.isfinite(rho_z)
    in_x = finite & (np.abs(e1x - rho_x) <= delta)
    in_y = finite & (np.abs(e1y - rho_y) <= delta)
    in_z = finite & (np.abs(e1z - rho_z) <= delta)
    return in_x, in_y, in_z


def convergence_times(
    t: np.ndarray,
    ex: np.ndarray,
    ey: np.ndarray,
    ez: np.ndarray,
    rho_x: np.ndarray,
    rho_y: np.ndarray,
    rho_z: np.ndarray,
    delta: float = BPPSMC_DELTA,
) -> dict[str, float]:
    """
    PPF first-entry times since arm [s].

    Literature (PPC/SMC): per-axis error enters prescribed bound; for UAV we report:
      - t_conv_x/y/z: per axis
      - t_conv_xy: first t when BOTH horizontal axes inside tube (mission plane)
      - t_conv_z: vertical alone (different time scale / thrust channel)
      - t_conv_all: max(t_conv_xy, t_conv_z) conservative joint
    """
    in_x, in_y, in_z = ppf_tube_mask(ex, ey, ez, rho_x, rho_y, rho_z, delta)
    t_x = _first_entry_time(t, in_x)
    t_y = _first_entry_time(t, in_y)
    t_z = _first_entry_time(t, in_z)
    in_xy = in_x & in_y
    t_xy = _first_entry_time(t, in_xy)
    t_all = float("nan")
    if np.isfinite(t_xy) and np.isfinite(t_z):
        t_all = max(t_xy, t_z)
    elif np.isfinite(t_xy):
        t_all = t_xy
    elif np.isfinite(t_z):
        t_all = t_z
    return {
        "t_conv_x_s": t_x,
        "t_conv_y_s": t_y,
        "t_conv_z_s": t_z,
        "t_conv_xy_s": t_xy,
        "t_conv_all_s": t_all,
    }


def control_energy(
    t: np.ndarray,
    ax: np.ndarray,
    ay: np.ndarray,
    az: np.ndarray,
) -> dict[str, float]:
    """
    Discrete L2 control energy J ≈ ∫ ||a||² dt (trapezoidal).
    Report horizontal (xy) separately from z: az is dominated by ~g compensation.
    """
    t = np.asarray(t, dtype=np.float64)
    ax = np.asarray(ax, dtype=np.float64)
    ay = np.asarray(ay, dtype=np.float64)
    az = np.asarray(az, dtype=np.float64)
    if len(t) < 2:
        return {
            "energy_xy": float("nan"),
            "energy_z": float("nan"),
            "energy_total": float("nan"),
            "mean_a2_xy": float("nan"),
        }
    a2_xy = ax ** 2 + ay ** 2
    a2_z = az ** 2
    a2_all = a2_xy + a2_z
    return {
        "energy_xy": float(np.trapz(a2_xy, t)),
        "energy_z": float(np.trapz(a2_z, t)),
        "energy_total": float(np.trapz(a2_all, t)),
        "mean_a2_xy": float(np.mean(a2_xy)),
    }
