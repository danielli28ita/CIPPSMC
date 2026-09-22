"""ULog time base and armed-flight window helpers for fair SITL comparisons.

Plot alignment policy: see Ldy skill ``references/sitl-plot-rules.md``.
"""

from __future__ import annotations

import numpy as np
from pyulog import ULog

# Match reference_generator default_mission.yaml mission_end_after_arm_s
MISSION_COMPARE_MAX_S = 80.0

# Unified SITL plot window — all stock PX4 / BPPSMC / compare plots share this.
FIGURE8_START_COMMAND_S = 15.0   # default_mission.yaml figure8_start_command_s
PLOT_WINDOW_SEC = 60.0
PLOT_XLIM = (0.0, PLOT_WINDOW_SEC)
PLOT_XLABEL = "Time since arm [s]"
PLOT_ALIGN_LABEL = "since_arm"

# Compare / BPPSMC plot scripts share the same arm-based axis (legacy names kept as aliases).
COMPARE_ALIGN_LABEL = PLOT_ALIGN_LABEL
COMPARE_XLABEL = PLOT_XLABEL
BPPSMC_PLOT_ALIGN_LABEL = PLOT_ALIGN_LABEL
BPPSMC_PLOT_XLABEL = PLOT_XLABEL

# Uniform resample grid for multi-log compare overlays.
RESAMPLE_DT_S = 0.02

TIME_TOPICS = (
    "trajectory_setpoint",
    "vehicle_local_position",
    "vehicle_local_position_setpoint",
    "bppsmc_debug",
    "actuator_armed",
    "vehicle_status",
)


def time_base_us(log: ULog) -> float:
    t0 = float("inf")
    for name in TIME_TOPICS:
        try:
            t0 = min(t0, float(log.get_dataset(name).data["timestamp"][0]))
        except (KeyError, IndexError):
            continue
    if not np.isfinite(t0):
        raise RuntimeError("Could not determine ULog time base")
    return t0


def armed_interval_us(log: ULog) -> tuple[float, float]:
    """Return (t_arm_us, t_disarm_us) absolute timestamps since boot."""
    aa = log.get_dataset("actuator_armed")
    t = np.asarray(aa.data["timestamp"], dtype=np.float64)
    armed = np.asarray(aa.data["armed"]).astype(bool)

    if len(t) < 2:
        raise RuntimeError("actuator_armed has too few samples")

    rising = np.where(np.diff(armed.astype(np.int8)) == 1)[0]
    falling = np.where(np.diff(armed.astype(np.int8)) == -1)[0]

    if len(rising) == 0:
        vs = log.get_dataset("vehicle_status")
        armed_time = np.asarray(vs.data["armed_time"], dtype=np.float64)
        valid = armed_time[armed_time > 0]
        if len(valid) == 0:
            raise RuntimeError("Could not find arm time in ULog")
        t_arm = float(valid[0])
        t_disarm = float(t[-1])
        return t_arm, t_disarm

    arm_idx = int(rising[0]) + 1
    t_arm = float(t[arm_idx])
    later = falling[falling >= rising[0]]
    if len(later) == 0:
        t_disarm = float(t[-1])
    else:
        t_disarm = float(t[int(later[0]) + 1])
    return t_arm, t_disarm


def armed_window_sec(log: ULog) -> tuple[float, float, float]:
    """Armed interval relative to shared log time base: (t_arm, t_disarm, duration) [s]."""
    t0 = time_base_us(log)
    t_arm_us, t_disarm_us = armed_interval_us(log)
    t_arm = (t_arm_us - t0) * 1e-6
    t_disarm = (t_disarm_us - t0) * 1e-6
    duration = max(t_disarm - t_arm, 0.0)
    return t_arm, t_disarm, duration


def sort_series_by_time(series: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Sort all per-sample arrays by ascending time (ULog topics may be non-monotonic)."""
    t = np.asarray(series["t"], dtype=np.float64)
    n = len(t)
    if n == 0:
        return series
    order = np.argsort(t, kind="stable")
    out: dict[str, np.ndarray] = {}
    for key, value in series.items():
        if isinstance(value, np.ndarray) and len(value) == n:
            out[key] = value[order]
        else:
            out[key] = value
    return out


def crop_series_since_arm(
    series: dict[str, np.ndarray],
    log: ULog,
    window_sec: float | None = None,
) -> dict[str, np.ndarray]:
    """Keep samples in [arm, arm + window_sec); re-zero time axis at arm."""
    cropped, _ = crop_series_plot_window(series, log, window_sec=window_sec)
    return cropped


def crop_series_plot_window(
    series: dict[str, np.ndarray],
    log: ULog,
    window_sec: float = PLOT_WINDOW_SEC,
) -> tuple[dict[str, np.ndarray], dict[str, float]]:
    """Crop to [arm, arm + window_sec]; re-zero time at arm (official flight start).

    All SITL plot scripts use this so PID / BPPSMC / compare share one 60 s axis
    and BPPSMC PPF convergence from arm is visible.
    """
    t_arm, t_disarm, armed_duration = armed_window_sec(log)
    if armed_duration <= 0:
        raise RuntimeError("Armed flight duration is zero")

    duration = min(float(window_sec), armed_duration, max(t_disarm - t_arm, 0.0))
    t_end = t_arm + duration
    t = np.asarray(series["t"], dtype=np.float64)
    mask = (t >= t_arm) & (t <= t_end)
    if not np.any(mask):
        raise RuntimeError(
            f"No samples in plot window [{t_arm:.2f}, {t_end:.2f}] s (arm + {window_sec}s)"
        )

    out: dict[str, np.ndarray] = {}
    n = len(t)
    for key, value in series.items():
        if isinstance(value, np.ndarray) and len(value) == n:
            out[key] = value[mask]
        else:
            out[key] = value
    out["t"] = t[mask] - t_arm

    meta: dict[str, float] = {
        "arm_log_s": t_arm,
        "plot_window_s": duration,
        "align_label": PLOT_ALIGN_LABEL,
    }
    try:
        t_cmd = detect_command_start_sec(log)
        meta["command_start_log_s"] = t_cmd
        meta["command_before_arm_s"] = t_cmd - t_arm
    except RuntimeError:
        pass
    try:
        t8, since_cmd = detect_figure8_start_since_command(log)
        meta["figure8_start_log_s"] = t8
        meta["figure8_since_arm_s"] = t8 - t_arm
        meta["figure8_since_command_s"] = since_cmd
    except RuntimeError:
        pass

    return sort_series_by_time(out), meta


def resample_since_arm(
    series: dict[str, np.ndarray],
    t_arm_log: float,
    duration: float,
    dt: float = RESAMPLE_DT_S,
) -> dict[str, np.ndarray]:
    """Uniform grid t=0..duration with t=0 at arm (log time ``t_arm_log``)."""
    t_log = np.asarray(series["t"], dtype=np.float64)
    n = len(t_log)
    t_rel = np.arange(0.0, duration + 0.5 * dt, dt)
    t_abs = t_arm_log + t_rel
    out: dict[str, np.ndarray] = {"t": t_rel}
    for key, val in series.items():
        if key == "t":
            continue
        if isinstance(val, np.ndarray) and len(val) == n:
            out[key] = np.interp(t_abs, t_log, val, left=np.nan, right=np.nan)
        else:
            out[key] = val
    return out


PLOT_MIN_FINITE_FRAC = 0.98


def validate_arm_plot_coverage(
    series: dict[str, np.ndarray],
    duration: float,
    *,
    keys: tuple[str, ...] = ("ex", "ey", "ez"),
) -> None:
    """Ensure the arm-aligned window has continuous data over [0, duration]."""
    t = np.asarray(series["t"], dtype=np.float64)
    if len(t) < int(duration / RESAMPLE_DT_S * 0.9):
        raise RuntimeError(
            f"Too few samples ({len(t)}) for {duration:.0f}s arm plot window"
        )
    if t[0] > 0.05:
        raise RuntimeError(
            f"Plot series must start at t=0 after arm (got t0={t[0]:.3f}s)"
        )
    if t[-1] < duration - 2 * RESAMPLE_DT_S:
        raise RuntimeError(
            f"Plot series ends at t={t[-1]:.2f}s, expected ~{duration:.0f}s"
        )
    for key in keys:
        if key not in series:
            continue
        y = np.asarray(series[key], dtype=np.float64)
        frac = float(np.mean(np.isfinite(y)))
        if frac < PLOT_MIN_FINITE_FRAC:
            raise RuntimeError(
                f"{key}: only {frac:.1%} finite samples in arm plot window"
            )


def prepare_arm_plot_series(
    series: dict[str, np.ndarray],
    log: ULog,
    window_sec: float = PLOT_WINDOW_SEC,
) -> tuple[dict[str, np.ndarray], dict[str, float]]:
    """Resample to uniform t=0..window from arm; reject sparse / empty windows."""
    t_arm, t_disarm, _ = armed_window_sec(log)
    duration = min(float(window_sec), max(t_disarm - t_arm, 0.0))
    if duration < 10.0:
        raise RuntimeError(
            f"Armed duration {duration:.1f}s too short for {window_sec:.0f}s plot"
        )

    resampled = resample_since_arm(series, t_arm, duration)
    validate_arm_plot_coverage(resampled, duration)

    meta: dict[str, float] = {
        "arm_log_s": t_arm,
        "plot_window_s": duration,
        "align_label": PLOT_ALIGN_LABEL,
    }
    try:
        t_cmd = detect_command_start_sec(log)
        meta["command_start_log_s"] = t_cmd
        meta["command_before_arm_s"] = t_cmd - t_arm
    except RuntimeError:
        pass
    try:
        t8, since_cmd = detect_figure8_start_since_command(log)
        meta["figure8_start_log_s"] = t8
        meta["figure8_since_arm_s"] = t8 - t_arm
        meta["figure8_since_command_s"] = since_cmd
    except RuntimeError:
        pass
    return resampled, meta


def detect_bppsmc_debug_start_sec(log: ULog) -> float:
    """First bppsmc_debug sample in log-relative seconds (ET-BPPSMC active / ULog start)."""
    t0 = time_base_us(log)
    dbg = log.get_dataset("bppsmc_debug")
    ts = np.asarray(dbg.data["timestamp"], dtype=np.float64)
    if len(ts) == 0:
        raise RuntimeError("bppsmc_debug has no samples")
    return (float(ts[0]) - t0) * 1e-6


def crop_series_since_log_t(
    series: dict[str, np.ndarray],
    t_start_log: float,
    window_sec: float = PLOT_WINDOW_SEC,
    t_end_log: float | None = None,
) -> tuple[dict[str, np.ndarray], dict[str, float]]:
    """Crop to [t_start_log, t_start_log + window_sec]; re-zero time at t_start_log."""
    t_end = t_start_log + window_sec if t_end_log is None else min(t_start_log + window_sec, t_end_log)
    t = np.asarray(series["t"], dtype=np.float64)
    mask = (t >= t_start_log) & (t <= t_end)
    if not np.any(mask):
        raise RuntimeError(
            f"No samples in plot window [{t_start_log:.2f}, {t_end:.2f}] s"
        )

    out: dict[str, np.ndarray] = {}
    n = len(t)
    for key, value in series.items():
        if isinstance(value, np.ndarray) and len(value) == n:
            out[key] = value[mask]
        else:
            out[key] = value
    out["t"] = t[mask] - t_start_log

    meta = {
        "window_start_log_s": float(t_start_log),
        "plot_window_s": float(t_end - t_start_log),
    }
    return sort_series_by_time(out), meta


def crop_series_compare_bppsmc_window(
    series: dict[str, np.ndarray],
    log: ULog,
    offset_since_arm_s: float,
    window_sec: float = PLOT_WINDOW_SEC,
) -> tuple[dict[str, np.ndarray], dict[str, float]]:
    """Crop using mission-phase offset since arm (for fair PID vs BPPSMC compare)."""
    t_arm, t_disarm, _ = armed_window_sec(log)
    t_start = t_arm + offset_since_arm_s
    if t_start >= t_disarm:
        raise RuntimeError(
            f"Compare start {offset_since_arm_s:.2f}s after arm is past disarm"
        )
    available = max(t_disarm - t_start, 0.0)
    duration = min(float(window_sec), available)
    cropped, meta = crop_series_since_log_t(series, t_start, window_sec=duration)
    meta["arm_log_s"] = t_arm
    meta["offset_since_arm_s"] = float(offset_since_arm_s)
    meta["align_label"] = COMPARE_ALIGN_LABEL
    return cropped, meta


def bppsmc_debug_offset_since_arm(log: ULog) -> float:
    """Seconds from arm to first bppsmc_debug sample."""
    t_arm, _, _ = armed_window_sec(log)
    return detect_bppsmc_debug_start_sec(log) - t_arm


def crop_series_bppsmc_plot_window(
    series: dict[str, np.ndarray],
    log: ULog, window_sec: float = PLOT_WINDOW_SEC,
) -> tuple[dict[str, np.ndarray], dict[str, float]]:
    """BPPSMC single-flight: [arm, arm + window_sec]; t=0 at arm."""
    return crop_series_plot_window(series, log, window_sec=window_sec)


def mission_compare_window_sec(*armed_durations: float) -> float:
    """Common evaluation window: min(60s mission, each log's armed duration)."""
    if not armed_durations:
        raise ValueError("armed_durations must not be empty")
    return min(MISSION_COMPARE_MAX_S, *armed_durations)


def _trajectory_arrays(log: ULog) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    t0 = time_base_us(log)
    ts = log.get_dataset("trajectory_setpoint")
    t = (np.asarray(ts.data["timestamp"], dtype=np.float64) - t0) * 1e-6
    px = np.asarray(ts.data["position[0]"], dtype=np.float64)
    py = np.asarray(ts.data["position[1]"], dtype=np.float64)
    pz = np.asarray(ts.data["position[2]"], dtype=np.float64)
    return t, px, py, pz


def detect_command_start_sec(log: ULog) -> float:
    """First valid hover setpoint (t_log when sp_z ≈ -5 m NED)."""
    t0 = time_base_us(log)
    ts = log.get_dataset("trajectory_setpoint")
    t = (np.asarray(ts.data["timestamp"], dtype=np.float64) - t0) * 1e-6
    pz = np.asarray(ts.data["position[2]"], dtype=np.float64)
    for i in range(len(t)):
        if np.isfinite(pz[i]) and abs(pz[i] + 5.0) < 0.5:
            return float(t[i])
    raise RuntimeError("Could not detect hover/command start in ULog")


def detect_figure8_start_since_command(
    log: ULog,
    figure8_start_command_s: float = 15.0,
) -> tuple[float, float]:
    """Return (t_log at figure8, seconds since hover/command start)."""
    t_cmd = detect_command_start_sec(log)
    t0 = time_base_us(log)
    ts = log.get_dataset("trajectory_setpoint")
    t = (np.asarray(ts.data["timestamp"], dtype=np.float64) - t0) * 1e-6
    px = np.asarray(ts.data["position[0]"], dtype=np.float64)
    py = np.asarray(ts.data["position[1]"], dtype=np.float64)
    t_nom = t_cmd + figure8_start_command_s
    mask = t >= t_cmd + figure8_start_command_s - 0.5
    if np.any(mask):
        t_m = t[mask]
        r_m = np.hypot(px[mask], py[mask])
        for i in range(len(t_m)):
            if r_m[i] > 0.5:
                return float(t_m[i]), float(t_m[i] - t_cmd)
    return t_nom, figure8_start_command_s


def crop_series_figure8_window(
    series: dict[str, np.ndarray],
    log: ULog,
    window_sec: float = PLOT_WINDOW_SEC,
    figure8_start_command_s: float = FIGURE8_START_COMMAND_S,
) -> tuple[dict[str, np.ndarray], dict[str, float]]:
    """Crop to [figure-8 start, figure-8 start + window_sec]; re-zero at figure-8 start.

    Used by tuning / batch metrics only; SITL plot scripts use crop_series_plot_window().
    """
    t8, since_cmd = detect_figure8_start_since_command(log, figure8_start_command_s)
    try:
        t_cmd = detect_command_start_sec(log)
    except RuntimeError:
        t_cmd = t8 - figure8_start_command_s

    t_end = t8 + window_sec
    t = np.asarray(series["t"], dtype=np.float64)
    mask = (t >= t8) & (t <= t_end)
    if not np.any(mask):
        raise RuntimeError(
            f"No samples in plot window [{t8:.2f}, {t_end:.2f}] s (figure-8 + {window_sec}s)"
        )

    out: dict[str, np.ndarray] = {}
    n = len(t)
    for key, value in series.items():
        if isinstance(value, np.ndarray) and len(value) == n:
            out[key] = value[mask]
        else:
            out[key] = value
    out["t"] = t[mask] - t8

    meta = {
        "command_start_log_s": t_cmd,
        "figure8_start_log_s": t8,
        "figure8_since_command_s": since_cmd,
        "plot_window_s": float(window_sec),
        "align_label": "figure8_start_command+15s",
    }
    return sort_series_by_time(out), meta


def detect_figure8_start_since_arm(
    log: ULog,
    post_arm_hold_s: float = 5.0,
    radius_threshold_m: float = 0.5,
    anchor_xy: tuple[float, float] = (0.0, 0.0),
) -> float:
    """Seconds after ULog arm when xy setpoint leaves hover (figure-8 start)."""
    t_arm, _, _ = armed_window_sec(log)
    t, px, py, _ = _trajectory_arrays(log)
    ta = t - t_arm
    mask = ta >= 0.0
    if not np.any(mask):
        raise RuntimeError("No trajectory samples after arm")
    ta = ta[mask]
    px = px[mask]
    py = py[mask]
    ax, ay = anchor_xy
    r = np.hypot(px - ax, py - ay)
    for i in range(len(ta)):
        if ta[i] >= max(0.0, post_arm_hold_s - 0.5) and r[i] > radius_threshold_m:
            return float(ta[i])
    return float(post_arm_hold_s)


def detect_return_start_since_arm(log: ULog, mission_end_s: float = 80.0) -> float:
    """Seconds after arm when mission enters return (setpoint recenters after figure-8)."""
    t_arm, _, _ = armed_window_sec(log)
    t, px, py, pz = _trajectory_arrays(log)
    ta = t - t_arm
    mask = ta >= mission_end_s - 2.0
    if not np.any(mask):
        return float(mission_end_s)
    ta_m = ta[mask]
    px_m = px[mask]
    py_m = py[mask]
    pz_m = pz[mask]
    n0 = min(30, max(1, len(ta_m) // 4))
    ax = float(np.median(px_m[:n0]))
    ay = float(np.median(py_m[:n0]))
    r = np.hypot(px_m - ax, py_m - ay)
    for i in range(len(ta_m)):
        if ta_m[i] >= mission_end_s - 0.5 and r[i] < 0.3 and abs(pz_m[i] + 5.0) < 0.2:
            return float(ta_m[i])
    return float(mission_end_s)


def crop_series_since_offset(
    series: dict[str, np.ndarray],
    log: ULog,
    offset_since_arm_s: float,
    window_sec: float | None = None,
) -> dict[str, np.ndarray]:
    """Crop to [arm + offset, arm + offset + window); re-zero time at offset."""
    t_arm, t_disarm, armed_duration = armed_window_sec(log)
    t_start = t_arm + offset_since_arm_s
    if t_start >= t_disarm:
        raise RuntimeError(f"Offset {offset_since_arm_s}s past disarm ({armed_duration}s armed)")

    available = max(t_disarm - t_start, 0.0)
    duration = available if window_sec is None else min(available, window_sec)
    t_end = t_start + duration
    t = series["t"]
    mask = (t >= t_start) & (t <= t_end)
    if not np.any(mask):
        raise RuntimeError("No samples in cropped mission window")

    out: dict[str, np.ndarray] = {}
    n = len(t)
    for key, value in series.items():
        if isinstance(value, np.ndarray) and len(value) == n:
            out[key] = value[mask]
        else:
            out[key] = value
    out["t"] = t[mask] - t_start
    out["_align_offset_s"] = np.array([offset_since_arm_s])
    out["_compare_window_s"] = np.array([duration])
    return out
