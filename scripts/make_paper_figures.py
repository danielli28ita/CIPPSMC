#!/usr/bin/env python3
"""Paper-style figures for the single real-flight sample.

Draws only log_19. The layout follows the paper's per-axis error figure and
commanded-acceleration figure: three stacked panels, no grid, the prescribed
corridor on the vertical axis only. It does not read or plot any simulator log.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog

C_TRACE = "#0B5CAD"
C_CENTER = "#6B7280"
C_LIMIT = "#B7791F"


def _dataset(ulog: ULog, name: str):
    for item in ulog.data_list:
        if item.name == name and item.multi_id == 0:
            return item.data
    raise KeyError(name)


def _seconds(ulog: ULog, timestamps) -> np.ndarray:
    return (np.asarray(timestamps, float) - ulog.start_timestamp) * 1e-6


def _interp(t_src, values, t_dst) -> np.ndarray:
    mask = np.isfinite(values)
    if int(mask.sum()) < 2:
        return np.full_like(t_dst, np.nan, dtype=float)
    return np.interp(t_dst, t_src[mask], values[mask])


def _resets(time_s: np.ndarray, count: np.ndarray) -> list[float]:
    jumps = np.flatnonzero(np.diff(count.astype(np.int64)) > 0) + 1
    return [float(time_s[i]) for i in jumps if 0 <= i < len(time_s)]


def _style():
    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 8,
        "axes.labelsize": 8,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 6.5,
        "axes.linewidth": 0.8,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": False,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })


def _panels(n: int):
    fig, axes = plt.subplots(n, 1, figsize=(7.16, 0.95 * n + 0.55), sharex=True)
    fig.subplots_adjust(left=0.12, right=0.97, top=0.96, bottom=0.10, hspace=0.28)
    return fig, np.atleast_1d(axes)


def _label(ax, text: str):
    ax.text(0.0, 1.08, text, transform=ax.transAxes, fontsize=8, fontweight="bold", va="bottom")


def load(path: Path):
    ulog = ULog(str(path))
    position = _dataset(ulog, "vehicle_local_position")
    reference = _dataset(ulog, "trajectory_setpoint")
    debug = _dataset(ulog, "bppsmc_debug")
    status = _dataset(ulog, "figure8_status")
    t = _seconds(ulog, position["timestamp"])
    t_ref = _seconds(ulog, reference["timestamp"])
    t_dbg = _seconds(ulog, debug["timestamp"])
    t_status = _seconds(ulog, status["timestamp"])
    stage = status["stage"][np.clip(np.searchsorted(t_status, t, side="right") - 1, 0, len(t_status) - 1)]
    params = ulog.initial_parameters
    bands = (float(params["BPP_BAND_XY"]), float(params["BPP_BAND_XY"]), float(params["BPP_BAND_Z"]))
    limits = (float(params["BPP_ACC_XY"]), float(params["BPP_ACC_XY"]), float(params["BPP_ACC_Z"]))
    delta = float(params["BPP_DELTA"])
    axes = {}
    for i, name in enumerate("xyz"):
        measured = np.asarray(position[name], float)
        reference_i = _interp(t_ref, np.asarray(reference[f"position[{i}]"], float), t)
        rho = _interp(t_dbg, np.asarray(debug[f"rho_center[{i}]"], float), t)
        e0 = _interp(t_dbg, np.asarray(debug[f"e0[{i}]"], float), t)
        half_width = np.maximum(delta * np.abs(e0), bands[i])
        axes[name] = {
            "error": measured - reference_i,
            "rho": rho,
            "half_width": half_width,
            "reference": reference_i,
            "measured": measured,
            "acceleration": _interp(t_dbg, np.asarray(debug[f"acceleration_sp[{i}]"], float), t),
            "limit": limits[i],
            "resets": _resets(t_dbg, np.asarray(debug[f"trigger_count[{i}]"])),
        }
    return t, stage, axes


def save(fig, out: Path, stem: str):
    out.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(out / f"{stem}.{ext}", dpi=200)
    plt.close(fig)
    print(out / f"{stem}.png")


def figure_error(t, axes_data, out: Path):
    fig, axes = _panels(3)
    for ax, name, panel in zip(axes, "xyz", ("(a)", "(b)", "(c)")):
        series = axes_data[name]
        if name == "z":
            ax.fill_between(t, series["rho"] - series["half_width"], series["rho"] + series["half_width"],
                            color="0.90", linewidth=0, label=r"corridor $\rho_c\pm b$", zorder=0)
            ax.plot(t, series["rho"], color=C_CENTER, linestyle="--", linewidth=0.8,
                    label=r"corridor center $\rho_c$", zorder=1)
        ax.plot(t, series["error"], color=C_TRACE, linewidth=1.2, label="position error", zorder=2)
        for instant in series["resets"]:
            ax.axvline(instant, color=C_TRACE, linewidth=0.6, ymin=0.0, ymax=0.12, zorder=3)
        ax.axhline(0.0, color="0.80", linewidth=0.5, zorder=0)
        ax.set_ylabel(f"$e_{name}$ (m)")
        _label(ax, panel)
    axes[2].legend(loc="upper right", frameon=True, framealpha=0.9, edgecolor="none", ncol=2)
    axes[-1].set_xlabel("Time from log start (s)")
    axes[-1].set_xlim(t[0], t[-1])
    save(fig, out, "error_time")


def figure_acceleration(t, axes_data, out: Path):
    fig, axes = _panels(3)
    for ax, name, panel in zip(axes, "xyz", ("(a)", "(b)", "(c)")):
        series = axes_data[name]
        limit = series["limit"]
        ax.axhline(limit, color=C_LIMIT, linestyle=":", linewidth=0.8, label=r"$a_{\mathrm{lim}}$")
        ax.axhline(-limit, color=C_LIMIT, linestyle=":", linewidth=0.8)
        ax.plot(t, series["acceleration"], color=C_TRACE, linewidth=1.0, label=r"$a_{sp}$")
        span = max(limit * 1.15, np.nanmax(np.abs(series["acceleration"])) * 1.05)
        ax.set_ylim(-span, span)
        ax.set_ylabel(f"$a_{{sp,{name}}}$ (m/s$^2$)")
        _label(ax, panel)
    axes[0].legend(loc="upper right", frameon=True, framealpha=0.9, edgecolor="none")
    axes[-1].set_xlabel("Time from log start (s)")
    axes[-1].set_xlim(t[0], t[-1])
    save(fig, out, "acceleration_time")


def figure_trajectory(t, stage, axes_data, out: Path):
    fig, ax = plt.subplots(figsize=(3.5, 3.4))
    mask = stage == 5
    if not np.any(mask):
        mask = np.ones(len(t), dtype=bool)
    ax.plot(axes_data["y"]["reference"][mask], axes_data["x"]["reference"][mask],
            color=C_CENTER, linestyle="--", linewidth=0.9, label="reference")
    ax.plot(axes_data["y"]["measured"][mask], axes_data["x"]["measured"][mask],
            color=C_TRACE, linewidth=1.2, label="estimate")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.legend(loc="best", frameon=True, framealpha=0.9, edgecolor="none")
    fig.subplots_adjust(left=0.18, right=0.97, bottom=0.14, top=0.96)
    save(fig, out, "trajectory_xy")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    _style()
    t, stage, axes_data = load(args.log)
    figure_error(t, axes_data, args.out)
    figure_acceleration(t, axes_data, args.out)
    figure_trajectory(t, stage, axes_data, args.out)


if __name__ == "__main__":
    main()
