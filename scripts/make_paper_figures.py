#!/usr/bin/env python3
"""Paper-style figures for the two real flights.

Flight A is drawn solid and Flight B dash-dot. Dense series have no markers.
The prescribed corridor is drawn only on the vertical error panel. Simulator
logs are not read.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog

# Okabe-Ito sky blue and orange, slightly lifted, with linestyle as a second cue.
FLIGHTS = (
    {"key": "A", "label": "Flight A", "color": "#5DADE2", "ls": "-", "lw": 1.5, "zorder": 2},
    {"key": "B", "label": "Flight B", "color": "#E69F00", "ls": "-.", "lw": 1.25, "zorder": 3},
)
C_LIMIT = "#6B7280"
TWO_COL = 7.16
ONE_COL = 3.5


def _dataset(ulog: ULog, name: str):
    for item in ulog.data_list:
        if item.name == name and item.multi_id == 0:
            return item.data
    raise KeyError(name)


def _arm_us(ulog: ULog) -> int:
    for message in ulog.logged_messages:
        if "Armed by" in message.message:
            return int(message.timestamp)
    armed = _dataset(ulog, "actuator_armed")
    idx = np.flatnonzero(np.asarray(armed["armed"]).astype(bool))
    return int(armed["timestamp"][idx[0]]) if len(idx) else int(ulog.start_timestamp)


def _seconds(origin_us: int, timestamps) -> np.ndarray:
    return (np.asarray(timestamps, np.int64) - origin_us) * 1e-6


def _interp(t_src, values, t_dst) -> np.ndarray:
    values = np.asarray(values, float)
    mask = np.isfinite(values)
    if int(mask.sum()) < 2:
        return np.full(np.shape(t_dst), np.nan)
    return np.interp(t_dst, t_src[mask], values[mask])


def _resets(time_s: np.ndarray, count: np.ndarray) -> list[float]:
    jumps = np.flatnonzero(np.diff(np.asarray(count, np.int64)) > 0) + 1
    return [float(time_s[i]) for i in jumps if time_s[i] > 0.5]


def load(path: Path) -> dict:
    ulog = ULog(str(path))
    origin = _arm_us(ulog)
    position = _dataset(ulog, "vehicle_local_position")
    reference = _dataset(ulog, "trajectory_setpoint")
    debug = _dataset(ulog, "bppsmc_debug")
    status = _dataset(ulog, "figure8_status")
    t = _seconds(origin, position["timestamp"])
    t_ref = _seconds(origin, reference["timestamp"])
    t_dbg = _seconds(origin, debug["timestamp"])
    t_status = _seconds(origin, status["timestamp"])
    stage = status["stage"][np.clip(np.searchsorted(t_status, t, side="right") - 1, 0, len(t_status) - 1)]
    params = ulog.initial_parameters
    bands = (float(params["BPP_BAND_XY"]), float(params["BPP_BAND_XY"]), float(params["BPP_BAND_Z"]))
    limits = (float(params["BPP_ACC_XY"]), float(params["BPP_ACC_XY"]), float(params["BPP_ACC_Z"]))
    delta = float(params["BPP_DELTA"])
    axes = {}
    for i, name in enumerate("xyz"):
        measured = np.asarray(position[name], float)
        reference_i = _interp(t_ref, reference[f"position[{i}]"], t)
        e0 = _interp(t_dbg, debug[f"e0[{i}]"], t)
        axes[name] = {
            "error": measured - reference_i,
            "rho": _interp(t_dbg, debug[f"rho_center[{i}]"], t),
            "half_width": np.maximum(delta * np.abs(e0), bands[i]),
            "reference": reference_i,
            "measured": measured,
            "acceleration": _interp(t_dbg, debug[f"acceleration_sp[{i}]"], t),
            "limit": limits[i],
            "resets": _resets(t_dbg, debug[f"trigger_count[{i}]"]),
        }
    return {"t": t, "stage": stage.astype(int), "axes": axes}


def _style():
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Nimbus Roman", "STIXGeneral", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 8,
        "axes.labelsize": 8,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 7,
        "axes.linewidth": 0.8,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": False,
        "legend.frameon": False,
        "xtick.direction": "out",
        "ytick.direction": "out",
        "axes.unicode_minus": False,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "savefig.dpi": 300,
    })


def _panels(n: int, width: float):
    fig, axes = plt.subplots(n, 1, figsize=(width, 0.92 * n + 0.62), sharex=True)
    fig.subplots_adjust(left=0.11, right=0.98, top=0.94, bottom=0.16, hspace=0.34)
    return fig, np.atleast_1d(axes)


def _panel_label(ax, text: str):
    ax.text(-0.11, 1.02, text, transform=ax.transAxes, fontsize=8, fontweight="bold", va="bottom", ha="left")


def _save(fig, out: Path, stem: str):
    out.mkdir(parents=True, exist_ok=True)
    for ext in ("pdf", "png"):
        fig.savefig(out / f"{stem}.{ext}", bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print(out / f"{stem}.png")


def figure_error(flights: list[dict], out: Path):
    fig, axes = _panels(3, TWO_COL)
    fig.subplots_adjust(top=0.88)
    t_end = max(float(item["t"][-1]) for item in flights)
    for ax, name, panel in zip(axes, "xyz", ("(a)", "(b)", "(c)")):
        if name == "z":
            band = flights[0]["axes"][name]
            ax.fill_between(
                flights[0]["t"], band["rho"] - band["half_width"], band["rho"] + band["half_width"],
                color="0.88", linewidth=0, zorder=0, label=r"Flight A corridor $\rho_c\pm b$",
            )
            ax.plot(flights[0]["t"], band["rho"], color="0.45", linestyle="--", linewidth=0.7,
                    zorder=1, label=r"Flight A center $\rho_c$")
        for spec, flight in zip(FLIGHTS, flights):
            ax.plot(flight["t"], flight["axes"][name]["error"], color=spec["color"], linestyle=spec["ls"],
                    linewidth=spec["lw"], zorder=spec["zorder"], label=spec["label"])
            for instant in flight["axes"][name]["resets"]:
                ax.axvline(instant, color=spec["color"], linewidth=0.5, ymin=0.0, ymax=0.08, zorder=4)
        ax.axhline(0.0, color="0.82", linewidth=0.4, zorder=0)
        ax.set_ylabel(f"$e_{name}$ (m)")
        _panel_label(ax, panel)
    axes[-1].set_xlim(0.0, t_end)
    axes[-1].set_xlabel("Time from arming (s)")
    handles, labels = axes[2].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.55, 0.995),
               ncol=4, frameon=False, handlelength=2.2, columnspacing=1.0)
    _save(fig, out, "error_time")


def figure_acceleration(flights: list[dict], out: Path):
    fig, axes = _panels(3, TWO_COL)
    t_end = max(float(item["t"][-1]) for item in flights)
    for ax, name, panel in zip(axes, "xyz", ("(a)", "(b)", "(c)")):
        limit = flights[0]["axes"][name]["limit"]
        ax.axhline(limit, color=C_LIMIT, linestyle=":", linewidth=0.7, zorder=0)
        ax.axhline(-limit, color=C_LIMIT, linestyle=":", linewidth=0.7, zorder=0, label=r"$a_{\mathrm{lim}}$")
        for spec, flight in zip(FLIGHTS, flights):
            ax.plot(flight["t"], flight["axes"][name]["acceleration"], color=spec["color"],
                    linestyle=spec["ls"], linewidth=spec["lw"] * 0.85, zorder=spec["zorder"], label=spec["label"])
        span = 1.15 * limit
        ax.set_ylim(-span, span)
        ax.set_ylabel(rf"$a_{{sp,{name}}}$ (m/s$^2$)")
        _panel_label(ax, panel)
    axes[-1].set_xlim(0.0, t_end)
    axes[-1].set_xlabel("Time from arming (s)")
    handles, labels = axes[0].get_legend_handles_labels()
    wanted = [r"$a_{\mathrm{lim}}$", "Flight A", "Flight B"]
    picked = [(handles[labels.index(name)], name) for name in wanted if name in labels]
    fig.legend([h for h, _ in picked], [l for _, l in picked], loc="lower center",
               bbox_to_anchor=(0.54, 0.0), ncol=3, frameon=False, handlelength=2.2)
    _save(fig, out, "acceleration_time")


def figure_trajectory(flights: list[dict], out: Path):
    fig, ax = plt.subplots(figsize=(ONE_COL, 3.35))
    reference_drawn = False
    for spec, flight in zip(FLIGHTS, flights):
        mask = flight["stage"] == 5
        if not np.any(mask):
            continue
        east_r = flight["axes"]["y"]["reference"][mask]
        north_r = flight["axes"]["x"]["reference"][mask]
        east = flight["axes"]["y"]["measured"][mask] - np.nanmean(east_r)
        north = flight["axes"]["x"]["measured"][mask] - np.nanmean(north_r)
        east_r = east_r - np.nanmean(east_r)
        north_r = north_r - np.nanmean(north_r)
        if not reference_drawn:
            ax.plot(east_r, north_r, color="0.45", linestyle="--", linewidth=0.8, zorder=1, label="reference")
            reference_drawn = True
        ax.plot(east, north, color=spec["color"], linestyle=spec["ls"], linewidth=spec["lw"],
                zorder=spec["zorder"], label=spec["label"])
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=3, frameon=False, handlelength=2.0)
    fig.subplots_adjust(left=0.18, right=0.97, bottom=0.22, top=0.97)
    _save(fig, out, "trajectory_xy")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flight-a", type=Path, required=True)
    parser.add_argument("--flight-b", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    _style()
    flights = [load(args.flight_a), load(args.flight_b)]
    figure_error(flights, args.out)
    figure_acceleration(flights, args.out)
    figure_trajectory(flights, args.out)


if __name__ == "__main__":
    main()
