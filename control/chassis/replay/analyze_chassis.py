#!/usr/bin/env python3
"""Read the chassis's logged path following from a recorded autonomous run and summarise it.

replay/chassis-auto.csv has one row per robot loop, converted from a driver-station capture of
the values the robot program and its path follower published. In the loops where the follower
published, a row holds the follower's target pose and the measured pose it read, both as logged;
every row holds the commanded and the measured chassis velocity. replay/reference-path.csv holds
the points of the two paths the follower was given. This script only reads those logged values:
it reports how far the measured pose was from the target pose, how far the measured velocity was
from the command, and the speeds reached, and plots the paths and the velocities. No controller
and no path follower is run.

Needs numpy and matplotlib.
"""

import argparse
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOG = os.path.join(HERE, "chassis-auto.csv")
DEFAULT_PATH = os.path.join(HERE, "reference-path.csv")
DEFAULT_OUT = os.path.join(HERE, "replay_out")
FIG_DPI = 150


def read_csv(path):
    """Columns as arrays; an empty cell (a value not logged in that row) reads as NaN."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) if r[k] != "" else np.nan for r in rows]) for k in rows[0]}


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def path_lengths(ref):
    out = {}
    for n in np.unique(ref["path"]).astype(int):
        m = ref["path"] == n
        out[n] = float(np.sum(np.hypot(np.diff(ref["x_m"][m]), np.diff(ref["y_m"][m]))))
    return out


def report(c, ref, path):
    t = c["time_s"]
    fol = ~np.isnan(c["path"])
    tf = t[fol]
    lengths = path_lengths(ref)
    print(f"clip: {os.path.basename(path)}   {len(t)} rows over {t[-1] - t[0]:.2f} s")
    print(f"path following: {len(lengths)} paths ("
          + " + ".join(f"{v:.2f} m" for v in lengths.values())
          + f"), {int(fol.sum())} follower updates from {tf[0]:.2f} s to {tf[-1]:.2f} s "
          f"({tf[-1] - tf[0]:.2f} s)")
    pos = np.hypot(c["target_x_m"] - c["measured_x_m"], c["target_y_m"] - c["measured_y_m"])[fol]
    head = wrap_deg(c["target_heading_deg"] - c["measured_heading_deg"])[fol]
    rms = lambda e: float(np.sqrt(np.mean(e ** 2)))
    i = int(np.argmax(pos))
    print(f"position error (target - measured, same loop): RMS {rms(pos):.3f} m, "
          f"max {pos.max():.3f} m at {tf[i]:.2f} s, median {np.median(pos):.3f} m")
    print(f"heading error: RMS {rms(head):.2f} deg, max {np.abs(head).max():.2f} deg")
    both = fol & ~np.isnan(c["cmd_vx_mps"]) & ~np.isnan(c["meas_vx_mps"])
    dv = np.hypot(c["cmd_vx_mps"] - c["meas_vx_mps"], c["cmd_vy_mps"] - c["meas_vy_mps"])[both]
    dw = (c["cmd_omega_radps"] - c["meas_omega_radps"])[both]
    print(f"velocity error while following (|v_cmd - v_meas|, {int(both.sum())} rows with both "
          f"logged): RMS {rms(dv):.3f} m/s; turn rate RMS {rms(dw):.3f} rad/s")
    speed = np.hypot(c["meas_vx_mps"], c["meas_vy_mps"])
    cmd_speed = np.hypot(c["cmd_vx_mps"], c["cmd_vy_mps"])
    print(f"measured speed up to {np.nanmax(speed[fol]):.2f} m/s while following "
          f"({np.nanmax(speed):.2f} m/s over the clip); commanded up to "
          f"{np.nanmax(cmd_speed[fol]):.2f} m/s while following")


def style():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ink, ink2, grid, surface = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": ink2, "axes.labelcolor": ink,
                         "xtick.color": ink2, "ytick.color": ink2, "axes.grid": True,
                         "grid.color": grid, "grid.linewidth": 0.6, "figure.facecolor": surface,
                         "axes.facecolor": surface, "savefig.facecolor": surface,
                         "axes.spines.top": False, "axes.spines.right": False})
    return plt, dict(ink=ink, ink2=ink2, surface=surface, blue="#2a78d6", orange="#eb6834",
                     band="#ebe9e2")


def plot_path(c, ref, out_dir):
    plt, k = style()
    fig, ax = plt.subplots(figsize=(7.2, 5.4))
    for j, n in enumerate(np.unique(ref["path"]).astype(int)):
        m = ref["path"] == n
        ax.plot(ref["x_m"][m], ref["y_m"][m], color=k["ink"], lw=0.9, ls=(0, (3, 2)), zorder=3,
                label="reference paths" if j == 0 else None)
        ax.annotate(f"path {n}", (ref["x_m"][m][0], ref["y_m"][m][0]), xytext=(4, 4),
                    textcoords="offset points", color=k["ink2"], fontsize=7.5)
    for j, n in enumerate(np.unique(c["path"][~np.isnan(c["path"])]).astype(int)):
        m = c["path"] == n
        ax.plot(c["target_x_m"][m], c["target_y_m"][m], color=k["orange"], lw=2.0,
                label="target pose (logged)" if j == 0 else None)
        ax.plot(c["measured_x_m"][m], c["measured_y_m"][m], color=k["blue"], lw=2.0,
                label="measured pose (logged)" if j == 0 else None)
    first = np.flatnonzero(~np.isnan(c["path"]))[0]
    ax.scatter([c["measured_x_m"][first]], [c["measured_y_m"][first]], s=36, color=k["blue"],
               edgecolors=k["surface"], linewidths=1.5, zorder=4, label="measured pose at start")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("field x (m)")
    ax.set_ylabel("field y (m)")
    ax.set_title("Path following: reference, target and measured pose", color=k["ink"], loc="left")
    ax.legend(frameon=False, fontsize=7.5, loc="best")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "path.png"), dpi=FIG_DPI)
    plt.close(fig)


def plot_velocity(c, out_dir):
    from matplotlib.patches import Patch

    plt, k = style()
    t = c["time_s"]
    fol = ~np.isnan(c["path"])
    edges = np.flatnonzero(np.diff(np.concatenate(([0], fol.astype(int), [0]))))
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(8.0, 5.4), sharex=True)
    for axis in (ax, ax2):
        for a, b in zip(edges[::2], edges[1::2]):
            axis.axvspan(t[a], t[b - 1], color=k["band"], lw=0, zorder=0)
    ax.plot(t, np.hypot(c["cmd_vx_mps"], c["cmd_vy_mps"]), color=k["ink"], lw=1.0,
            ls=(0, (3, 2)), label="commanded (logged)")
    ax.plot(t, np.hypot(c["meas_vx_mps"], c["meas_vy_mps"]), color=k["blue"], lw=1.4,
            label="measured (logged)")
    ax.set_ylabel("speed (m/s)")
    ax.set_title("Chassis speed", color=k["ink"], loc="left")
    handles, labels = ax.get_legend_handles_labels()
    handles.append(Patch(facecolor=k["band"], edgecolor=k["ink2"], lw=0.5))
    labels.append("path follower publishing")
    ax.legend(handles, labels, frameon=False, fontsize=7.5, loc="upper center")
    ax2.plot(t, c["cmd_omega_radps"], color=k["ink"], lw=1.0, ls=(0, (3, 2)),
             label="commanded (logged)")
    ax2.plot(t, c["meas_omega_radps"], color=k["blue"], lw=1.4, label="measured (logged)")
    ax2.set_ylabel("turn rate (rad/s)")
    ax2.set_title("Chassis turn rate", color=k["ink"], loc="left")
    ax2.legend(frameon=False, fontsize=7.5, loc="upper right")
    ax2.set_xlabel("time in clip (s)")
    ax2.set_xlim(t[0], t[-1])
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "velocity.png"), dpi=FIG_DPI)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(
        description="Summarise the chassis's logged path following: position and heading error "
                    "between the follower's target pose and the measured pose (RMS, max), the "
                    "error between commanded and measured velocity, the speeds reached and the "
                    "duration. Reads the CSVs' logged columns only. Writes path.png and "
                    "velocity.png.")
    ap.add_argument("--log", default=DEFAULT_LOG,
                    help="per-loop CSV to read (default: the bundled chassis-auto.csv)")
    ap.add_argument("--path", default=DEFAULT_PATH,
                    help="reference path CSV (default: the bundled reference-path.csv)")
    ap.add_argument("--out", default=DEFAULT_OUT,
                    help="directory for the PNGs (default: replay_out/ next to this script)")
    args = ap.parse_args()

    c = read_csv(args.log)
    ref = read_csv(args.path)
    report(c, ref, args.log)

    os.makedirs(args.out, exist_ok=True)
    plot_path(c, ref, args.out)
    plot_velocity(c, args.out)
    shown = os.path.relpath(args.out) if args.out == DEFAULT_OUT else args.out
    for name in ("path.png", "velocity.png"):
        print(f"wrote {os.path.join(shown, name)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
