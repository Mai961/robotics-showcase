#!/usr/bin/env python3
"""Read the turret's logged aiming and shot compensation from a recorded drive and summarise it.

replay/turret-shoot-30s.csv has one row per robot loop, converted from a driver-station capture of
the values the robot program published. This script only reads the logged columns: the turret's
target and measured angle (robot frame), the mode, the measured chassis velocity, whether the shot
compensation was on, and the compensation angle, lateral speed and saturation flag the robot
logged while it was on. It reports how long the compensation ran while the robot moved, the range
of lateral speed and compensation angle it logged, and how closely the turret followed its target.
It plots command against measurement over the clip, and the logged compensation angle against the
logged lateral speed. No control law and no shot solution is run.

Needs numpy and matplotlib.
"""

import argparse
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOG = os.path.join(HERE, "turret-shoot-30s.csv")
DEFAULT_OUT = os.path.join(HERE, "replay_out")
MOVING_MPS = 0.5
FIG_DPI = 150
TEXT_COLUMNS = ("turret_mode",)


def load(path):
    """Columns as arrays; an empty cell (a value not logged in that row) reads as NaN."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    c = {k: np.array([float(r[k]) if r[k] != "" else np.nan for r in rows])
         for k in rows[0] if k not in TEXT_COLUMNS}
    for k in TEXT_COLUMNS:
        c[k] = np.array([r[k] for r in rows])
    for k in ("compensation_enabled", "compensation_saturated", "shooting"):
        c[k] = c[k] == 1.0
    return c


def spans(t, mask):
    """(start, end) times of each run of True rows; a run ends at the next row's time."""
    edges = np.flatnonzero(np.diff(np.concatenate(([0], mask.astype(int), [0]))))
    return [(t[a], t[min(b, len(t) - 1)]) for a, b in zip(edges[::2], edges[1::2])]


def report(c, path):
    t, mode = c["time_s"], c["turret_mode"]
    dt = np.diff(t)
    speed = np.hypot(c["chassis_vx_mps"], c["chassis_vy_mps"])
    on = c["compensation_enabled"]
    track = mode == "TRACKING"
    comp = on & track & (speed > MOVING_MPS)
    modes = ", ".join(f"{m} {int(np.sum(mode == m))}" for m in sorted(set(mode)))
    print(f"clip: {os.path.basename(path)}   {len(t)} rows over {t[-1] - t[0]:.1f} s ({modes})")
    print(f"chassis up to {speed.max():.2f} m/s (measured), above {MOVING_MPS} m/s on "
          f"{100 * np.mean(speed > MOVING_MPS):.0f} % of the rows")
    print(f"shot compensation on in {len(spans(t, on))} periods, {np.sum(dt[on[:-1]]):.2f} s in all; "
          f"on while TRACKING above {MOVING_MPS} m/s: {np.sum(dt[comp[:-1]]):.2f} s "
          f"({int(comp.sum())} rows)")
    ang, lat = c["compensation_angle_deg"][comp], c["lateral_speed_mps"][comp]
    print(f"  over those rows: lateral speed {lat.min():+.3f} to {lat.max():+.3f} m/s, "
          f"compensation angle {ang.min():+.2f} to {ang.max():+.2f} deg (both as logged)")
    sat = c["compensation_saturated"]
    for a, b in spans(t, sat):
        print(f"  compensation logged as saturated from {a:.2f} s to {b:.2f} s "
              f"({int(np.sum(sat & (t >= a) & (t < b)))} rows)")
    err = c["target_angle_robot_deg"] - c["measured_angle_robot_deg"]
    rms = lambda e: np.sqrt(np.mean(e ** 2))
    print(f"tracking error (target - measured): RMS {rms(err[comp]):.2f} deg while compensating "
          f"({int(comp.sum())} rows), RMS {rms(err[track]):.2f} deg over all TRACKING rows "
          f"({int(track.sum())})")
    return err, comp


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
    return plt, dict(ink=ink, ink2=ink2, grid=grid, surface=surface, blue="#2a78d6",
                     orange="#eb6834", aqua="#1baf7a", band="#ebe9e2")


def plot_tracking(c, err, out_dir):
    from matplotlib.patches import Patch

    plt, k = style()
    t = c["time_s"]
    fig, (ax, ax2, ax3) = plt.subplots(3, 1, figsize=(8.0, 6.6), sharex=True,
                                       gridspec_kw={"height_ratios": [2.2, 1.4, 0.45]})
    for a, b in spans(t, c["compensation_enabled"]):
        for axis in (ax, ax2):
            axis.axvspan(a, b, color=k["band"], lw=0, zorder=0)
    ax.plot(t, c["target_angle_robot_deg"], color=k["ink"], lw=0.9, ls=(0, (3, 2)),
            label="target (logged)")
    ax.plot(t, c["measured_angle_robot_deg"], color=k["blue"], lw=1.2, label="measured (logged)")
    ax.set_ylabel("turret angle, robot frame (deg)")
    ax.set_title("Turret target against measurement", color=k["ink"], loc="left")
    handles, labels = ax.get_legend_handles_labels()
    handles.append(Patch(facecolor=k["band"], edgecolor=k["ink2"], lw=0.5))
    labels.append("shot compensation on")
    ax.legend(handles, labels, frameon=False, fontsize=7.5, loc="best")

    track = c["turret_mode"] == "TRACKING"
    rms = np.sqrt(np.mean(err[track] ** 2))
    ax2.plot(t, np.where(track, err, np.nan), color=k["orange"], lw=0.9)
    for a, b in spans(t, ~track):
        ax2.axvspan(a, b, facecolor="none", edgecolor=k["ink2"], hatch="////", lw=0, zorder=0)
        ax2.annotate("SEEKING, not plotted", (b, 0.92), xycoords=("data", "axes fraction"),
                     xytext=(4, 0), textcoords="offset points", ha="left", va="top",
                     color=k["ink2"], fontsize=7.5)
    for s in (rms, -rms):
        ax2.axhline(s, color=k["ink2"], lw=0.8, ls="--")
    ax2.annotate(f"+/- RMS {rms:.2f} deg (all TRACKING rows)", (0.0, rms),
                 xycoords=("axes fraction", "data"), xytext=(4, 3), textcoords="offset points",
                 ha="left", va="bottom", color=k["ink2"], fontsize=7.5)
    ax2.set_ylabel("error (deg)")
    ax2.set_title("Tracking error in TRACKING, target - measured", color=k["ink"], loc="left")

    names = sorted(set(c["turret_mode"]))
    level = np.array([names.index(m) for m in c["turret_mode"]])
    ax3.step(t, level, where="post", color=k["aqua"], lw=1.5)
    ax3.set_yticks(range(len(names)), names)
    ax3.set_ylim(-0.5, len(names) - 0.5)
    ax3.set_title("Mode", color=k["ink"], loc="left")
    ax3.set_xlabel("time in clip (s)")
    ax3.set_xlim(t[0], t[-1])
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "tracking.png"), dpi=FIG_DPI)
    plt.close(fig)


def plot_compensation(c, comp, out_dir):
    plt, k = style()
    on = c["compensation_enabled"]
    sat = on & c["compensation_saturated"]
    other = on & ~comp
    lat, ang = c["lateral_speed_mps"], c["compensation_angle_deg"]
    fig, ax = plt.subplots(figsize=(6.4, 4.8))
    ax.axhline(0.0, color=k["ink2"], lw=0.6)
    ax.axvline(0.0, color=k["ink2"], lw=0.6)
    ax.scatter(lat[comp], ang[comp], s=14, color=k["blue"], edgecolors=k["surface"],
               linewidths=0.6, zorder=3,
               label=f"TRACKING above {MOVING_MPS} m/s ({int(comp.sum())} rows)")
    ax.scatter(lat[other], ang[other], s=14, facecolors="none", edgecolors=k["ink2"],
               linewidths=0.8, zorder=3, label=f"other rows ({int(other.sum())})")
    ax.scatter(lat[sat], ang[sat], s=64, facecolors="none", edgecolors=k["orange"],
               linewidths=1.4, zorder=4, label=f"logged as saturated ({int(sat.sum())})")
    ax.set_xlabel("lateral speed, as logged (m/s)")
    ax.set_ylabel("compensation angle, as logged (deg)")
    ax.set_title("Shot compensation against lateral speed, rows with compensation on",
                 color=k["ink"], loc="left")
    ax.legend(frameon=False, fontsize=7.5, loc="best")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "compensation_vs_lateral_speed.png"), dpi=FIG_DPI)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(
        description="Summarise the turret's logged aiming and shot compensation: time with the "
                    "compensation on while moving, the logged lateral speed and compensation "
                    "angle ranges, the saturation flag, and the tracking error (RMS while "
                    "compensating and over all TRACKING rows). Reads the CSV's logged columns "
                    "only. Writes tracking.png and compensation_vs_lateral_speed.png.")
    ap.add_argument("--log", default=DEFAULT_LOG, help="CSV to read (default: the bundled one)")
    ap.add_argument("--out", default=DEFAULT_OUT,
                    help="directory for the PNGs (default: replay_out/ next to this script)")
    args = ap.parse_args()

    c = load(args.log)
    err, comp = report(c, args.log)

    os.makedirs(args.out, exist_ok=True)
    plot_tracking(c, err, args.out)
    plot_compensation(c, comp, args.out)
    shown = os.path.relpath(args.out) if args.out == DEFAULT_OUT else args.out
    for name in ("tracking.png", "compensation_vs_lateral_speed.png"):
        print(f"wrote {os.path.join(shown, name)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
