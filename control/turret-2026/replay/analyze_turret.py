#!/usr/bin/env python3
"""Read the turret's logged tracking from the recorded drive and summarise it.

replay/turret-tracking-30s.csv has one row per robot loop in which the turret logged its loop
time, converted from the robot program's log. This script only reads the logged columns: the
turret's target and measured angle (robot frame), the mode, the logged aim error, the loop period
and the chassis velocity. It reports how closely the turret followed its target, and plots
command against measurement, the tracking error and the mode over time. No control law is run.

The shot compensation was off while this clip was recorded, so the CSV holds no logged
compensation values and there is nothing to plot against the lateral speed.

Needs numpy and matplotlib.
"""

import argparse
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOG = os.path.join(HERE, "turret-tracking-30s.csv")
DEFAULT_OUT = os.path.join(HERE, "replay_out")
MOVING_MPS = 0.5
FIG_DPI = 150


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    num = {k: np.array([float(r[k]) for r in rows])
           for k in rows[0] if k != "turret_mode"}
    num["turret_mode"] = np.array([r["turret_mode"] for r in rows])
    return num


def report(c, path):
    t, mode = c["time_s"], c["turret_mode"]
    err = c["target_angle_robot_deg"] - c["measured_angle_robot_deg"]
    speed = np.hypot(c["chassis_vx_mps"], c["chassis_vy_mps"])
    track = mode == "TRACKING"
    modes = ", ".join(f"{m} {int(np.sum(mode == m))}" for m in sorted(set(mode)))
    print(f"clip: {os.path.basename(path)}   {len(t)} rows over {t[-1] - t[0]:.1f} s ({modes})")
    target = c["target_angle_robot_deg"]
    swing = max(np.ptp(target[(t >= t0) & (t < t0 + 1.0)]) for t0 in t)
    print(f"chassis up to {speed.max():.1f} m/s and {np.abs(c['chassis_omega_radps']).max():.1f} "
          f"rad/s; robot-frame target swings up to {swing:.0f} deg within 1 s")
    print(f"loop period median {np.median(c['loop_dt_ms']):.0f} ms, "
          f"max {c['loop_dt_ms'].max():.0f} ms")
    e = err[track]
    print(f"tracking error in TRACK (target - measured, {int(track.sum())} rows): "
          f"RMS {np.sqrt(np.mean(e ** 2)):.2f} deg, median |e| {np.median(np.abs(e)):.2f} deg, "
          f"90 % under {np.percentile(np.abs(e), 90):.2f} deg, max {np.abs(e).max():.2f} deg")
    moving = speed > MOVING_MPS
    a = c["aim_error_abs_deg"][moving]
    print(f"logged aim error above {MOVING_MPS} m/s ({int(moving.sum())} rows, "
          f"{100 * moving.mean():.0f} %): median {np.median(a):.2f} deg, "
          f"90 % under {np.percentile(a, 90):.2f} deg")
    return err


def plot(c, err, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ink, ink2, grid, surface = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
    blue, orange, aqua = "#2a78d6", "#eb6834", "#1baf7a"
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": ink2, "axes.labelcolor": ink,
                         "xtick.color": ink2, "ytick.color": ink2, "axes.grid": True,
                         "grid.color": grid, "grid.linewidth": 0.6, "figure.facecolor": surface,
                         "axes.facecolor": surface, "savefig.facecolor": surface,
                         "axes.spines.top": False, "axes.spines.right": False})

    t = c["time_s"]
    fig, (ax, ax2, ax3) = plt.subplots(3, 1, figsize=(8.0, 6.6), sharex=True,
                                       gridspec_kw={"height_ratios": [2.2, 1.4, 0.45]})
    ax.plot(t, c["target_angle_robot_deg"], color=ink, lw=0.9, ls=(0, (3, 2)),
            label="target (logged)")
    ax.plot(t, c["measured_angle_robot_deg"], color=blue, lw=1.2, label="measured (logged)")
    ax.set_ylabel("turret angle, robot frame (deg)")
    ax.set_title("Turret target against measurement", color=ink, loc="left")
    ax.legend(frameon=False, fontsize=7.5, loc="best")

    rms = np.sqrt(np.mean(err ** 2))
    ax2.plot(t, err, color=orange, lw=0.9, label="target - measured")
    ax2.axhline(rms, color=ink2, lw=0.8, ls="--")
    ax2.axhline(-rms, color=ink2, lw=0.8, ls="--")
    ax2.annotate(f"+/- RMS {rms:.2f} deg", (1.0, rms), xycoords=("axes fraction", "data"),
                 xytext=(-4, 3), textcoords="offset points", ha="right", va="bottom",
                 color=ink2, fontsize=7.5)
    ax2.set_ylabel("error (deg)")
    ax2.set_title("Tracking error", color=ink, loc="left")

    names = sorted(set(c["turret_mode"]))
    level = np.array([names.index(m) for m in c["turret_mode"]])
    ax3.step(t, level, where="post", color=aqua, lw=1.5)
    ax3.set_yticks(range(len(names)), names)
    ax3.set_ylim(-0.5, len(names) - 0.5)
    ax3.set_title("Mode", color=ink, loc="left")
    ax3.set_xlabel("time in clip (s)")
    ax3.set_xlim(t[0], t[-1])
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "tracking.png"), dpi=FIG_DPI)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(
        description="Summarise the turret's logged tracking: target against measured angle, "
                    "tracking error in TRACK (RMS, median, 90th percentile), the logged aim error "
                    "while the robot moves, loop period and mode. Reads the CSV's logged columns "
                    "only. Writes tracking.png.")
    ap.add_argument("--log", default=DEFAULT_LOG, help="CSV to read (default: the bundled one)")
    ap.add_argument("--out", default=DEFAULT_OUT,
                    help="directory for the PNG (default: replay_out/ next to this script)")
    args = ap.parse_args()

    c = load(args.log)
    err = report(c, args.log)
    print("shot compensation: compensation was off in this clip; no logged compensation "
          "values, so no compensation plot")

    os.makedirs(args.out, exist_ok=True)
    plot(c, err, args.out)
    shown = os.path.relpath(args.out) if args.out == DEFAULT_OUT else args.out
    print(f"wrote {os.path.join(shown, 'tracking.png')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
