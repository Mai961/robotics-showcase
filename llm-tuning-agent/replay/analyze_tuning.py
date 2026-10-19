#!/usr/bin/env python3
"""Read the measured outcome of every episode of one bench tuning session and summarise it.

replay/shoulder-session-episodes.csv has one row per episode of a shoulder tuning session on the
test bench, converted from the bench tuner's log: the search episodes of the gain sweeps, the
speed steps and the verify episodes, with each episode's measured overshoot, settle time, settling
error and rising error, as the tuner logged them, and whether it was aborted. The gain values and
the limits are not in the file; `gain_set` only marks the episode that ran the starting (seed)
gains and the episodes that ran the final ones. This script reads those columns, counts the
experiments, compares the seed episode with the mean of the verify episodes, and prints the
search episode that ran the final gains, the same kind of episode as the seed. No tuner, no cost
function and no controller is run.

Needs numpy and matplotlib.
"""

import argparse
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOG = os.path.join(HERE, "shoulder-session-episodes.csv")
DEFAULT_OUT = os.path.join(HERE, "replay_out")
FIG_DPI = 150
TEXT_COLUMNS = ("phase", "gain_set")
METRICS = (("settle_time_s", "settle time", "s", 2),
           ("settling_error_rad", "settling error", "rad", 4),
           ("overshoot_rad", "overshoot", "rad", 4),
           ("rising_error_rad", "rising error", "rad", 3))
PHASES = (("search", "search"), ("speed", "speed step"), ("verify", "verify"))


def load(path):
    """Columns as arrays; an empty cell (an outcome the tuner did not log) reads as NaN."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    c = {k: np.array([float(r[k]) if r[k] != "" else np.nan for r in rows])
         for k in rows[0] if k not in TEXT_COLUMNS}
    for k in TEXT_COLUMNS:
        c[k] = np.array([r[k] for r in rows])
    c["aborted"] = c["aborted"] == 1.0
    return c


def report(c, path):
    phase = c["phase"]
    n = {p: int(np.sum(phase == p)) for p, _ in PHASES}
    sweeps = len(set(c["sweep"][phase == "search"][~np.isnan(c["sweep"][phase == "search"])]))
    print(f"session: {os.path.basename(path)}   {len(phase)} episodes: {n['search']} search "
          f"({sweeps} sweeps), {n['speed']} speed steps, {n['verify']} verify; "
          f"{int(c['aborted'].sum())} aborted")
    seed = np.flatnonzero(c["gain_set"] == "seed")[0]
    verify = phase == "verify"
    print(f"seed (episode {int(c['episode'][seed])}) against the mean of the {n['verify']} verify "
          f"episodes (final gains):")
    for key, name, unit, nd in METRICS:
        before = c[key][seed]
        v = c[key][verify]
        if np.all(np.isnan(v)):
            first_speed = np.flatnonzero((phase == "speed") & (c["gain_set"] == "final"))[0]
            after = c[key][first_speed]
            note = (f"verify episodes do not log it; episode {int(c['episode'][first_speed])}, "
                    f"the first speed step with the final gains")
        else:
            after = np.nanmean(v)
            note = "rose" if after > before else "fell"
        print(f"  {name:<15} {before:.{nd}f} {unit} -> {after:.{nd}f} {unit}   ({note})")
    final = np.flatnonzero((phase == "search") & (c["gain_set"] == "final"))
    if len(final):
        i = final[-1]
        print(f"final gains in a search episode (episode {int(c['episode'][i])}, the same episode "
              f"type as the seed): " + ", ".join(f"{name} {c[key][i]:.{nd}f} {unit}"
                                                 for key, name, unit, nd in METRICS))


def plot(c, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ink, ink2, grid, surface = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
    colors = {"search": "#2a78d6", "speed": "#eb6834", "verify": "#1baf7a"}
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": ink2, "axes.labelcolor": ink,
                         "xtick.color": ink2, "ytick.color": ink2, "axes.grid": True,
                         "grid.color": grid, "grid.linewidth": 0.6, "figure.facecolor": surface,
                         "axes.facecolor": surface, "savefig.facecolor": surface,
                         "axes.spines.top": False, "axes.spines.right": False})
    ep = c["episode"]
    seed = c["gain_set"] == "seed"
    fig, axes = plt.subplots(4, 1, figsize=(8.0, 8.0), sharex=True)
    for ax, (key, name, unit, _) in zip(axes, METRICS):
        y = c[key]
        for p, label in PHASES:
            m = (c["phase"] == p) & ~c["aborted"] & ~np.isnan(y)
            ax.plot(ep[m], y[m], ls="none", marker="o", ms=5, color=colors[p],
                    markeredgecolor=surface, markeredgewidth=0.8, label=label)
        ax.plot(ep[seed], y[seed], ls="none", marker="o", ms=11, markerfacecolor="none",
                markeredgecolor=ink, markeredgewidth=1.0, label="seed gains")
        ab = c["aborted"]
        ax.plot(ep[ab], np.full(int(ab.sum()), np.nanmax(y)), ls="none", marker="x", ms=7,
                color=ink, label=f"aborted ({int(ab.sum())})")
        v = y[c["phase"] == "verify"]
        if not np.all(np.isnan(v)):
            ax.axhline(np.nanmean(v), color=ink2, lw=0.8, ls="--")
            gap = (ep[c["phase"] == "search"].max() + ep[c["phase"] == "verify"].min()) / 2
            ax.annotate("verify mean", (gap, np.nanmean(v)), xytext=(0, 3),
                        textcoords="offset points", ha="center", va="bottom", color=ink2,
                        fontsize=7.5)
        ax.set_ylabel(unit)
        ax.set_title(name.capitalize(), color=ink, loc="left")
    axes[0].legend(frameon=False, fontsize=7.5, loc="upper center", ncol=5,
                   bbox_to_anchor=(0.5, 1.45))
    axes[-1].set_xlabel("episode")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "episodes.png"), dpi=FIG_DPI)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(
        description="Summarise one bench tuning session's per-episode outcomes: the number of "
                    "search, speed-step and verify episodes and aborts, and the seed episode "
                    "against the mean of the verify episodes for settle time, settling error, "
                    "overshoot and rising error, and the search episode that ran the final gains. "
                    "Reads the CSV's logged columns only. Writes episodes.png.")
    ap.add_argument("--log", default=DEFAULT_LOG, help="CSV to read (default: the bundled one)")
    ap.add_argument("--out", default=DEFAULT_OUT,
                    help="directory for the PNG (default: replay_out/ next to this script)")
    args = ap.parse_args()

    c = load(args.log)
    report(c, args.log)

    os.makedirs(args.out, exist_ok=True)
    plot(c, args.out)
    shown = os.path.relpath(args.out) if args.out == DEFAULT_OUT else args.out
    print(f"wrote {os.path.join(shown, 'episodes.png')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
