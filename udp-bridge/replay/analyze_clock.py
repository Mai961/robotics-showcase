#!/usr/bin/env python3
"""Read the UDP bridge's clock-sync estimate from the recorded match and summarise it.

Every message on /talos_udp/clock in replay/clock-sync-154s.mcap is one estimate the bridge
published: [offset_s, rtt_s, age_s, valid]. The estimate is the sample with the smallest round
trip among the recent pings, so `age_s` grows until a newer sample with a smaller round trip
replaces it. This script only reads those logged values: it splits the two interleaved series by
the size of their offset, prints offset, round trip and age for each, and marks the messages where
the selected sample changed (its offset and round trip differ from the previous message's).
Nothing is recomputed from pings.

Needs numpy, matplotlib and mcap-ros2-support.
"""

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOG = os.path.join(HERE, "clock-sync-154s.mcap")
DEFAULT_OUT = os.path.join(HERE, "replay_out")
TOPIC = "/talos_udp/clock"
SPLIT_S = 1.0     # offsets above this belong to the series between two different clocks
FIG_DPI = 150


def load(path):
    """Rows of [t_s, offset_s, rtt_s, age_s, valid], t_s from the clip's first message."""
    from mcap_ros2.reader import read_ros2_messages

    rows = [[m.log_time_ns] + list(m.ros_msg.data)
            for m in read_ros2_messages(path, topics=[TOPIC])]
    a = np.array(rows, dtype=float)
    a[:, 0] = (a[:, 0] - a[0, 0]) * 1e-9
    return a


def selection_changes(s):
    """Indices where the estimate switched to another sample: offset or round trip changed."""
    return np.flatnonzero((np.diff(s[:, 1]) != 0) | (np.diff(s[:, 2]) != 0)) + 1


def report(series, path, n_total):
    print(f"clip: {os.path.basename(path)}   {TOPIC}: {n_total} messages over "
          f"{max(s[-1, 0] for s in series.values()):.0f} s")
    for name, s in series.items():
        period = np.median(np.diff(s[:, 0]))
        print(f"{name}: {len(s)} messages, one per {period:.2f} s, valid on {int(s[:, 4].sum())}")
        if name.startswith("offset near"):
            print(f"  offset within {1e6 * np.abs(s[:, 1]).max():.0f} us of zero "
                  f"(from {1e6 * s[:, 1].min():+.1f} to {1e6 * s[:, 1].max():+.1f} us)")
        else:
            print(f"  offset {s[0, 1]:.3f} s at the start, {s[-1, 1]:.3f} s at the end "
                  f"(drift {1e3 * (s[-1, 1] - s[0, 1]):+.1f} ms over {s[-1, 0] - s[0, 0]:.0f} s)")
        print(f"  round trip median {1e3 * np.median(s[:, 2]):.2f} ms "
              f"({1e3 * s[:, 2].min():.2f} to {1e3 * s[:, 2].max():.2f} ms)")
        print(f"  age of the selected sample up to {s[:, 3].max():.1f} s; "
              f"selection changed {len(selection_changes(s))} times")


def plot(series, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ink, ink2, grid, surface = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
    blue, orange = "#2a78d6", "#eb6834"
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": ink2, "axes.labelcolor": ink,
                         "xtick.color": ink2, "ytick.color": ink2, "axes.grid": True,
                         "grid.color": grid, "grid.linewidth": 0.6, "figure.facecolor": surface,
                         "axes.facecolor": surface, "savefig.facecolor": surface,
                         "axes.spines.top": False, "axes.spines.right": False})

    (name_a, a), (name_b, b) = series.items()
    fig, axes = plt.subplots(4, 1, figsize=(8.0, 8.4), sharex=True,
                             gridspec_kw={"height_ratios": [1, 1, 1, 1]})
    for ax, (name, s, color, scale, unit) in zip(
            axes[:2], ((name_a, a, blue, 1e3, "ms"), (name_b, b, orange, 1e6, "us"))):
        ax.plot(s[:, 0], scale * s[:, 1], color=color, lw=1.0, label="logged offset")
        k = selection_changes(s)
        ax.scatter(s[k, 0], scale * s[k, 1], s=16, color=surface, edgecolors=color,
                   linewidths=1.0, zorder=3, label="another min-round-trip sample selected")
        ax.set_ylabel(f"offset ({unit})")
        ax.set_title(name, color=ink, loc="left")
        ax.legend(frameon=False, fontsize=7.5, loc="best")
    for name, s, color in ((name_a, a, blue), (name_b, b, orange)):
        axes[2].plot(s[:, 0], 1e3 * s[:, 2], color=color, lw=1.0, label=name)
        axes[3].plot(s[:, 0], s[:, 3], color=color, lw=1.0, label=name)
    axes[2].set_ylabel("round trip (ms)")
    axes[2].set_ylim(0.0, 1.35e3 * a[:, 2].max())
    axes[2].set_title("Round trip of the selected sample", color=ink, loc="left")
    axes[2].legend(frameon=False, fontsize=7.5, loc="upper right", ncol=2)
    axes[3].set_ylabel("age (s)")
    axes[3].set_title("Age of the selected sample", color=ink, loc="left")
    axes[3].set_xlabel("time in clip (s)")
    axes[3].set_xlim(left=0.0)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "clock_sync.png"), dpi=FIG_DPI)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(
        description="Summarise the logged clock-sync estimate on /talos_udp/clock: offset, round "
                    "trip and age of each of the two interleaved series, read from the log only. "
                    "Writes clock_sync.png.")
    ap.add_argument("--log", default=DEFAULT_LOG, help="MCAP clip to read (default: the bundled one)")
    ap.add_argument("--out", default=DEFAULT_OUT,
                    help="directory for the PNG (default: replay_out/ next to this script)")
    args = ap.parse_args()

    a = load(args.log)
    far = np.abs(a[:, 1]) > SPLIT_S
    series = {"offset between two clocks": a[far], "offset near zero": a[~far]}
    report(series, args.log, len(a))

    os.makedirs(args.out, exist_ok=True)
    plot(series, args.out)
    shown = os.path.relpath(args.out) if args.out == DEFAULT_OUT else args.out
    print(f"wrote {os.path.join(shown, 'clock_sync.png')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
