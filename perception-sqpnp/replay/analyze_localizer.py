#!/usr/bin/env python3
"""Read the AprilTag localizer's logged output from the recorded match and summarise it.

The match recording, state-estimation/replay/match-20s.mcap, holds what the localizer node logged
during 20 s of a match: ~/detections (one JSON record per processed frame: every detection,
accepted or rejected and why, and the solve's summary), ~/pose (camera in field with its 6x6
covariance) and the per-frame numbers on /viz/det/*. This script only reads those messages. It counts the frames by how the node
solved them (all accepted tags pooled into one solve, a single tag, withheld, or no tag accepted),
takes the chosen tag's reprojection error from /viz/det/chosen_reproj_err_px as logged, and reads
the position uncertainty from the logged covariance. Nothing is re-solved.

Needs numpy, matplotlib and mcap-ros2-support.
"""

import argparse
import collections
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOG = os.path.normpath(os.path.join(HERE, "..", "..", "state-estimation", "replay",
                                            "match-20s.mcap"))
DEFAULT_OUT = os.path.join(HERE, "replay_out")

NODE = "/odin_apriltag_localizer"
DETECTIONS, POSE = NODE + "/detections", NODE + "/pose"
REPROJ = "/viz/det/chosen_reproj_err_px"
SOLVED = ("pooled", "single")
FIG_DPI = 150


def load(path):
    """Detection records, published poses (stamp_ns -> position, 6x6 covariance) and the logged
    reprojection errors. Times are seconds from the clip's first message."""
    from mcap_ros2.reader import read_ros2_messages

    frames, poses, reproj, t0 = [], {}, [], None
    for m in read_ros2_messages(path, topics=[DETECTIONS, POSE, REPROJ]):
        t0 = t0 or m.log_time_ns
        t = (m.log_time_ns - t0) * 1e-9
        if m.channel.topic == DETECTIONS:
            d = json.loads(m.ros_msg.data)
            d["t_s"] = t
            frames.append(d)
        elif m.channel.topic == POSE:
            h, p = m.ros_msg.header, m.ros_msg.pose.pose.position
            poses[h.stamp.sec * 1_000_000_000 + h.stamp.nanosec] = (
                t, np.array([p.x, p.y, p.z]), np.array(m.ros_msg.pose.covariance).reshape(6, 6))
        else:
            reproj.append(m.ros_msg.data)
    return frames, poses, np.array(reproj)


def outcome(d):
    """How the node handled one frame, as its record says."""
    s = d["solve"]
    if s["status"] in SOLVED and s["cov_ok"]:
        return s["status"]
    if any(det["accepted"] for det in d["detections"]):
        return "withheld"
    return "none"


def report(frames, poses, reproj, path):
    n = len(frames)
    kind = collections.Counter(outcome(d) for d in frames)
    print(f"clip: {os.path.basename(path)}   frames processed (~/detections): {n}")
    pct = lambda k: f"{kind[k]:3d} ({100 * kind[k] / n:4.1f} %)"
    tags = collections.Counter(d["solve"]["n_tags"] for d in frames if outcome(d) == "pooled")
    print(f"  pooled (2+ tags solved as one): {pct('pooled')}   "
          + ", ".join(f"{k} tags: {v}" for k, v in sorted(tags.items())))
    print(f"  single tag:                     {pct('single')}")
    print(f"  withheld (tags accepted, no pose): {pct('withheld')}")
    print(f"  no tag accepted:                {pct('none')}")
    rejected = collections.Counter(det["reject"] for d in frames for det in d["detections"]
                                   if not det["accepted"])
    n_det = sum(len(d["detections"]) for d in frames)
    print(f"detections: {n_det}, accepted {n_det - sum(rejected.values())}, rejected "
          f"{sum(rejected.values())} (" + ", ".join(f"{k} {v}" for k, v in
                                                    sorted(rejected.items())) + ")")
    solved_stamps = {d["stamp_ns"] for d in frames if outcome(d) in SOLVED}
    print(f"~/pose: {len(poses)} messages; {len(solved_stamps & set(poses))} match a solved frame's "
          f"record, {len(set(poses) - {d['stamp_ns'] for d in frames})} have no ~/detections "
          f"record in the clip")
    print(f"chosen tag reprojection error, as logged ({REPROJ}, {len(reproj)} frames): median "
          f"{np.median(reproj):.3f} px, 90 % under {np.percentile(reproj, 90):.3f} px, max "
          f"{reproj.max():.3f} px")
    sx = np.array([1e3 * np.sqrt(c[0, 0]) for _, _, c in poses.values()])
    print(f"sigma x from the logged covariance: median {np.median(sx):.1f} mm over {len(sx)} poses")
    by_tags = position_sigma_by_tags(frames, poses)
    print("position sigma, sqrt of the covariance trace (x y z), by tags in the solve: "
          + "; ".join(f"{k} tag{'s' if k > 1 else ''}: median {np.median(v):.0f} mm ({len(v)})"
                      for k, v in sorted(by_tags.items())))


def position_sigma_by_tags(frames, poses):
    out = collections.defaultdict(list)
    for d in frames:
        if outcome(d) in SOLVED and d["stamp_ns"] in poses:
            cov = poses[d["stamp_ns"]][2]
            out[d["solve"]["n_tags"]].append(1e3 * np.sqrt(np.trace(cov[:3, :3])))
    return out


def plot(frames, poses, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker
    from matplotlib.patches import Ellipse

    ink, ink2, grid, surface = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
    blue, orange, aqua, yellow = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": ink2, "axes.labelcolor": ink,
                         "xtick.color": ink2, "ytick.color": ink2, "axes.grid": True,
                         "grid.color": grid, "grid.linewidth": 0.6, "figure.facecolor": surface,
                         "axes.facecolor": surface, "savefig.facecolor": surface,
                         "axes.spines.top": False, "axes.spines.right": False})

    status = {d["stamp_ns"]: outcome(d) for d in frames}
    fig, ax = plt.subplots(figsize=(7.6, 5.0))
    xy = np.array([p[:2] for _, p, _ in poses.values()])
    for stamp, (_, p, cov) in sorted(poses.items()):
        s = status.get(stamp)
        color = blue if s == "pooled" else orange if s == "single" else ink2
        w, v = np.linalg.eigh(cov[:2, :2])
        angle = np.degrees(np.arctan2(v[1, 1], v[0, 1]))
        ax.add_patch(Ellipse(p[:2], 4 * np.sqrt(w[1]), 4 * np.sqrt(w[0]), angle=angle,
                             fill=False, lw=0.6, color=color, alpha=0.55))
    for kind, color, marker, label in (("pooled", blue, "o", "pooled solve (2+ tags)"),
                                       ("single", orange, "^", "single tag"),
                                       (None, ink2, "s", "no ~/detections record in the clip")):
        pts = np.array([p[:2] for st, (_, p, _) in poses.items() if status.get(st) == kind])
        if len(pts):
            ax.scatter(pts[:, 0], pts[:, 1], s=12, marker=marker, color=color, edgecolors=surface,
                       linewidths=0.4, zorder=3, label=label)
    pad = 0.4
    ax.set_xlim(xy[:, 0].min() - pad, xy[:, 0].max() + pad)
    ax.set_ylim(xy[:, 1].min() - pad, xy[:, 1].max() + pad)
    ax.set_aspect("equal")
    ax.set_xlabel("field x (m)")
    ax.set_ylabel("field y (m)")
    ax.set_title("Camera position per ~/pose, 2-sigma ellipses from its logged covariance",
                 color=ink, loc="left")
    ax.legend(frameon=False, fontsize=7.5, loc="best")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "pose_xy.png"), dpi=FIG_DPI)
    plt.close(fig)

    rows = [(d["solve"]["n_tags"], d["solve"]["weighted_mean_edge_px"],
             1e3 * np.sqrt(np.trace(poses[d["stamp_ns"]][2][:3, :3])))
            for d in frames if outcome(d) in SOLVED and d["stamp_ns"] in poses]
    r = np.array(rows, dtype=float)
    colors = {1: orange, 2: blue, 3: aqua, 4: yellow}
    jitter = np.random.default_rng(0).uniform(-0.18, 0.18, len(r))
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10.0, 4.2), sharey=True)
    for k in sorted(set(r[:, 0].astype(int))):
        sel = r[:, 0] == k
        label = f"{k} tag{'s' if k > 1 else ''} ({int(sel.sum())})"
        a1.scatter(r[sel, 0] + jitter[sel], r[sel, 2], s=12, color=colors.get(k, ink2),
                   edgecolors=surface, linewidths=0.4, zorder=3)
        a1.plot([k - 0.3, k + 0.3], [np.median(r[sel, 2])] * 2, color=ink, lw=1.2, zorder=4)
        a2.scatter(r[sel, 1], r[sel, 2], s=12, color=colors.get(k, ink2), edgecolors=surface,
                   linewidths=0.4, zorder=3, label=label)
    a1.set_yscale("log")
    a1.yaxis.set_major_formatter(matplotlib.ticker.FormatStrFormatter("%g"))
    a1.set_xticks(sorted(set(r[:, 0].astype(int))))
    a1.set_xlabel("tags in the solve")
    a1.set_ylabel("sqrt of the position covariance trace (mm)")
    a1.set_title("Logged uncertainty against the number of tags", color=ink, loc="left")
    a2.set_xscale("log")
    a2.xaxis.set_major_formatter(matplotlib.ticker.FormatStrFormatter("%g"))
    a2.xaxis.set_minor_formatter(matplotlib.ticker.FormatStrFormatter("%g"))
    a2.set_xlabel("mean apparent tag edge in the solve, as logged (px)")
    a2.set_title("... and against their apparent size", color=ink, loc="left")
    a2.legend(frameon=False, fontsize=7.5, loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "cov_trace_vs_ntags.png"), dpi=FIG_DPI)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(
        description="Summarise the localizer's logged output: frames processed and the share "
                    "solved as one pooled target, as a single tag, withheld or with no tag "
                    "accepted; rejections by reason; the chosen tag's reprojection error as "
                    "logged; and the position uncertainty from the logged covariance, by number "
                    "of tags. Reads logged messages only. Writes pose_xy.png and "
                    "cov_trace_vs_ntags.png.")
    ap.add_argument("--log", default=DEFAULT_LOG,
                    help="MCAP clip to read (default: the match recording in "
                         "state-estimation/replay/)")
    ap.add_argument("--out", default=DEFAULT_OUT,
                    help="directory for the PNGs (default: replay_out/ next to this script)")
    args = ap.parse_args()

    frames, poses, reproj = load(args.log)
    report(frames, poses, reproj, args.log)

    os.makedirs(args.out, exist_ok=True)
    plot(frames, poses, args.out)
    shown = os.path.relpath(args.out) if args.out == DEFAULT_OUT else args.out
    print(f"wrote {os.path.join(shown, 'pose_xy.png')} and "
          f"{os.path.join(shown, 'cov_trace_vs_ntags.png')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
