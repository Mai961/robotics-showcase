#!/usr/bin/env python3
"""Read the field EKF's logged inputs and outputs from the recorded match and summarise them.

replay/match-20s.mcap holds what the EKF node read and wrote during 20 s of a match:
the VIO odometry, the tag fixes with their covariance, the fused pose /odin_tree/field_base, the
field->odom transform it published, and one record of logged values per fix (/viz/fix/* and
/field_localizer/fix_event). This script only reads those messages. It reports the rates, the
number of updates and the logged per-fix values, and checks the published track against the
fixes:

- Post-fit residual: each tag fix's camera position minus the EKF's camera position at the
  image's capture time, taken from the field->odom transform published right after that fix was
  applied and the VIO camera pose at the capture stamp. Its squared Mahalanobis distance uses the
  fix's own logged xy covariance. The log has no innovation covariance, so this is not an NIS.
- VIO only: the VIO camera track placed on the field with the first field->odom transform in the
  clip, and never corrected.

No filter step is run. The camera mount is the logged /tf_static transform.

Needs numpy, matplotlib and mcap-ros2-support.
"""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOG = os.path.join(HERE, "match-20s.mcap")
DEFAULT_OUT = os.path.join(HERE, "replay_out")

FIELD_BASE = "/odin_tree/field_base"
ODOM = "/odin1/odometry_highfreq"
TAG = "/odin_apriltag_localizer/pose"
FIX_EVENT = "/field_localizer/fix_event"
FIX_VALUES = ("innovation_xy_m", "innovation_yaw_deg", "sigma_pos_m", "k_pos", "n_tags", "applied",
              "rotation_applied", "update_count")
TOPICS = [FIELD_BASE, ODOM, TAG, FIX_EVENT, "/tf", "/tf_static"] + [f"/viz/fix/{k}" for k in FIX_VALUES]
CHI2_95_2DOF = 5.991
FIG_DPI = 150


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def vec(v):
    return np.array([v.x, v.y, v.z])


def load(path):
    """Everything the analysis reads, as arrays. Log times are seconds from the clip's start;
    header stamps stay in nanoseconds of the robot's clock."""
    from mcap_ros2.reader import read_ros2_messages

    c = {"fb": [], "odom": [], "tag": [], "event": [], "field_odom": [], "mount": None,
         "fix": {k: [] for k in FIX_VALUES}, "t0": None}
    for m in read_ros2_messages(path, topics=TOPICS):
        c["t0"] = c["t0"] or m.log_time_ns
        t, msg, lt = m.channel.topic, m.ros_msg, m.log_time_ns
        if t == FIELD_BASE:
            c["fb"].append((lt, stamp_ns(msg.header), vec(msg.pose.position),
                            rotation(msg.pose.orientation)))
        elif t == ODOM:
            c["odom"].append((stamp_ns(msg.header), vec(msg.pose.pose.position)))
        elif t == TAG:
            cov = np.array(msg.pose.covariance).reshape(6, 6)
            c["tag"].append((lt, stamp_ns(msg.header), vec(msg.pose.pose.position), cov))
        elif t == FIX_EVENT:
            c["event"].append((lt, json.loads(msg.data)))
        elif t == "/tf_static":
            for tr in msg.transforms:
                if tr.header.frame_id == "base" and tr.child_frame_id == "odin_camera":
                    c["mount"] = vec(tr.transform.translation)
        elif t == "/tf":
            for tr in msg.transforms:
                if tr.header.frame_id == "field" and tr.child_frame_id == "odom":
                    c["field_odom"].append((lt, vec(tr.transform.translation),
                                            rotation(tr.transform.rotation)))
        else:
            c["fix"][t.rsplit("/", 1)[1]].append((lt, msg.data))
    return c


def analyse(c):
    t0 = c["t0"]
    fb_lt = np.array([f[0] for f in c["fb"]])
    fb_st = np.array([f[1] for f in c["fb"]], dtype=np.int64)
    fb_xy = np.array([f[2][:2] for f in c["fb"]])
    # the EKF's camera position: its base pose moved through the logged mount
    cam_ekf = np.array([p + R @ c["mount"] for _, _, p, R in c["fb"]])
    od_st = np.array([o[0] for o in c["odom"]], dtype=np.int64)
    od_p = np.array([o[1] for o in c["odom"]])
    fo_lt = np.array([f[0] for f in c["field_odom"]])

    def vio_at(st):
        return np.array([np.interp(st, od_st, od_p[:, i]) for i in range(3)])

    # VIO only: placed on the field once, with the first field->odom transform in the clip
    _, T0, R0 = c["field_odom"][0]
    cam_vio = od_p @ R0.T + T0

    fix = {k: np.array([v for _, v in c["fix"][k]], dtype=float) for k in FIX_VALUES}
    fix_t = np.array([(lt - t0) * 1e-9 for lt, _ in c["fix"]["innovation_xy_m"]])
    events = {round(e["stamp"] * 1e9): (lt, e) for lt, e in c["event"]}

    post = []
    for lt, st, p, cov in c["tag"]:
        hit = events.get(st) or next((v for k, v in events.items() if abs(k - st) <= 1000), None)
        if hit is None:
            continue
        k = np.searchsorted(fo_lt, hit[0], side="right")
        if k >= len(fo_lt):
            continue
        _, T, R = c["field_odom"][k]
        r = (p - (R @ vio_at(st) + T))[:2]
        d2 = float(r @ np.linalg.solve(cov[:2, :2], r))
        tightest_mm = 1e3 * np.sqrt(np.linalg.eigvalsh(cov[:2, :2])[0])
        post.append(((hit[0] - t0) * 1e-9, d2, 100 * np.hypot(*r), hit[1]["n_tags"] >= 2,
                     tightest_mm))

    return {"fb_t": (fb_lt - t0) * 1e-9, "fb_st": fb_st, "fb_xy": fb_xy, "cam_ekf": cam_ekf,
            "od_st": od_st, "cam_vio": cam_vio, "fix": fix, "fix_t": fix_t,
            "tag_xy": np.array([tg[2][:2] for tg in c["tag"]]),
            "tag_multi": np.array([e["n_tags"] >= 2 for _, e in c["event"]]),
            "post": post, "event_keys": sorted(c["event"][0][1]) if c["event"] else []}


def rate_hz(stamps_ns):
    return (len(stamps_ns) - 1) / ((stamps_ns[-1] - stamps_ns[0]) * 1e-9)


def report(a, c, path):
    fix, n = a["fix"], len(a["fix"]["innovation_xy_m"])
    print(f"clip: {os.path.basename(path)}   {a['fb_t'][-1]:.1f} s")
    print(f"EKF output {FIELD_BASE}: {len(a['fb_st'])} poses, {rate_hz(a['fb_st']):.0f} Hz;  "
          f"VIO {ODOM}: {len(a['od_st'])} samples, {rate_hz(a['od_st']):.0f} Hz")
    path_m = np.sum(np.hypot(*np.diff(a["fb_xy"], axis=0).T))
    print(f"distance covered by the EKF pose: {path_m:.1f} m")
    multi = int(np.sum(fix["n_tags"] >= 2))
    print(f"tag fixes {TAG}: {len(c['tag'])};  logged per-fix values (/viz/fix/*): {n}")
    print(f"updates in the clip: {n} (the node's update_count {fix['update_count'][0]:.0f} to "
          f"{fix['update_count'][-1]:.0f}); applied {int(fix['applied'].sum())}; "
          f"multi-tag {multi}, single-tag {n - multi}; heading corrected on "
          f"{int(fix['rotation_applied'].sum())}")
    print(f"logged innovation: xy median {100 * np.median(fix['innovation_xy_m']):.1f} cm, "
          f"max {100 * fix['innovation_xy_m'].max():.1f} cm; yaw median "
          f"{np.median(np.abs(fix['innovation_yaw_deg'])):.2f} deg, max "
          f"{np.abs(fix['innovation_yaw_deg']).max():.2f} deg")
    print(f"logged position gain k_pos: median {np.median(fix['k_pos']):.2f} "
          f"({fix['k_pos'].min():.2f} to {fix['k_pos'].max():.2f}); logged sigma_pos_m: median "
          f"{100 * np.median(fix['sigma_pos_m']):.1f} cm")
    print(f"fix_event carries: {', '.join(a['event_keys'])}")
    print("innovation covariance: not logged, so no NIS; post-fit residual against the fix's "
          "logged covariance instead")
    d2 = np.array([p[1] for p in a["post"]])
    above = int(np.sum(d2 > CHI2_95_2DOF))
    print(f"post-fit residual (tag fix minus EKF at capture time, xy) over {len(d2)} fixes: "
          f"median {np.median([p[2] for p in a['post']]):.1f} cm, max "
          f"{max(p[2] for p in a['post']):.1f} cm; squared Mahalanobis median {np.median(d2):.2f}, "
          f"above the chi-square 95 % line (2 dof, {CHI2_95_2DOF}): {above} of {len(d2)} "
          f"({100 * above / len(d2):.1f} %)")
    hi = [p for p in a["post"] if p[1] > CHI2_95_2DOF]
    if hi:
        print(f"  above the line: {sum(p[3] for p in hi)} of {len(hi)} multi-tag; tightest logged "
              f"sigma {min(p[4] for p in hi):.0f} to {max(p[4] for p in hi):.0f} mm, residual "
              f"{min(p[2] for p in hi):.1f} to {max(p[2] for p in hi):.1f} cm")
    gap = 100 * np.hypot(*(a["cam_vio"][:, :2] - np.array(
        [np.interp(a["od_st"], a["fb_st"], a["cam_ekf"][:, i]) for i in range(2)]).T).T)
    print(f"VIO only (placed with the first field->odom transform, never corrected): "
          f"{gap[-1]:.1f} cm from the EKF at the end, max {gap.max():.1f} cm")
    return gap


def plot(a, gap, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker

    ink, ink2, grid, surface = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
    blue, orange, aqua = "#2a78d6", "#eb6834", "#1baf7a"
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": ink2, "axes.labelcolor": ink,
                         "xtick.color": ink2, "ytick.color": ink2, "axes.grid": True,
                         "grid.color": grid, "grid.linewidth": 0.6, "figure.facecolor": surface,
                         "axes.facecolor": surface, "savefig.facecolor": surface,
                         "axes.spines.top": False, "axes.spines.right": False})

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(10.0, 4.6), gridspec_kw={"width_ratios": [1.15, 1]})
    ax.plot(a["cam_vio"][:, 0], a["cam_vio"][:, 1], color=aqua, lw=1.2, label="VIO only")
    ax.plot(a["cam_ekf"][:, 0], a["cam_ekf"][:, 1], color=blue, lw=1.5,
            label="EKF (/odin_tree/field_base, moved to the camera)")
    multi, xy = a["tag_multi"], a["tag_xy"]
    ax.scatter(xy[multi, 0], xy[multi, 1], s=12, color=orange, edgecolors=surface,
               linewidths=0.4, zorder=4, label="tag fix, multi-tag")
    ax.scatter(xy[~multi, 0], xy[~multi, 1], s=14, marker="^", color=orange, edgecolors=ink,
               linewidths=0.4, zorder=4, label="tag fix, single tag")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("field x (m)")
    ax.set_ylabel("field y (m)")
    ax.set_title("Camera on the field, as logged", color=ink, loc="left")
    ax.legend(frameon=False, fontsize=7.5, loc="best")

    t_od = (a["od_st"] - a["od_st"][0]) * 1e-9 + a["fb_t"][0]
    ax2.plot(t_od, gap, color=aqua, lw=1.2)
    ax2.set_xlabel("time in clip (s)")
    ax2.set_xlim(left=0.0)
    ax2.set_ylabel("horizontal distance from the EKF (cm)")
    ax2.set_title("VIO only, against the EKF", color=ink, loc="left")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "trajectory.png"), dpi=FIG_DPI)
    plt.close(fig)

    fix, t = a["fix"], a["fix_t"]
    multi = fix["n_tags"] >= 2
    fig, (b1, b2, b3) = plt.subplots(3, 1, figsize=(7.6, 7.0), sharex=True)
    for sel, marker, color, edge, label in ((multi, "o", blue, surface, "multi-tag fix"),
                                            (~multi, "^", orange, ink, "single-tag fix")):
        b1.scatter(t[sel], 100 * fix["innovation_xy_m"][sel], s=14, marker=marker, color=color,
                   edgecolors=edge, linewidths=0.4, zorder=3, label=label)
        b3.scatter(t[sel], fix["k_pos"][sel], s=14, marker=marker, color=color, edgecolors=edge,
                   linewidths=0.4, zorder=3)
    b1.set_ylabel("innovation, xy (cm)")
    b1.set_title("Logged innovation per fix", color=ink, loc="left")
    b1.legend(frameon=False, loc="upper right", fontsize=7.5)

    p = np.array([(tt, d2, m) for tt, d2, _, m, _ in a["post"]])
    for sel, marker, color, edge in ((p[:, 2] == 1, "o", blue, surface),
                                     (p[:, 2] == 0, "^", orange, ink)):
        b2.scatter(p[sel, 0], p[sel, 1], s=14, marker=marker, color=color, edgecolors=edge,
                   linewidths=0.4, zorder=3)
    b2.axhline(CHI2_95_2DOF, color=ink2, lw=1.0, ls="--", zorder=2)
    b2.annotate(f"chi-square 95 % line, 2 dof ({CHI2_95_2DOF:.2f})", (1.0, CHI2_95_2DOF),
                xycoords=("axes fraction", "data"), xytext=(-4, 3), textcoords="offset points",
                ha="right", va="bottom", color=ink2, fontsize=7.5)
    b2.set_yscale("log")
    b2.yaxis.set_major_formatter(matplotlib.ticker.FormatStrFormatter("%g"))
    b2.set_ylabel("squared Mahalanobis")
    b2.set_title("Post-fit residual against the fix's logged xy covariance (not an NIS)",
                 color=ink, loc="left")

    b3.set_ylabel("k_pos")
    b3.set_ylim(0.0, 1.0)
    b3.set_title("Logged position gain", color=ink, loc="left")
    b3.set_xlabel("time in clip (s)")
    b3.set_xlim(left=0.0)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "innovation.png"), dpi=FIG_DPI)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(
        description="Summarise the field EKF's logged inputs and outputs: pose and VIO rates, the "
                    "number of tag updates, the node's logged innovation, gain and sigma per fix, "
                    "a post-fit residual of each fix against the published track (its squared "
                    "Mahalanobis distance under the fix's logged covariance and the share above "
                    "the chi-square 95 % line), and how far uncorrected VIO drifts from the EKF. "
                    "Reads logged messages only. Writes trajectory.png and innovation.png.")
    ap.add_argument("--log", default=DEFAULT_LOG, help="MCAP clip to read (default: the bundled one)")
    ap.add_argument("--out", default=DEFAULT_OUT,
                    help="directory for the PNGs (default: replay_out/ next to this script)")
    args = ap.parse_args()

    c = load(args.log)
    a = analyse(c)
    gap = report(a, c, args.log)

    os.makedirs(args.out, exist_ok=True)
    plot(a, gap, args.out)
    shown = os.path.relpath(args.out) if args.out == DEFAULT_OUT else args.out
    print(f"wrote {os.path.join(shown, 'trajectory.png')} and "
          f"{os.path.join(shown, 'innovation.png')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
