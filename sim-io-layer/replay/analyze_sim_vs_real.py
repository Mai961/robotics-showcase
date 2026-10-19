#!/usr/bin/env python3
"""Compare one logged whole-body cycle in simulation with one on the real robot.

Both recordings hold one DRIVE -> APPROACH -> TASK -> RETRACT -> DRIVE cycle of the same
whole-body controller toward the same goal: replay/sim-cycle.mcap with the robot program in WPILib
simulation (the simulated IO backend), and control/wholebody-2026's shop recording with the real
robot (the TalonFX backend). This script only reads what the two runs logged: the measured and
reference joints (/wholebody/measured, /wholebody/references) and the phase events
(/wholebody/event), all at their log times. It shifts each run so that its APPROACH -> TASK event
is at t = 0, samples both on a common 5 ms grid by linear interpolation, and reports the RMS of sim
minus real per joint, the phase durations of each run, and each run's tracking RMS (measured minus
reference). No controller, no solver and no plant model is run.

Needs numpy, matplotlib and mcap-ros2-support.
"""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SIM = os.path.join(HERE, "sim-cycle.mcap")
DEFAULT_REAL = os.path.normpath(os.path.join(HERE, "..", "..", "control", "wholebody-2026",
                                             "replay", "wholebody-cycle-shop.mcap"))
DEFAULT_OUT = os.path.join(HERE, "replay_out")
JOINTS = ("q1", "d", "q3")
UNITS = {"q1": "rad", "d": "m", "q3": "rad"}
TITLES = {"q1": "Shoulder q1", "d": "Telescope d", "q3": "Wrist q3"}
DT = 0.005      # common grid for the comparison, s
PAD_S = 2.0     # the full-cycle window reaches this far before the latch and after DRIVE
FIG_DPI = 150


def load(path):
    """Measured and reference joints as [t, q1, d, q3] rows, and the first time of each event.

    Times are log times in seconds from the recording's first message.
    """
    from mcap.reader import make_reader
    from mcap_ros2.reader import read_ros2_messages

    with open(path, "rb") as f:
        t0 = make_reader(f).get_summary().statistics.message_start_time
    rows = {"meas": [], "ref": []}
    ev = {}
    topics = ["/wholebody/measured", "/wholebody/references", "/wholebody/event"]
    for m in read_ros2_messages(path, topics=topics):
        t = (m.log_time_ns - t0) * 1e-9
        if m.channel.topic == "/wholebody/event":
            e = json.loads(m.ros_msg.data)
            if e.get("event") == "goal_latched":
                ev.setdefault("latch", t)
                ev.setdefault("goal", e.get("goal"))
            elif e.get("event") == "goal_released":
                ev.setdefault("release", t)
            elif e.get("event") == "phase_transition":
                ev.setdefault(f"{e['from']}->{e['to']}", t)
        else:
            pos = dict(zip(m.ros_msg.name, m.ros_msg.position))
            key = "meas" if m.channel.topic.endswith("measured") else "ref"
            rows[key].append([t] + [pos[j] for j in JOINTS])
    run = {k: np.array(v, dtype=float) for k, v in rows.items()}
    run["ev"] = ev
    return run


def phases(ev):
    return {"APPROACH": ev["APPROACH->TASK"] - ev["DRIVE->APPROACH"],
            "TASK": ev["TASK->RETRACT"] - ev["APPROACH->TASK"],
            "RETRACT": ev["RETRACT->DRIVE"] - ev["TASK->RETRACT"],
            "latch -> DRIVE": ev["RETRACT->DRIVE"] - ev["latch"]}


def aligned(run):
    """Copies of the joint series with the run's APPROACH -> TASK event at t = 0."""
    a = run["ev"]["APPROACH->TASK"]
    out = {}
    for k in ("meas", "ref"):
        s = run[k].copy()
        s[:, 0] -= a
        out[k] = s
    out["ev"] = {k: v - a for k, v in run["ev"].items() if k != "goal"}
    return out


def on_grid(series, win):
    g = np.arange(win[0], win[1] + 1e-9, DT)
    return np.column_stack([np.interp(g, series[:, 0], series[:, 1 + j])
                            for j in range(len(JOINTS))])


def rms(a):
    return np.sqrt(np.mean(a ** 2, axis=0))


def compare(sim, real):
    s, r = aligned(sim), aligned(real)
    task = (0.0, min(phases(sim["ev"])["TASK"], phases(real["ev"])["TASK"]))
    full = (max(s["ev"]["latch"] - PAD_S, r["ev"]["latch"] - PAD_S,
                s["meas"][0, 0], r["meas"][0, 0]),
            min(s["ev"]["RETRACT->DRIVE"] + PAD_S, r["ev"]["RETRACT->DRIVE"] + PAD_S,
                s["meas"][-1, 0], r["meas"][-1, 0]))
    res = {"windows": {"TASK overlap": task, "full cycle": full}, "diff": {}, "track": {}}
    for name, win in res["windows"].items():
        res["diff"][name] = rms(on_grid(s["meas"], win) - on_grid(r["meas"], win))
    for name, run in (("sim", s), ("real", r)):
        res["track"][name] = rms(on_grid(run["meas"], task) - on_grid(run["ref"], task))
    return s, r, res


def report(sim, real, res, sim_path, real_path):
    for name, run, path in (("sim", sim, sim_path), ("real", real, real_path)):
        rate = len(run["meas"]) / (run["meas"][-1, 0] - run["meas"][0, 0])
        print(f"{name}: {os.path.basename(path)}   goal {run['ev'].get('goal')}, "
              f"{len(run['meas'])} measured-joint messages ({rate:.0f} Hz), "
              f"APPROACH -> TASK at {run['ev']['APPROACH->TASK']:.3f} s")
    print("phase durations (s)      sim     real")
    ps, pr = phases(sim["ev"]), phases(real["ev"])
    for k in ps:
        print(f"  {k:<20} {ps[k]:7.3f}  {pr[k]:7.3f}")
    print("measured joints, RMS of sim - real, both aligned at APPROACH -> TASK:")
    for name, (a, b) in res["windows"].items():
        d = res["diff"][name]
        print(f"  {name} ({a:+.2f} s to {b:+.2f} s): "
              + ", ".join(f"{j} {d[i]:.4f} {UNITS[j]}" for i, j in enumerate(JOINTS)))
    print("tracking in TASK overlap, RMS of measured - reference:")
    for name in ("sim", "real"):
        d = res["track"][name]
        print(f"  {name}: " + ", ".join(f"{j} {d[i]:.4f} {UNITS[j]}"
                                       for i, j in enumerate(JOINTS)))


def plot(s, r, res, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    ink, ink2, grid, surface, band = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb", "#ebe9e2"
    sim_c, real_c = "#2a78d6", "#eb6834"
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": ink2, "axes.labelcolor": ink,
                         "xtick.color": ink2, "ytick.color": ink2, "axes.grid": True,
                         "grid.color": grid, "grid.linewidth": 0.6, "figure.facecolor": surface,
                         "axes.facecolor": surface, "savefig.facecolor": surface,
                         "axes.spines.top": False, "axes.spines.right": False})
    task, full = res["windows"]["TASK overlap"], res["windows"]["full cycle"]
    fig, axes = plt.subplots(3, 1, figsize=(8.0, 7.4), sharex=True)
    for j, (ax, k) in enumerate(zip(axes, JOINTS)):
        ax.axvspan(task[0], task[1], color=band, lw=0, zorder=0)
        for run, color in ((s, sim_c), (r, real_c)):
            ax.plot(run["ref"][:, 0], run["ref"][:, 1 + j], color=color, lw=1.0,
                    ls=(0, (3, 2)), zorder=2)
            ax.plot(run["meas"][:, 0], run["meas"][:, 1 + j], color=color, lw=1.8, zorder=3)
            for e in ("latch", "RETRACT->DRIVE"):
                ax.axvline(run["ev"][e], color=color, lw=0.8, ls=":", zorder=1)
        ax.axvline(0.0, color=ink2, lw=0.9, zorder=1)
        ax.set_ylabel(f"{UNITS[k]}")
        ax.set_title(f"{TITLES[k]}: RMS sim - real {res['diff']['TASK overlap'][j]:.3f} {UNITS[k]} "
                     f"in TASK, {res['diff']['full cycle'][j]:.3f} {UNITS[k]} over the cycle",
                     color=ink, loc="left")
    handles = [Line2D([], [], color=sim_c, lw=1.8), Line2D([], [], color=real_c, lw=1.8),
               Line2D([], [], color=sim_c, lw=1.0, ls=(0, (3, 2))),
               Line2D([], [], color=real_c, lw=1.0, ls=(0, (3, 2))),
               Line2D([], [], color=ink2, lw=0.8, ls=":"),
               Patch(facecolor=band, edgecolor=ink2, lw=0.5)]
    labels = ["sim measured", "real measured", "sim reference", "real reference",
              "latch and back in DRIVE (each run)", "TASK overlap"]
    fig.legend(handles, labels, frameon=False, fontsize=7.5, loc="upper center", ncol=3,
               bbox_to_anchor=(0.5, 1.0))
    axes[-1].set_xlabel("time from APPROACH -> TASK (s)")
    axes[-1].set_xlim(full[0], full[1])
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(os.path.join(out_dir, "sim_vs_real.png"), dpi=FIG_DPI)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(
        description="Compare a logged whole-body cycle in simulation with one on the real robot: "
                    "both aligned at their APPROACH -> TASK event, RMS of sim minus real measured "
                    "joints over the TASK overlap and the full cycle, phase durations, and each "
                    "run's tracking RMS in TASK. Reads the logged joints and events only. Writes "
                    "sim_vs_real.png.")
    ap.add_argument("--sim", default=DEFAULT_SIM,
                    help="simulated cycle, MCAP (default: the bundled sim-cycle.mcap)")
    ap.add_argument("--real", default=DEFAULT_REAL,
                    help="real cycle, MCAP (default: control/wholebody-2026's "
                         "wholebody-cycle-shop.mcap)")
    ap.add_argument("--out", default=DEFAULT_OUT,
                    help="directory for the PNG (default: replay_out/ next to this script)")
    args = ap.parse_args()

    sim, real = load(args.sim), load(args.real)
    s, r, res = compare(sim, real)
    report(sim, real, res, args.sim, args.real)

    os.makedirs(args.out, exist_ok=True)
    plot(s, r, res, args.out)
    shown = os.path.relpath(args.out) if args.out == DEFAULT_OUT else args.out
    print(f"wrote {os.path.join(shown, 'sim_vs_real.png')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
