"""Field-scale analysis of field_record.py recordings (kenobi TdpMotionPlay runs).

Each change of the commanded target starts a move. For every move (the first one, the
positioning move, is dropped, as are pure rotations < 0.1 m):
  dir_deg      travel direction in the robot frame at the start (0 = forward, 90 = left)
  a_plan       trajectory acceleration the firmware planned (m/s^2)
  a_kf         achieved acceleration: slope of the estimated velocity along the move
               during the planned acceleration phase
  a_imu        same from the accelerometer (median), as a cross-check
  xtrack_mm    max distance from the straight start->target line (the ablation's
               "curves off its path")
  hdg_dev_deg  max heading error vs the trajectory heading
  track_mm     RMS |trajectory - estimate| while the trajectory moves
  t_reach_s    time to within 2 cm of the target
  i_max        max wheel current setpoint (A); tlim = any motor torque_limited flag

Figures (pdf + png):
  field_accel_polar   a_kf and a_plan vs direction over the model's feasible accel
  field_deviation     cross-track and heading deviation vs direction, one series per input
  field_thermal       replayed I2T budget use and FF limit vs time (--thermal)
  field_paths         XY paths of the first input

    python3 analysis/tdp/fig_field.py shipped=analysis/data/tdp/field_star_shipped_r2.npz \
        nocap=analysis/data/tdp/field_star_nocap_r2.npz clip=analysis/data/tdp/field_star_clip_r2.npz
    python3 analysis/tdp/fig_field.py soak=analysis/data/tdp/field_soak_r2.npz --thermal --name-prefix soak_
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "calibration"))
import tdp_common as tc  # noqa: E402
from common import load  # noqa: E402

GLOBAL_POSITION = 10


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def segments(t, g):
    tgt = np.stack([g("cmd_x"), g("cmd_y"), g("cmd_th")], 1)
    valid = ~np.isnan(tgt).any(axis=1) & (g("cmd_mode") == GLOBAL_POSITION)
    change = np.where(valid[1:] & (np.abs(np.diff(tgt, axis=0)) > 1e-6).any(axis=1))[0] + 1
    first_valid = np.argmax(valid) if valid.any() else None
    starts = ([first_valid] if first_valid is not None else []) + list(change)
    bounds = list(zip(starts, starts[1:] + [len(t)]))
    return bounds[1:]  # drop the positioning move


def analyze(t, g):
    pos = np.stack([g("x"), g("y")], 1)
    th = np.unwrap(g("th"))
    v_l = np.stack([g("vx_l"), g("vy_l")], 1)
    c, s = np.cos(th), np.sin(th)
    v_g = np.stack([c * v_l[:, 0] - s * v_l[:, 1], s * v_l[:, 0] + c * v_l[:, 1]], 1)
    a_imu_l = np.stack([g("imu_ax"), g("imu_ay")], 1)
    traj_p = np.stack([g("traj_p_x"), g("traj_p_y")], 1)
    traj_v = np.stack([g("traj_v_x"), g("traj_v_y")], 1)
    I = np.stack([g(f"i_sp{i}") for i in range(4)], 1)
    tlim = np.stack([g(f"tlim{i}") for i in range(4)], 1)
    rows = []
    for a, b in segments(t, g):
        T = np.array([g("cmd_x")[a], g("cmd_y")[a]])
        start = pos[a]
        L = np.linalg.norm(T - start)
        if L < 0.1 or b - a < 10:
            continue
        d = (T - start) / L
        dir_local = math.degrees(math.atan2(d[1], d[0]) - th[a]) % 360.0
        tt = t[a:b] - t[a]
        v_along_traj = traj_v[a:b] @ d
        peak = np.argmax(v_along_traj)
        acc = np.arange(0, max(peak, 1))
        acc = acc[v_along_traj[acc] < 0.9 * v_along_traj[peak]] if peak > 2 else acc
        if len(acc) < 3:
            continue
        a_plan = float(np.polyfit(tt[acc], v_along_traj[acc], 1)[0])
        a_kf = float(np.polyfit(tt[acc], v_g[a:b][acc] @ d, 1)[0])
        u_local = np.stack([np.cos(math.atan2(d[1], d[0]) - th[a:b]), np.sin(math.atan2(d[1], d[0]) - th[a:b])], 1)
        a_imu = float(np.median(np.sum(a_imu_l[a:b] * u_local, axis=1)[acc]))
        rel = pos[a:b] - start
        xtrack = np.abs(rel[:, 0] * d[1] - rel[:, 1] * d[0])
        hdg = np.abs(wrap(g("th")[a:b] - g("traj_p_w")[a:b]))
        moving = np.linalg.norm(traj_v[a:b], axis=1) > 0.05
        track = np.linalg.norm(traj_p[a:b] - pos[a:b], axis=1)
        reach = np.where(np.linalg.norm(pos[a:b] - T, axis=1) < 0.02)[0]
        rows.append(dict(dir_deg=dir_local, length_m=float(L), cmd_amax=float(g("cmd_amax")[a]),
                         a_plan=a_plan, a_kf=a_kf, a_imu=a_imu,
                         a_model=tc.feasible_linear_accel((math.cos(math.radians(dir_local)),
                                                           math.sin(math.radians(dir_local)))),
                         xtrack_mm=float(xtrack.max() * 1e3), hdg_dev_deg=float(np.degrees(hdg.max())),
                         track_mm=float(np.sqrt((track[moving] ** 2).mean()) * 1e3) if moving.any() else np.nan,
                         t_reach_s=float(tt[reach[0]]) if len(reach) else np.nan,
                         i_max=float(np.abs(I[a:b]).max()), tlim=bool(tlim[a:b].any())))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="label=recording.npz")
    ap.add_argument("--thermal", action="store_true", help="also plot the replayed thermal budget")
    ap.add_argument("--name-prefix", default="")
    ap.add_argument("--out", default="analysis/tdp/figures")
    args = ap.parse_args()
    plt = tc.lncs_style()
    runs = []
    for item in args.logs:
        label, _, path = item.partition("=")
        t, g = load(path)
        rows = analyze(t, g)
        runs.append((label, t, g, rows))
        print(f"\n{label}: {len(rows)} moves ({path})")
        print(" dir  len  a_cmd a_plan  a_kf a_imu a_model  xtrk  hdg  track reach  Imax tlim")
        for r in sorted(rows, key=lambda r: r["dir_deg"]):
            print(f"{r['dir_deg']:4.0f} {r['length_m']:4.2f} {r['cmd_amax']:5.2f} {r['a_plan']:6.2f} {r['a_kf']:5.2f}"
                  f" {r['a_imu']:5.2f} {r['a_model']:7.2f} {r['xtrack_mm']:5.0f} {r['hdg_dev_deg']:4.1f}"
                  f" {r['track_mm']:6.1f} {r['t_reach_s']:5.2f} {r['i_max']:5.2f} {'Y' if r['tlim'] else '-'}")
        Path(args.out).mkdir(parents=True, exist_ok=True)
        (Path(args.out) / f"{args.name_prefix}field_moves_{label}.json").write_text(json.dumps(rows, indent=1))

    # accel vs direction
    ang = np.radians(np.arange(0, 360.5, 1.0))
    model = np.array([tc.feasible_linear_accel((math.cos(a), math.sin(a))) for a in ang])
    fig = plt.figure(figsize=(tc.LNCS_WIDTH_IN * 0.6, tc.LNCS_WIDTH_IN * 0.6))
    ax = fig.add_subplot(projection="polar")
    ax.plot(ang, model, "k-", lw=0.9, label="model feasible")
    ax.plot(ang, tc.FEASIBLE_ACCEL_MARGIN * model, "k:", lw=0.9, label=f"planner cap (x{tc.FEASIBLE_ACCEL_MARGIN})")
    for label, _, _, rows in runs:
        if rows:
            dd = np.radians([r["dir_deg"] for r in rows])
            line, = ax.plot(dd, [r["a_kf"] for r in rows], "o", ms=3, label=f"{label} achieved")
            ax.plot(dd, [r["a_plan"] for r in rows], "x", ms=3, color=line.get_color(), alpha=0.6)
    ax.set_theta_zero_location("N")
    ax.set_rlabel_position(112)
    ax.set_title("acceleration vs direction (m/s$^2$)\nrobot frame, forward up; x = planned", pad=10)
    fig.legend(loc="outside lower center", ncol=2)
    tc.save(fig, args.out, f"{args.name_prefix}field_accel_polar")

    # deviation vs direction
    fig, axs = plt.subplots(1, 2, figsize=(tc.LNCS_WIDTH_IN, 1.9), sharex=True)
    for label, _, _, rows in runs:
        rs = sorted(rows, key=lambda r: r["dir_deg"])
        dd = [r["dir_deg"] for r in rs]
        axs[0].plot(dd, [r["xtrack_mm"] for r in rs], "o-", ms=3, label=label)
        axs[1].plot(dd, [r["hdg_dev_deg"] for r in rs], "o-", ms=3, label=label)
        print(f"{label}: median cross-track {np.median([r['xtrack_mm'] for r in rows]):.0f} mm, "
              f"median heading dev {np.median([r['hdg_dev_deg'] for r in rows]):.1f} deg, "
              f"median track {np.nanmedian([r['track_mm'] for r in rows]):.1f} mm")
    axs[0].set_ylabel("max cross-track (mm)")
    axs[1].set_ylabel("max heading error (deg)")
    for ax in axs:
        ax.set_xlabel("direction in robot frame (deg)")
        ax.set_xticks(range(0, 361, 90))
    fig.legend(*axs[0].get_legend_handles_labels(), loc="outside lower center", ncol=len(runs))
    tc.save(fig, args.out, f"{args.name_prefix}field_deviation")

    # paths
    label, t, g, _ = runs[0]
    fig, ax = plt.subplots(figsize=(tc.LNCS_WIDTH_IN * 0.7, tc.LNCS_WIDTH_IN * 0.45))
    ax.plot(g("x"), g("y"), lw=0.6, label=f"{label} estimate")
    ok = ~np.isnan(g("cmd_x"))
    ax.plot(g("cmd_x")[ok], g("cmd_y")[ok], "k+", ms=5, label="targets")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("field x (m)")
    ax.set_ylabel("field y (m)")
    ax.legend(loc="upper right")
    tc.save(fig, args.out, f"{args.name_prefix}field_paths")

    if args.thermal:
        fig, axs = plt.subplots(3, 1, figsize=(tc.LNCS_WIDTH_IN, 3.2), sharex=True)
        for label, t, g, _ in runs:
            meas = np.stack([g(f"i_meas{i}") for i in range(4)], 1)
            used, lim = tc.thermal_replay(t, meas)
            tt = t - t[0]
            line, = axs[0].plot(tt, np.abs(meas).max(axis=1), lw=0.6, label=label)
            axs[1].plot(tt, used.max(axis=1), color=line.get_color())
            axs[2].plot(tt, lim.min(axis=1), color=line.get_color())
            tl = np.stack([g(f"tlim{i}") for i in range(4)], 1).any(axis=1)
            if tl.any():
                axs[2].plot(tt[tl], np.full(tl.sum(), tc.FF_DERATED_A - 0.1), "r|", ms=4, label=f"{label} motor limit")
            print(f"{label}: peak budget use {used.max():.0%}, min FF limit {lim.min():.2f} A, "
                  f"motor torque_limited samples {int(tl.sum())}")
        axs[0].set_ylabel("max |I| (A)")
        axs[1].set_ylabel("I$^2$t budget used")
        axs[2].set_ylabel("FF limit (A)")
        axs[2].set_xlabel("time (s)")
        axs[0].legend(loc="upper right")
        fig.align_ylabels(axs)
        tc.save(fig, args.out, f"{args.name_prefix}field_thermal")


if __name__ == "__main__":
    main()
