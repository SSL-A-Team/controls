"""Model-only TDP figures (no robot needed).

1. feasible_accel_polar: largest linear acceleration the planner may use, vs travel
   direction in the robot frame, with fresh (3.5 A) and thermally derated (1.9 A)
   feed-forward limits (ControlContext::feasible_linear_accel).
2. budget_vs_clip: for a commanded acceleration above what the wheels can deliver,
   what the robot actually gets with the firmware's friction-first,
   direction-preserving budget vs. naive per-wheel clipping: heading error of the
   net linear acceleration and the spurious yaw acceleration (this is the "curves off
   its path" failure).
3. friction_direction: the model's Coulomb compensation force vs direction,
   C * k(dir), i.e. what fig_friction.py's measured data should collapse onto.

    python analysis/tdp/fig_model.py --out analysis/tdp/figures
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import tdp_common as tc  # noqa: E402


def delivered(currents, v_dir):
    """Net local acceleration from wheel currents while moving along v_dir (above the
    Coulomb ramp), using the identified efficiency and friction."""
    F = tc.currents_to_local_force(currents) * np.array([tc.EFF_XY, tc.EFF_XY, tc.EFF_W])
    F_fric = tc.friction_force(np.array([*v_dir * 0.5, 0.0]))
    return np.linalg.solve(tc.INERTIA, F - F_fric)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="analysis/tdp/figures")
    ap.add_argument("--accel", type=float, default=3.0, help="commanded acceleration for budget_vs_clip (m/s^2)")
    ap.add_argument("--limit", type=float, default=tc.FF_LIMIT_A, help="FF current limit for budget_vs_clip (A)")
    args = ap.parse_args()
    plt = tc.lncs_style()

    ang = np.radians(np.arange(0, 360.5, 1.0))
    dirs = np.stack([np.cos(ang), np.sin(ang)], 1)

    # 1. feasible acceleration polar
    fig = plt.figure(figsize=(tc.LNCS_WIDTH_IN * 0.55, tc.LNCS_WIDTH_IN * 0.55))
    ax = fig.add_subplot(projection="polar")
    for lim, ls, lab in ((tc.FF_LIMIT_A, "-", f"{tc.FF_LIMIT_A} A (fresh)"),
                         (tc.FF_DERATED_A, "--", f"{tc.FF_DERATED_A} A (thermally derated)")):
        a = np.array([tc.feasible_linear_accel(d, lim) for d in dirs])
        ax.plot(ang, a, ls, label=lab)
        print(f"feasible accel @ {lim} A: forward {a[0]:.2f}, sideways {a[90]:.2f}, "
              f"diagonal {a[45]:.2f} m/s^2 (planner uses x{tc.FEASIBLE_ACCEL_MARGIN})")
    ax.set_theta_zero_location("N")  # robot forward (local +x) up
    ax.set_rlabel_position(120)
    ax.set_rticks([0.5, 1.0, 1.5, 2.0])
    ax.set_title("feasible linear accel (m/s$^2$)\nvs. direction (robot frame, forward up)", pad=10)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.32))
    tc.save(fig, args.out, "feasible_accel_polar")

    # 2. direction-preserving budget vs per-wheel clipping
    err = {"budget": [], "clip": []}
    yaw = {"budget": [], "clip": []}
    mag = {"budget": [], "clip": []}
    for d in dirs:
        u = np.array([*d, 0.0])
        i_fric = tc.accel_to_current(tc.ff_accel(u * 0.5, np.zeros(3)))
        i_acc = tc.accel_to_current(tc.ff_accel(np.zeros(3), u * args.accel))
        for name, fn in (("budget", tc.budget_direction_preserving), ("clip", tc.budget_per_wheel_clip)):
            a = delivered(fn(i_fric, i_acc, args.limit), d)
            # no meaningful direction when friction eats the whole budget (net accel ~0 or braking)
            e = math.degrees(math.remainder(math.atan2(a[1], a[0]) - math.atan2(d[1], d[0]), 2 * math.pi))
            err[name].append(e if a[:2] @ d > 0.05 else np.nan)
            yaw[name].append(a[2])
            mag[name].append(math.hypot(a[0], a[1]))
    deg = np.degrees(ang)
    fig, axs = plt.subplots(3, 1, figsize=(tc.LNCS_WIDTH_IN, 3.6), sharex=True)
    for name, ls in (("clip", "-"), ("budget", "--")):
        lab = "per-wheel clipping" if name == "clip" else "friction-first, direction-preserving"
        axs[0].plot(deg, mag[name], ls, label=lab)
        axs[1].plot(deg, err[name], ls)
        axs[2].plot(deg, yaw[name], ls)
    axs[0].axhline(args.accel, color="k", lw=0.6, ls=":")
    axs[0].set_ylabel("|a| (m/s$^2$)")
    axs[1].set_ylabel("dir. err (deg)")
    axs[2].set_ylabel("yaw acc. (rad/s$^2$)")
    fig.align_ylabels(axs)
    axs[2].set_xlabel("commanded direction in robot frame (deg, 0 = forward)")
    axs[2].set_xticks(range(0, 361, 45))
    axs[0].legend(loc="upper right")
    axs[0].set_title(f"commanded {args.accel} m/s$^2$ at {args.limit} A feed-forward limit (model)")
    tc.save(fig, args.out, "budget_vs_clip")
    print(f"per-wheel clipping: max direction error {np.nanmax(np.abs(err['clip'])):.1f} deg, "
          f"max yaw accel {np.max(np.abs(yaw['clip'])):.1f} rad/s^2; budget: "
          f"{np.nanmax(np.abs(err['budget'])):.2f} deg, {np.max(np.abs(yaw['budget'])):.2f} rad/s^2")

    # 3. model friction vs direction
    fig = plt.figure(figsize=(tc.LNCS_WIDTH_IN * 0.55, tc.LNCS_WIDTH_IN * 0.55))
    ax = fig.add_subplot(projection="polar")
    k = np.array([tc.k_dir(d) for d in dirs])
    ax.plot(ang, tc.COULOMB_XY * k, label=f"C k(dir), C = {tc.COULOMB_XY} N")
    ax.set_theta_zero_location("N")
    ax.set_title("model Coulomb force (N, ground side)", pad=10)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.25))
    tc.save(fig, args.out, "friction_direction_model")
    print(f"k(forward) = {k[0]:.3f}, k(diagonal) = {k[45]:.3f}, k(sideways) = {k[90]:.3f}")


if __name__ == "__main__":
    main()
