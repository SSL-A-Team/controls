"""Per-setup: vision noise and the EKF vision measurement std (R, DEFAULT_EKF_VISION_VAR_*).

1. Records vision at rest and reports its noise (the floor for R).
2. Runs a sideways move pair and a diagonal + 90 deg move pair and reports the
   real-time estimate error: each applied vision sample compared with the KF estimate
   the robot had at that capture time. At rest this should be ~1 mm; a larger constant
   offset means a bias (e.g. accelerometer) rather than R.
3. With --r-values, repeats step 2 for each R (radio parameter KF_MEASUREMENT_STD, id 1,
   [std_lin_m, std_ang_rad, 0, 0]). Pick the largest R that is close to the best moving
   error; smaller R trusts vision more, which hurts with noisy multi-camera vision.

Run vision_latency.py first: with a wrong latency, small R makes the estimate worse.
Result -> DEFAULT_EKF_VISION_VAR_M / DEFAULT_EKF_VISION_VAR_RAD (std squared) in
ateam-controls/src/defaults.rs.

    python analysis/calibration/vision_noise.py --home=0,0,3.14 --heading-ref 3.14 \
        --r-values 0.05,0.1 0.01,0.01 0.003,0.005
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from common import add_common_args, load, move, record_during, set_param  # noqa: E402
from vision_latency import firmware_default  # noqa: E402


def estimate_error(t, g, ekf_delay_s):
    v = (g("vis_upd") > 0) & (g("vis_old") == 0)
    tc = t[v] - ekf_delay_s
    ex = (g("vis_x")[v] - np.interp(tc, t, g("x"))) * 1e3
    ey = (g("vis_y")[v] - np.interp(tc, t, g("y"))) * 1e3
    eth = np.degrees(np.remainder(g("vis_th")[v] - np.interp(tc, t, np.unwrap(g("th"))) + np.pi, 2 * np.pi) - np.pi)
    sp = np.interp(tc, t, np.hypot(g("vx_l"), g("vy_l")))
    w = np.abs(np.interp(tc, t, g("gyro")))
    for label, m in (("moving", (sp > 0.1) | (w > 0.3)), ("at rest", (sp < 0.02) & (w < 0.05))):
        if m.sum() < 3:
            continue
        e = np.hypot(ex[m], ey[m])
        print(f"    {label:8s} n {m.sum():4d}: XY error rms {np.sqrt((e ** 2).mean()):.1f} mm (p95 "
              f"{np.percentile(e, 95):.1f}), mean offset ({ex[m].mean():+.1f}, {ey[m].mean():+.1f}) mm | "
              f"heading rms {np.sqrt((eth[m] ** 2).mean()):.2f} deg")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--ekf-delay-ms", type=float, default=firmware_default("DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US"))
    ap.add_argument("--rest", type=float, default=10.0, help="seconds of rest recording")
    ap.add_argument("--r-values", nargs="*", default=[],
                    help="R std pairs 'lin_m,ang_rad' to compare (sets radio param 1 for each)")
    args = ap.parse_args()

    t, g = record_during(args, [], duration=args.rest)
    v = g("vis_upd") > 0
    print(f"vision at rest ({v.sum()} samples): std x {g('vis_x')[v].std() * 1e3:.2f} mm, "
          f"y {g('vis_y')[v].std() * 1e3:.2f} mm, theta {np.degrees(g('vis_th')[v].std()):.3f} deg")

    moves = [move(args, 0, 0.2), move(args, 0.15, 0.15, 90)]
    for r in args.r_values or [None]:
        if r is not None:
            if not set_param(args.robot, 1, f"{r},0,0"):
                print(f"R {r}: could not set parameter")
                continue
        print(f"  R = {r or 'as flashed'}:")
        t, g = record_during(args, moves, duration=36)
        estimate_error(t, g, args.ekf_delay_ms * 1e-3)
    if args.r_values:
        print("\nreflash (or set param 1 back) to restore the compiled R")


if __name__ == "__main__":
    main()
