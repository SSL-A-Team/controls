"""Per-robot: wheel slip feed-forward gain (WHEEL_SLIP_COMP in
control-board/src/motion/params/controller_params.rs).

Under linear load the wheel surface outruns the ground, roughly in proportion to the
wheel force, so the firmware raises each wheel's velocity setpoint by
K * (linear part of its feed-forward current).

1. Fit: records sideways and forward position move pairs and fits, per wheel,
       wheel speed - kinematic speed from the KF body velocity = K * I_linear + b
   where I_linear is the wheel current setpoint with its torque (rotation) part
   removed. Also runs two fast theta ramps and reports the same fit against the
   rotation current, which should be ~0 (rotation does not slip; the firmware only
   compensates the linear part).
2. Validate (--validate): A/B candidate K values (radio parameter WHEEL_SLIP_COMP, id 19)
   on the move set; on robot 2 the fit gave ~1.0 and 1.2 tracked best.

    python analysis/calibration/wheel_slip.py --home=0,0,3.14 --heading-ref 3.14
    python analysis/calibration/wheel_slip.py --home=0,0,3.14 --heading-ref 3.14 --skip-fit --validate 0 0.8 1.2 1.6
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from common import M, R, add_common_args, move, record_during, test_cmd, validate, wheels  # noqa: E402

M_PINV = np.linalg.pinv(M)  # local [Fx, Fy, Tz] -> wheel forces


def split_current(I):
    """Wheel currents -> (linear part, rotation part), via the body force they produce."""
    F = I @ M.T
    lin = np.stack([F[:, 0], F[:, 1], np.zeros(len(F))], 1) @ M_PINV.T
    return lin, I - lin


def slip_fit(g, rotation):
    W = wheels(g, "enc")
    kv = np.stack([g("vx_l"), g("vy_l"), g("gyro")], 1)
    excess = W - kv @ M / R  # wheel speed beyond the body motion (rad/s)
    lin, rot = split_current(wheels(g, "i_sp"))
    if rotation:
        sel = np.abs(g("gyro")) > 0.3
        x = rot
    else:
        sel = np.hypot(g("vx_l"), g("vy_l")) > 0.05
        x = lin
    if sel.sum() < 30:
        raise SystemExit(f"only {sel.sum()} samples in motion")
    fits = [np.polyfit(x[sel, i], excess[sel, i], 1) for i in range(4)]
    r2 = [np.corrcoef(x[sel, i], excess[sel, i])[0, 1] ** 2 for i in range(4)]
    return np.array([f[0] for f in fits]), np.array(r2), int(sel.sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--skip-fit", action="store_true", help="only run --validate")
    ap.add_argument("--validate", nargs="*", default=[], help="WHEEL_SLIP_COMP candidates to A/B")
    ap.add_argument("--repeats", type=int, default=2, help="move-set repeats per validation candidate")
    args = ap.parse_args()

    if not args.skip_fit:
        g = record_during(args, [move(args, 0, 0.2, 0), move(args, 0.2, 0, 0)], duration=28)[1]
        k, r2, n = slip_fit(g, rotation=False)
        print(f"linear slip over {n} samples: K per wheel (FL, BL, BR, FR) {np.round(k, 2)}, R^2 {np.round(r2, 2)}")
        k_lin = float(np.mean(k))
        print(f"  K = {k_lin:.2f} (rad/s)/A -> WHEEL_SLIP_COMP (fitted; validate before using)")
        ramp = test_cmd("theta_ramp_test", args, "--w", 1.8, "--ramp", 0.5, "--hold-w", 0.2)
        g = record_during(args, [ramp], duration=16)[1]
        k, r2, n = slip_fit(g, rotation=True)
        print(f"rotation slip over {n} samples: K per wheel {np.round(k, 2)} (expect ~0; if clearly > 0.3 the "
              "firmware's linear-only compensation needs revisiting)")
        if not args.validate:
            print(f"\nnext: --skip-fit --validate 0 {k_lin:.1f} {k_lin + 0.4:.1f} (the fitted K and neighbours)")
    if args.validate:
        validate(args, 19, args.validate, "WHEEL_SLIP_COMP")


if __name__ == "__main__":
    main()
