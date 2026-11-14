"""Per-robot: linear output efficiency and Coulomb friction
(DEFAULT_PHYS_EFFICIENCY_XY, DEFAULT_PHYS_COULOMB_XY in ateam-controls/src/defaults.rs).

1. Fit: records forward, sideways and diagonal position moves, then least-squares fits
       m * a_local = eff * F_motor_local - k(dir) * coulomb * dir
   over moving samples, where F_motor comes from the wheel current setpoints, a_local
   from the KF pose (Savitzky-Golay second derivative), and k(dir) is the firmware's
   roller direction factor (1 sideways). The coulomb printed is in the firmware's units
   (ground force for sideways motion, N).
2. Validate (--validate): the fit only covers the model's terms; on robot 2 the fitted
   3.7 N tracked worse than 4.5 N. A/B candidate PHYS_FRICTION_MODEL values (radio
   parameter 6: [coulomb_xy, viscous_xy, coulomb_w, viscous_w, eff_xy, eff_w]) on the
   move set and keep the one with the lowest tracking error.

    python analysis/calibration/efficiency_friction.py --home=0,0,3.14 --heading-ref 3.14
    python analysis/calibration/efficiency_friction.py --home=0,0,3.14 --heading-ref 3.14 \
        --skip-fit --validate 4.5,0,0.1,0,0.75,0.8 3.7,0,0.1,0,0.75,0.8
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np
from scipy.signal import savgol_filter

sys.path.insert(0, os.path.dirname(__file__))
from common import M, R, add_common_args, load, move, record_during, validate, wheels  # noqa: E402

KT = 0.0335  # motor torque constant (N m / A), DF45M024053-A2


def direction_factor(dx, dy):
    """Firmware RobotModel::friction_direction_factor: roller speed along (dx, dy),
    normalized to 1 for sideways (local y) motion."""
    D = M[:2].T  # wheel drive directions
    roller = lambda x, y: np.abs(x * D[:, 1] - y * D[:, 0]).sum()
    return roller(dx, dy) / roller(0.0, 1.0)


def fit(t, g, mass):
    # uniform 100 Hz resample of the KF pose and current setpoints
    T = np.arange(t[0], t[-1], 0.01)
    x, y = np.interp(T, t, g("x")), np.interp(T, t, g("y"))
    th = np.interp(T, t, np.unwrap(g("th")))
    sg = lambda s, k: savgol_filter(s, 15, 3, deriv=k, delta=0.01)
    vx, vy, ax, ay = sg(x, 1), sg(y, 1), sg(x, 2), sg(y, 2)
    c, s = np.cos(th), np.sin(th)
    v_l = np.stack([c * vx + s * vy, -s * vx + c * vy], 1)
    a_l = np.stack([c * ax + s * ay, -s * ax + c * ay], 1)
    I = np.stack([np.interp(T, t, col) for col in wheels(g, "i_sp").T], 1)
    F = (KT / R) * I @ M.T  # motor-side local [Fx, Fy, Tz]

    speed = np.linalg.norm(v_l, axis=1)
    sel = np.where((speed > 0.1) & (np.abs(np.interp(T, t, g("gyro"))) < 1.0))[0][::2]
    rows, rhs = [], []
    for k in sel:
        u = v_l[k] / speed[k]
        kd = direction_factor(*u)
        for j in range(2):
            rows.append([F[k, j], -kd * u[j]])
            rhs.append(mass * a_l[k, j])
    A, b = np.array(rows), np.array(rhs)
    if len(sel) < 30:
        raise SystemExit(f"only {len(sel)} moving samples; did the moves run?")
    (eff, coulomb), *_ = np.linalg.lstsq(A, b, rcond=None)
    resid = np.sqrt(np.mean((A @ np.array([eff, coulomb]) - b) ** 2))
    return eff, coulomb, resid, len(sel)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--mass", type=float, default=2.7, help="robot mass (kg), DEFAULT_PHYS_MASS")
    ap.add_argument("--no-run", action="store_true", help="fit an existing --out recording")
    ap.add_argument("--validate", nargs="*", default=[],
                    help="PHYS_FRICTION_MODEL candidates 'coul_xy,visc_xy,coul_w,visc_w,eff_xy,eff_w' to A/B")
    ap.add_argument("--repeats", type=int, default=2, help="move-set repeats per validation candidate")
    ap.add_argument("--skip-fit", action="store_true", help="only run --validate")
    args = ap.parse_args()

    if not args.skip_fit:
        if args.no_run:
            t, g = load(args.out)
        else:
            moves = [move(args, 0.2, 0, 0), move(args, 0, 0.2, 0), move(args, 0.15, 0.15, 0)]
            t, g = record_during(args, moves, duration=40)
        eff, coulomb, resid, n = fit(t, g, args.mass)
        print(f"fit over {n} moving samples (residual {resid:.2f} N):")
        print(f"  efficiency_xy = {eff:.3f}   -> DEFAULT_PHYS_EFFICIENCY_XY")
        print(f"  coulomb_xy    = {coulomb:.2f} N -> DEFAULT_PHYS_COULOMB_XY (fitted; validate before using)")
        print(f"  (forward motion sees k = {direction_factor(1, 0):.2f} of that)")
        if not args.validate:
            print("\nnext: --validate with the current values and fitted variants, e.g.\n  --validate "
                  f"4.5,0,0.1,0,0.75,0.8 {coulomb:.2f},0,0.1,0,{eff:.2f},0.8 4.5,0,0.1,0,{eff:.2f},0.8")
    if args.validate:
        validate(args, 6, args.validate, "PHYS_FRICTION_MODEL")


if __name__ == "__main__":
    main()
