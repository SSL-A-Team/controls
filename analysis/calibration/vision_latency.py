"""Per-setup: vision latency (camera exposure -> robot), DEFAULT_VISION_MIN_LATENCY_US.

The firmware estimates the robot/vision clock skew assuming the fastest vision packet
arrives DEFAULT_VISION_MIN_LATENCY_US after SSL-Vision's t_capture. Any camera-pipeline
delay before t_capture is stamped must be in that number too, or vision is fused at the
wrong instant (the estimate lags or leads during motion).

Method: two fast theta ramps; integrate the gyro (and, independently, the wheel
encoders) to a heading, place each applied vision heading at its EKF time
(t_robot - EKF delay, which is when telemetry reports it) and find the time shift d that
best aligns the two. d = 0 when the parameter is right; otherwise
    corrected latency = current latency - d.

    python analysis/calibration/vision_latency.py --home=0,0,3.14 --heading-ref 3.14
    python analysis/calibration/vision_latency.py --home=... --heading-ref ... --no-run --out rec.npz
"""

from __future__ import annotations

import argparse
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from common import CONTROLS_ROOT, M, R, add_common_args, load, record_during, test_cmd, wheels  # noqa: E402


def firmware_default(name):
    src = (CONTROLS_ROOT / "ateam-controls/src/defaults.rs").read_text()
    m = re.search(rf"{name}: u64 = ([\d_]+)", src)
    return int(m.group(1).replace("_", "")) / 1000.0 if m else None


def fit_shift(t, rate, tc, vth, moving):
    """Best shift d (s) aligning vision heading at tc + d with the integrated rate."""
    heading = np.concatenate([[0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * np.diff(t))])
    best = None
    for d in np.arange(-0.15, 0.15, 0.001):
        h = np.interp(tc + d, t, heading)
        # vision = scale * heading + offset + drift * t  (drift absorbs gyro bias)
        A = np.stack([h, np.ones_like(h), tc - tc[0]], 1)[moving]
        sol = np.linalg.lstsq(A, vth[moving], rcond=None)[0]
        res = np.sqrt(np.mean((A @ sol - vth[moving]) ** 2))
        if best is None or res < best[1]:
            best = (d, res, sol[0])
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--min-latency-ms", type=float, default=firmware_default("DEFAULT_VISION_MIN_LATENCY_US"),
                    help="DEFAULT_VISION_MIN_LATENCY_US the robot is running (default: defaults.rs)")
    ap.add_argument("--ekf-delay-ms", type=float, default=firmware_default("DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US"),
                    help="EKF delay = DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US the robot is running")
    ap.add_argument("--w", type=float, default=1.8, help="peak rotation rate (rad/s)")
    ap.add_argument("--no-run", action="store_true", help="analyze an existing --out recording")
    args = ap.parse_args()

    if args.no_run:
        t, g = load(args.out)
    else:
        ramp = test_cmd("theta_ramp_test", args, "--w", args.w, "--ramp", 0.5, "--hold-w", 0.2)
        t, g = record_during(args, [ramp, ramp], duration=32)

    v = (g("vis_upd") > 0) & (g("vis_old") == 0)
    if v.sum() < 50:
        raise SystemExit(f"only {v.sum()} applied vision samples")
    age = g("vis_age")[v] * 1e3
    tc = t[v] - args.ekf_delay_ms * 1e-3
    vth = np.unwrap(g("vis_th")[v])
    gyro = g("gyro")
    moving = np.interp(tc, t, np.convolve(np.abs(gyro), np.ones(30) / 30, "same")) > 0.15
    print(f"running min latency {args.min_latency_ms:.0f} ms, EKF delay {args.ekf_delay_ms:.0f} ms; "
          f"{v.sum()} vision samples ({moving.sum()} while rotating)")
    print(f"applied vision age ms: p5 {np.percentile(age, 5):.0f} p50 {np.median(age):.0f} "
          f"p95 {np.percentile(age, 95):.0f} max {age.max():.0f}; rejected as too old: {int((g('vis_old') > 0).sum())}")
    if moving.sum() < 30:
        raise SystemExit("not enough rotation recorded")

    enc_rate = (R * wheels(g, "enc") @ np.linalg.pinv(M))[:, 2]
    results = []
    for name, rate in (("gyro", gyro), ("encoders", enc_rate)):
        d, res, scale = fit_shift(t, rate, tc, vth, moving)
        results.append(d)
        print(f"  {name:8s}: shift {d * 1e3:+.0f} ms (residual {np.degrees(res):.2f} deg, scale {scale:.3f})")
    d = float(np.mean(results)) * 1e3
    rec = args.min_latency_ms - d
    print(f"\nvision placed {abs(d):.0f} ms too {'late' if d < 0 else 'early'}")
    print(f"recommended DEFAULT_VISION_MIN_LATENCY_US = {rec * 1e3:,.0f}".replace(",", "_")
          + f"   ({rec:.0f} ms; |shift| <= ~3 ms means it is already right)")
    max_age = np.percentile(age, 99.5) + (rec - args.min_latency_ms)
    print(f"applied ages would then reach ~{max_age:.0f} ms: keep DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US above "
          f"that and DEFAULT_EKF_BUFF_LEN (1 ms frames) above the max age")


if __name__ == "__main__":
    main()
