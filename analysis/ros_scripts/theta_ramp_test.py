"""Slow joystick-like ramp of the commanded local angular velocity (BCM_LOCAL_VELOCITY).

Ramps omega 0 -> +W over --ramp seconds, holds --hold-w seconds, ramps back to 0,
pauses, then the same toward -W (so the robot returns near its start heading).
Uses the small_move_test safety monitor; logs like vel_test.py.

    python analysis/ros_scripts/theta_ramp_test.py --home=0,0,3.14 --heading-ref 3.14 --w 1.0 --ramp 1.0
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor

from vel_test import VelTest
from small_move_test import spin_for


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--home", type=str, required=True)
    ap.add_argument("--heading-ref", type=float, required=True)
    ap.add_argument("--w", type=float, default=1.0, help="peak angular velocity (rad/s)")
    ap.add_argument("--ramp", type=float, default=1.0, help="ramp duration (s)")
    ap.add_argument("--hold-w", type=float, default=0.3, help="time at peak (s)")
    ap.add_argument("--vx", type=float, default=0.0, help="constant local vx during the ramps (m/s)")
    ap.add_argument("--amax", type=float, default=2.0)
    ap.add_argument("--alphamax", type=float, default=10.0)
    ap.add_argument("--vmax", type=float, default=0.4)
    ap.add_argument("--wmax", type=float, default=3.0)
    ap.add_argument("--hold", type=float, default=1.0)
    ap.add_argument("--out", type=str, default="analysis/data/theta_ramp_test")
    args = ap.parse_args()
    angle = abs(args.w) * (0.75 * args.ramp + args.hold_w)
    if angle > math.radians(100) or abs(args.w) > 3.0 or abs(args.vx) * (2 * args.ramp + args.hold_w) > 0.1:
        raise SystemExit(f"profile too large for the tether box ({math.degrees(angle):.0f} deg)")

    rclpy.init()
    node = VelTest(args)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        t_end = time.monotonic() + 10.0
        while (node.pose is None or node.last_vision is None) and time.monotonic() < t_end:
            time.sleep(0.01)
        if node.pose is None or node.last_vision is None:
            node.get_logger().error("no telemetry/vision")
            return 1
        spin_for(node, 0.5)
        home = np.array([float(x) for x in args.home.split(",")])
        node.start = home.copy()
        node.heading_ref = args.heading_ref
        node.target = home
        ok = spin_for(node, 3.0)

        def ramp(w0, w1, seconds, vx):
            t0 = time.monotonic()
            nonlocal ok
            while ok and (t := time.monotonic() - t0) < seconds:
                f = t / seconds
                node.vel_target = np.array([vx, 0.0, w0 + (w1 - w0) * f])
                ok = spin_for(node, 0.01)

        for sign in (1.0, -1.0):
            if not ok:
                break
            w = sign * args.w
            node.get_logger().info(f"ramp to {w:+.2f} rad/s")
            ramp(0.0, w, args.ramp, sign * args.vx)
            ramp(w, w, args.hold_w, sign * args.vx)
            ramp(w, 0.0, args.ramp / 2, sign * args.vx)
            node.vel_target = np.zeros(3)
            ok = ok and spin_for(node, args.hold)
        node.vel_target = None
        node.target = home
        ok = ok and spin_for(node, 3.0)
    finally:
        node.target = None
        node.vel_target = None
        for _ in range(20):
            node.send_off()
            time.sleep(0.01)
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        f = out / time.strftime("theta_ramp_%Y%m%d_%H%M%S.npz")
        np.savez(f, data=np.array(node.log, dtype=object), vel_target=np.array(node.vel_log[:len(node.log)]),
                 tripped=node.tripped or "")
        print(f"saved {f}; tripped: {node.tripped}")
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
