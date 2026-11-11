"""Joystick position-mode emulation with the small_move_test safety monitor.

Mimics ateam_joystick_control position mode: anchor = HOME, target = anchor +
linear_range * stick (x/y in the anchor heading frame) and theta = anchor theta +
angular_range * stick, with a smoothed random-walk "stick" published at 100 Hz.

    python analysis/ros_scripts/joy_pos_test.py --home=0,0,3.14 --heading-ref 3.14
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

from small_move_test import SmallMoveTest, spin_for


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--home", type=str, required=True)
    ap.add_argument("--heading-ref", type=float, required=True)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--stick-xy", type=float, default=0.4, help="max |stick| for x/y")
    ap.add_argument("--stick-th", type=float, default=0.5, help="max |stick| for theta")
    ap.add_argument("--stick-tau", type=float, default=0.3, help="stick smoothing time constant (s)")
    ap.add_argument("--linear-range", type=float, default=0.25)
    ap.add_argument("--angular-range", type=float, default=math.pi / 2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--creep", type=float, default=0.0,
                    help="barely-moving stick: drift each axis at this rate (stick units/s), "
                         "reversing every 1-3 s, so the target changes slightly every packet")
    ap.add_argument("--jitter", type=float, default=0.002, help="per-packet stick jitter (creep mode)")
    # trajectory limits sent with the command; 0 = firmware defaults (like the joystick)
    ap.add_argument("--vmax", type=float, default=0.0)
    ap.add_argument("--amax", type=float, default=0.0)
    ap.add_argument("--wmax", type=float, default=0.0)
    ap.add_argument("--alphamax", type=float, default=0.0)
    ap.add_argument("--out", type=str, default="analysis/data/joy_pos_test")
    args = ap.parse_args()
    if args.stick_xy * args.linear_range > 0.15 or args.stick_th * args.angular_range > math.radians(80):
        raise SystemExit("stick range too large for the tether box")

    rclpy.init()
    node = SmallMoveTest(args)
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
        ok = spin_for(node, 4.0)

        rng = np.random.default_rng(args.seed)
        lim = np.array([args.stick_xy, args.stick_xy, args.stick_th])
        goal = np.zeros(3)
        stick = np.zeros(3)
        dt = 0.01
        c, s = math.cos(home[2]), math.sin(home[2])
        t_start = time.monotonic()
        next_goal = 0.0
        drift = np.zeros(3)
        while ok and time.monotonic() - t_start < args.duration:
            t = time.monotonic() - t_start
            if args.creep > 0:
                if t >= next_goal:
                    drift = args.creep * rng.choice([-1.0, 1.0], 3)
                    next_goal = t + rng.uniform(1.0, 3.0)
                stick = np.clip(stick + drift * dt + args.jitter * rng.standard_normal(3), -lim, lim)
                drift = np.where(np.abs(stick) >= lim, -np.sign(stick) * args.creep, drift)
            else:
                if t >= next_goal:
                    goal = lim * rng.uniform(-1, 1, 3) * (rng.uniform() < 0.8)
                    next_goal = t + rng.uniform(0.3, 1.5)
                stick += (goal - stick) * dt / args.stick_tau
            dx, dy = args.linear_range * stick[0], args.linear_range * stick[1]
            node.target = np.array([home[0] + c * dx - s * dy, home[1] + s * dx + c * dy,
                                    math.remainder(home[2] + args.angular_range * stick[2], 2 * math.pi)])
            ok = spin_for(node, dt)
        node.target = home
        ok = ok and spin_for(node, 3.0)
    finally:
        node.target = None
        for _ in range(20):
            node.send_off()
            time.sleep(0.01)
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        f = out / time.strftime("joy_pos_test_%Y%m%d_%H%M%S.npz")
        np.savez(f, data=np.array(node.log, dtype=object), tripped=node.tripped or "")
        print(f"saved {f}; tripped: {node.tripped}")
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
