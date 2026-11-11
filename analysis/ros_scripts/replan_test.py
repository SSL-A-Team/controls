"""Position control under constant replanning toward a distant target.

BCM_GLOBAL_POSITION with a "carrot" target CARROT m away from home along one axis
(beyond the tether box), re-noised every packet so the firmware replans every
command, like a path planner streaming waypoints. When the robot passes +/-FLIP
from home the carrot flips to the other side, giving repeated
cruise -> brake -> reverse cycles. Uses the small_move_test safety monitor.

    python analysis/ros_scripts/replan_test.py --home=0,0,3.14 --heading-ref 3.14 --axis x
"""

from __future__ import annotations

import argparse
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
    ap.add_argument("--axis", choices=("x", "y"), default="x", help="global axis to run along")
    ap.add_argument("--carrot", type=float, default=0.6, help="target distance from home (m)")
    ap.add_argument("--flip", type=float, default=0.08, help="flip the carrot past this offset (m)")
    ap.add_argument("--noise-xy", type=float, default=0.01, help="per-packet target noise std (m)")
    ap.add_argument("--noise-th", type=float, default=1.0, help="per-packet heading noise std (deg)")
    ap.add_argument("--reversals", type=int, default=8)
    ap.add_argument("--pre-hold", type=float, default=4.0, help="hold at home before moving (s)")
    ap.add_argument("--vmax", type=float, default=0.3)
    ap.add_argument("--amax", type=float, default=2.0)
    ap.add_argument("--wmax", type=float, default=3.0)
    ap.add_argument("--alphamax", type=float, default=12.0)
    ap.add_argument("--out", type=str, default="analysis/data/replan_test")
    args = ap.parse_args()
    if args.flip > 0.12:
        raise SystemExit("flip offset too large for the tether box")

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
        ok = spin_for(node, args.pre_hold)

        k = 0 if args.axis == "x" else 1
        rng = np.random.default_rng(0)
        direction, reversals = 1.0, 0
        node.get_logger().info(f"carrot {args.carrot} m along {args.axis}, flip at +/-{args.flip} m")
        while ok and reversals < args.reversals:
            off = node.pose[k] - home[k]
            if direction * off > args.flip:
                direction = -direction
                reversals += 1
            tgt = home.copy()
            tgt[k] += direction * args.carrot
            tgt[:2] += args.noise_xy * rng.standard_normal(2)
            tgt[2] += np.radians(args.noise_th) * rng.standard_normal()
            node.target = tgt
            ok = spin_for(node, 0.01)
        node.target = home
        ok = ok and spin_for(node, 3.0)
    finally:
        node.target = None
        for _ in range(20):
            node.send_off()
            time.sleep(0.01)
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        f = out / time.strftime("replan_test_%Y%m%d_%H%M%S.npz")
        np.savez(f, data=np.array(node.log, dtype=object), tripped=node.tripped or "")
        print(f"saved {f}; tripped: {node.tripped}")
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
