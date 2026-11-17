"""Velocity-mode (joystick-like) test with the small_move_test safety monitor.

Holds HOME in BCM_GLOBAL_POSITION, then runs short BCM_LOCAL_VELOCITY pulses, each
followed by a zero-velocity hold: forward/back, sideways/back, constant rotation
rate +/-, and a noisy forward command like a joystick. Logs telemetry like
small_move_test (target columns hold [vx, vy, w] during velocity phases).

    python analysis/ros_scripts/vel_test.py --home=0,0,3.14 --heading-ref 3.14
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

from ateam_msgs.msg import RobotMotionCommand
from small_move_test import SmallMoveTest, spin_for


class VelTest(SmallMoveTest):
    def __init__(self, args):
        super().__init__(args)
        self.vel_target = None
        self.vel_log = []

    def on_telem(self, msg):
        super().on_telem(msg)
        self.vel_log.append(np.full(3, np.nan) if self.vel_target is None else self.vel_target.copy())

    def publish(self):
        if self.tripped or (self.target is None and self.vel_target is None):
            self.send_off()
            return
        if self.vel_target is None:
            return super().publish()
        a = self.args
        msg = RobotMotionCommand()
        msg.body_control_mode = RobotMotionCommand.BCM_LOCAL_VELOCITY
        msg.velocity.x, msg.velocity.y, msg.velocity.theta = map(float, self.vel_target)
        msg.limit_acc_linear = a.amax
        msg.limit_acc_angular = a.alphamax
        msg.kick_request = RobotMotionCommand.KR_DISABLE
        self.cmd_pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--home", type=str, required=True)
    ap.add_argument("--heading-ref", type=float, required=True)
    ap.add_argument("--v", type=float, default=0.3, help="linear pulse speed (m/s)")
    ap.add_argument("--t", type=float, default=0.4, help="linear pulse duration (s)")
    ap.add_argument("--w", type=float, default=1.5, help="rotation rate (rad/s)")
    ap.add_argument("--hold", type=float, default=1.5)
    ap.add_argument("--amax", type=float, default=2.0)
    ap.add_argument("--alphamax", type=float, default=12.0)
    ap.add_argument("--vmax", type=float, default=0.4)
    ap.add_argument("--wmax", type=float, default=3.0)
    ap.add_argument("--out", type=str, default="analysis/data/vel_test")
    args = ap.parse_args()
    if args.v * args.t > 0.15 or abs(args.w) * 0.9 > math.radians(80):
        raise SystemExit("pulse too large for the tether box")

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
        ok = spin_for(node, 4.0)

        def vel(v, seconds):
            node.vel_target = np.array(v, dtype=float)
            return spin_for(node, seconds)

        def stop():
            return vel((0.0, 0.0, 0.0), args.hold)

        steps = [
            ("forward", (args.v, 0, 0), args.t), ("back", (-args.v, 0, 0), args.t),
            ("left", (0, args.v, 0), args.t), ("right", (0, -args.v, 0), args.t),
            ("rot+", (0, 0, args.w), 0.9), ("rot-", (0, 0, -args.w), 0.9),
        ]
        for name, v, t in steps:
            if not ok:
                break
            node.get_logger().info(name)
            ok = vel(v, t) and stop()
        if ok:
            node.get_logger().info("noisy forward/back")
            rng = np.random.default_rng(0)
            for sign in (1, -1):
                t_end = time.monotonic() + args.t
                while ok and time.monotonic() < t_end:
                    ok = vel((sign * args.v * (1 + 0.1 * rng.standard_normal()), 0.02 * rng.standard_normal(),
                              0.1 * rng.standard_normal()), 0.02)
                ok = ok and stop()
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
        f = out / time.strftime("vel_test_%Y%m%d_%H%M%S.npz")
        np.savez(f, data=np.array(node.log, dtype=object), vel_target=np.array(node.vel_log[:len(node.log)]),
                 tripped=node.tripped or "")
        print(f"saved {f}; tripped: {node.tripped}")
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
