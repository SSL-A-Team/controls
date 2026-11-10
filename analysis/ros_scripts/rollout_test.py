"""TIGERs-style roll-out identification of linear Coulomb/viscous friction.

After TIGERs Mannheim's identifyBotModelV2 (Sumatra matlab/identification): push the
robot, cut motor torque, and fit the coast-down on

    m * dv/dt = -(B * |v| + C)

If the Coulomb+viscous fit is not >5% better than Coulomb-only, B = 0 (TIGERs rule).
Pushes are short and alternate direction so the robot stays near its start pose
(tethered robot). The wheel velocity loop is disabled for the test so BCM_OFF means
zero current (coasting); it is restored afterwards.

    python analysis/ros_scripts/rollout_test.py -r 2 --axis y --trials 6
"""

from __future__ import annotations

import argparse
import math
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor

from ateam_msgs.msg import RobotMotionCommand
from small_move_test import SmallMoveTest

MASS_KG = 2.7
IZ_KGM2 = 0.008


def set_wheel_vel_loop(enabled: bool) -> None:
    subprocess.run(["ros2", "param", "set", "/radio_bridge", "controls_enabled.wheel_vel",
                    "true" if enabled else "false"], check=True, capture_output=True, timeout=10)


class RolloutTest(SmallMoveTest):
    def __init__(self, args):
        super().__init__(args)
        self.accel_cmd = None  # (ax, ay, aw) local, or None for BCM_OFF
        self.phase = "rest"

    def publish(self):
        if self.tripped or self.accel_cmd is None:
            self.send_off()
            return
        msg = RobotMotionCommand()
        msg.body_control_mode = RobotMotionCommand.BCM_LOCAL_ACCEL_OPEN_LOOP
        msg.acceleration.x, msg.acceleration.y, msg.acceleration.theta = map(float, self.accel_cmd)
        msg.kick_request = RobotMotionCommand.KR_DISABLE
        self.cmd_pub.publish(msg)

    def on_telem(self, msg):
        super().on_telem(msg)
        if self.log:
            self.log[-1] = np.append(self.log[-1], 1.0 if self.phase == "coast" else 0.0)


def fit_coast(t, speed, inertia):
    """TIGERs fricXY/fricW: fit inertia*dv/dt = -(B v + C) on the coast-down."""
    m = speed > 0.05
    t, speed = t[m], speed[m]
    if len(t) < 6:
        return None
    acc = np.diff(speed) / np.diff(t)
    v = speed[1:]
    b = acc * inertia
    A = np.column_stack([v, np.ones_like(v)])
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    visc, coul = -x[0], -x[1]
    coul_only = -np.mean(b)
    err_vc = np.sqrt(np.sum((A @ x - b) ** 2)) / len(b)
    err_c = np.sqrt(np.sum((-coul_only - b) ** 2)) / len(b)
    return dict(viscous=visc, coulomb=coul, coulomb_only=coul_only, err_vc=err_vc, err_c=err_c, n=len(b))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--axis", choices=["x", "y", "w"], default="y")
    ap.add_argument("--trials", type=int, default=6)
    ap.add_argument("--accel", type=float, default=2.0, help="push accel (m/s^2 or rad/s^2)")
    ap.add_argument("--push", type=float, default=0.15, help="push duration (s)")
    ap.add_argument("--coast", type=float, default=1.0)
    ap.add_argument("--rest", type=float, default=1.0)
    ap.add_argument("--out", type=str, default="analysis/data/rollout")
    # unused by this test but required by SmallMoveTest
    ap.add_argument("--vmax", type=float, default=0.3)
    ap.add_argument("--amax", type=float, default=1.0)
    ap.add_argument("--wmax", type=float, default=2.0)
    ap.add_argument("--alphamax", type=float, default=6.0)
    args = ap.parse_args()
    if args.push * args.accel > (2.0 if args.axis == "w" else 1.0):
        raise SystemExit("push too strong for the tether box")

    rclpy.init()
    node = RolloutTest(args)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    comp = {"x": 0, "y": 1, "w": 2}[args.axis]
    trials = []
    try:
        end = time.monotonic() + 10
        while (node.pose is None or node.last_vision is None) and time.monotonic() < end:
            time.sleep(0.01)
        if node.pose is None or node.last_vision is None:
            node.get_logger().error("no telemetry/vision")
            return 1
        set_wheel_vel_loop(False)
        time.sleep(0.5)
        node.start = node.pose.copy()
        node.target = node.start  # arms the watchdog's stale checks
        for i in range(args.trials):
            if node.tripped:
                break
            sign = 1.0 if i % 2 == 0 else -1.0
            # push from the side we're on back toward/through the start
            if args.axis != "w":
                d_local = np.array([[math.cos(-node.pose[2]), -math.sin(-node.pose[2])],
                                    [math.sin(-node.pose[2]), math.cos(-node.pose[2])]]) @ (node.pose[:2] - node.start[:2])
                if abs(d_local[comp]) > 0.02:
                    sign = -math.copysign(1.0, d_local[comp])
            else:
                dw = math.remainder(node.pose[2] - node.start[2], 2 * math.pi)
                if abs(dw) > 0.1:
                    sign = -math.copysign(1.0, dw)
            cmd = [0.0, 0.0, 0.0]
            cmd[comp] = sign * args.accel
            n0 = len(node.log)
            node.phase = "push"
            node.accel_cmd = cmd
            time.sleep(args.push)
            node.accel_cmd = None
            node.phase = "coast"
            time.sleep(args.coast)
            node.phase = "rest"
            seg = np.array([r for r in node.log[n0:] if r[-1] == 1.0])
            res = None
            if len(seg):
                t = seg[:, 0]
                vel = seg[:, 4:7]
                speed = np.abs(vel[:, 2]) if args.axis == "w" else np.hypot(vel[:, 0], vel[:, 1])
                res = fit_coast(t, speed, IZ_KGM2 if args.axis == "w" else MASS_KG)
                node.get_logger().info(f"trial {i + 1} dir {sign:+.0f}: peak {speed.max():.3f}, fit {res}")
            trials.append(res)
            time.sleep(args.rest)
    finally:
        node.accel_cmd = None
        node.target = None
        for _ in range(20):
            node.send_off()
            time.sleep(0.01)
        try:
            set_wheel_vel_loop(True)
        except Exception as e:  # noqa: BLE001
            node.get_logger().error(f"could not restore wheel vel loop: {e}")
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        f = out / time.strftime(f"rollout_{args.axis}_%Y%m%d_%H%M%S.npz")
        np.savez(f, data=np.array(node.log, dtype=object), tripped=str(node.tripped))
        ok = [r for r in trials if r]
        if ok:
            errs = np.mean([r["err_vc"] for r in ok]) / np.mean([r["err_c"] for r in ok])
            if 0.95 < errs < 1.05:
                print(f"RESULT {args.axis}: viscous-free model; coulomb = {np.mean([r['coulomb_only'] for r in ok]):.3f}")
            else:
                print(f"RESULT {args.axis}: coulomb = {np.mean([r['coulomb'] for r in ok]):.3f}, viscous = {np.mean([r['viscous'] for r in ok]):.3f}")
        node.get_logger().info(f"saved {f}; tripped={node.tripped}")
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.tripped is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
