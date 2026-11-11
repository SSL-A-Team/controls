"""Open-loop acceleration steps for output-efficiency identification (TIGERs-style).

With the wheel velocity loop disabled, a BCM_LOCAL_ACCEL_OPEN_LOOP command maps to
feed-forward wheel currents: (a_cmd + friction/m) / efficiency. Set the firmware
friction model to zero and efficiency to 1.0; with the friction measured separately
(as motor-side force F_fric), efficiency = m * a_meas / (F_motor - F_fric), where
F_motor comes from the current setpoints (which track measured current 1:1) and
a_meas from the encoder-derived body velocity. Each step pushes for --push seconds then cuts to
BCM_OFF (coast). Steps alternate sign to stay near the start pose.

    python analysis/ros_scripts/accel_steps.py -r 2 --steps 0:1.0,90:0.4,w:8
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

from rollout_test import RolloutTest, set_wheel_vel_loop

A, B, L, R, KT = math.pi / 6, math.pi / 4, 0.0814, 0.03, 0.0335
M = np.array([[-math.cos(A), -math.cos(B), math.cos(B), math.cos(A)],
              [math.sin(A), -math.sin(B), -math.sin(B), math.sin(A)],
              [L, L, L, L]])
WHEEL_TO_LOCAL = R * np.linalg.pinv(M.T)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--steps", type=str, default="0:1.0,90:0.4,w:8",
                    help="dir:accel list; dir in degrees (local) or 'w'; accel m/s^2 or rad/s^2")
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--push", type=float, default=0.35)
    ap.add_argument("--coast", type=float, default=1.0)
    ap.add_argument("--rest", type=float, default=0.6)
    ap.add_argument("--out", type=str, default="../data/accel_steps")
    ap.add_argument("--vmax", type=float, default=0.3)
    ap.add_argument("--amax", type=float, default=1.0)
    ap.add_argument("--wmax", type=float, default=2.0)
    ap.add_argument("--alphamax", type=float, default=6.0)
    args = ap.parse_args()

    rclpy.init()
    node = RolloutTest(args)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    results = []
    try:
        end = time.monotonic() + 10
        while (node.pose is None or node.last_vision is None) and time.monotonic() < end:
            time.sleep(0.01)
        if node.pose is None:
            node.get_logger().error("no telemetry")
            return 1
        set_wheel_vel_loop(False)
        time.sleep(0.5)
        node.start = node.pose.copy()
        node.target = node.start
        for spec in args.steps.split(","):
            d, acc = spec.split(":")
            acc = float(acc)
            for rep in range(args.reps):
                if node.tripped:
                    break
                sign = 1.0 if rep % 2 == 0 else -1.0
                if d == "w":
                    cmd = (0.0, 0.0, sign * acc)
                else:
                    ang = math.radians(float(d))
                    cmd = (sign * acc * math.cos(ang), sign * acc * math.sin(ang), 0.0)
                n0 = len(node.log)
                node.accel_cmd = cmd
                time.sleep(args.push)
                node.accel_cmd = None
                n1 = len(node.log)
                time.sleep(args.coast)
                seg = np.array([np.asarray(r, dtype=float)[:16] for r in node.log[n0:n1]])
                if len(seg) > 8:
                    t = seg[:, 0]
                    vel = (WHEEL_TO_LOCAL @ seg[:, 12:16].T).T  # encoder body velocity (local)
                    # fit the acceleration over the middle of the push (skip current rise / slope limit)
                    m = (t > t[0] + 0.08) & (t < t[-1] - 0.02)
                    k = 2 if d == "w" else (0 if abs(cmd[0]) >= abs(cmd[1]) else 1)
                    a_meas = np.polyfit(t[m], vel[m, k], 1)[0] if m.sum() > 4 else float("nan")
                    a_cmd = cmd[k]
                    i_mean = seg[m, 8:12].mean(axis=0)
                    f_motor = M @ (i_mean * KT / R)  # body force/torque produced at the motors (local)
                    rec = dict(dir=d, sign=sign, a_cmd=round(a_cmd, 3), a_meas=round(float(a_meas), 3),
                               ratio=round(float(a_meas / a_cmd), 3), f_motor=np.round(f_motor, 3).tolist(),
                               I=np.round(i_mean, 2).tolist())
                    results.append(rec)
                    node.get_logger().info(str(rec))
                time.sleep(args.rest)
    finally:
        node.accel_cmd = None
        node.target = None
        for _ in range(20):
            node.send_off()
            time.sleep(0.01)
        try:
            set_wheel_vel_loop(True)
        finally:
            out = Path(args.out)
            out.mkdir(parents=True, exist_ok=True)
            f = out / time.strftime("accel_steps_%Y%m%d_%H%M%S.npz")
            np.savez(f, data=np.array([np.asarray(r, float)[:16] for r in node.log]),
                     results=np.array(results, dtype=object), tripped=str(node.tripped))
            node.get_logger().info(f"saved {f}; tripped={node.tripped}")
            ex.shutdown()
            node.destroy_node()
            rclpy.shutdown()
    return 0 if node.tripped is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
