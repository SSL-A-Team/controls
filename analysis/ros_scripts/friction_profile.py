"""Static-friction (breakaway) profiling with open-loop body force.

With the wheel velocity loop disabled and the friction model zeroed, a
BCM_LOCAL_ACCEL_OPEN_LOOP command maps directly to feed-forward wheel currents. Each
trial ramps the commanded local acceleration along one direction (linear angle in the
robot frame and/or rotation) until the wheels start turning (breakaway), then cuts to
BCM_OFF so the robot coasts to a stop after moving a few cm. Directions alternate sign
to keep the robot near its start pose. Logs per-wheel current setpoints at breakaway.

    python analysis/ros_scripts/friction_profile.py -r 2 --dirs 0,45,90,135,w,90w --reps 2
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

MASS_KG = 2.7
IZ_KGM2 = 0.008
# Equivalent rotational "accel" per unit linear accel for combined directions: rotation
# at the same per-wheel current as linear motion needs roughly m*l/Iz times the accel.
W_PER_LIN = MASS_KG * 0.0814 / IZ_KGM2
BREAKAWAY_WHEEL_RADS = 2.0
BREAKAWAY_SAMPLES = 3


def parse_dir(token: str):
    """'90' -> linear at 90 deg; 'w' -> pure rotation; '90w' -> both (equal weight)."""
    lin, rot = None, 0.0
    if token.endswith("w"):
        rot = 1.0
        token = token[:-1]
    if token:
        lin = math.radians(float(token))
    return lin, rot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--dirs", type=str, default="0,45,90,135,w,90w")
    ap.add_argument("--reps", type=int, default=2, help="trials per direction (alternating sign)")
    ap.add_argument("--ramp", type=float, default=2.0, help="linear accel ramp rate (m/s^2 per s)")
    ap.add_argument("--max-accel", type=float, default=4.0, help="linear accel at which to give up")
    ap.add_argument("--coast", type=float, default=0.8)
    ap.add_argument("--rest", type=float, default=0.8)
    ap.add_argument("--out", type=str, default="../data/friction_profile")
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
        for token in args.dirs.split(","):
            lin, rot = parse_dir(token)
            for rep in range(args.reps):
                if node.tripped:
                    break
                sign = 1.0 if rep % 2 == 0 else -1.0
                # steer back toward the start if we drifted
                if lin is not None:
                    c, s = math.cos(-node.pose[2]), math.sin(-node.pose[2])
                    d = node.pose[:2] - node.start[:2]
                    d_local = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
                    along = d_local @ np.array([math.cos(lin), math.sin(lin)])
                    if abs(along) > 0.02:
                        sign = -math.copysign(1.0, along)
                elif rot:
                    dw = math.remainder(node.pose[2] - node.start[2], 2 * math.pi)
                    if abs(dw) > 0.15:
                        sign = -math.copysign(1.0, dw)
                n0 = len(node.log)
                node.phase = "push"
                t0 = time.monotonic()
                broke = None
                moving = 0
                while not node.tripped:
                    a = args.ramp * (time.monotonic() - t0)
                    if a > args.max_accel:
                        break
                    ax = ay = aw = 0.0
                    if lin is not None:
                        ax, ay = sign * a * math.cos(lin), sign * a * math.sin(lin)
                    if rot:
                        aw = sign * a * W_PER_LIN
                    node.accel_cmd = (ax, ay, aw)
                    time.sleep(0.005)
                    if node.latest_wheel_vel is not None and \
                            max(abs(w) for w in node.latest_wheel_vel) > BREAKAWAY_WHEEL_RADS:
                        moving += 1
                        if moving >= BREAKAWAY_SAMPLES:
                            broke = a
                            break
                    else:
                        moving = 0
                node.accel_cmd = None
                node.phase = "coast"
                seg = np.array([np.asarray(r, dtype=float) for r in node.log[n0:]])
                cur_at = seg[-1, 8:12] if len(seg) else np.zeros(4)
                wv_at = seg[-1, 12:16] if len(seg) else np.zeros(4)
                time.sleep(args.coast)
                node.phase = "rest"
                rec = dict(dir=token, sign=sign, breakaway_accel=broke, currents=np.round(cur_at, 2).tolist(),
                           wheel_vel=np.round(wv_at, 1).tolist())
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
        except Exception as e:  # noqa: BLE001
            node.get_logger().error(f"could not restore wheel vel loop: {e}")
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        f = out / time.strftime("friction_profile_%Y%m%d_%H%M%S.npz")
        np.savez(f, data=np.array(node.log, dtype=object), results=np.array(results, dtype=object),
                 tripped=str(node.tripped))
        node.get_logger().info(f"saved {f}; tripped={node.tripped}")
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.tripped is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
