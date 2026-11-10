"""Kinetic friction profiling at constant velocity (TIGERs plateau method).

Drives at a constant local velocity (BCM_LOCAL_VELOCITY, wheel velocity loop on) along
one direction for a short plateau and averages the wheel current setpoints there. At
constant velocity the applied force balances friction, so the generalized friction
force (Fx, Fy, Tz) vs. velocity can be computed from the currents. Directions alternate
sign to keep the robot near its start pose.

    python analysis/ros_scripts/plateau_profile.py -r 2 --dirs 0,90,45,w,90w --speeds 0.1,0.2
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
from small_move_test import SmallMoveTest

A, B, L, R, KT, EFF = math.pi / 6, math.pi / 4, 0.0814, 0.03, 0.0335, 0.8
W = np.array([[-math.cos(A), -math.cos(B), math.cos(B), math.cos(A)],
              [math.sin(A), -math.sin(B), -math.sin(B), math.sin(A)],
              [L, L, L, L]])
W_PER_LIN = 1.0 / L  # rotation (rad/s) per linear speed (m/s): same wheel speed


class PlateauTest(SmallMoveTest):
    def __init__(self, args):
        super().__init__(args)
        self.vel_cmd = None
        self.phase = "rest"

    def publish(self):
        if self.tripped or self.vel_cmd is None:
            self.send_off()
            return
        a = self.args
        msg = RobotMotionCommand()
        msg.body_control_mode = RobotMotionCommand.BCM_LOCAL_VELOCITY
        msg.velocity.x, msg.velocity.y, msg.velocity.theta = map(float, self.vel_cmd)
        msg.limit_acc_linear = a.amax
        msg.limit_acc_angular = a.amax * W_PER_LIN
        msg.kick_request = RobotMotionCommand.KR_DISABLE
        self.cmd_pub.publish(msg)

    def on_telem(self, msg):
        super().on_telem(msg)
        if self.log:
            vis = np.array(msg.body_control_telemetry.vision_pose[:3], dtype=float)
            self.log[-1] = np.concatenate([self.log[-1], vis, [1.0 if self.phase == "plateau" else 0.0]])


def parse_dir(token):
    lin, rot = None, 0.0
    if token.endswith("w"):
        rot, token = 1.0, token[:-1]
    if token:
        lin = math.radians(float(token))
    return lin, rot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--dirs", type=str, default="0,90,45,w,90w")
    ap.add_argument("--speeds", type=str, default="0.1,0.2", help="wheel-equivalent linear speeds (m/s)")
    ap.add_argument("--plateau", type=float, default=0.4)
    ap.add_argument("--amax", type=float, default=1.0)
    ap.add_argument("--rest", type=float, default=0.8)
    ap.add_argument("--out", type=str, default="../data/plateau_profile")
    ap.add_argument("--vmax", type=float, default=0.3)
    ap.add_argument("--wmax", type=float, default=2.0)
    ap.add_argument("--alphamax", type=float, default=6.0)
    args = ap.parse_args()

    rclpy.init()
    node = PlateauTest(args)
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
        node.start = node.pose.copy()
        node.target = node.start
        for token in args.dirs.split(","):
            lin, rot = parse_dir(token)
            for speed in [float(s) for s in args.speeds.split(",")]:
                for sign in (1.0, -1.0):
                    if node.tripped:
                        break
                    vx = vy = vw = 0.0
                    if lin is not None:
                        vx, vy = sign * speed * math.cos(lin), sign * speed * math.sin(lin)
                    if rot:
                        vw = sign * speed * W_PER_LIN
                    if lin is not None and rot:
                        # split the wheel-speed budget between translation and rotation
                        vx, vy, vw = vx * 0.5, vy * 0.5, vw * 0.5
                    t_acc = speed / args.amax
                    node.vel_cmd = (vx, vy, vw)
                    time.sleep(t_acc + 0.15)
                    n0 = len(node.log)
                    node.phase = "plateau"
                    time.sleep(args.plateau)
                    node.phase = "rest"
                    seg = np.array([np.asarray(r, dtype=float) for r in node.log[n0:] if r[-1] == 1.0])
                    node.vel_cmd = None
                    if len(seg) > 5:
                        cur = seg[:, 8:12].mean(axis=0)
                        vel = seg[:, 4:7].mean(axis=0)
                        F = W @ (cur * KT / R * EFF)
                        # slip check: vision velocity (local frame) vs encoder-implied velocity
                        vrows = seg[np.any(seg[:, -4:-1] != 0.0, axis=1)]
                        t, vis = vrows[:, 0], vrows[:, -4:-1]
                        th = np.unwrap(vis[:, 2])
                        gvx, gvy, gvw = (np.polyfit(t, c, 1)[0] for c in (vis[:, 0], vis[:, 1], th))
                        c, s_ = math.cos(th.mean()), math.sin(th.mean())
                        vis_vel = np.array([c * gvx + s_ * gvy, -s_ * gvx + c * gvy, gvw])
                        enc_vel = R * np.linalg.pinv(W.T) @ seg[:, 12:16].mean(axis=0)
                        rec = dict(dir=token, sign=sign, speed=speed, cmd=(round(vx, 3), round(vy, 3), round(vw, 3)),
                                   vel=np.round(vel, 3).tolist(), I=np.round(cur, 2).tolist(),
                                   F=np.round(F, 3).tolist(), I_std=np.round(seg[:, 8:12].std(axis=0), 2).tolist(),
                                   vis_vel=np.round(vis_vel, 3).tolist(), enc_vel=np.round(enc_vel, 3).tolist())
                        results.append(rec)
                        node.get_logger().info(str(rec))
                    time.sleep(args.rest)
    finally:
        node.vel_cmd = None
        node.target = None
        for _ in range(20):
            node.send_off()
            time.sleep(0.01)
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        f = out / time.strftime("plateau_profile_%Y%m%d_%H%M%S.npz")
        np.savez(f, data=np.array(node.log, dtype=object), results=np.array(results, dtype=object),
                 tripped=str(node.tripped))
        node.get_logger().info(f"saved {f}; tripped={node.tripped}")
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.tripped is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
