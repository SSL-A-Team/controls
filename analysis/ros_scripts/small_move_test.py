"""Small precise move test with a fast safety monitor.

Holds the robot at its starting pose, then moves back and forth between HOME
and HOME + (dx, dy, dtheta) in BCM_GLOBAL_POSITION with conservative trajectory
limits, logging telemetry to an .npz.

Safety: on every ExtendedTelemetry message (and on a 500 Hz watchdog timer) the
monitor checks the pose box, heading, speeds, wheel current setpoints, motor
errors, vision freshness and telemetry staleness. On any violation it latches
and publishes BCM_OFF at the timer rate until the script exits.

    ros2 launch ateam_bringup bringup_physical_core.launch.py controls_enabled_wheel_vel:=True
    python analysis/ros_scripts/small_move_test.py -r 2 --dx 0.05 --dtheta-deg 15 --cycles 3
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
from rclpy.node import Node

from ateam_msgs.msg import RobotMotionCommand
from ateam_radio_msgs.msg import ExtendedTelemetry

MOTORS = ("front_left_motor", "back_left_motor", "back_right_motor", "front_right_motor")


class SafetyLimits:
    """Tether box relative to the start pose, plus the absolute field box below."""
    xy_abs = 0.25
    heading_abs = math.radians(95.0)
    field_x = (0.0, 0.25)  # m, absolute
    field_y_abs = 0.25     # m, absolute
    speed = 0.6            # m/s, KF estimate
    ang_speed = 4.0        # rad/s, gyro
    wheel_current = 4.2    # A, motor current setpoint (4 A boost cap)
    wheel_speed = 60.0     # rad/s, encoder
    telem_stale_s = 0.4    # radio telemetry gaps up to ~0.35 s observed; firmware stops at 0.5 s without commands
    vision_stale_s = 0.4


class SmallMoveTest(Node):
    def __init__(self, args):
        super().__init__("small_move_test")
        self.args = args
        self.lim = SafetyLimits()
        self.cmd_pub = self.create_publisher(
            RobotMotionCommand, f"/robot_motion_commands/robot{args.robot}", 10)
        self.create_subscription(
            ExtendedTelemetry, f"/robot_feedback/extended/robot{args.robot}", self.on_telem, 50)
        self.tripped: str | None = None
        self.last_telem = None
        self.last_vision = None
        self.pose = None
        self.start = None
        self.latest_wheel_vel = None
        self.target = None
        self.t0 = time.monotonic()
        self.log = []
        self._tick = 0
        self.create_timer(0.005, self.on_timer)

    # ---------------- safety ----------------
    def trip(self, reason: str):
        if self.tripped is None:
            self.tripped = reason
            self.get_logger().error(f"SAFETY TRIP: {reason} -> BCM_OFF")
        self.send_off()

    def send_off(self):
        msg = RobotMotionCommand()
        msg.body_control_mode = RobotMotionCommand.BCM_OFF
        msg.kick_request = RobotMotionCommand.KR_DISABLE
        self.cmd_pub.publish(msg)

    def watchdog(self):
        now = time.monotonic()
        if self.tripped:
            self.send_off()
            return
        if self.target is not None:
            if self.last_telem is None or now - self.last_telem > self.lim.telem_stale_s:
                self.trip("telemetry stale")
            elif self.last_vision is None or now - self.last_vision > self.lim.vision_stale_s:
                self.trip("vision stale")

    def on_telem(self, msg):
        now = time.monotonic()
        self.last_telem = now
        b = msg.body_control_telemetry
        if b.vision_update:
            self.last_vision = now
        pos = np.array(b.kf_body_pos_estimate[:3], dtype=float)
        vel = np.array(b.kf_body_vel_estimate[:3], dtype=float)
        gyro = float(b.imu_gyro[2])
        cur = np.array([getattr(msg, m).current_telemetry.current_setpoint_ma for m in MOTORS]) / 1000.0
        wv = np.array([getattr(msg, m).velocity_telemetry.wheel_vel_rads for m in MOTORS])
        err = any(getattr(msg, m).master_error for m in MOTORS)
        self.pose = pos
        self.latest_wheel_vel = wv
        traj = np.array(b.body_traj_pos[:3], dtype=float)
        self.log.append(np.concatenate([[now - self.t0], pos, vel, [gyro], cur, wv, traj,
                                        np.array(b.body_vel_u[:3]), np.array(b.body_accel_u_fric_comp[:3]),
                                        self.target if self.target is not None else np.full(3, np.nan),
                                        [float(b.vision_update), float(b.body_control_mode)],
                                        [getattr(msg, m).velocity_telemetry.vel_setpoint_rads for m in MOTORS],
                                        [getattr(msg, m).motion_control_type for m in MOTORS],
                                        [((msg.timestamp_us_hi << 32) | msg.timestamp_us_lo) * 1e-6]]))
        if self.target is None or self.tripped:
            return
        L = self.lim
        d = pos - self.start
        if (not L.field_x[0] < pos[0] < L.field_x[1] or abs(pos[1]) >= L.field_y_abs
                or abs(d[0]) > L.xy_abs or abs(d[1]) > L.xy_abs):
            self.trip(f"pose out of box {pos[:2]} (start {self.start[:2]})")
        elif abs(dh := math.remainder(pos[2] - self.heading_ref, 2 * math.pi)) > L.heading_abs:
            self.trip(f"heading {math.degrees(dh):.1f} deg from reference")
        elif math.hypot(vel[0], vel[1]) > L.speed:
            self.trip(f"speed {math.hypot(vel[0], vel[1]):.2f} m/s")
        elif abs(gyro) > L.ang_speed:
            self.trip(f"gyro {gyro:.2f} rad/s")
        elif np.max(np.abs(cur)) > L.wheel_current:
            self.trip(f"wheel current {cur}")
        elif np.max(np.abs(wv)) > L.wheel_speed:
            self.trip(f"wheel speed {wv}")
        elif err:
            self.trip("motor master error")

    def on_timer(self):
        self.watchdog()
        self._tick += 1
        if self._tick % 2 == 0:
            self.publish()

    # ---------------- command ----------------
    def publish(self):
        if self.tripped or self.target is None:
            self.send_off()
            return
        a = self.args
        msg = RobotMotionCommand()
        msg.body_control_mode = RobotMotionCommand.BCM_GLOBAL_POSITION
        msg.pose.x, msg.pose.y, msg.pose.theta = map(float, self.target)
        msg.limit_vel_linear = a.vmax
        msg.limit_acc_linear = a.amax
        msg.limit_vel_angular = a.wmax
        msg.limit_acc_angular = a.alphamax
        msg.kick_request = RobotMotionCommand.KR_DISABLE
        self.cmd_pub.publish(msg)


def spin_for(node, seconds):
    """ROS spins in a background thread; just wait, returning early on a trip."""
    end = time.monotonic() + seconds
    while time.monotonic() < end and rclpy.ok():
        if node.tripped:
            return False
        time.sleep(0.005)
    return node.tripped is None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--dx", type=float, default=0.05)
    ap.add_argument("--dy", type=float, default=0.0)
    ap.add_argument("--dtheta-deg", type=float, default=15.0)
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--hold", type=float, default=1.5, help="seconds at each end")
    ap.add_argument("--vmax", type=float, default=0.3)
    ap.add_argument("--amax", type=float, default=1.0)
    ap.add_argument("--wmax", type=float, default=2.0)
    ap.add_argument("--alphamax", type=float, default=6.0)
    ap.add_argument("--heading-ref", type=float, default=None,
                    help="absolute heading (rad) the +/-95 deg tether limit is measured from; default: start heading")
    ap.add_argument("--home", type=str, default=None,
                    help="absolute home pose 'x,y,theta' to move to (and hold) before the cycles; default: current pose")
    ap.add_argument("--sine-period", type=float, default=0.0,
                    help="if > 0: sweep the target home -> away -> home sinusoidally (period s) for --cycles periods")
    ap.add_argument("--out", type=str, default="analysis/data/small_move")
    args = ap.parse_args()

    rclpy.init()
    node = SmallMoveTest(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.get_logger().info("waiting for telemetry + vision")
        end = time.monotonic() + 10.0
        while (node.pose is None or node.last_vision is None) and time.monotonic() < end:
            time.sleep(0.01)
        if node.pose is None or node.last_vision is None:
            node.get_logger().error("no telemetry/vision; aborting")
            return 1
        spin_for(node, 0.5)
        node.start = node.pose.copy()
        node.heading_ref = node.start[2] if args.heading_ref is None else args.heading_ref
        home = node.pose.copy() if args.home is None else np.array([float(v) for v in args.home.split(",")])
        away = home + np.array([args.dx, args.dy, math.radians(args.dtheta_deg)])
        away[2] = math.remainder(away[2], 2 * math.pi)
        L = node.lim
        if (abs(args.dx) > 0.21 or abs(args.dy) > 0.21 or abs(args.dtheta_deg) > 91 or args.hold > 10
                or not all(L.field_x[0] + 0.01 < p[0] < L.field_x[1] - 0.01 and abs(p[1]) < L.field_y_abs - 0.01
                           for p in (home, away))
                or np.any(np.abs((np.array([home, away]) - node.start)[:, :2]) > L.xy_abs - 0.01)
                or any(abs(math.remainder(p[2] - node.heading_ref, 2 * math.pi)) > math.radians(91) for p in (home, away))):
            node.get_logger().error("move too large for the tether box")
            return 1
        node.get_logger().info(f"home {home}, away {away}")
        node.target = home
        ok = spin_for(node, args.hold if args.home is None else max(args.hold, 4.0))
        if args.sine_period > 0:
            node.get_logger().info(f"sine sweep: {args.cycles} x {args.sine_period} s")
            delta = away - home
            delta[2] = math.remainder(delta[2], 2 * math.pi)
            t_start = time.monotonic()
            while ok and (t := time.monotonic() - t_start) < args.cycles * args.sine_period:
                node.target = home + delta * 0.5 * (1.0 - math.cos(2.0 * math.pi * t / args.sine_period))
                ok = spin_for(node, 0.01)
            node.target = home
            ok = ok and spin_for(node, args.hold)
        for i in range(args.cycles if args.sine_period <= 0 else 0):
            if not ok:
                break
            node.get_logger().info(f"cycle {i + 1}: -> away")
            node.target = away
            ok = spin_for(node, args.hold)
            if not ok:
                break
            node.get_logger().info(f"cycle {i + 1}: -> home")
            node.target = home
            ok = spin_for(node, args.hold)
    finally:
        node.target = None
        for _ in range(20):
            node.send_off()
            time.sleep(0.01)
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        f = out / time.strftime("small_move_%Y%m%d_%H%M%S.npz")
        np.savez(f, data=np.array(node.log), tripped=str(node.tripped))
        node.get_logger().info(f"saved {f}; tripped={node.tripped}")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.tripped is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
