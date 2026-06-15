#!/usr/bin/env python3
"""
PID-tuning signal generator.

Drives a single robot with a parameterized signal (pulse / sinusoid / step /
chirp) on a chosen axis (x / y / theta) in either global position or global
velocity mode. Used to characterize the firmware's controller response and
to record telemetry for offline visualization.

State machine::

    WAIT_FOR_VISION    (latch center pose from /<team>_team/robot<id>)
        |
    WARMUP             (hold rest command for --warmup-duration seconds)
        |
    RUN                (publish signal for --duration seconds, or forever)
        |
    HOLD               (hold rest command, then exit if --duration > 0)

Example::

    python ros_scripts/signal_input.py \\
        --robot-id 2 --axis x --signal sinusoid \\
        --amplitude 0.3 --sine-frequency 1.5 \\
        --warmup-duration 2.0 --duration 10.0
"""

from __future__ import annotations

import argparse
import math
import sys
from enum import auto, Enum

from ateam_msgs.msg import RobotMotionCommand, Twist2D, VisionStateRobot
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


AXIS_INDEX = {"x": 0, "y": 1, "theta": 2}


class State(Enum):
    WAIT_FOR_VISION = auto()
    WARMUP = auto()
    RUN = auto()
    HOLD = auto()


def yaw_from_quat(q) -> float:
    """Extract yaw from a planar quaternion (assumes roll=pitch=0)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SignalInputNode(Node):

    def __init__(self, args: argparse.Namespace):
        super().__init__("signal_input")
        self.args = args
        self.axis_idx = AXIS_INDEX[args.axis]

        if args.signal == "square" and args.control_mode != "position":
            raise ValueError(
                "--signal square requires --control-mode position "
                "(it commands 2D waypoints; velocity mode is not supported)."
            )

        # CCW square offsets per side, in the order +x, +y, -x, -y
        # (each entry is the dx, dy increment applied each `side_duration`).
        self._square_dirs = (
            (+1.0, 0.0),
            (0.0, +1.0),
            (-1.0, 0.0),
            (0.0, -1.0),
        )

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        topic_robot = f"/{args.team_color}_team/robot{args.robot_id}"
        self.robot_sub = self.create_subscription(
            VisionStateRobot, topic_robot, self.robot_cb, sensor_qos)

        topic_cmd = f"/robot_motion_commands/robot{args.robot_id}"
        # Use default RELIABLE QoS. The radio_bridge subscriber requests
        # BEST_EFFORT, which is compatible with a RELIABLE publisher
        # (publisher's offered quality >= subscriber's request). Using
        # BEST_EFFORT here was wrong: it prevented any other RELIABLE
        # subscribers (e.g. `ros2 topic echo`) from receiving messages,
        # without solving any real problem.
        self.cmd_pub = self.create_publisher(
            RobotMotionCommand, topic_cmd, 10)

        self.robot = None
        self.center_pose = None  # (x, y, theta)
        self.state = State.WAIT_FOR_VISION
        self.state_entered = self.get_clock().now()
        self.done = False
        self.hold_value = 0.0  # latched signal value at end of RUN
        self.hold_command = None  # full latched command for 2D signals (square)

        self.timer = self.create_timer(1.0 / args.rate_hz, self.tick)

        self.get_logger().info(
            f"signal_input started: robot={args.robot_id} team={args.team_color} "
            f"mode={args.control_mode} axis={args.axis} signal={args.signal} "
            f"amp={args.amplitude} warmup={args.warmup_duration}s "
            f"duration={args.duration}s rate={args.rate_hz}Hz"
        )
        self.get_logger().info(f"State: {self.state.name}")

    # ---------------------------------------------------------------- callbacks
    def robot_cb(self, msg: VisionStateRobot):
        self.robot = msg

    # ------------------------------------------------------------------ helpers
    def transition(self, new_state: State):
        self.get_logger().info(f"State {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_entered = self.get_clock().now()

    def time_in_state(self) -> float:
        return (self.get_clock().now()
                - self.state_entered).nanoseconds * 1e-9

    # ---------------------------------------------------------- signal generators
    def signal_value(self, t: float) -> float:
        """Compute the scalar signal value at time `t` (seconds since RUN start)."""
        a = self.args
        sig = a.signal
        amp = a.amplitude

        if sig == "pulse":
            period = 1.0 / a.pulse_frequency if a.pulse_frequency > 0 else 0.0
            if period <= 0.0:
                return 0.0
            phase = t % period
            return amp if phase < a.pulse_width else 0.0

        if sig == "sinusoid":
            return amp * math.sin(2.0 * math.pi * a.sine_frequency * t)

        if sig == "step":
            return amp if t >= a.step_time else 0.0

        if sig == "chirp":
            if t > a.chirp_duration:
                return 0.0
            f0 = a.chirp_start_freq
            f1 = a.chirp_end_freq
            T = a.chirp_duration
            phi = 2.0 * math.pi * (f0 * t + (f1 - f0) / (2.0 * T) * t * t)
            return amp * math.sin(phi)

        return 0.0

    # ---------------------------------------------------- command construction
    def make_square_command(self, t: float) -> RobotMotionCommand:
        """2D square path waypoint command.

        Cycles +x -> +y -> -x -> -y (CCW). The target waypoint is the sum of
        the per-corner increments for every side that has fully elapsed by
        time `t` (seconds since RUN start). Side i is "fully elapsed" when
        `t >= (i+1) * side_duration`. While side i is in progress, the
        waypoint sits at the corner reached at the end of side i-1 (so the
        bang-bang firmware drives to that target during the side's time
        budget).
        """
        a = self.args
        side_len = float(a.square_side_length)
        side_dur = float(a.square_side_duration)

        # How many sides have finished?
        if side_dur > 0.0:
            sides_done = int(t // side_dur) + 1
        else:
            sides_done = 0

        dx = 0.0
        dy = 0.0
        for i in range(sides_done):
            ux, uy = self._square_dirs[i % 4]
            dx += side_len * ux
            dy += side_len * uy

        cx, cy, _ = self.center_pose

        cmd = RobotMotionCommand()
        cmd.body_control_mode = RobotMotionCommand.BCM_GLOBAL_POSITION
        cmd.pose = Twist2D(x=float(cx + dx), y=float(cy + dy), theta=0.0)
        cmd.velocity = Twist2D()
        cmd.acceleration = Twist2D()
        cmd.limit_vel_linear = float(a.limit_vel_linear)
        cmd.limit_vel_angular = float(a.limit_vel_angular)
        cmd.limit_acc_linear = float(a.limit_acc_linear)
        cmd.limit_acc_angular = float(a.limit_acc_angular)
        cmd.kick_request = RobotMotionCommand.KR_DISABLE
        cmd.kick_speed = 0.0
        cmd.dribbler_speed = 0.0
        return cmd

    def make_command(self, signal: float) -> RobotMotionCommand:
        a = self.args
        cmd = RobotMotionCommand()
        cmd.pose = Twist2D()
        cmd.velocity = Twist2D()
        cmd.acceleration = Twist2D()
        cmd.limit_vel_linear = float(a.limit_vel_linear)
        cmd.limit_vel_angular = float(a.limit_vel_angular)
        cmd.limit_acc_linear = float(a.limit_acc_linear)
        cmd.limit_acc_angular = float(a.limit_acc_angular)
        cmd.kick_request = RobotMotionCommand.KR_DISABLE
        cmd.kick_speed = 0.0
        cmd.dribbler_speed = 0.0

        if a.control_mode == "position":
            cmd.body_control_mode = RobotMotionCommand.BCM_GLOBAL_POSITION
            cx, cy, ctheta = self.center_pose
            cmd.pose.x = float(cx)
            cmd.pose.y = float(cy)
            cmd.pose.theta = float(ctheta)
            if self.axis_idx == 0:
                cmd.pose.x += signal
            elif self.axis_idx == 1:
                cmd.pose.y += signal
            else:
                cmd.pose.theta += signal
        else:  # velocity
            cmd.body_control_mode = RobotMotionCommand.BCM_GLOBAL_VELOCITY
            if self.axis_idx == 0:
                cmd.velocity.x = float(signal)
            elif self.axis_idx == 1:
                cmd.velocity.y = float(signal)
            else:
                cmd.velocity.theta = float(signal)

        return cmd

    # ------------------------------------------------------------------- tick
    def tick(self):
        s = self.state

        if s == State.WAIT_FOR_VISION:
            if self.robot is None or not self.robot.visible:
                return
            p = self.robot.pose.position
            theta = yaw_from_quat(self.robot.pose.orientation)
            self.center_pose = (p.x, p.y, theta)
            self.get_logger().info(
                f"Latched center pose: ({p.x:.3f}, {p.y:.3f}, {theta:.3f})"
            )
            if self.args.warmup_duration > 0.0:
                self.transition(State.WARMUP)
            else:
                self.transition(State.RUN)
            return

        if s == State.WARMUP:
            self.cmd_pub.publish(self.make_command(0.0))
            if self.time_in_state() >= self.args.warmup_duration:
                self.transition(State.RUN)
            return

        if s == State.RUN:
            t = self.time_in_state()
            if self.args.signal == "square":
                cmd = self.make_square_command(t)
                self.cmd_pub.publish(cmd)
                if self.args.duration > 0.0 and t >= self.args.duration:
                    # Latch final waypoint so HOLD doesn't snap back to start
                    self.hold_command = cmd
                    self.transition(State.HOLD)
            else:
                value = self.signal_value(t)
                self.cmd_pub.publish(self.make_command(value))
                if self.args.duration > 0.0 and t >= self.args.duration:
                    # Latch final signal value so HOLD doesn't snap back to 0
                    # (matters for `step` and the high half of `pulse`).
                    self.hold_value = value
                    self.hold_command = None
                    self.transition(State.HOLD)
            return

        if s == State.HOLD:
            if self.hold_command is not None:
                self.cmd_pub.publish(self.hold_command)
            else:
                self.cmd_pub.publish(self.make_command(self.hold_value))
            # Brief settle, then signal completion so main can shutdown.
            if self.time_in_state() >= 0.2:
                self.done = True
            return


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--robot-id", type=int, default=2)
    p.add_argument("--team-color", type=str, default="blue",
                   choices=["blue", "yellow"])
    p.add_argument("--control-mode", type=str, default="position",
                   choices=["position", "velocity"])
    p.add_argument("--axis", type=str, default="x",
                   choices=["x", "y", "theta"])
    p.add_argument("--signal", type=str, default="pulse",
                   choices=["pulse", "sinusoid", "step", "chirp", "square"],
                   help="`square` drives a 2D square path in position mode "
                        "(--axis is ignored). All others are 1D on --axis.")
    p.add_argument("--amplitude", type=float, default=0.2,
                   help="meters or m/s for x/y; radians or rad/s for theta")

    p.add_argument("--pulse-frequency", type=float, default=1.0,
                   help="Hz; period = 1 / pulse_frequency")
    p.add_argument("--pulse-width", type=float, default=0.2,
                   help="seconds; high-time per period (signal = +amp during "
                        "pulse, 0 for rest of period)")

    p.add_argument("--sine-frequency", type=float, default=1.0, help="Hz")

    p.add_argument("--chirp-start-freq", type=float, default=0.1, help="Hz")
    p.add_argument("--chirp-end-freq", type=float, default=5.0, help="Hz")
    p.add_argument("--chirp-duration", type=float, default=30.0, help="seconds")

    p.add_argument("--step-time", type=float, default=1.0,
                   help="seconds after warmup at which the step transitions")

    p.add_argument("--square-side-length", type=float, default=0.5,
                   help="meters per side for `--signal square`")
    p.add_argument("--square-side-duration", type=float, default=2.0,
                   help="seconds per side for `--signal square`")

    p.add_argument("--warmup-duration", type=float, default=2.0,
                   help="seconds holding center pose / zero velocity before signal")
    p.add_argument("--duration", type=float, default=0.0,
                   help="signal-phase duration (0 = run indefinitely)")
    p.add_argument("--rate-hz", type=float, default=100.0)

    p.add_argument("--limit-vel-linear", type=float, default=0.0,
                   help="0 = firmware default")
    p.add_argument("--limit-vel-angular", type=float, default=0.0,
                   help="0 = firmware default")
    p.add_argument("--limit-acc-linear", type=float, default=0.0,
                   help="0 = firmware default")
    p.add_argument("--limit-acc-angular", type=float, default=0.0,
                   help="0 = firmware default")

    return p


def main(argv=None):
    args = build_parser().parse_args(argv)

    rclpy.init()
    node = SignalInputNode(args)
    try:
        # spin() with a separate "done" check via a short-period guard
        # keeps the executor running tight (no 100ms-class waits between
        # ticks that can stall publishing).
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)

        def _check_done():
            if node.done:
                executor.shutdown()
        guard = node.create_timer(0.05, _check_done)

        try:
            executor.spin()
        finally:
            node.destroy_timer(guard)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
