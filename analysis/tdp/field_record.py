"""Record a field-scale test (kenobi's TdpMotionPlay, or anything else driving the robot)
for fig_field.py: extended telemetry plus the motion command the robot was sent.

Unlike telem_record.py this keeps the trajectory, the body accel before/after the
current budget, the IMU accelerometer, and the commanded target and limits, which is
what segmenting moves and measuring achieved acceleration need. Same npz layout
(data + columns), so calibration/common.load() reads it.

    python3 analysis/tdp/field_record.py -r 2 --out analysis/data/tdp/field_star_shipped_r2.npz
    (Ctrl-C to stop, or --duration)
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node

from ateam_msgs.msg import RobotMotionCommand
from ateam_radio_msgs.msg import ExtendedTelemetry

MOTORS = ("front_left_motor", "back_left_motor", "back_right_motor", "front_right_motor")
MOTOR_FIELDS = ("enc", "vel_sp", "i_sp", "i_meas", "tlim", "vbus")
V3 = lambda p: [f"{p}_x", f"{p}_y", f"{p}_w"]  # noqa: E731
COLUMNS = (["t_host", "t_robot", "gyro", "imu_ax", "imu_ay", "vis_x", "vis_y", "vis_th", "vis_upd",
            "vis_age", "vis_old", "x", "y", "th", "vx_l", "vy_l", "vw_l", "bcm"]
           + V3("traj_p") + V3("traj_v") + V3("vel_u") + V3("acc_u") + V3("acc_ff")
           + [f"{f}{i}" for i in range(4) for f in MOTOR_FIELDS]
           + ["cmd_mode", "cmd_x", "cmd_y", "cmd_th", "cmd_vmax", "cmd_amax", "cmd_wmax", "cmd_alphamax"])


NO_CMD = [np.nan] * 8


def command_fields(c):
    """RobotMotionCommand -> the cmd_* columns."""
    return [float(c.body_control_mode), c.pose.x, c.pose.y, c.pose.theta,
            c.limit_vel_linear, c.limit_acc_linear, c.limit_vel_angular, c.limit_acc_angular]


def v3(values):
    """Exactly 3 floats. The generated message fields are unbounded arrays; the bridge
    always fills 3, but a short or empty one must not shift every later column."""
    out = [float(v) for v in list(values)[:3]]
    return out + [float("nan")] * (3 - len(out))


def telemetry_row(m, t_host, cmd):
    """ExtendedTelemetry (+ the latest command's cmd_* fields) -> one row of COLUMNS.
    Shared with bag2npz.py so live recordings and bag conversions are identical."""
    b = m.body_control_telemetry
    row = [t_host, ((m.timestamp_us_hi << 32) | m.timestamp_us_lo) * 1e-6, v3(b.imu_gyro)[2],
           *v3(b.imu_accel)[:2], *v3(b.vision_pose), float(b.vision_update),
           b.vision_insert_age_us * 1e-6, float(b.vision_old),
           *v3(b.kf_body_pos_estimate), *v3(b.kf_body_vel_estimate), float(b.body_control_mode),
           *v3(b.body_traj_pos), *v3(b.body_traj_vel), *v3(b.body_vel_u), *v3(b.body_accel_u),
           *v3(b.body_accel_u_fric_comp)]
    for name in MOTORS:
        mot = getattr(m, name)
        c, v = mot.current_telemetry, mot.velocity_telemetry
        samples = list(c.current_samples_ma)
        row += [v.wheel_vel_rads, v.vel_setpoint_rads, c.current_setpoint_ma / 1000.0,
                (sum(samples) / len(samples) / 1000.0) if samples else 0.0, float(mot.torque_limited),
                c.bus_voltage_mv / 1000.0]
    row += list(cmd)
    assert len(row) == len(COLUMNS), (len(row), len(COLUMNS))
    return row


class FieldRecorder(Node):
    def __init__(self, robot):
        super().__init__("tdp_field_record")
        self.rows = []
        self.cmd = NO_CMD
        self.create_subscription(ExtendedTelemetry, f"/robot_feedback/extended/robot{robot}", self.on_telem, 200)
        self.create_subscription(RobotMotionCommand, f"/robot_motion_commands/robot{robot}", self.on_cmd, 50)

    def on_cmd(self, c):
        self.cmd = command_fields(c)

    def on_telem(self, m):
        self.rows.append(telemetry_row(m, time.monotonic(), self.cmd))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--duration", type=float, default=600.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = FieldRecorder(args.robot)
    end = time.monotonic() + args.duration
    last = 0.0
    try:
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - last > 5.0:
                last = time.monotonic()
                print(f"\r{len(node.rows)} rows", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        np.savez(args.out, data=np.array(node.rows, dtype=float), columns=np.array(COLUMNS))
        print(f"\nsaved {len(node.rows)} rows to {args.out}")
        if node.rows and np.isnan(np.array(node.rows)[:, -1]).all():
            print("warning: no motion commands seen on /robot_motion_commands/robot%d" % args.robot)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
