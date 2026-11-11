"""Record robot ExtendedTelemetry (body + all motor fields) to an npz for offline analysis.

Runs alongside a test script; stop with Ctrl-C or --duration.

    python analysis/ros_scripts/telem_record.py -r 2 --duration 20 --out /tmp/rec.npz

Columns (see COLUMNS): robot time, gyro z, vision pose (0 when no update this frame),
KF pose and local velocity, body control mode, then per motor (FL, BL, BR, FR):
encoder speed, hall speed, speed setpoint, current setpoint, measured current (mean
of the 20 samples, unsigned), bus voltage, motor voltage command, motion type,
torque_limited.
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node

from ateam_radio_msgs.msg import ExtendedTelemetry

MOTORS = ("front_left_motor", "back_left_motor", "back_right_motor", "front_right_motor")
MOTOR_FIELDS = ("enc", "hall", "vel_sp", "i_sp", "i_meas", "vbus", "vmot", "mct", "tlim")
COLUMNS = (["t_host", "t_robot", "gyro", "vis_x", "vis_y", "vis_th", "vis_upd", "x", "y", "th",
            "vx_l", "vy_l", "vw_l", "bcm"]
           + [f"{f}{i}" for i in range(4) for f in MOTOR_FIELDS])


class Recorder(Node):
    def __init__(self, robot):
        super().__init__("telem_record")
        self.rows = []
        self.create_subscription(ExtendedTelemetry, f"/robot_feedback/extended/robot{robot}", self.cb, 200)

    def cb(self, m):
        b = m.body_control_telemetry
        row = [time.monotonic(), ((m.timestamp_us_hi << 32) | m.timestamp_us_lo) * 1e-6, b.imu_gyro[2],
               *b.vision_pose[:3], float(b.vision_update), *b.kf_body_pos_estimate[:3],
               *b.kf_body_vel_estimate[:3], float(b.body_control_mode)]
        for name in MOTORS:
            mot = getattr(m, name)
            c, v = mot.current_telemetry, mot.velocity_telemetry
            samples = list(c.current_samples_ma)
            row += [v.wheel_vel_rads, c.hall_vel_est_drads / 10.0, v.vel_setpoint_rads,
                    c.current_setpoint_ma / 1000.0, (sum(samples) / len(samples) / 1000.0) if samples else 0.0,
                    c.bus_voltage_mv / 1000.0, c.motor_voltage_cmd_mv / 1000.0,
                    float(mot.motion_control_type), float(mot.torque_limited)]
        self.rows.append(row)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    rclpy.init()
    node = Recorder(args.robot)
    end = time.monotonic() + args.duration
    try:
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        np.savez(args.out, data=np.array(node.rows, dtype=float), columns=np.array(COLUMNS))
        print(f"saved {len(node.rows)} rows to {args.out}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
