"""Exercise the pivot and line maneuvers on a tethered robot, with the small_move_test
safety monitor and log format (so mv.py / summarize.py style tools apply).

Each test is a list of timed phases; a phase sends one RobotMotionCommand, or holds
`--home` in position mode. The robot also returns home between tests.

    python analysis/ros_scripts/maneuver_test.py --home=0,0,3.14 --heading-ref 3.14 --test heading_pivot
    tests: heading_pivot, point_pivot, heading_line, point_line, all
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

M = RobotMotionCommand


class ManeuverTest(SmallMoveTest):
    def __init__(self, args):
        super().__init__(args)
        self.cmd = None  # callable(msg) filling a maneuver command, or None for position hold

    def publish(self):
        if self.tripped or self.target is None:
            self.send_off()
            return
        if self.cmd is None:
            return super().publish()
        msg = M()
        msg.kick_request = M.KR_DISABLE
        self.cmd(msg)
        self.cmd_pub.publish(msg)


def heading_pivot(theta, a):
    def f(m):
        m.body_control_mode = M.BCM_HEADING_PIVOT
        m.pivot_global_theta = float(theta)
        m.pivot_max_angular_vel = a.pivot_w
        m.pivot_max_angular_acc = a.pivot_alpha
        m.pivot_orbit_radius = a.radius
        m.pivot_compute_inset_angle = True
    return f


def point_pivot(px, py, a):
    def f(m):
        m.body_control_mode = M.BCM_POINT_PIVOT
        m.pivot_target_x, m.pivot_target_y = float(px), float(py)
        m.pivot_max_angular_vel = a.pivot_w
        m.pivot_max_angular_acc = a.pivot_alpha
        m.pivot_orbit_radius = a.radius
        m.pivot_compute_inset_angle = True
    return f


def line(home, v, a, theta=None, point=None):
    def f(m):
        if point is None:
            m.body_control_mode = M.BCM_HEADING_LINE
            m.line_global_theta = float(theta)
        else:
            m.body_control_mode = M.BCM_POINT_LINE
            m.line_target_x, m.line_target_y = map(float, point)
        # line along global y through home, offset in x so the robot has to converge onto it
        m.line_start_x, m.line_start_y = float(home[0] + a.line_offset), float(home[1])
        m.line_dir_x, m.line_dir_y = 0.0, 1.0
        m.line_velocity = float(v)
        m.line_max_vel_colinear = abs(float(v))
        m.line_max_vel_perp = a.line_vperp
        m.line_max_vel_angular = 3.0
        m.line_max_accel_colinear = a.line_acc
        m.line_max_accel_perp = a.line_acc
        m.line_max_accel_angular = 10.0
        m.line_colinear_start_thresh = 0.01
    return f


def phases(test, home, a):
    th = home[2]
    turn = math.radians(a.turn_deg)
    # facing point on the robot's current heading, a.point_dist away
    fx, fy = home[0] + a.point_dist * math.cos(th), home[1] + a.point_dist * math.sin(th)
    side = a.point_dist * math.tan(turn)  # lateral offset of the point to turn by `turn`
    nx, ny = -math.sin(th), math.cos(th)
    t_line = a.line_len / a.line_v
    # The pivot orbits a center off to one side of the heading (inset angle), chosen per
    # command by shortest travel, so out-and-back pivots do not return to the start:
    # re-home in position mode (None) after each pivot to stay in the tether box.
    hold = (None, a.hold + 0.8)
    if test == "heading_pivot":
        return [(heading_pivot(th + turn, a), a.dwell), hold, (heading_pivot(th - turn, a), a.dwell), hold]
    if test == "point_pivot":
        return [(point_pivot(fx + side * nx, fy + side * ny, a), a.dwell), hold,
                (point_pivot(fx - side * nx, fy - side * ny, a), a.dwell), hold]
    if test in ("heading_line", "point_line"):
        kw = dict(theta=th) if test == "heading_line" else dict(point=(fx, fy))
        return [(line(home, a.line_v, a, **kw), t_line), (line(home, 0.0, a, **kw), a.dwell),
                (line(home, -a.line_v, a, **kw), 2 * t_line), (line(home, 0.0, a, **kw), a.dwell),
                (line(home, a.line_v, a, **kw), t_line), (line(home, 0.0, a, **kw), a.dwell)]
    raise ValueError(test)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--home", type=str, required=True)
    ap.add_argument("--heading-ref", type=float, required=True)
    ap.add_argument("--box", type=float, default=0.25)
    ap.add_argument("--test", default="all",
                    choices=("heading_pivot", "point_pivot", "heading_line", "point_line", "all"))
    ap.add_argument("--turn-deg", type=float, default=60.0, help="pivot turn each way")
    ap.add_argument("--radius", type=float, default=0.12, help="pivot orbit radius (m)")
    ap.add_argument("--pivot-w", type=float, default=4.0)
    ap.add_argument("--pivot-alpha", type=float, default=10.0)
    ap.add_argument("--point-dist", type=float, default=1.0, help="facing point distance (m)")
    ap.add_argument("--line-v", type=float, default=0.3)
    ap.add_argument("--line-len", type=float, default=0.1, help="travel per leg from home (m)")
    ap.add_argument("--line-acc", type=float, default=2.0)
    ap.add_argument("--line-vperp", type=float, default=0.5)
    ap.add_argument("--line-offset", type=float, default=0.03, help="line x offset from home (m)")
    ap.add_argument("--dwell", type=float, default=1.2)
    ap.add_argument("--hold", type=float, default=1.0, help="position hold at home between tests (s)")
    ap.add_argument("--vmax", type=float, default=1.0)
    ap.add_argument("--amax", type=float, default=2.0)
    ap.add_argument("--wmax", type=float, default=3.0)
    ap.add_argument("--alphamax", type=float, default=10.0)
    ap.add_argument("--out", type=str, default="analysis/data/maneuver_test")
    a = ap.parse_args()
    home = np.array([float(x) for x in a.home.split(",")])

    rclpy.init()
    node = ManeuverTest(a)
    node.lim.field_x = (-a.box, a.box)
    node.lim.field_y_abs = a.box
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    end = time.monotonic() + 5
    while (node.pose is None or node.last_vision is None) and time.monotonic() < end:
        time.sleep(0.05)
    if node.pose is None or node.last_vision is None:
        print("no telemetry/vision")
        return
    node.start = node.pose.copy()
    node.heading_ref = a.heading_ref
    tests = ("heading_pivot", "point_pivot", "heading_line", "point_line") if a.test == "all" else (a.test,)
    ok = True
    node.target = home.copy()
    ok = spin_for(node, a.hold + 1.0)
    for test in tests:
        if not ok:
            break
        node.get_logger().info(f"{test}")
        for cmd, dur in phases(test, home, a):
            node.cmd = cmd
            if not (ok := spin_for(node, dur)):
                break
        node.cmd = None
        ok = ok and spin_for(node, a.hold + 0.8)
    node.target = None
    spin_for(node, 0.3)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    f = out / time.strftime(f"maneuver_{a.test}_%Y%m%d_%H%M%S.npz")
    np.savez(f, data=np.array(node.log, dtype=object), tripped=str(node.tripped))
    print(f"saved {f}; tripped: {node.tripped}")
    ex.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
