#!/usr/bin/env python3
"""Pivot trajectory animation — robot pivoting from one heading to another.

Creates a time-optimal bang-bang pivot trajectory and animates the robot
moving in the X/Y plane while keeping its front face in contact with a ball
at the origin.

The robot is drawn as a filled circle with a line from its centre to its
perimeter indicating the current heading.

Usage (from the analysis/ directory):
    python pivot_traj_visualize.py
    python pivot_traj_visualize.py --theta-start 0 --theta-end 180
    python pivot_traj_visualize.py --theta-start -90 --theta-end 90 --fps 60
"""

import argparse
import ctypes
import numpy as np
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

from build import compile_controls


def main():
    parser = argparse.ArgumentParser(
        description="Animate a pivot trajectory from one heading to another.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--theta-start", type=float, default=0.0,
        help="Starting heading in degrees",
    )
    parser.add_argument(
        "--theta-end", type=float, default=180.0,
        help="Target heading in degrees",
    )
    parser.add_argument(
        "--fps", type=float, default=30.0,
        help="Animation playback speed in frames per second",
    )
    parser.add_argument(
        "--inset-angle", type=float, default=None,
        help="Angle (degrees) from the radius line pointing at the orbit center "
             "to the robot heading, positive toward the direction of travel "
             "(0 = face center, +90 = tangent forward, -90 = tangent backward), "
             "held constant for the whole pivot (default: pivot parameter default)",
    )
    parser.add_argument(
        "--orbit-radius", type=float, default=None,
        help="Orbit radius in meters (default: pivot parameter default)",
    )
    parser.add_argument(
        "--max-angular-vel", type=float, default=None,
        help="Max orbit angular velocity in rad/s (default: pivot parameter default)",
    )
    parser.add_argument(
        "--max-angular-acc", type=float, default=None,
        help="Max orbit angular acceleration in rad/s^2 (default: pivot parameter default)",
    )
    args = parser.parse_args()

    compile_controls()

    from ateam_controls import (
        Vector6C,
        default_pivot_params,
        pivot_traj_new,
    )
    from visualization.trajectory import sample_pivot_trajectory, animate_robot_trajectory

    theta_start = np.deg2rad(args.theta_start)
    theta_end = np.deg2rad(args.theta_end)

    # Build trajectory ---------------------------------------------------
    # Start from the pivot parameter defaults; only override fields whose CLI
    # arg was explicitly provided.
    params = default_pivot_params()
    if args.inset_angle is not None:
        params.inset_angle = float(np.deg2rad(args.inset_angle))
    if args.orbit_radius is not None:
        params.orbit_radius = float(args.orbit_radius)
    if args.max_angular_vel is not None:
        params.max_vel_angular = float(args.max_angular_vel)
    if args.max_angular_acc is not None:
        params.max_accel_angular = float(args.max_angular_acc)

    orbit_radius = params.orbit_radius      # ball_radius + robot_radius
    robot_radius = 0.090                    # display radius of robot body
    ball_radius = 0.0215

    # Start the robot at the origin with the given start heading.
    init_state_c = Vector6C(
        data=(ctypes.c_float * 6)(
            0.0, 0.0, float(theta_start), 0.0, 0.0, 0.0
        )
    )

    traj = pivot_traj_new(init_state_c, float(theta_end), params)

    # Actual orbit center derived by the trajectory (depends on orbit direction).
    center_x = float(traj.center_x)
    center_y = float(traj.center_y)

    # Sample at exactly fps Hz so each frame = 1/fps seconds of real time ----
    from ateam_controls import pivot_traj_end_time
    duration = float(pivot_traj_end_time(traj))
    n_frames = max(2, round(duration * args.fps) + 1)
    times, states = sample_pivot_trajectory(traj, n=n_frames)

    # Reference orbit circle drawn at the trajectory's actual orbit center.
    extra_patches = [
        mpatches.Circle(
            (center_x, center_y), orbit_radius,
            fill=False, edgecolor="gray", linewidth=1.0, linestyle=":",
            alpha=0.7, zorder=1, label="orbit",
        ),
    ]

    title = (
        f"Pivot trajectory  "
        f"θ: {args.theta_start:.0f}° → {args.theta_end:.0f}°  "
        f"(T = {duration:.2f} s)"
    )

    fig, anim = animate_robot_trajectory(
        times, states,
        robot_radius=robot_radius,
        title=title,
        fps=args.fps,
        extra_patches=extra_patches,
        front_ball_radius=ball_radius,
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
