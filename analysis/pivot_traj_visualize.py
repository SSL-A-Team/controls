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
    args = parser.parse_args()

    compile_controls()

    from ateam_controls import (
        Vector2C,
        Vector6C,
        default_pivot_params,
        pivot_traj_new,
    )
    from visualization.trajectory import sample_pivot_trajectory, animate_robot_trajectory

    theta_start = np.deg2rad(args.theta_start)
    theta_end = np.deg2rad(args.theta_end)

    # Build trajectory ---------------------------------------------------
    params = default_pivot_params()
    orbit_radius = params.orbit_radius      # ball_radius + robot_radius
    robot_radius = 0.090                    # display radius of robot body
    ball_radius = orbit_radius - robot_radius  # ≈ 0.0215 m

    # Robot initially positioned on the orbit at theta_start, ball at origin
    init_x = float(-orbit_radius * np.cos(theta_start))
    init_y = float(-orbit_radius * np.sin(theta_start))
    init_state_c = Vector6C(
        data=(ctypes.c_float * 6)(
            init_x, init_y, float(theta_start), 0.0, 0.0, 0.0
        )
    )
    center_c = Vector2C(x=0.0, y=0.0)

    traj = pivot_traj_new(init_state_c, center_c, float(theta_end), params)

    # Sample at exactly fps Hz so each frame = 1/fps seconds of real time ----
    from ateam_controls import pivot_traj_end_time
    duration = float(pivot_traj_end_time(traj))
    n_frames = max(2, round(duration * args.fps) + 1)
    times, states = sample_pivot_trajectory(traj, init_state_c, n=n_frames)

    # Extra static decorations -------------------------------------------
    extra_patches = [
        # Reference orbit circle
        mpatches.Circle(
            (0.0, 0.0), orbit_radius,
            fill=False, edgecolor="gray", linewidth=1.0, linestyle=":",
            alpha=0.7, zorder=1, label="orbit",
        ),
        # Ball at origin
        mpatches.Circle(
            (0.0, 0.0), ball_radius,
            fill=True, facecolor="orange", edgecolor="darkorange",
            linewidth=1.0, alpha=0.9, zorder=2, label="ball",
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
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
