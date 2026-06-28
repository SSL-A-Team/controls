#!/usr/bin/env python3
"""Linear (line-following) trajectory animation.

Builds a ``LinearTrajectory`` that drives the robot perpendicular onto a line,
rotates it to a target heading (braking the colinear velocity to rest during the
approach), then accelerates along the line to a target speed once the robot is
within the perpendicular threshold. Past the trajectory end the robot coasts
along the line at the target speed forever.

The plot shows the robot (a filled circle with a heading indicator) animated in
the X/Y plane, the target line, the ±threshold band around it, the start pose,
and the engagement point where the colinear acceleration begins.

Usage (from the analysis/ directory):
    python linear_traj_visualize.py
    python linear_traj_visualize.py --fps 60 --coast-time 3

To explore the different scenarios / edge cases, uncomment exactly ONE of the
``scenario = Scenario(...)`` blocks in ``main()`` below (and comment out the
others).
"""

import argparse
import ctypes
from dataclasses import dataclass, field

import numpy as np
import matplotlib.pyplot as plt

from build import compile_controls


@dataclass
class Scenario:
    """A single starting condition for the line-following trajectory.

    Attributes
    ----------
    name : str
        Human-readable label shown in the plot title.
    init_state : list[float]
        Seed robot state ``[x, y, θ, ẋ, ẏ, θ̇]`` (metres, radians, m/s, rad/s).
    target_theta_deg : float
        Desired final heading in degrees.
    start_point : list[float]
        A point ``[x, y]`` the line passes through (metres).
    line_dir : list[float]
        Direction of the line ``[dx, dy]`` (normalized internally).
    line_vel : float
        Target velocity along ``line_dir`` once on the line (m/s).
    param_overrides : dict
        Optional ``LinearParams`` field overrides, e.g.
        ``{"colinear_start_thresh_linear": 0.3, "max_accel_colinear": 1.0}``.
    """

    name: str
    init_state: list
    target_theta_deg: float
    start_point: list
    line_dir: list
    line_vel: float
    param_overrides: dict = field(default_factory=dict)


def main():
    parser = argparse.ArgumentParser(
        description="Animate a line-following (linear) trajectory.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--fps", type=float, default=30.0,
        help="Animation playback speed in frames per second",
    )
    parser.add_argument(
        "--coast-time", type=float, default=2.0,
        help="Seconds of steady-state line following to show past the "
             "trajectory end time",
    )
    parser.add_argument(
        "--robot-radius", type=float, default=0.090,
        help="Robot body radius (m) used only for drawing",
    )
    args = parser.parse_args()

    # =====================================================================
    # Scenario selection
    # ---------------------------------------------------------------------
    # Uncomment exactly ONE scenario below (and comment out the others) to
    # view it. State is [x, y, θ, ẋ, ẏ, θ̇]; angles in the constructor below
    # are converted from degrees.
    # =====================================================================

    # 1) Baseline: 1 m perpendicular offset from a line along +x, heading
    #    already aligned, drive at 1.5 m/s. The robot slides straight onto the
    #    line and accelerates once inside the threshold.
    scenario = Scenario(
        name="baseline approach",
        init_state=[0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        target_theta_deg=0.0,
        start_point=[0.0, 0.0],
        line_dir=[1.0, 0.0],
        line_vel=1.5,
    )

    # # 2) Already inside the threshold → colinear acceleration engages
    # #    immediately (engagement time t_start = 0), no braking phase.
    # scenario = Scenario(
    #     name="already on line (immediate engage)",
    #     init_state=[0.0, 0.05, 0.0, 0.0, 0.0, 0.0],
    #     target_theta_deg=0.0,
    #     start_point=[0.0, 0.0],
    #     line_dir=[1.0, 0.0],
    #     line_vel=2.0,
    # )

    # # 3) Moving fast ALONG the line in the wrong direction during the
    # #    approach. The colinear velocity must brake to rest before the robot
    # #    engages and accelerates to the target.
    # scenario = Scenario(
    #     name="brake colinear then engage",
    #     init_state=[0.0, 1.5, 0.0, -2.5, 0.0, 0.0],
    #     target_theta_deg=0.0,
    #     start_point=[0.0, 0.0],
    #     line_dir=[1.0, 0.0],
    #     line_vel=1.5,
    # )

    # #   # 4) Diagonal line at 30°, robot offset to the side, large heading change.
    # scenario = Scenario(
    #     name="diagonal line (30°)",
    #     init_state=[-1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    #     target_theta_deg=30.0,
    #     start_point=[0.5, -0.5],
    #     line_dir=[0.8660254, 0.5],  # [cos30, sin30]
    #     line_vel=1.0,
    # )

    #   # 5) 180° heading flip while approaching (perpendicular approach with a
    #   #    full turn-around in θ).
    # scenario = Scenario(
    #     name="180° heading flip",
    #     init_state=[0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    #     target_theta_deg=180.0,
    #     start_point=[0.0, 0.0],
    #     line_dir=[1.0, 0.0],
    #     line_vel=1.5,
    # )

    #   # 6) Negative target velocity → travel along -line_dir once on the line.
    # scenario = Scenario(
    #     name="negative line velocity",
    #     init_state=[0.0, 1.0, np.pi, 0.0, 0.0, 0.0],
    #     target_theta_deg=180.0,
    #     start_point=[0.0, 0.0],
    #     line_dir=[1.0, 0.0],
    #     line_vel=-1.5,
    # )

    #   # 7) Initial velocity pointed at the line (perpendicular inbound). The
    #   #    perpendicular bang-bang must brake to avoid overshooting the line.
    # scenario = Scenario(
    #     name="perpendicular inbound velocity",
    #     init_state=[0.0, 1.5, 0.0, 0.0, -2.0, 0.0],
    #     target_theta_deg=0.0,
    #     start_point=[0.0, 0.0],
    #     line_dir=[1.0, 0.0],
    #     line_vel=1.5,
    # )

    #   # 8) Vertical line not through the origin; robot far to the side and
    #   #    rotating to face +y.
    # scenario = Scenario(
    #     name="vertical line, offset origin",
    #     init_state=[2.0, -1.0, 0.0, 0.0, 0.0, 0.0],
    #     target_theta_deg=90.0,
    #     start_point=[1.0, 0.0],
    #     line_dir=[0.0, 1.0],
    #     line_vel=1.0,
    # )

    #   # 9) Large threshold so colinear acceleration begins early, far from the
    #   #    line (override the default 0.1 m threshold).
    # scenario = Scenario(
    #     name="large engage threshold (0.5 m)",
    #     init_state=[0.0, 1.5, 0.0, 0.0, 0.0, 0.0],
    #     target_theta_deg=0.0,
    #     start_point=[0.0, 0.0],
    #     line_dir=[1.0, 0.0],
    #     line_vel=1.5,
    #     param_overrides={"colinear_start_thresh_linear": 0.5},
    # )

    #   # 10) Gentle limits: low accelerations stretch out the approach and
    #   #     engagement so each phase is easy to see.
    # scenario = Scenario(
    #     name="gentle accel limits",
    #     init_state=[-0.5, 1.0, 0.0, 0.0, 0.0, 0.0],
    #     target_theta_deg=0.0,
    #     start_point=[0.0, 0.0],
    #     line_dir=[1.0, 0.0],
    #     line_vel=1.0,
    #     param_overrides={
    #         "max_accel_perp": 0.8,
    #         "max_accel_colinear": 0.8,
    #         "max_vel_perp": 1.0,
    #     },
    # )

    # =====================================================================

    compile_controls()

    from ateam_controls import (
        Vector2C,
        Vector6C,
        default_linear_params,
        linear_traj_from_line,
        linear_traj_end_time,
        linear_traj_state_at,
    )
    from visualization.trajectory import (
        sample_linear_trajectory,
        animate_robot_trajectory,
    )

    # Build params from defaults, applying any per-scenario overrides.
    params = default_linear_params()
    for key, value in scenario.param_overrides.items():
        if not hasattr(params, key):
            raise AttributeError(f"Unknown LinearParams field: {key!r}")
        setattr(params, key, float(value))

    init_state_c = Vector6C(
        data=(ctypes.c_float * 6)(*[float(v) for v in scenario.init_state])
    )
    start_point_c = Vector2C(
        x=float(scenario.start_point[0]), y=float(scenario.start_point[1])
    )
    line_dir_c = Vector2C(
        x=float(scenario.line_dir[0]), y=float(scenario.line_dir[1])
    )
    target_theta = float(np.deg2rad(scenario.target_theta_deg))

    traj = linear_traj_from_line(
        init_state_c,
        target_theta,
        start_point_c,
        line_dir_c,
        float(scenario.line_vel),
        params,
    )

    end_time = float(linear_traj_end_time(traj))
    duration = end_time + max(0.0, args.coast_time)
    n_frames = max(2, round(duration * args.fps) + 1)
    times, states = sample_linear_trajectory(
        traj, n=n_frames, extra_time=args.coast_time
    )

    # Normalized line geometry for drawing.
    sp = np.array([scenario.start_point[0], scenario.start_point[1]], dtype=float)
    ld = np.array([scenario.line_dir[0], scenario.line_dir[1]], dtype=float)
    ld = ld / np.linalg.norm(ld)
    perp = np.array([-ld[1], ld[0]])
    thresh = float(params.colinear_start_thresh_linear)

    # Engagement point: where the colinear acceleration begins.
    t_start = float(traj.colinear_accel_start_time)
    engage_c = linear_traj_state_at(traj, ctypes.c_float(t_start))
    engage_xy = (float(engage_c.data[0]), float(engage_c.data[1]))

    title = (
        f"Linear trajectory — {scenario.name}\n"
        f"target {scenario.target_theta_deg:.0f}°, "
        f"v={scenario.line_vel:.2f} m/s, "
        f"engage @ t={t_start:.2f}s, T={end_time:.2f}s"
    )

    fig, anim = animate_robot_trajectory(
        times, states,
        robot_radius=args.robot_radius,
        title=title,
        fps=args.fps,
    )

    # ---------------------------------------------------------------------
    # Static annotations: line, ±threshold band, start pose, engage point.
    # Drawn after the animation set up the axes/limits.
    # ---------------------------------------------------------------------
    ax = fig.axes[0]

    # Span the line across the colinear extent of the trajectory (plus margin).
    rel = states[:, :2] - sp
    s_coords = rel @ ld
    s_lo, s_hi = float(s_coords.min()), float(s_coords.max())
    margin = 0.5 + (s_hi - s_lo) * 0.1
    s_a, s_b = s_lo - margin, s_hi + margin
    line_a = sp + s_a * ld
    line_b = sp + s_b * ld
    ax.plot(
        [line_a[0], line_b[0]], [line_a[1], line_b[1]],
        color="green", linewidth=1.6, linestyle="-", alpha=0.8,
        zorder=1.5, label="line",
    )
    # ±threshold band around the line.
    for sign in (+1.0, -1.0):
        off = sign * thresh * perp
        ax.plot(
            [line_a[0] + off[0], line_b[0] + off[0]],
            [line_a[1] + off[1], line_b[1] + off[1]],
            color="green", linewidth=1.0, linestyle=":", alpha=0.5,
            zorder=1.4,
            label=("±threshold" if sign > 0 else None),
        )

    # Start pose marker.
    ax.plot(
        [scenario.init_state[0]], [scenario.init_state[1]],
        marker="o", color="black", markersize=6, zorder=6, label="start",
    )
    # Engagement point marker (colinear accel begins).
    ax.plot(
        [engage_xy[0]], [engage_xy[1]],
        marker="*", color="red", markersize=14, zorder=6, label="engage",
    )

    ax.relim()
    ax.autoscale_view()
    ax.legend(loc="upper right", fontsize=8)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
