"""Robot trajectory sampling and X/Y animation utilities.

Functions
---------
sample_pivot_trajectory(traj, init_state_c, n) → (times, states)
    Sample a PivotTrajectory via FFI at evenly-spaced time points.

animate_robot_trajectory(times, states, ...) → (fig, anim)
    Create a matplotlib animation of a circular robot (with heading indicator)
    moving through the X/Y plane along pre-sampled trajectory states.
"""

import ctypes
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation

from ateam_controls import (
    Vector6C,
    pivot_traj_end_time,
    pivot_traj_state_at,
)


# ============================================================================
# Trajectory sampling
# ============================================================================

def sample_pivot_trajectory(traj, init_state_c, n=300):
    """Sample a PivotTrajectory at *n* evenly-spaced time points.

    Parameters
    ----------
    traj : PivotTrajectory
        The pivot trajectory object returned by ``pivot_traj_new``.
    init_state_c : Vector6C
        Robot state at trajectory start: ``[x, y, θ, ẋ, ẏ, θ̇]``.
    n : int
        Number of samples (default 300).

    Returns
    -------
    times : np.ndarray, shape (n,)
        Time values in seconds.
    states : np.ndarray, shape (n, 6)
        Robot state ``[x, y, θ, ẋ, ẏ, θ̇]`` at each sample.
    """
    end = float(pivot_traj_end_time(traj))
    times = np.linspace(0.0, end, n)
    states = np.empty((n, 6), dtype=np.float64)
    for i, t in enumerate(times):
        st = pivot_traj_state_at(
            traj,
            init_state_c,
            ctypes.c_float(0.0),
            ctypes.c_float(float(t)),
        )
        states[i] = [st.data[j] for j in range(6)]
    return times, states


# ============================================================================
# Robot animation
# ============================================================================

def animate_robot_trajectory(
    times,
    states,
    robot_radius=0.090,
    title="Robot Trajectory",
    fps=30.0,
    extra_patches=None,
):
    """Animate a robot moving through X/Y space along a pre-sampled trajectory.

    The robot body is drawn as a filled circle. A line from the centre to the
    perimeter indicates the current heading angle θ.

    Parameters
    ----------
    times : array-like, shape (N,)
        Time values in seconds.
    states : array-like, shape (N, 6)
        Robot state ``[x, y, θ, ẋ, ẏ, θ̇]`` at each sample.
    robot_radius : float
        Robot body radius in metres used for drawing (default 0.090 m).
    title : str
        Figure and axes title.
    fps : float
        Animation playback speed in frames per second (default 30).
    extra_patches : list of matplotlib.patches.Patch, optional
        Additional static patches (e.g. an orbit circle, a ball dot) to add
        to the axes before the animation starts.  Patches should be created
        with a ``label`` if they should appear in the legend.

    Returns
    -------
    fig : matplotlib.figure.Figure
    anim : matplotlib.animation.FuncAnimation
        **Keep a reference to** ``anim`` **in the calling scope** to prevent
        garbage collection while the window is open.
    """
    times = np.asarray(times, dtype=np.float64)
    states = np.asarray(states, dtype=np.float64)
    xs = states[:, 0]
    ys = states[:, 1]
    thetas = states[:, 2]

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal")
    ax.set_title(title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.3)

    # Static background: full trajectory path
    ax.plot(
        xs, ys,
        color="lightgray", linewidth=1.2, linestyle="--",
        label="trajectory path", zorder=1,
    )

    # Optional caller-supplied static patches (orbit circle, ball, etc.)
    if extra_patches:
        for patch in extra_patches:
            ax.add_patch(patch)

    # Robot body circle (animated)
    robot_circle = mpatches.Circle(
        (xs[0], ys[0]), robot_radius,
        fill=True, facecolor="steelblue", edgecolor="navy",
        alpha=0.55, linewidth=1.5, zorder=3, label="robot",
    )
    ax.add_patch(robot_circle)

    # Heading indicator: line from centre to perimeter (animated)
    hx0 = xs[0] + robot_radius * np.cos(thetas[0])
    hy0 = ys[0] + robot_radius * np.sin(thetas[0])
    (heading_line,) = ax.plot(
        [xs[0], hx0], [ys[0], hy0],
        color="navy", linewidth=2.5, zorder=4, label="heading",
    )

    # Time readout
    time_text = ax.text(
        0.02, 0.96,
        f"t = {times[0]:.3f} s",
        transform=ax.transAxes, fontsize=10, va="top",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
        zorder=5,
    )

    # Axis limits with generous padding so the robot body is never clipped
    pad = robot_radius * 2.5
    ax.set_xlim(xs.min() - pad, xs.max() + pad)
    ax.set_ylim(ys.min() - pad, ys.max() + pad)
    ax.legend(loc="upper right")

    # -----------------------------------------------------------------------
    # Animation callbacks
    # -----------------------------------------------------------------------

    def _update(frame):
        x = xs[frame]
        y = ys[frame]
        theta = thetas[frame]
        robot_circle.center = (x, y)
        heading_line.set_data(
            [x, x + robot_radius * np.cos(theta)],
            [y, y + robot_radius * np.sin(theta)],
        )
        time_text.set_text(f"t = {times[frame]:.3f} s")
        return robot_circle, heading_line, time_text

    interval_ms = 1000.0 / fps
    anim = animation.FuncAnimation(
        fig,
        _update,
        frames=len(times),
        interval=interval_ms,
        blit=False,
    )

    return fig, anim
