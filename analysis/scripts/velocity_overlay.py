"""
Sliding-window vision-derived velocity overlay for kalman_visualize.py.

For every vision-update time, computes the OLS best-fit slope of the
position measurements inside a centred window of configurable width,
producing a continuous velocity curve plotted on the velocity row
(row 1) of the 3x3 subplot grid.

Toggled together with TrajectoryOverlay via the Space key.

Window size is adjustable at runtime with '[' / ']' keys or set
from the CLI via --velocity-window.
"""

import numpy as np


_LINE_STYLE = dict(color="lime", linewidth=1.5, linestyle="-", alpha=0.85)


def _compute_sliding_velocity(vision_t, vision_pos, window):
    """Compute OLS slope at each vision time over a centred window.

    Parameters
    ----------
    vision_t : ndarray, shape (M,)
        Sorted times of vision measurements.
    vision_pos : ndarray, shape (M, 3)
        Position (x, y, theta) at each vision time.
    window : float
        Total window width in seconds (centred on each evaluation point).

    Returns
    -------
    vel : ndarray, shape (M, 3)
        Estimated velocity at each vision time.  NaN where fewer than
        2 samples fall inside the window.
    """
    M = len(vision_t)
    vel = np.full((M, 3), np.nan)
    half = window / 2.0

    for i in range(M):
        t_c = vision_t[i]
        lo = np.searchsorted(vision_t, t_c - half, side="left")
        hi = np.searchsorted(vision_t, t_c + half, side="right")  # exclusive
        if hi - lo < 2:
            continue
        t_win = vision_t[lo:hi]
        n = len(t_win)
        t_mean = t_win.mean()
        denom = (t_win * t_win).sum() - n * t_mean * t_mean
        if abs(denom) < 1e-30:
            continue
        for j in range(3):
            y_win = vision_pos[lo:hi, j]
            y_mean = y_win.mean()
            vel[i, j] = ((t_win * y_win).sum() - n * t_mean * y_mean) / denom

    return vel


class VelocityOverlay:
    """Plots a vision-derived sliding-window velocity curve.

    Parameters
    ----------
    fig : matplotlib.figure.Figure
    axs : ndarray of Axes, shape (3, 3)
    t : ndarray, shape (N,)
        Full time array in seconds.
    telemetry : dict-like
        Loaded NPZ telemetry; must contain ``vision_pose`` (N, 3) and
        ``vision_update`` (N,).
    window : float
        Initial sliding window size in seconds.
    """

    _WINDOW_STEP = 0.05  # seconds added/removed per key press

    def __init__(self, fig, axs, t, telemetry, window=0.2):
        self.fig = fig
        self.axs = axs
        self.t = t
        self.window = window

        # Vision data at frames where vision fired.
        self.vision_mask = telemetry["vision_update"].astype(bool)
        self.vision_t = t[self.vision_mask]
        self.vision_pos = telemetry["vision_pose"][self.vision_mask]  # (M, 3)

        self._active = False

        # Pre-create line artists on the velocity row.
        self._vel_lines = {}
        for j in range(3):
            ax = self.axs[1, j]
            line, = ax.plot([], [], **_LINE_STYLE, label="vision_slope_vel", visible=False)
            self._vel_lines[j] = line

        self._window_text = self.fig.text(
            0.01, 0.01, "", fontsize=9, color="lime",
            visible=False, transform=self.fig.transFigure,
        )

        # Eagerly compute the initial curve.
        self._recompute()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def activate(self):
        if self._active:
            return
        self._active = True
        self._key_cid = self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self._show()
        self.fig.canvas.draw_idle()

    def deactivate(self):
        if not self._active:
            return
        self._active = False
        self.fig.canvas.mpl_disconnect(self._key_cid)
        self._hide()
        self.fig.canvas.draw_idle()

    def toggle(self):
        if self._active:
            self.deactivate()
        else:
            self.activate()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _on_key(self, event):
        if event.key == "[":
            self.window = max(self._WINDOW_STEP, self.window - self._WINDOW_STEP)
            self._recompute()
            self._show()
            self.fig.canvas.draw_idle()
        elif event.key == "]":
            self.window += self._WINDOW_STEP
            self._recompute()
            self._show()
            self.fig.canvas.draw_idle()

    def _recompute(self):
        """Recompute the full velocity curve with the current window."""
        self._vel = _compute_sliding_velocity(
            self.vision_t, self.vision_pos, self.window
        )

    def _show(self):
        for j in range(3):
            line = self._vel_lines[j]
            line.set_data(self.vision_t, self._vel[:, j])
            line.set_visible(True)
        self._window_text.set_text(f"vel window: {self.window:.2f}s")
        self._window_text.set_visible(True)

    def _hide(self):
        for j in range(3):
            self._vel_lines[j].set_data([], [])
            self._vel_lines[j].set_visible(False)
        self._window_text.set_visible(False)
