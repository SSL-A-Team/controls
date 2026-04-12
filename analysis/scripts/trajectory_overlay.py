"""
Interactive trajectory overlay for kalman_visualize.py.

Adds a scrollable vertical time cursor (arrow keys) to all subplots,
and overlays the optimal bang-bang trajectory computed from the robot's
state estimate at the cursor time toward the commanded pose.

Usage (from kalman_visualize):
    from trajectory_overlay import TrajectoryOverlay
    overlay = TrajectoryOverlay(fig, axs, t, state_est, telemetry)
    overlay.activate()
"""

import numpy as np
import matplotlib.pyplot as plt
from rust_interface import compute_trajectory


# Maps (row, col) in the 3x3 subplot grid to the trajectory DataFrame column.
_TRAJ_COLUMN_MAP = {
    (0, 0): "x",
    (0, 1): "y",
    (0, 2): "theta",
    (1, 0): "xd",
    (1, 1): "yd",
    (1, 2): "thetad",
    (2, 0): "xdd",
    (2, 1): "ydd",
    (2, 2): "thetadd",
}


class TrajectoryOverlay:
    """Manages a vertical time cursor and trajectory overlay on a 3x3 subplot figure.

    Parameters
    ----------
    fig : matplotlib.figure.Figure
    axs : ndarray of Axes, shape (3, 3)
    t : ndarray, shape (N,)
        Time array in seconds.
    state_est : ndarray, shape (N, 6)
        State estimates [x, y, theta, xd, yd, thetad] per time step.
    telemetry : dict-like
        Loaded NPZ telemetry; must contain ``body_control_telemetry__body_cmd`` (N, 3) and
        ``body_control_telemetry__body_control_mode`` (N,).
    """

    # Matplotlib keymap names whose default bindings conflict with overlay keys.
    _CURSOR_STYLE = dict(color="black", linewidth=0.8, linestyle="--", alpha=0.6)
    _TRAJ_STYLE = dict(color="magenta", linewidth=1.5, linestyle="-", alpha=0.85, label="optimal trajectory")

    def __init__(self, fig, axs, t, state_est, telemetry, param_json=None):
        self.fig = fig
        self.axs = axs
        self.t = t
        self.state_est = state_est
        self.body_cmd = telemetry["body_control_telemetry__body_cmd"]
        self.pose_ctrl = (telemetry["body_control_telemetry__body_control_mode"] == 1)  # BCM_GLOBAL_POSE
        self.param_json = param_json

        self._idx = 0
        self._active = False
        self._updating = False

        # Pre-create artists so updates are fast (just set_data / set_xdata).
        self._cursor_lines = {}
        self._traj_lines = {}
        for i in range(3):
            for j in range(3):
                ax = self.axs[i, j]
                vline = ax.axvline(t[0], **self._CURSOR_STYLE, visible=False)
                tline, = ax.plot([], [], **self._TRAJ_STYLE, visible=False)
                self._cursor_lines[(i, j)] = vline
                self._traj_lines[(i, j)] = tline

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def activate(self):
        """Connect keyboard events and show the cursor / trajectory."""
        if self._active:
            return
        self._active = True
        self._click_cid = self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self._xlim_cid = self.fig.canvas.mpl_connect("draw_event", self._on_draw)
        self._last_xlim = self.axs[0, 0].get_xlim()
        # Place cursor at the left edge of the current view.
        self._idx = min(int(np.searchsorted(self.t, self._last_xlim[0], side="left")),
                        len(self.t) - 1)
        self._set_visible(True)
        self._update()
        print(
            "[TrajectoryOverlay] Activated.\n"
            "  Click      move cursor to clicked time\n"
            "  Space      toggle overlay"
        )

    def deactivate(self):
        """Disconnect keyboard/draw events and hide the overlay."""
        if not self._active:
            return
        self._active = False
        self.fig.canvas.mpl_disconnect(self._click_cid)
        self.fig.canvas.mpl_disconnect(self._xlim_cid)
        self._set_visible(False)
        self._hide_traj_lines()
        self.fig.canvas.draw_idle()
        print("[TrajectoryOverlay] Deactivated.")

    def toggle(self):
        """Toggle the overlay on or off."""
        if self._active:
            self.deactivate()
        else:
            self.activate()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _on_click(self, event):
        """Move cursor to the clicked time position."""
        if event.inaxes is None:
            return
        # Only respond to left clicks (button 1).
        if event.button != 1:
            return
        # Find the nearest time index to the clicked x coordinate.
        new_idx = int(np.searchsorted(self.t, event.xdata, side="left"))
        new_idx = np.clip(new_idx, 0, len(self.t) - 1)
        self._idx = new_idx
        self._update()

    def _on_draw(self, event):
        """Snap cursor to window start when the user zooms/pans."""
        if self._updating:
            return
        xlim = self.axs[0, 0].get_xlim()
        if xlim == self._last_xlim:
            return
        self._last_xlim = xlim
        # Find the first time index at or after the left edge of the window.
        new_idx = int(np.searchsorted(self.t, xlim[0], side="left"))
        new_idx = min(new_idx, len(self.t) - 1)
        if new_idx != self._idx:
            self._idx = new_idx
            self._update()

    def _update(self):
        """Reposition the cursor and recompute the trajectory overlay."""
        self._updating = True
        t_cursor = self.t[self._idx]

        # Move all cursor lines.
        for vline in self._cursor_lines.values():
            vline.set_xdata([t_cursor, t_cursor])

        # Compute trajectory if pose control is active at this time step.
        if self.pose_ctrl[self._idx]:
            init_state = self.state_est[self._idx].tolist()
            target_pose = self.body_cmd[self._idx].tolist()

            # Only sample within the visible time window.
            xlim = self.axs[0, 0].get_xlim()
            t_start = max(0.0, xlim[0] - t_cursor)
            t_end = max(0.0, xlim[1] - t_cursor)

            try:
                traj_data = compute_trajectory(
                    init_state, target_pose,
                    param_json=self.param_json,
                    t_start=t_start, t_end=t_end,
                )
                traj_t = traj_data["time"].values + t_cursor  # shift to absolute time
                for (i, j), col in _TRAJ_COLUMN_MAP.items():
                    line = self._traj_lines[(i, j)]
                    line.set_data(traj_t, traj_data[col].values)
                    line.set_visible(True)
            except Exception as exc:
                print(f"[TrajectoryOverlay] trajectory computation failed: {exc}")
                self._hide_traj_lines()
        else:
            self._hide_traj_lines()

        self.fig.canvas.draw_idle()
        self._updating = False

    def _hide_traj_lines(self):
        for line in self._traj_lines.values():
            line.set_data([], [])
            line.set_visible(False)

    def _set_visible(self, visible):
        for vline in self._cursor_lines.values():
            vline.set_visible(visible)
        # Trajectory lines start hidden until the first update.
        if not visible:
            self._hide_traj_lines()
