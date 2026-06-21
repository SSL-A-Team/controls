"""Interactive overlays for the telemetry visualization.

TrajectoryOverlay — scrollable time cursor with bang-bang trajectory overlay.
VelocityOverlay  — sliding-window vision-derived velocity curve.
"""

import ctypes
import numpy as np
import pandas as pd

from ateam_controls import (
    Vector3C,
    Vector6C,
    TrajectoryParams,
    traj_from_target_pose,
    traj_from_target_twist,
    traj_end_time,
    traj_state_at,
    traj_accel_at,
)
from .body import local_to_global_xy


# ============================================================================
# Trajectory sampling helper
# ============================================================================

_TRAJ_COLUMNS = ["time", "x", "y", "theta", "xd", "yd", "thetad", "xdd", "ydd", "thetadd"]


def _sample_trajectory(traj, t_start, t_end):
    """Sample a bang-bang trajectory at 1 ms resolution."""
    end = traj_end_time(traj)
    sample_start = max(0.0, t_start)
    sample_end = min(end, t_end) if not np.isinf(end) else t_end
    if sample_start > sample_end or np.isinf(sample_end):
        return pd.DataFrame(columns=_TRAJ_COLUMNS)

    time_resolution = 0.001
    times = np.arange(sample_start, sample_end + time_resolution * 0.5, time_resolution)
    n = len(times)
    rows = np.empty((n, 10), dtype=np.float32)
    for i, t in enumerate(times):
        st = traj_state_at(traj, ctypes.c_float(t))
        ac = traj_accel_at(traj, ctypes.c_float(t))
        rows[i] = [t, st.data[0], st.data[1], st.data[2],
                      st.data[3], st.data[4], st.data[5],
                      ac.x, ac.y, ac.z]
    return pd.DataFrame(rows, columns=_TRAJ_COLUMNS)


# ============================================================================
# TrajectoryOverlay
# ============================================================================

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

# body_control_mode → skill name
_BCM_GLOBAL_POS = 1
_BCM_GLOBAL_VEL = 2
_BCM_LOCAL_VEL = 3
_BCM_TRAJ_MODES = {_BCM_GLOBAL_POS, _BCM_GLOBAL_VEL, _BCM_LOCAL_VEL}


def _extract_pose_cmd(telemetry):
    """Build an (N, 3) global-position command array from telemetry."""
    prefix = "body_control_telemetry/skill_global_pos/cmd_echo/"
    x = telemetry[prefix + "global_x"]
    y = telemetry[prefix + "global_y"]
    theta = telemetry[prefix + "global_theta"]
    return np.column_stack([x, y, theta])


def _extract_vel_cmd(telemetry, theta):
    """Build an (N, 3) global-frame velocity command array from telemetry.

    For global vel mode the commands are used directly. For local vel mode
    the commands are rotated into the global frame using the robot heading.
    The caller selects which samples to use via the control-mode mask.
    """
    bcm = telemetry["body_control_telemetry/body_control_mode"]

    global_vx = np.zeros(len(bcm), dtype=np.float32)
    global_vy = np.zeros(len(bcm), dtype=np.float32)
    global_omega = np.zeros(len(bcm), dtype=np.float32)

    # Global vel (mode 2)
    mask_gv = bcm == _BCM_GLOBAL_VEL
    if np.any(mask_gv):
        prefix = "body_control_telemetry/skill_global_vel/cmd_echo/"
        global_vx[mask_gv] = telemetry[prefix + "global_xd"][mask_gv]
        global_vy[mask_gv] = telemetry[prefix + "global_yd"][mask_gv]
        global_omega[mask_gv] = telemetry[prefix + "global_omega"][mask_gv]

    # Local vel (mode 3) — rotate to global
    mask_lv = bcm == _BCM_LOCAL_VEL
    if np.any(mask_lv):
        prefix = "body_control_telemetry/skill_local_vel/cmd_echo/"
        local_xd = telemetry[prefix + "local_xd"]
        local_yd = telemetry[prefix + "local_yd"]
        gx, gy = local_to_global_xy(theta, local_xd, local_yd)
        global_vx[mask_lv] = gx[mask_lv]
        global_vy[mask_lv] = gy[mask_lv]
        global_omega[mask_lv] = telemetry[prefix + "local_omega"][mask_lv]

    return np.column_stack([global_vx, global_vy, global_omega])


def _extract_traj_params(telemetry):
    """Build per-timestep trajectory parameters from cmd_echo fields.

    Returns (max_accel_linear, max_accel_angular, max_vel_linear, max_vel_angular)
    as (N,) arrays. Velocity limits are only available from global_pos commands.
    """
    bcm = telemetry["body_control_telemetry/body_control_mode"]
    n = len(bcm)

    max_accel_linear = np.zeros(n, dtype=np.float32)
    max_accel_angular = np.zeros(n, dtype=np.float32)
    max_vel_linear = np.zeros(n, dtype=np.float32)
    max_vel_angular = np.zeros(n, dtype=np.float32)

    # Mode → (prefix, has_vel_limits)
    _mode_prefix = {
        _BCM_GLOBAL_POS: ("body_control_telemetry/skill_global_pos/cmd_echo/", True),
        _BCM_GLOBAL_VEL: ("body_control_telemetry/skill_global_vel/cmd_echo/", False),
        _BCM_LOCAL_VEL:  ("body_control_telemetry/skill_local_vel/cmd_echo/", False),
    }
    for mode, (prefix, has_vel) in _mode_prefix.items():
        mask = bcm == mode
        if not np.any(mask):
            continue
        max_accel_linear[mask] = telemetry[prefix + "max_linear_acc"][mask]
        max_accel_angular[mask] = telemetry[prefix + "max_angular_acc"][mask]
        if has_vel:
            max_vel_linear[mask] = telemetry[prefix + "max_linear_vel"][mask]
            max_vel_angular[mask] = telemetry[prefix + "max_angular_vel"][mask]

    return max_accel_linear, max_accel_angular, max_vel_linear, max_vel_angular


class TrajectoryOverlay:
    """Manages a vertical time cursor and trajectory overlay on a 3x3 subplot figure."""

    _CURSOR_STYLE = dict(color="black", linewidth=0.8, linestyle="--", alpha=0.6)
    _TRAJ_STYLE = dict(color="magenta", linewidth=1.5, linestyle="-", alpha=0.85)

    def __init__(self, fig, axs, t, state_est, telemetry, param_json=None):
        self.fig = fig
        self.axs = axs
        self.t = t
        self.state_est = state_est
        self.bcm = telemetry["body_control_telemetry/body_control_mode"]
        self.pose_cmd = _extract_pose_cmd(telemetry)
        self.vel_cmd = _extract_vel_cmd(telemetry, state_est[:, 2])
        self.traj_params = _extract_traj_params(telemetry)
        self.param_json = param_json

        self._idx = 0
        self._active = False
        self._updating = False

        self._cursor_lines = {}
        self._traj_lines = {}
        for i in range(3):
            for j in range(3):
                ax = self.axs[i, j]
                vline = ax.axvline(t[0], **self._CURSOR_STYLE, visible=False)
                tline, = ax.plot([], [], **self._TRAJ_STYLE, label="_nolegend_", visible=False)
                self._cursor_lines[(i, j)] = vline
                self._traj_lines[(i, j)] = tline

    def activate(self):
        if self._active:
            return
        self._active = True
        self._click_cid = self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self._xlim_cid = self.fig.canvas.mpl_connect("draw_event", self._on_draw)
        self._last_xlim = self.axs[0, 0].get_xlim()
        self._idx = min(int(np.searchsorted(self.t, self._last_xlim[0], side="left")),
                        len(self.t) - 1)
        self._set_visible(True)
        for line in self._traj_lines.values():
            line.set_label("optimal trajectory")
        self._refresh_legends()
        self._update()
        print(
            "[TrajectoryOverlay] Activated.\n"
            "  Click      move cursor to clicked time\n"
            "  Space      toggle overlay"
        )

    def deactivate(self):
        if not self._active:
            return
        self._active = False
        self.fig.canvas.mpl_disconnect(self._click_cid)
        self.fig.canvas.mpl_disconnect(self._xlim_cid)
        self._set_visible(False)
        self._hide_traj_lines()
        for line in self._traj_lines.values():
            line.set_label("_nolegend_")
        self._refresh_legends()
        self.fig.canvas.draw_idle()
        print("[TrajectoryOverlay] Deactivated.")

    def toggle(self):
        if self._active:
            self.deactivate()
        else:
            self.activate()

    def _on_click(self, event):
        if event.inaxes is None:
            return
        if event.button != 1:
            return
        new_idx = int(np.searchsorted(self.t, event.xdata, side="left"))
        new_idx = np.clip(new_idx, 0, len(self.t) - 1)
        self._idx = new_idx
        self._update()

    def _on_draw(self, event):
        if self._updating:
            return
        xlim = self.axs[0, 0].get_xlim()
        if xlim == self._last_xlim:
            return
        self._last_xlim = xlim
        new_idx = int(np.searchsorted(self.t, xlim[0], side="left"))
        new_idx = min(new_idx, len(self.t) - 1)
        if new_idx != self._idx:
            self._idx = new_idx
            self._update()

    def _update(self):
        self._updating = True
        t_cursor = self.t[self._idx]

        for vline in self._cursor_lines.values():
            vline.set_xdata([t_cursor, t_cursor])

        mode = self.bcm[self._idx]
        if mode in _BCM_TRAJ_MODES:
            init_state = self.state_est[self._idx].tolist()

            xlim = self.axs[0, 0].get_xlim()
            t_start = max(0.0, xlim[0] - t_cursor)
            t_end = max(0.0, xlim[1] - t_cursor)

            max_al, max_aa, max_vl, max_va = [a[self._idx] for a in self.traj_params]
            params = TrajectoryParams(
                max_vel_linear=float(max_vl),
                max_vel_angular=float(max_va),
                max_accel_linear=float(max_al),
                max_accel_angular=float(max_aa),
            )

            try:
                state_c = Vector6C(data=(ctypes.c_float * 6)(*[float(v) for v in init_state]))

                if mode == _BCM_GLOBAL_POS:
                    tp = self.pose_cmd[self._idx]
                    target_c = Vector3C(x=float(tp[0]), y=float(tp[1]), z=float(tp[2]))
                    traj = traj_from_target_pose(state_c, target_c, params)
                else:
                    tw = self.vel_cmd[self._idx]
                    target_tw = Vector3C(x=float(tw[0]), y=float(tw[1]), z=float(tw[2]))
                    traj = traj_from_target_twist(state_c, target_tw, params)

                traj_data = _sample_trajectory(traj, t_start, t_end)
                traj_t = traj_data["time"].values + t_cursor
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
        if not visible:
            self._hide_traj_lines()

    def _refresh_legends(self):
        for i in range(3):
            for j in range(3):
                self.axs[i, j].legend()


# ============================================================================
# VelocityOverlay
# ============================================================================

_VEL_LINE_STYLE = dict(color="lime", linewidth=1.5, linestyle="-", alpha=0.85, label="vision_slope_vel")


def _compute_sliding_velocity(vision_t, vision_pos, window):
    """Compute OLS slope at each vision time over a centred window."""
    M = len(vision_t)
    vel = np.full((M, 3), np.nan)
    half = window / 2.0

    for i in range(M):
        t_c = vision_t[i]
        lo = np.searchsorted(vision_t, t_c - half, side="left")
        hi = np.searchsorted(vision_t, t_c + half, side="right")
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
    """Plots a vision-derived sliding-window velocity curve."""

    _WINDOW_STEP = 0.05

    def __init__(self, fig, axs, t, telemetry, window=0.2):
        self.fig = fig
        self.axs = axs
        self.t = t
        self.window = window

        self.vision_mask = telemetry["body_control_telemetry/vision_update"].astype(bool)
        self.vision_t = t[self.vision_mask]
        self.vision_pos = telemetry["body_control_telemetry/vision_pose"][self.vision_mask]

        self._active = False

        self._vel_lines = {}
        for j in range(3):
            ax = self.axs[1, j]
            line, = ax.plot([], [], **_VEL_LINE_STYLE, visible=False)
            self._vel_lines[j] = line

        self._window_text = self.fig.text(
            0.01, 0.01, "", fontsize=9, color="lime",
            visible=False, transform=self.fig.transFigure,
        )

        self._recompute()

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
        self._vel = _compute_sliding_velocity(
            self.vision_t, self.vision_pos, self.window
        )

    def _show(self):
        for j in range(3):
            line = self._vel_lines[j]
            line.set_data(self.vision_t, self._vel[:, j])
            line.set_visible(True)
            self.axs[1, j].legend()
        self._window_text.set_text(f"vel window: {self.window:.2f}s")
        self._window_text.set_visible(True)

    def _hide(self):
        for j in range(3):
            self._vel_lines[j].set_data([], [])
            self._vel_lines[j].set_visible(False)
            self.axs[1, j].legend()
        self._window_text.set_visible(False)
