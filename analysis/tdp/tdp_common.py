"""Shared helpers for the TDP 2027 figure scripts.

* A numpy mirror of the firmware's robot model and feed-forward current budget
  (ateam-controls/src/robot_model.rs, control-board/src/motion/body_controller.rs,
  control_context.rs::feasible_linear_accel). Keep the constants below in sync with
  defaults.rs / controller_params.rs if they change.
* Column layout of the small_move_test logs and of telem_record recordings.
* Matplotlib style sized for the LNCS/LNAI text block (12.2 cm wide).
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Robot model (ateam-controls/src/defaults.rs, robot_model.rs)
# ---------------------------------------------------------------------------
ALPHA, BETA = math.pi / 6, math.pi / 4  # DEFAULT_PHYS_ALPHA / BETA
L_M, R_M = 0.0814, 0.030                # DEFAULT_PHYS_L / R
MASS, IZ = 2.7, 0.008                   # DEFAULT_PHYS_MASS / IZ
KT = 0.0335                             # DEFAULT_PHYS_MOTOR_TORQUE_CONSTANT (N m / A)
COULOMB_XY, COULOMB_W = 4.5, 0.1        # DEFAULT_PHYS_COULOMB_XY / W
EFF_XY, EFF_W = 0.75, 0.8               # DEFAULT_PHYS_EFFICIENCY_XY / W
FRICTION_XY_RAMP_MPS, FRICTION_W_RAMP_RADPS = 0.05, 0.2

# controller_params.rs, wheel_thermal.rs, control_context.rs
FF_LIMIT_A = 3.5                 # WHEEL_FF_CURRENT_LIMIT_A
FF_DERATED_A = 1.9               # FF_CONTINUOUS_CURRENT_A
FEASIBLE_ACCEL_MARGIN = 0.9
MIN_PLANNED_ACCEL_LINEAR = 0.3

WHEEL_NAMES = ("FL", "BL", "BR", "FR")

# Columns are D_i over l: FL, BL, BR, FR (robot_model.rs update_params)
WHEEL_MAT = np.array([
    [-math.cos(ALPHA), -math.cos(BETA), math.cos(BETA), math.cos(ALPHA)],
    [math.sin(ALPHA), -math.sin(BETA), -math.sin(BETA), math.sin(ALPHA)],
    [L_M, L_M, L_M, L_M],
])
INERTIA = np.diag([MASS, MASS, IZ])
# local accel -> wheel torque (N m): pinv(W^T)^T * I * r
ACCEL2WHEEL = np.linalg.pinv(WHEEL_MAT.T).T @ INERTIA * R_M
# local twist -> wheel speed (rad/s)
VEL2WHEEL = WHEEL_MAT.T / R_M


def roller_speed(d):
    D = WHEEL_MAT[:2]
    return float(np.abs(d[0] * D[1] - d[1] * D[0]).sum())


ROLLER_REF = roller_speed((0.0, 1.0))


def k_dir(d):
    """friction_direction_factor: roller speed along unit local d, 1 for sideways."""
    d = np.asarray(d, float)
    return roller_speed(d / np.linalg.norm(d)) / ROLLER_REF


def friction_force(v_local, coulomb_xy=COULOMB_XY, coulomb_w=COULOMB_W):
    """RobotModel::friction_comp_force (viscous terms are 0 in the defaults)."""
    vx, vy, w = v_local
    s = math.hypot(vx, vy)
    fx = fy = 0.0
    if s > 0:
        d = np.array([vx, vy]) / s
        c = min(s / FRICTION_XY_RAMP_MPS, 1.0) * coulomb_xy * k_dir(d)
        fx, fy = d * c
    tz = math.copysign(min(abs(w) / FRICTION_W_RAMP_RADPS, 1.0) * coulomb_w, w) if w else 0.0
    return np.array([fx, fy, tz])


def ff_accel(v_local, a_local, eff_xy=EFF_XY, eff_w=EFF_W, **fric):
    """RobotModel::feedforward_local_accel."""
    acc = np.asarray(a_local, float) + np.linalg.solve(INERTIA, friction_force(v_local, **fric))
    return acc / np.array([eff_xy, eff_xy, eff_w])


def accel_to_current(a_local):
    return ACCEL2WHEEL @ np.asarray(a_local, float) / KT


def feasible_linear_accel(dir_local, limit=FF_LIMIT_A, **model):
    """ControlContext::feasible_linear_accel: the largest linear acceleration along
    dir_local that every wheel can add on top of full Coulomb compensation."""
    u = np.array([*np.asarray(dir_local, float) / np.linalg.norm(dir_local), 0.0])
    i_fric = accel_to_current(ff_accel(u * 0.5, np.zeros(3), **model))
    i_unit = accel_to_current(ff_accel(np.zeros(3), u, **model))
    limits = np.broadcast_to(np.asarray(limit, float), (4,))
    s = math.inf
    for i in range(4):
        if abs(i_unit[i]) < 1e-6:
            continue
        head = limits[i] - i_fric[i] if i_unit[i] > 0 else limits[i] + i_fric[i]
        s = min(s, head / abs(i_unit[i]))
    return max(s, 0.0)


def budget_direction_preserving(i_fric, i_accel, limits):
    """body_controller.rs: friction first, then one scale s on the acceleration."""
    limits = np.broadcast_to(np.asarray(limits, float), (4,))
    fs = min([1.0] + [limits[i] / abs(i_fric[i]) for i in range(4) if abs(i_fric[i]) > limits[i]])
    if fs < 1.0:
        return i_fric * fs
    s = 1.0
    for i in range(4):
        head = limits[i] - i_fric[i] if i_accel[i] > 0 else limits[i] + i_fric[i]
        if abs(i_accel[i]) * s > head:
            s = head / abs(i_accel[i])
    return i_fric + i_accel * min(max(s, 0.0), 1.0)


def budget_per_wheel_clip(i_fric, i_accel, limits):
    """The naive alternative: clip each wheel's total current independently."""
    limits = np.broadcast_to(np.asarray(limits, float), (4,))
    return np.clip(i_fric + i_accel, -limits, limits)


MOTOR_CONTINUOUS_A, MOTOR_I2T_BUDGET_A2S = 2.16, 11.3  # wheel_thermal.rs
THERMAL_BUDGET_FRACTION, THERMAL_DERATE_START = 0.8, 0.75


def thermal_replay(t, currents, peak=FF_LIMIT_A):
    """Replay the control board's WheelThermalBudget (wheel_thermal.rs) on logged wheel
    currents (N, 4). Returns (used fraction (N, 4), FF limit A (N, 4)). The firmware
    integrates the applied current every 1 ms; at 100 Hz telemetry this is approximate."""
    budget = THERMAL_BUDGET_FRACTION * MOTOR_I2T_BUDGET_A2S
    start = THERMAL_DERATE_START * budget
    cont = min(FF_DERATED_A, peak)
    used = np.zeros(4)
    used_out, lim_out = np.zeros((len(t), 4)), np.zeros((len(t), 4))
    dts = np.diff(t, prepend=t[0])
    for k, (dt, i) in enumerate(zip(dts, np.asarray(currents, float))):
        used = np.clip(used + (i ** 2 - MOTOR_CONTINUOUS_A ** 2) * min(max(dt, 0.0), 0.1), 0.0, budget)
        frac = np.clip((used - start) / (budget - start), 0.0, 1.0)
        used_out[k], lim_out[k] = used / budget, peak + (cont - peak) * frac
    return used_out, lim_out


def currents_to_local_force(I):
    """Wheel currents (..., 4) -> motor-side local [Fx, Fy, Tz] (N, N m)."""
    return (np.asarray(I, float) * KT / R_M) @ WHEEL_MAT.T


# ---------------------------------------------------------------------------
# Log formats
# ---------------------------------------------------------------------------
# small_move_test.py row layout (and its subclasses: the first 39 columns)
SM = dict(t=0, pos=slice(1, 4), vel=slice(4, 7), gyro=7, cur=slice(8, 12), wv=slice(12, 16),
          traj=slice(16, 19), vel_u=slice(19, 22), acc_u=slice(22, 25), target=slice(25, 28),
          vis=28, bcm=29, vel_sp=slice(30, 34), mct=slice(34, 38), t_robot=38)


def load_small_move(path):
    d = np.load(path, allow_pickle=True)
    a = np.array([np.asarray(r, float)[:39] for r in d["data"]])
    return a, str(d["tripped"]) if "tripped" in d else "None"


def slip_residual(wheel_speeds):
    """Wheel speed outside the rigid-body solution (null space of the 4->3 map), m/s
    at the wheel surface. Four wheels over-determine the 3-DOF twist, so any
    residual is slip (or encoder error)."""
    W = np.asarray(wheel_speeds, float)
    fit = W @ np.linalg.pinv(VEL2WHEEL).T @ VEL2WHEEL.T
    return (W - fit) * R_M


def load_json(path):
    return json.loads(Path(path).read_text())


# ---------------------------------------------------------------------------
# Figure style (LNCS text block: 12.2 cm = 4.8 in)
# ---------------------------------------------------------------------------
LNCS_WIDTH_IN = 4.8


def lncs_style():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 8, "legend.fontsize": 7,
        "xtick.labelsize": 7, "ytick.labelsize": 7, "lines.linewidth": 1.2,
        "axes.grid": True, "grid.alpha": 0.3, "savefig.bbox": "tight", "savefig.dpi": 300,
        "font.family": "serif", "pdf.fonttype": 42, "figure.constrained_layout.use": True,
    })
    return plt


def save(fig, out_dir, name):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    for ext in ("pdf", "png"):
        fig.savefig(out / f"{name}.{ext}")
    print(f"wrote {out / name}.pdf/.png")
