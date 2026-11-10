"""Closed-loop safety sim of the firmware motion output stage.

Mirrors control-board `ControlContext::calc_tracking_setpoints` (TIGERs-style
position P into the velocity setpoint, anti-jitter fade) and the
`BodyController` output stage (friction compensation, output efficiency, accel
clamp, uniform feed-forward current scaling, current slope limit), plus the
wheel-torque motor firmware's P velocity loop and current limits. Trajectories
and kinematics come from the real controls library via the C FFI.

The plant is a rigid body (no wheel slip) with its own "true" friction and
efficiency, so model error is exercised. Run before flashing new controller
logic or uploading new parameters:

    python analysis/feedback_sim.py            # all scenarios, assert bounds
    python analysis/feedback_sim.py --plot     # also plot the 5 cm / 15 deg move
"""

from __future__ import annotations

import argparse
import ctypes
import math
from dataclasses import dataclass, field, replace

import numpy as np

import ateam_controls as ac

DT = 0.001


@dataclass
class Gains:
    pose_fb_xy: tuple = (8.0, 0.3, 0.01)            # K, MAX, ANTI_JITTER (firmware defaults)
    pose_fb_w: tuple = (8.0, 2.0, 0.01, 2.0)        # POS_K, POS_MAX, ANTI_JITTER, VEL_K
    wheel_vel_fb: tuple = (500.0, 0.0, 1.5, 4.0)    # KP mA/(rad/s), KI, AJ rad/s, MAX_CURRENT A
    ff_current_limit: float = 3.0
    slope_a_per_s: float = 500.0
    accel_clamp: tuple = (20.0, 100.0)
    divergence: tuple = (1.0, 1.57, 5.0, 10.0)      # pos lin, pos ang, vel lin, vel ang


@dataclass
class Model:
    """Friction/output model (firmware PHYS_FRICTION_MODEL order)."""
    coulomb_xy: float = 0.0
    viscous_xy: float = 0.0
    coulomb_w: float = 0.0
    viscous_w: float = 0.0
    eff_xy: float = 0.8
    eff_w: float = 0.8


@dataclass
class Plant:
    mass: float = 2.7
    iz: float = 0.008
    kt: float = 0.0335
    coulomb_xy: float = 5.0      # along local y (direction-scaled like the model); ~measured, incl. tether
    viscous_xy: float = 0.5
    coulomb_w: float = 0.05
    viscous_w: float = 0.005
    eff_xy: float = 0.8
    eff_w: float = 0.75


@dataclass
class Result:
    t: np.ndarray
    pose: np.ndarray
    traj_pose: np.ndarray
    vel_cmd: np.ndarray
    current: np.ndarray
    wheel_vel: np.ndarray
    diverged_at: float | None = None
    notes: list = field(default_factory=list)


def _mat(m, rows, cols) -> np.ndarray:
    return np.array(list(m.data), dtype=float).reshape(cols, rows).T


def _v6(v) -> "ac.Vector6C":
    return ac.Vector6C((ctypes.c_float * 6)(*[float(x) for x in v]))


def _v3(v) -> "ac.Vector3C":
    return ac.Vector3C(*[float(x) for x in v])


def _sample(traj):
    st, acc = ac.traj_sample(traj)
    return np.array(list(st.data), dtype=float), np.array([acc.x, acc.y, acc.z], dtype=float)


def wheel_drive_dirs(phys) -> np.ndarray:
    a, b = phys.alpha_rad, phys.beta_rad
    return np.array([
        [-math.cos(a), -math.cos(b), math.cos(b), math.cos(a)],
        [math.sin(a), -math.sin(b), -math.sin(b), math.sin(a)],
    ])


def direction_factor(dirs: np.ndarray, d: np.ndarray) -> float:
    roller = lambda u: float(np.sum(np.abs(u[0] * dirs[1] - u[1] * dirs[0])))
    return roller(d) / roller(np.array([0.0, 1.0]))


def friction_force(dirs, v, c_xy, b_xy, c_w, b_w, smooth=0.0) -> np.ndarray:
    """TIGERs ModelFricGetXY/W (smooth > 0 gives a tanh Coulomb for the plant)."""
    f = np.zeros(3)
    speed = math.hypot(v[0], v[1])
    if smooth > 0.0 or speed > 0.01:
        if speed > 1e-9:
            d = v[:2] / speed
            k = direction_factor(dirs, d)
            mag = math.tanh(speed / smooth) if smooth > 0.0 else 1.0
            f[0] = d[0] * k * (c_xy * mag + b_xy * abs(v[0]))
            f[1] = d[1] * k * (c_xy * mag + b_xy * abs(v[1]))
    w = v[2]
    if smooth > 0.0:
        f[2] = math.tanh(w / (smooth * 10)) * c_w + b_w * w
    elif abs(w) > 1e-3:
        f[2] = math.copysign(c_w + b_w * abs(w), w)
    return f


def rot(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def aj_scale(err: float, thresh: float) -> float:
    return 1.0 if thresh <= 0.0 else min(max(err / thresh, 0.0), 1.0)


def hw_current_limit(w_rads: float) -> float:
    # Wheel-torque firmware: 4 A stall boost (I2T budget not modeled; tests are short).
    return 4.0 if abs(w_rads) < 2.0 * math.pi else 6.3


def simulate(waypoints, durations, traj_params, gains=Gains(), model=Model(), plant=Plant(),
             start=(0.125, 0.0, 0.0), estimate_jump=None) -> Result:
    phys = ac.default_phys_params()
    phys.mass_kg, phys.iz_kgm2, phys.motor_torque_constant_nmpa = plant.mass, plant.iz, plant.kt
    rm = ac.robot_model_new(phys)
    vel2wheel = _mat(ac.robot_model_transform_vel2wheel_local(rm), 4, 3)
    accel2wheel = _mat(ac.robot_model_transform_accel2wheel_local(rm), 4, 3)
    wheel2accel = _mat(ac.robot_model_transform_wheel2accel_local(rm), 3, 4)
    dirs = wheel_drive_dirs(phys)
    inertia = np.array([plant.mass, plant.mass, plant.iz])

    pose = np.array(start, dtype=float)
    vel = np.zeros(3)                     # global
    traj = None
    target = None
    cur_out = np.zeros(4)                 # slope limiter state (A)
    motor_cmd = (np.zeros(4), np.zeros(4))  # (I_ff, w_set) delivered with 1 tick delay
    w_filt = np.zeros(4)
    alpha = 1.0 - math.exp(-2.0 * math.pi * 200.0 * DT)
    kp, ki, aj_w, i_max = gains.wheel_vel_fb

    n = int(round(sum(durations) / DT))
    log = {k: [] for k in ("t", "pose", "traj", "vel_cmd", "cur", "wv")}
    res_notes, diverged_at = [], None
    seg_ends = np.cumsum(durations)

    for k in range(n):
        t = k * DT
        seg = int(np.searchsorted(seg_ends, t, side="right"))
        seg = min(seg, len(waypoints) - 1)
        est_pose = pose.copy()
        if estimate_jump is not None and t >= estimate_jump[0]:
            est_pose += np.array(estimate_jump[1])
        est_vel = vel.copy()

        # --- maneuver: plan on command change, seed from traj state if any ---
        if target is None or tuple(waypoints[seg]) != target:
            target = tuple(waypoints[seg])
            seed = (_sample(traj)[0] if traj is not None
                    else np.concatenate([est_pose, est_vel]))
            traj = ac.traj_from_target_pose(_v6(seed), _v3(target), traj_params)
        tstate, taccel = _sample(traj)
        ac.traj_tick(traj, DT)

        # --- divergence check (firmware brakes; sim stops here) ---
        e_pos = math.hypot(*(tstate[:2] - est_pose[:2]))
        e_ang = abs(math.remainder(tstate[2] - est_pose[2], 2 * math.pi))
        e_vel = math.hypot(*(tstate[3:5] - est_vel[:2]))
        e_w = abs(tstate[5] - est_vel[2])
        dv = gains.divergence
        if diverged_at is None and (e_pos > dv[0] or e_ang > dv[1] or e_vel > dv[2] or e_w > dv[3]):
            diverged_at = t
            res_notes.append(f"divergence recovery at t={t:.3f}s")

        if diverged_at is not None:
            twist_out = np.zeros(3)
            accel_out = np.zeros(3)
        else:
            # --- tracking law (ControlContext::calc_tracking_setpoints) ---
            k_xy, max_xy, aj_xy = gains.pose_fb_xy
            err = tstate[:2] - est_pose[:2]
            fb = err * k_xy
            nfb = float(np.linalg.norm(fb))
            if nfb > max_xy:
                fb *= max_xy / nfb
            s_xy = aj_scale(float(np.linalg.norm(err)), aj_xy)
            k_w, max_w, aj_ww, kv_w = gains.pose_fb_w
            err_w = math.remainder(tstate[2] - est_pose[2], 2 * math.pi)
            fb_w = min(max(err_w * k_w, -max_w), max_w) + (tstate[5] - est_vel[2]) * kv_w
            s_w = aj_scale(abs(err_w), aj_ww)
            twist_out = np.array([tstate[3] + fb[0] * s_xy, tstate[4] + fb[1] * s_xy,
                                  tstate[5] + fb_w * s_w])
            accel_out = np.array([taccel[0] * s_xy, taccel[1] * s_xy, taccel[2] * s_w])

        # --- output stage (BodyController::control_update) ---
        heading = est_pose[2] + est_vel[2] * DT * 0.5
        to_local = rot(-heading)
        tw_l = to_local @ twist_out
        ac_l = to_local @ accel_out
        ff = ac_l + friction_force(dirs, tw_l, model.coulomb_xy, model.viscous_xy,
                                   model.coulomb_w, model.viscous_w) / inertia
        ff = ff / np.array([model.eff_xy, model.eff_xy, model.eff_w])
        ff = np.clip(ff, [-gains.accel_clamp[0]] * 2 + [-gains.accel_clamp[1]],
                     [gains.accel_clamp[0]] * 2 + [gains.accel_clamp[1]])
        target_cur = accel2wheel @ ff / plant.kt
        m = float(np.max(np.abs(target_cur)))
        if m > gains.ff_current_limit:
            target_cur *= gains.ff_current_limit / m
        step = gains.slope_a_per_s * DT
        cur_out += np.clip(target_cur - cur_out, -step, step)
        w_set = vel2wheel @ tw_l
        if diverged_at is not None:
            w_set = np.zeros(4)  # active brake approximated as vel-hold at zero

        # --- motor firmware (previous tick's command) ---
        i_ff, w_cmd = motor_cmd
        motor_cmd = (cur_out.copy(), w_set.copy())
        w_true = vel2wheel @ (rot(-pose[2]) @ vel)
        w_filt += alpha * (w_true - w_filt)
        e = w_cmd - w_filt
        corr = kp * e / 1000.0
        sc = np.where(np.abs(e) < aj_w, np.abs(e) / aj_w, 1.0) if aj_w > 0 else 1.0
        i_tot = np.clip(i_ff + corr * sc, -i_max, i_max)
        lim = np.array([hw_current_limit(w) for w in w_true])
        i_tot = np.clip(i_tot, -lim, lim)

        # --- plant ---
        v_l = rot(-pose[2]) @ vel
        torque = i_tot * plant.kt * np.array([plant.eff_xy] * 4)
        a_l = wheel2accel @ torque
        a_l[2] *= plant.eff_w / plant.eff_xy
        a_l -= friction_force(dirs, v_l, plant.coulomb_xy, plant.viscous_xy,
                              plant.coulomb_w, plant.viscous_w, smooth=0.005) / inertia
        vel = vel + (rot(pose[2]) @ a_l) * DT
        pose = pose + vel * DT

        log["t"].append(t)
        log["pose"].append(pose.copy())
        log["traj"].append(tstate[:3].copy())
        log["vel_cmd"].append(twist_out.copy())
        log["cur"].append(i_tot.copy())
        log["wv"].append(w_true.copy())

    ac.robot_model_free(rm)
    return Result(np.array(log["t"]), np.array(log["pose"]), np.array(log["traj"]),
                  np.array(log["vel_cmd"]), np.array(log["cur"]), np.array(log["wv"]),
                  diverged_at, res_notes)


def small_move_traj_params():
    p = ac.default_traj_params()
    p.max_vel_linear, p.max_accel_linear = 0.3, 1.0
    p.max_vel_angular, p.max_accel_angular = 2.0, 6.0
    return p


HOME = (0.125, 0.0, 0.0)
AWAY = (0.175, 0.0, math.radians(15.0))
SMALL_MOVE = ([HOME, AWAY] * 4, [1.5] * 8)


def summarize(name, r: Result, current_bound=4.0 + 1e-6, rest_window=0.3) -> bool:
    i_max = float(np.max(np.abs(r.current)))
    v_max = float(np.max(np.hypot(r.vel_cmd[:, 0], r.vel_cmd[:, 1])))
    w_max = float(np.max(np.abs(r.vel_cmd[:, 2])))
    rest = r.t >= r.t[-1] - rest_window
    i_rest = float(np.max(np.abs(r.current[rest])))
    final_err = r.pose[-1] - r.traj_pose[-1]
    ok = i_max <= current_bound and np.all(np.isfinite(r.current))
    print(f"{name:42s} |I|max={i_max:5.2f}A  |v_cmd|max={v_max:5.2f}m/s  |w_cmd|max={w_max:5.2f}rad/s  "
          f"|I|rest={i_rest:5.3f}A  final err=({final_err[0]*1e3:+.1f}mm,{final_err[1]*1e3:+.1f}mm,"
          f"{math.degrees(final_err[2]):+.2f}deg)  {'OK' if ok else 'FAIL'}")
    for n in r.notes:
        print(f"    {n}")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()
    tp = small_move_traj_params()
    test_gains = Gains(divergence=(0.05, 0.3, 0.3, 2.0))
    all_ok = True

    r = simulate(*SMALL_MOVE, tp, gains=test_gains)
    all_ok &= summarize("small move, model off, test gains", r)
    small = r

    r = simulate(*SMALL_MOVE, tp, gains=test_gains,
                 model=Model(5.0, 0.5, 0.05, 0.005, 0.8, 0.75))
    all_ok &= summarize("small move, model = plant", r)
    r = simulate(*SMALL_MOVE, tp, gains=test_gains, model=Model(coulomb_xy=4.0))
    all_ok &= summarize("small move, coulomb_xy 4 N only", r)
    r = simulate(*SMALL_MOVE, tp, gains=test_gains, model=Model(coulomb_xy=7.0))
    all_ok &= summarize("small move, coulomb_xy 7 N (over)", r)

    for kp in (100.0, 500.0, 1000.0, 2000.0):
        g = replace(test_gains, wheel_vel_fb=(kp, 0.0, 1.5, 2.0))
        r = simulate(*SMALL_MOVE, tp, gains=g)
        all_ok &= summarize(f"small move, wheel kP={kp:.0f}", r)

    bad = Model(coulomb_xy=10.0, viscous_xy=10.0, coulomb_w=10.0, viscous_w=10.0, eff_xy=0.2, eff_w=0.2)
    r = simulate(*SMALL_MOVE, tp, gains=test_gains, model=bad)
    all_ok &= summarize("worst accepted params (fric 10, eff 0.2)", r)

    r = simulate(*SMALL_MOVE, tp, gains=Gains(pose_fb_xy=(8.0, 2.0, 0.01), pose_fb_w=(8.0, 8.0, 0.01, 2.0)),
                 estimate_jump=(2.0, (0.3, 0.0, 0.0)))
    all_ok &= summarize("vision jump 0.3m, TIGERs fb limits", r)
    r = simulate(*SMALL_MOVE, tp, gains=test_gains, estimate_jump=(2.0, (0.3, 0.0, 0.0)))
    all_ok &= summarize("vision jump 0.3m, test gains + divergence", r)
    r = simulate(*SMALL_MOVE, tp, gains=replace(test_gains, divergence=Gains().divergence),
                 estimate_jump=(2.0, (0.0, 0.0, 1.0)))
    all_ok &= summarize("heading jump 1 rad, test gains, no div.", r)

    print("ALL OK" if all_ok else "FAILURES")
    if args.plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, sharex=True)
        ax[0].plot(small.t, small.pose[:, 0], label="x"); ax[0].plot(small.t, small.traj_pose[:, 0], "--", label="traj x")
        ax[1].plot(small.t, np.degrees(small.pose[:, 2]), label="theta"); ax[1].plot(small.t, np.degrees(small.traj_pose[:, 2]), "--")
        ax[2].plot(small.t, small.current); ax[2].set_ylabel("A")
        for a in ax:
            a.grid(); a.legend()
        plt.show()
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
