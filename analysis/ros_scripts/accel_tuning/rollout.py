"""Roll-out (coast-down) friction tuning routine.

Spins the robot up along one axis with an open-loop acceleration pulse, then
coasts (zero commanded acceleration, friction compensation for the axis
turned off) until it stops. The deceleration measured during the coast is the
friction acting on the robot, as a function of speed. Fitting it over several
trials gives the full friction curve for the axis::

    F(s) = c_coul + (c_static - c_coul) * exp(-(s / v_stribeck)^2) + c_visc * s

expressed in the firmware's commanded-force units: ``F = I * decel / k``, with
``I`` the model mass (or ``iz``) and ``k`` the linear friction direction
factor for the axis (1 for ``x`` and ``theta``). This assumes the model's
accel-to-motion gain is already tuned (``eta`` for linear axes, ``iz`` for
``theta``), so a commanded friction force of ``I * decel`` cancels the real
friction.

The fit writes ``c_coul``/``c_visc`` into ``PHYS_FRICTION_MODEL`` and
``c_static``/``v_stribeck`` into ``PHYS_STRIBECK_MODEL`` for the axis.
"""

from __future__ import annotations

import argparse
import math
from typing import Optional

import numpy as np
import rclpy

from .base import (
    ANGULAR_AXES,
    BaseTuneNode,
    PHASE_COAST,
    PHASE_PULSE_POS,
    PHASE_REST,
    TurnaroundTimeout,
)
from .checkpoint import CheckpointDir, TrialRecord, save_trial, write_json
from .coulomb import (
    axis_friction_factor,
    axis_friction_indices,
    axis_inertia,
    axis_stribeck_indices,
    empty_stribeck_array,
)
from ._search import log_lines


# ===== EDIT ME =====
DEFAULT_TRIALS = 5
DEFAULT_T_SPINUP = 0.5          # s of +amplitude before coasting
DEFAULT_T_COAST = 2.0           # s of coast (long enough to stop)
DEFAULT_T_REST = 0.5            # s at BCM_OFF after the coast
DEFAULT_V_FIT_MIN_LIN = 0.02    # m/s, ignore slower coast samples
DEFAULT_V_FIT_MIN_ANG = 0.2     # rad/s
DEFAULT_DERIV_HALF_WINDOW_S = 0.04  # s, half-width of the local slope fit
DEFAULT_STRIBECK_GRID = 40      # v_stribeck candidates in the fit
# ===================


class RolloutRunner:
    """Tick-driven spin-up → coast → rest profile for one trial."""

    def __init__(self, node: BaseTuneNode, amplitude: float,
                 t_spinup: float, t_coast: float, t_rest: float):
        self.node = node
        self.amplitude = float(amplitude)
        self.t_spinup = float(t_spinup)
        self.t_coast = float(t_coast)
        self.t_rest = float(t_rest)
        self._t0: Optional[float] = None
        self.done = False

    def tick(self) -> bool:
        if self.done:
            self.node.publish_off()
            return True
        t = self.node.trial_elapsed()
        if self._t0 is None:
            self._t0 = t
        dt = t - self._t0
        if dt < self.t_spinup:
            self.node.set_phase(PHASE_PULSE_POS)
            self.node.publish_accel(self.amplitude)
        elif dt < self.t_spinup + self.t_coast:
            self.node.set_phase(PHASE_COAST)
            self.node.publish_accel(0.0)
        else:
            self.node.set_phase(PHASE_REST)
            self.node.publish_off()
            if dt >= self.t_spinup + self.t_coast + self.t_rest:
                self.done = True
        return self.done


def local_slope(t: np.ndarray, v: np.ndarray, half_window_s: float) -> np.ndarray:
    """Per-sample dv/dt from a linear fit over a +/- ``half_window_s`` window."""
    a = np.full(t.shape, np.nan)
    for i in range(t.size):
        m = np.abs(t - t[i]) <= half_window_s
        if m.sum() >= 3:
            a[i] = np.polyfit(t[m], v[m], 1)[0]
    return a


def _nonneg_lstsq(cols: list[np.ndarray], y: np.ndarray) -> tuple[np.ndarray, float]:
    """Least squares with nonnegative coefficients by trying every column subset."""
    n = len(cols)
    best_coef = np.zeros(n)
    best_sse = float(np.sum(y * y))
    for mask in range(1, 1 << n):
        idx = [j for j in range(n) if mask >> j & 1]
        a = np.stack([cols[j] for j in idx], axis=1)
        coef, *_ = np.linalg.lstsq(a, y, rcond=None)
        if np.any(coef < 0):
            continue
        sse = float(np.sum((a @ coef - y) ** 2))
        if sse < best_sse:
            best_sse = sse
            best_coef = np.zeros(n)
            best_coef[idx] = coef
    return best_coef, best_sse


def fit_friction_curve(speed: np.ndarray, force: np.ndarray,
                       grid_n: int = DEFAULT_STRIBECK_GRID) -> dict:
    """Fit ``force(speed)`` to coulomb + stribeck + viscous (all >= 0).

    Grid-searches ``v_stribeck``; for each, the model is linear in
    ``[c_coul, c_static - c_coul, c_visc]``. ``v_stribeck = 0`` means no
    Stribeck term.
    """
    ones = np.ones_like(speed)
    coef, sse = _nonneg_lstsq([ones, speed], force)
    best = {"c_coul": coef[0], "delta": 0.0, "c_visc": coef[1],
            "v_stribeck": 0.0, "sse": sse}
    s_lo, s_hi = float(np.min(speed)), float(np.max(speed))
    for v_s in np.geomspace(max(s_lo, 1e-3) / 2.0, s_hi, grid_n):
        decay = np.exp(-(speed / v_s) ** 2)
        coef, sse = _nonneg_lstsq([ones, decay, speed], force)
        if sse < best["sse"] and coef[1] > 0:
            best = {"c_coul": coef[0], "delta": coef[1], "c_visc": coef[2],
                    "v_stribeck": float(v_s), "sse": sse}
    n = max(speed.size, 1)
    return {
        "c_coul": float(best["c_coul"]),
        "c_static": float(best["c_coul"] + best["delta"]),
        "v_stribeck": float(best["v_stribeck"]),
        "c_visc": float(best["c_visc"]),
        "rmse": float(math.sqrt(best["sse"] / n)),
        "n_samples": int(speed.size),
    }


def binned_curve(speed: np.ndarray, force: np.ndarray, n_bins: int = 12) -> list[dict]:
    if speed.size == 0:
        return []
    edges = np.linspace(float(np.min(speed)), float(np.max(speed)), n_bins + 1)
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (speed >= lo) & (speed <= hi)
        if m.any():
            out.append({"speed": float(np.mean(speed[m])),
                        "force_mean": float(np.mean(force[m])),
                        "force_std": float(np.std(force[m])),
                        "n": int(m.sum())})
    return out


class RolloutRoutine(BaseTuneNode):
    """Spin up, coast with no friction compensation, fit friction vs speed."""

    def __init__(self, robot_id: int, axis: str, amplitude: float,
                 trials: int, v_fit_min: Optional[float] = None,
                 dry_run: bool = False, skip_turnaround: bool = False):
        super().__init__("rollout_tune", robot_id, axis, dry_run=dry_run,
                         skip_turnaround=skip_turnaround)
        self.amplitude = float(amplitude)
        self.trials = int(trials)
        self.v_fit_min = (float(v_fit_min) if v_fit_min is not None
                          else DEFAULT_V_FIT_MIN_ANG if axis in ANGULAR_AXES
                          else DEFAULT_V_FIT_MIN_LIN)

    def _test_params(self, params: dict) -> dict:
        """Baseline params with the axis's friction compensation removed."""
        coul_idx, visc_idx = axis_friction_indices(self.axis)
        static_idx, vel_idx = axis_stribeck_indices(self.axis)
        fric = list(params["PHYS_FRICTION_MODEL"])
        fric[coul_idx] = 0.0
        fric[visc_idx] = 0.0
        strib = list(params.get("PHYS_STRIBECK_MODEL", empty_stribeck_array()))
        strib[static_idx] = 0.0
        strib[vel_idx] = 0.0
        return {**params, "PHYS_FRICTION_MODEL": fric, "PHYS_STRIBECK_MODEL": strib}

    def _run_trial(self) -> None:
        self.wait_for_vision()
        self.begin_trial()
        t_spinup = self.t_pulse if self.t_pulse is not None else DEFAULT_T_SPINUP
        t_coast = self.t_coast if self.t_coast is not None else DEFAULT_T_COAST
        runner = RolloutRunner(self, self.amplitude, t_spinup, t_coast, DEFAULT_T_REST)
        original_on_tick = self.on_tick
        self.on_tick = runner.tick  # type: ignore[assignment]
        try:
            while rclpy.ok() and not runner.done:
                rclpy.spin_once(self, timeout_sec=1.0 / self.PUBLISH_RATE_HZ)
        finally:
            self.on_tick = original_on_tick  # type: ignore[assignment]
            self.publish_off()

    def _coast_samples(self) -> tuple[np.ndarray, np.ndarray]:
        """(speed, friction force) samples from this trial's coast."""
        t, v = self.buffer.axis_velocity(PHASE_COAST, self.axis)
        if t.size < 3:
            return np.zeros(0), np.zeros(0)
        a = local_slope(t, v, DEFAULT_DERIV_HALF_WINDOW_S)
        direction = math.copysign(1.0, self.amplitude)
        moving = (direction * v > self.v_fit_min) & np.isfinite(a)
        speed = direction * v[moving]
        decel = -direction * a[moving]
        return speed, decel

    def run(self, ckpt: CheckpointDir, params: dict) -> dict:
        params = dict(params)
        inertia = axis_inertia(self.axis, params)
        k = axis_friction_factor(self.axis, params)
        self.push_params(self._test_params(params),
                         only_keys=["PHYS_INERTIA", "PHYS_MOTOR_MODEL",
                                    "PHYS_FRICTION_MODEL", "PHYS_STRIBECK_MODEL"])
        speeds: list[np.ndarray] = []
        forces: list[np.ndarray] = []
        aborted = False
        try:
            for i in range(self.trials):
                log_lines(self.get_logger(), [
                    f"▶ ROLLOUT · {self.axis} · TRIAL {i:04d} "
                    f"(spin-up {self.amplitude}, coast, no friction comp)"])
                self._run_trial()
                speed, decel = self._coast_samples()
                force = inertia * decel / k
                speeds.append(speed)
                forces.append(force)
                record = TrialRecord(
                    routine="rollout", axis=self.axis, trial_idx=i,
                    candidate={"amplitude": float(self.amplitude)},
                    metrics={
                        "n_fit_samples": int(speed.size),
                        "max_coast_speed": float(np.max(speed)) if speed.size else 0.0,
                        "mean_friction_force": float(np.mean(force)) if force.size else None,
                    },
                    accepted=True,
                    notes="rollout coast (friction comp off for this axis)",
                )
                save_trial(ckpt, record, telemetry=self.buffer.to_arrays())
                self.get_logger().info(
                    f"  coast samples={speed.size}  "
                    f"max speed={np.max(speed) if speed.size else 0:.3f}  "
                    f"mean friction={np.mean(force) if force.size else float('nan'):.4f}"
                )
                if self.axis not in ANGULAR_AXES:
                    self.wait_for_turnaround()
                else:
                    self.wait_for_turnaround(prompt=False)
        except TurnaroundTimeout as exc:
            self.get_logger().error(f"rollout aborted: {exc}")
            aborted = True

        speed = np.concatenate(speeds) if speeds else np.zeros(0)
        force = np.concatenate(forces) if forces else np.zeros(0)
        result: dict = {
            "axis": self.axis,
            "amplitude": float(self.amplitude),
            "aborted": aborted,
            "inertia_used": inertia,
            "direction_factor": k,
            "v_fit_min": self.v_fit_min,
            "params_used": params,
        }
        if speed.size < 10:
            result["fit"] = None
            self.get_logger().warn("rollout: too few coast samples to fit")
        else:
            fit = fit_friction_curve(speed, force)
            result["fit"] = fit
            result["curve"] = binned_curve(speed, force)
            coul_idx, visc_idx = axis_friction_indices(self.axis)
            static_idx, vel_idx = axis_stribeck_indices(self.axis)
            fric = list(params["PHYS_FRICTION_MODEL"])
            fric[coul_idx] = fit["c_coul"]
            fric[visc_idx] = fit["c_visc"]
            strib = list(params.get("PHYS_STRIBECK_MODEL", empty_stribeck_array()))
            strib[static_idx] = fit["c_static"]
            strib[vel_idx] = fit["v_stribeck"]
            result["params_after"] = {**params, "PHYS_FRICTION_MODEL": fric,
                                      "PHYS_STRIBECK_MODEL": strib}
            self.get_logger().info(
                f"[rollout][{self.axis}] fit: c_coul={fit['c_coul']:.4f} "
                f"c_static={fit['c_static']:.4f} v_stribeck={fit['v_stribeck']:.3f} "
                f"c_visc={fit['c_visc']:.4f} rmse={fit['rmse']:.4f} "
                f"(n={fit['n_samples']})"
            )
        write_json(ckpt.routine_result("rollout", self.axis), result)
        return result


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--trials", type=int, default=DEFAULT_TRIALS,
                   help=f"Number of spin-up/coast trials (default {DEFAULT_TRIALS})")
    p.add_argument("--amplitude", type=float, default=None,
                   help="Spin-up acceleration; must break static friction with "
                        "friction compensation off")
    p.add_argument("--v-fit-min", type=float, default=None,
                   help="Ignore coast samples slower than this (m/s or rad/s)")
    return p
