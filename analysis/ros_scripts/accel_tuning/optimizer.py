"""End-to-end optimizer: runs coulomb → viscous → efficiency → inertia.

Sequence (default):

  1. coulomb_x  (eta=1, c_visc=0, c_coul=0, default iz)
  2. viscous_x  (search c_visc, recompute c_coul each candidate)
  3. efficiency_x (search eta; coulomb recomputed)
  4. coulomb_theta (uses tuned linear params for eta and friction)
  5. viscous_theta
  6. inertia_theta (uses linear eta; iz searched)
  7. coulomb_theta re-run (one convergence pass, since iz changed)

Each step prompts the user (linear axes only) to physically rotate the
robot between trials. The optimizer writes ``summary.json`` with the full
before/after parameter delta.
"""

from __future__ import annotations

from typing import Optional

import rclpy

from .base import default_params_dict
from .checkpoint import CheckpointDir, write_json
from .coulomb import (
    CoulombRoutine,
    DEFAULT_MIN_SAMPLES,
    DEFAULT_PULSE_DURATION,
    DEFAULT_REST_DURATION,
    DEFAULT_VEL_THRESHOLD_WHEEL,
    default_max_accel,
    default_step,
    empty_friction_array,
)
from .efficiency import (DEFAULT_ETA_INITIAL_GUESS, DEFAULT_ETA_LOW,
                          DEFAULT_ETA_HIGH, EfficiencyRoutine)
from .inertia import InertiaRoutine
from .triangular import default_pulse_accel
from .viscous import ViscousRoutine, viscous_bounds


def _run_coulomb(robot_id: int, axis: str, ckpt: CheckpointDir,
                  params: dict) -> dict:
    node = CoulombRoutine(
        robot_id=robot_id, axis=axis,
        step=default_step(axis), start_accel=0.0,
        pulse_duration=DEFAULT_PULSE_DURATION,
        rest_duration=DEFAULT_REST_DURATION,
        max_accel=default_max_accel(axis),
        velocity_threshold=DEFAULT_VEL_THRESHOLD_WHEEL,
        min_samples=DEFAULT_MIN_SAMPLES,
    )
    try:
        return node.run(ckpt, current_params=params)
    finally:
        node.destroy_node()


def _run_viscous(robot_id: int, axis: str, ckpt: CheckpointDir,
                  params: dict, a_min: float, v_min: float) -> dict:
    node = ViscousRoutine(robot_id=robot_id, axis=axis,
                           amplitude=default_pulse_accel(axis))
    try:
        return node.run_search(ckpt, params, a_min, v_min,
                                bounds=viscous_bounds(axis))
    finally:
        node.destroy_node()


def _run_efficiency(robot_id: int, axis: str, ckpt: CheckpointDir,
                     params: dict, a_min: float, v_min: float) -> dict:
    node = EfficiencyRoutine(robot_id=robot_id, axis=axis,
                               amplitude=default_pulse_accel(axis))
    try:
        return node.run_search(ckpt, params, a_min, v_min,
                                initial=DEFAULT_ETA_INITIAL_GUESS,
                                bounds=(DEFAULT_ETA_LOW, DEFAULT_ETA_HIGH))
    finally:
        node.destroy_node()


def _run_inertia(robot_id: int, ckpt: CheckpointDir,
                  params: dict, a_min: float, v_min: float) -> dict:
    node = InertiaRoutine(robot_id=robot_id, amplitude=default_pulse_accel("theta"))
    try:
        return node.run_search(ckpt, params, a_min, v_min)
    finally:
        node.destroy_node()


def run_optimizer(robot_id: int, ckpt: CheckpointDir) -> dict:
    """Run all four routines in sequence and return the summary dict."""
    baseline = default_params_dict()
    ckpt.write_baseline(baseline)

    # --- Linear (x) ---
    params = dict(baseline)
    params["PHYS_MOTOR_MODEL"][1] = 1.0
    params["PHYS_FRICTION_MODEL"] = empty_friction_array()

    coul_x = _run_coulomb(robot_id, "x", ckpt, params)
    if not coul_x.get("detected"):
        write_json(ckpt.root / "summary.json",
                    {"status": "failed", "reason": "coulomb_x no motion"})
        return {"status": "failed"}
    params = coul_x["params_after"]
    a_min_x, v_min_x = coul_x["a_min"], coul_x["v_min"]

    visc_x = _run_viscous(robot_id, "x", ckpt, params, a_min_x, v_min_x)
    params = visc_x["params_after"]

    eff_x = _run_efficiency(robot_id, "x", ckpt, params, a_min_x, v_min_x)
    params = eff_x["params_after"]

    # --- Angular (theta) ---
    coul_t = _run_coulomb(robot_id, "theta", ckpt, params)
    if not coul_t.get("detected"):
        write_json(ckpt.root / "summary.json",
                    {"status": "failed", "reason": "coulomb_theta no motion",
                     "params_at_failure": params})
        return {"status": "failed"}
    params = coul_t["params_after"]
    a_min_t, v_min_t = coul_t["a_min"], coul_t["v_min"]

    visc_t = _run_viscous(robot_id, "theta", ckpt, params, a_min_t, v_min_t)
    params = visc_t["params_after"]

    inertia_t = _run_inertia(robot_id, ckpt, params, a_min_t, v_min_t)
    params = inertia_t["params_after"]

    # Re-run angular coulomb once now that iz changed, to keep the formula
    # consistent.
    coul_t2 = _run_coulomb(robot_id, "theta", ckpt, params)
    if coul_t2.get("detected"):
        params = coul_t2["params_after"]

    summary = {
        "status": "ok",
        "baseline_params": baseline,
        "final_params": params,
        "steps": {
            "coulomb_x": coul_x,
            "viscous_x": visc_x,
            "efficiency_x": eff_x,
            "coulomb_theta_initial": coul_t,
            "viscous_theta": visc_t,
            "inertia_theta": inertia_t,
            "coulomb_theta_final": coul_t2,
        },
    }
    ckpt.write_summary(summary)
    return summary
