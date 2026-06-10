#!/usr/bin/env python3
"""Feed-forward acceleration model tuning — multi-step CLI.

Tune the parameters of ``ateam_controls::RobotModel`` that participate in
the feed-forward acceleration → wheel-current pipeline. Each step is
independently runnable for safety; an ``all`` mode chains them.

Tuning steps (order matters when chaining):

    coulomb     — find minimum-motion accel; compute c_coul.
    viscous     — triangular profile, search c_visc keeping the profile linear.
    efficiency  — triangular profile, search motor_efficiency_factor
                   so realized accel matches commanded.
    inertia     — angular only; search iz so realized angular accel matches
                   commanded under the tuned linear model.
    all         — run the full pipeline (coulomb_x → viscous_x → efficiency_x
                   → coulomb_θ → viscous_θ → inertia_θ → coulomb_θ).

Examples:

    # Single-step coulomb tuning on x:
    python accel_model_tune.py --routine coulomb --axis x --robot-id 0

    # Search c_visc on theta with a custom amplitude (requires a_min/v_min
    # from a prior coulomb step):
    python accel_model_tune.py --routine viscous --axis theta --robot-id 0 \\
        --a-min 5.2 --v-min 0.3 --amplitude 2.5

    # End-to-end automated run (uses defaults from the Rust controls lib):
    python accel_model_tune.py --routine all --robot-id 0

The ``y`` axis is allowed for diagnostic-only runs (no firmware param
changes are pushed). Checkpoints and per-trial telemetry are written to
``analysis/data/accel_tuning/<run_id>/``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

import rclpy

SCRIPTS_DIR = Path(__file__).resolve().parent
CONTROLS_DIR = next(p for p in SCRIPTS_DIR.parents if p.name == "controls")
sys.path.insert(0, str(CONTROLS_DIR / "analysis"))

from accel_tuning import (  # noqa: E402
    coulomb as coulomb_mod,
    efficiency as efficiency_mod,
    inertia as inertia_mod,
    optimizer as optimizer_mod,
    verify as verify_mod,
    viscous as viscous_mod,
)
from accel_tuning.base import (  # noqa: E402
    ANGULAR_AXES,
    DIAGNOSTIC_ONLY_AXES,
    default_params_dict,
)
from accel_tuning.checkpoint import CheckpointDir  # noqa: E402
from accel_tuning.triangular import default_pulse_accel  # noqa: E402


def _add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("-r", "--robot-id", type=int, required=True,
                         help="Robot ID")
    parser.add_argument("--axis", choices=["x", "y", "theta"], default="x",
                         help="Axis to tune (default x). y is diagnostic-only "
                              "(no firmware param push).")
    parser.add_argument("--run-id", type=str, default=None,
                         help="Checkpoint dir name (default: timestamp)")
    parser.add_argument("--baseline-params", type=str, default=None,
                         help="Path to a result.json from a prior step (its "
                              "'params_after' is used as the starting param "
                              "set). If omitted, defaults from the Rust lib "
                              "are used.")
    parser.add_argument("--motor-efficiency", type=float, default=None,
                         help="Override motor_efficiency_factor in the loaded "
                              "baseline params before running. Use to start "
                              "the viscous/efficiency search at a non-default "
                              "η (e.g. 10.0) instead of editing JSON by hand.")
    parser.add_argument("--no-turnaround", action="store_true",
                         help="Skip the manual-rotation prompt between trials "
                              "(linear axes only). Useful when the robot has "
                              "enough room that ±A pulses balance out.")
    parser.add_argument("--plot", action="store_true",
                         help="After the routine completes, open the "
                              "analysis.visualization.body 3×3 telemetry "
                              "plot for all trials in this run.")
    parser.add_argument("--dry-run", action="store_true",
                         help="Don't publish motion commands or push params; "
                              "exercise the state machines only.")


def _load_baseline_params(args) -> dict:
    """Return the starting param dict for this routine.

    If ``--baseline-params`` points at a routine ``result.json`` (e.g.
    ``.../coulomb/x/result.json``), its ``params_after`` is returned. If it
    points at any other JSON object containing PARAM_MAP keys, that's used
    directly. Otherwise the Rust-library defaults are returned. After
    loading, ``--motor-efficiency`` overrides ``PHYS_MOTOR_MODEL[1]`` if set.
    """
    if args.baseline_params is None:
        params = default_params_dict()
    else:
        import json
        with open(args.baseline_params) as f:
            data = json.load(f)
        if isinstance(data, dict) and "params_after" in data:
            params = data["params_after"]
        else:
            params = data
    if getattr(args, "motor_efficiency", None) is not None:
        # Always keep two slots: [Kt, η].
        mm = list(params.get("PHYS_MOTOR_MODEL",
                              default_params_dict()["PHYS_MOTOR_MODEL"]))
        mm[1] = float(args.motor_efficiency)
        params["PHYS_MOTOR_MODEL"] = mm
    return params


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="routine", required=True)

    # coulomb
    p_coul = sub.add_parser("coulomb",
                             help="Pulse-and-step minimum-motion search",
                             parents=[coulomb_mod.build_arg_parser()])
    _add_common(p_coul)

    # viscous
    p_visc = sub.add_parser("viscous",
                             help="Triangular-profile c_visc search",
                             parents=[viscous_mod.build_arg_parser()])
    _add_common(p_visc)

    # efficiency
    p_eff = sub.add_parser("efficiency",
                            help="Triangular-profile motor-efficiency search",
                            parents=[efficiency_mod.build_arg_parser()])
    _add_common(p_eff)

    # inertia
    p_iz = sub.add_parser("inertia",
                           help="Triangular-profile iz search (angular only)",
                           parents=[inertia_mod.build_arg_parser()])
    _add_common(p_iz)
    # Force axis=theta for inertia.
    p_iz.set_defaults(axis="theta")

    # all
    p_all = sub.add_parser("all",
                            help="Run the full coulomb→viscous→efficiency→inertia pipeline")
    _add_common(p_all)
    # 'axis' arg added by _add_common is ignored by the optimizer.

    # verify
    p_ver = sub.add_parser("verify",
                            help="Upload a tuned param set and run N back-and-forth "
                                 "triangular trials to check open-loop performance.",
                            parents=[verify_mod.build_arg_parser()])
    _add_common(p_ver)

    return parser


# --------------------------------------------------------------------------
# Per-routine dispatch
# --------------------------------------------------------------------------

def _amplitude(args, axis: str) -> float:
    explicit = getattr(args, "amplitude", None)
    return float(explicit) if explicit is not None else default_pulse_accel(axis)


def _baseline_for(axis: str) -> dict:
    params = default_params_dict()
    # Coulomb baseline: η=1, c_*=0 (only relevant if user starts with coulomb).
    return params


def _dispatch_coulomb(args, ckpt: CheckpointDir) -> dict:
    step = args.step if args.step is not None else coulomb_mod.default_step(args.axis)
    max_accel = (args.max_accel if args.max_accel is not None
                 else coulomb_mod.default_max_accel(args.axis))
    node = coulomb_mod.CoulombRoutine(
        robot_id=args.robot_id, axis=args.axis,
        step=step, start_accel=args.start_accel,
        pulse_duration=args.pulse_duration,
        rest_duration=args.rest_duration,
        max_accel=max_accel,
        velocity_threshold=args.velocity_threshold,
        min_samples=args.min_samples,
        dry_run=args.dry_run,
    )
    try:
        # If user supplied a baseline, use it (e.g. for angular coulomb after
        # linear tuning); otherwise the routine forces η=1, c_*=0 itself.
        params = (_load_baseline_params(args)
                  if args.baseline_params is not None else None)
        return node.run(ckpt, current_params=params)
    finally:
        node.destroy_node()


def _require_a_min(args) -> None:
    if args.a_min is None:
        raise SystemExit(
            "ERROR: --a-min is required for this routine; rerun the coulomb "
            "step first (or pass --a-min / --v-min from its result.json)."
        )


def _dispatch_viscous(args, ckpt: CheckpointDir) -> dict:
    _require_a_min(args)
    amp = _amplitude(args, args.axis)
    node = viscous_mod.ViscousRoutine(
        robot_id=args.robot_id, axis=args.axis,
        amplitude=amp, dry_run=args.dry_run,
        skip_turnaround=args.no_turnaround,
    )
    try:
        params = _load_baseline_params(args)
        if args.mode == "single":
            if args.value is None:
                raise SystemExit("ERROR: --value is required with --mode single")
            return node.run_single(ckpt, params, args.a_min, args.v_min,
                                    args.value,
                                    replicates=args.replicates,
                                    max_replacements=args.max_replacements,
                                    outlier_mad=args.outlier_mad)
        bounds = None
        if args.bounds_low is not None and args.bounds_high is not None:
            bounds = (args.bounds_low, args.bounds_high)
        return node.run_search(ckpt, params, args.a_min, args.v_min,
                                bounds=bounds, tol=args.tol,
                                max_iter=args.max_iter,
                                replicates=args.replicates,
                                max_replacements=args.max_replacements,
                                outlier_mad=args.outlier_mad)
    finally:
        node.destroy_node()


def _dispatch_efficiency(args, ckpt: CheckpointDir) -> dict:
    _require_a_min(args)
    amp = _amplitude(args, args.axis)
    node = efficiency_mod.EfficiencyRoutine(
        robot_id=args.robot_id, axis=args.axis,
        amplitude=amp, dry_run=args.dry_run,
        skip_turnaround=args.no_turnaround,
    )
    try:
        params = _load_baseline_params(args)
        if args.mode == "single":
            if args.value is None:
                raise SystemExit("ERROR: --value is required with --mode single")
            return node.run_single(ckpt, params, args.a_min, args.v_min,
                                    args.value,
                                    replicates=args.replicates,
                                    max_replacements=args.max_replacements,
                                    outlier_mad=args.outlier_mad)
        return node.run_search(ckpt, params, args.a_min, args.v_min,
                                initial=args.initial,
                                bounds=(args.bounds_low, args.bounds_high),
                                tol=args.tol, max_iter=args.max_iter,
                                replicates=args.replicates,
                                max_replacements=args.max_replacements,
                                outlier_mad=args.outlier_mad,
                                tol_x_frac=args.tol_x_frac,
                                min_iter=args.min_iter,
                                expand_factor=args.expand_factor)
    finally:
        node.destroy_node()


def _dispatch_inertia(args, ckpt: CheckpointDir) -> dict:
    _require_a_min(args)
    amp = _amplitude(args, "theta")
    node = inertia_mod.InertiaRoutine(
        robot_id=args.robot_id, amplitude=amp, dry_run=args.dry_run,
        skip_turnaround=args.no_turnaround,
    )
    try:
        params = _load_baseline_params(args)
        if args.mode == "single":
            if args.value is None:
                raise SystemExit("ERROR: --value is required with --mode single")
            return node.run_single(ckpt, params, args.a_min, args.v_min,
                                    args.value,
                                    replicates=args.replicates,
                                    max_replacements=args.max_replacements,
                                    outlier_mad=args.outlier_mad)
        return node.run_search(ckpt, params, args.a_min, args.v_min,
                                initial=args.initial,
                                bounds=(args.bounds_low, args.bounds_high),
                                tol=args.tol, max_iter=args.max_iter,
                                replicates=args.replicates,
                                max_replacements=args.max_replacements,
                                outlier_mad=args.outlier_mad,
                                tol_x_frac=args.tol_x_frac,
                                min_iter=args.min_iter,
                                expand_factor=args.expand_factor)
    finally:
        node.destroy_node()


def _dispatch_all(args, ckpt: CheckpointDir) -> dict:
    if args.dry_run:
        raise SystemExit(
            "ERROR: --dry-run is not supported with --routine all; run each "
            "step individually for dry-run testing."
        )
    return optimizer_mod.run_optimizer(args.robot_id, ckpt)


def _dispatch_verify(args, ckpt: CheckpointDir) -> dict:
    amp = _amplitude(args, args.axis)
    node = verify_mod.VerifyRoutine(
        robot_id=args.robot_id, axis=args.axis,
        amplitude=amp, trials=args.trials,
        dry_run=args.dry_run,
        skip_turnaround=args.no_turnaround,
    )
    try:
        params = _load_baseline_params(args)
        if not params:
            raise SystemExit(
                "ERROR: verify needs a parameter set. Use --baseline-params "
                "to point at a result.json from a tuning step (or another JSON "
                "with PARAM_MAP keys)."
            )
        return node.run(ckpt, params)
    finally:
        node.destroy_node()


DISPATCH = {
    "coulomb": _dispatch_coulomb,
    "viscous": _dispatch_viscous,
    "efficiency": _dispatch_efficiency,
    "inertia": _dispatch_inertia,
    "all": _dispatch_all,
    "verify": _dispatch_verify,
}


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    result = None
    try:
        ckpt = CheckpointDir.create(run_id=args.run_id)
        ckpt.write_metadata(args.robot_id, vars(args))
        ckpt.write_baseline(default_params_dict())
        print(f"Run dir: {ckpt.root}")
        try:
            result = DISPATCH[args.routine](args, ckpt)
            print("Result:")
            import json
            print(json.dumps(result, indent=2, default=str))
        except Exception as exc:
            # Lazy-import the typed exception so the CLI works even when ROS
            # isn't sourced (the import path is the same either way).
            from accel_tuning.base import TurnaroundTimeout
            if isinstance(exc, TurnaroundTimeout):
                print(f"\nAborted: {exc}")
                print(f"To resume, rerun the same command with "
                      f"`--run-id {ckpt.run_id}`. Previously-evaluated "
                      "candidates will be served from "
                      f"{ckpt.root}/<routine>/<axis>/progress.json without "
                      "re-pulsing the robot.")
                return 2
            raise
    finally:
        rclpy.shutdown()

    if getattr(args, "plot", False) and result is not None:
        from accel_tuning.plot import show_run_plot
        if args.routine == "all":
            for routine_name in ("coulomb", "viscous", "efficiency", "inertia"):
                for ax in ("x", "y", "theta"):
                    if (ckpt.root / routine_name / ax).is_dir():
                        show_run_plot(ckpt.root, routine_name, ax)
        else:
            show_run_plot(ckpt.root, args.routine, args.axis)
    return 0


if __name__ == "__main__":
    sys.exit(main())
