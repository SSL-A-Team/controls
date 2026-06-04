"""Motor-efficiency tuning routine.

Runs the triangular profile and searches the motor-efficiency factor
``eta`` so the realized acceleration matches the commanded amplitude.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

import rclpy

from .analysis import accel_match_error
from .base import ANGULAR_AXES, BaseTuneNode
from .checkpoint import CheckpointDir, TrialRecord, save_trial, write_json
from .coulomb import axis_friction_indices, axis_inertia, compute_coulomb
from ._search import (
    bracketed_root_search,
    format_trial_end_banner,
    format_trial_start_banner,
    log_lines,
    replicated_evaluate,
    run_triangular_trial,
)
from .progress import ProgressCache
from .triangular import default_pulse_accel

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
CONTROLS_DIR = next(p for p in SCRIPTS_DIR.parents if p.name == "controls")
sys.path.insert(0, str(CONTROLS_DIR / "analysis"))
from params import merge_params_dict  # noqa: E402


# ===== EDIT ME =====
DEFAULT_ETA_LOW = 1.0
DEFAULT_ETA_HIGH = 25.0
DEFAULT_ETA_INITIAL_GUESS = 13.0  # starting point per user note
DEFAULT_ETA_TOL = 0.05            # |realized - commanded| stop threshold (tol_y)
DEFAULT_ETA_MAX_ITER = 8
# Bracketed-root tuning knobs (replaces the old plain-secant early-exit):
#   tol_x_frac: bracket-width target as a fraction of (bounds[1]-bounds[0]).
#     Once both endpoints sit within this fraction of the full search range
#     AND |f|<=tol_y AND we've done at least min_iter evaluations, the
#     search exits. Lower => stricter localisation.
#   min_iter:   never accept a result on fewer evaluations than this, no
#     matter how good the |f| is. Forces real exploration.
#   expand_factor: when both seeds have the same sign (root not bracketed),
#     expand the seed-pair outward by this multiple per step (clamped by
#     bounds) until a bracket is found.
DEFAULT_TOL_X_FRAC = 0.05
DEFAULT_MIN_ITER = 4
DEFAULT_EXPAND_FACTOR = 1.5
# Per-candidate noise control. See ``viscous.py`` for the same flags;
# efficiency replicates the signed accel-match error (the search variable)
# and feeds the median to the bracketed root search.
DEFAULT_REPLICATES_PER_CANDIDATE = 3
DEFAULT_MAX_REPLACEMENTS_PER_CANDIDATE = 2
DEFAULT_OUTLIER_MAD = 3.0
# ===================


class EfficiencyRoutine(BaseTuneNode):
    """Triangular-profile secant search on motor efficiency."""

    def __init__(self, robot_id: int, axis: str, amplitude: float,
                 dry_run: bool = False, skip_turnaround: bool = False):
        super().__init__("efficiency_tune", robot_id, axis, dry_run=dry_run,
                         skip_turnaround=skip_turnaround)
        self.amplitude = float(amplitude)
        self._ckpt: Optional[CheckpointDir] = None
        self._params: dict = {}
        self._a_min: float = 0.0
        self._v_min: float = 0.0
        self._trial_idx = 0
        self._cache: Optional[ProgressCache] = None
        # Track running-best by |error_signed| since that's what the secant
        # search is driving to zero.
        self._best: Optional[tuple[float, float]] = None  # (eta, err_signed)
        # Per-candidate noise control (set by run_search / run_single).
        self._replicates: int = DEFAULT_REPLICATES_PER_CANDIDATE
        self._max_replacements: int = DEFAULT_MAX_REPLACEMENTS_PER_CANDIDATE
        self._outlier_mad: float = DEFAULT_OUTLIER_MAD
        self._candidate_history: list[dict] = []

    def _apply_candidate(self, eta: float) -> dict:
        # Updating eta changes the c_coul balance, so recompute it for this
        # axis's slot.
        coul_idx, visc_idx = axis_friction_indices(self.axis)
        c_visc = self._params["PHYS_FRICTION_MODEL"][visc_idx]
        inertia = axis_inertia(self.axis, self._params)
        c_coul = compute_coulomb(eta, inertia, self._a_min, c_visc, self._v_min)
        new_friction = list(self._params["PHYS_FRICTION_MODEL"])
        new_friction[coul_idx] = c_coul
        new_motor = list(self._params["PHYS_MOTOR_MODEL"])
        new_motor[1] = float(eta)
        updated = merge_params_dict(self._params, {
            "PHYS_MOTOR_MODEL": new_motor,
            "PHYS_FRICTION_MODEL": new_friction,
        })
        self.push_params(updated, only_keys=["PHYS_MOTOR_MODEL",
                                              "PHYS_FRICTION_MODEL"])
        return updated

    def _run_single_trial(self, eta: float, replicate_idx: int,
                          replicates_total: int) -> float:
        """One physical triangular pulse at ``eta``; returns signed accel
        match error. Does NOT touch ``self._best`` or the progress cache —
        those are owned by ``_evaluate`` after replicate aggregation.
        """
        updated = self._apply_candidate(eta)
        candidate = {
            "eta": float(eta),
            "replicate_idx": int(replicate_idx),
            "replicates_total": int(replicates_total),
        }
        best_desc = (f"eta={self._best[0]:.3f}  |err|={abs(self._best[1]):.4f}"
                     if self._best is not None else None)
        log_lines(self.get_logger(), format_trial_start_banner(
            "efficiency", self.axis, self._trial_idx, candidate, updated,
            best_desc=best_desc,
        ))
        outcome = run_triangular_trial(
            self, self.amplitude, err_fn=accel_match_error,
        )
        record = TrialRecord(
            routine="efficiency", axis=self.axis,
            trial_idx=self._trial_idx,
            candidate=candidate,
            metrics=outcome.metrics,
            accepted=False,
            notes=(f"amplitude={self.amplitude}; "
                   f"replicate {replicate_idx + 1}/{replicates_total}"),
        )
        save_trial(self._ckpt, record, telemetry=self.buffer.to_arrays())  # type: ignore[arg-type]
        best_desc = (f"eta={self._best[0]:.3f}  |err|={abs(self._best[1]):.4f}"
                     if self._best is not None else "(none yet)")
        log_lines(self.get_logger(), format_trial_end_banner(
            "efficiency", self.axis, self._trial_idx, outcome,
            best_desc=best_desc,
        ))
        self.get_logger().info(
            f"  err_signed={outcome.error_signed:+.4f}  (target 0)"
        )
        self._trial_idx += 1
        self._params = updated
        if self.axis not in ANGULAR_AXES:
            self.wait_for_turnaround()
        else:
            self.wait_for_turnaround(prompt=False)
        return float(outcome.error_signed)

    def _evaluate(self, eta: float) -> float:
        if self._cache is not None:
            cached = self._cache.find(eta)
            if cached is not None:
                self.get_logger().info(
                    f"[efficiency][{self.axis}] cache hit eta={eta:.3f} "
                    f"→ err={cached['score']:.4f} (median of "
                    f"{cached.get('extra', {}).get('replicates_total', '?')} "
                    f"replicates)"
                )
                self._params = self._apply_candidate(eta)
                return float(cached["score"])

        self.get_logger().info(
            f"[efficiency][{self.axis}] EVALUATE eta={eta:.3f}: "
            f"{self._replicates} replicates "
            f"(max {self._max_replacements} outlier replacements, "
            f"MAD threshold {self._outlier_mad}σ)"
        )
        summary = replicated_evaluate(
            candidate_label=f"[efficiency][{self.axis}] eta={eta:.3f}",
            logger=self.get_logger(),
            run_trial=lambda i, n: self._run_single_trial(eta, i, n),
            replicates=self._replicates,
            max_replacements=self._max_replacements,
            outlier_mad=self._outlier_mad,
        )
        aggregated_err = summary["aggregated"]
        candidate_record = {
            "eta": float(eta),
            "errors_signed": summary["scores"],
            "error_median": aggregated_err,
            "error_mean": summary["mean"],
            "error_std": summary["std"],
            "error_min": summary["min"],
            "error_max": summary["max"],
            "replicates_total": summary["replicates_total"],
            "replacements": summary["replacements"],
            "outlier_threshold_final": summary["outlier_threshold_final"],
        }
        self._candidate_history.append(candidate_record)
        self.get_logger().info(
            f"[efficiency][{self.axis}] eta={eta:.3f} AGGREGATED: "
            f"err_median={aggregated_err:+.4f}  mean={summary['mean']:+.4f} "
            f"± {summary['std']:.4f}  "
            f"(min={summary['min']:+.4f}, max={summary['max']:+.4f}, "
            f"replacements={summary['replacements_done']}/{self._max_replacements}, "
            f"threshold=±{summary['outlier_threshold_final']:.4f})"
        )
        if self._best is None or abs(aggregated_err) < abs(self._best[1]):
            self._best = (eta, aggregated_err)
        if self._cache is not None:
            self._cache.append(eta, aggregated_err, extra={
                "trial_idx_end": self._trial_idx,
                "replicates_total": summary["replicates_total"],
                "replacements": summary["replacements_done"],
                "raw_errors_signed": summary["scores"],
            })
        return aggregated_err

    def run_single(self, ckpt: CheckpointDir, params: dict,
                    a_min: float, v_min: float, eta_value: float,
                    replicates: int = DEFAULT_REPLICATES_PER_CANDIDATE,
                    max_replacements: int = DEFAULT_MAX_REPLACEMENTS_PER_CANDIDATE,
                    outlier_mad: float = DEFAULT_OUTLIER_MAD,
                    ) -> dict:
        self._ckpt = ckpt
        self._params = dict(params)
        self._a_min = float(a_min)
        self._v_min = float(v_min)
        self._replicates = max(1, int(replicates))
        self._max_replacements = max(0, int(max_replacements))
        self._outlier_mad = float(outlier_mad)
        self._candidate_history = []
        self._cache = ProgressCache.load_or_new(
            ckpt.routine_dir("efficiency", self.axis) / "progress.json")
        self._trial_idx = (len(self._cache.entries)
                           * (self._replicates + self._max_replacements))
        err = self._evaluate(eta_value)
        result = {
            "axis": self.axis, "mode": "single",
            "eta": float(eta_value),
            "error_signed": float(err),
            "params_after": self._params,
            "candidates": self._candidate_history,
            "replicates_per_candidate": self._replicates,
            "max_replacements_per_candidate": self._max_replacements,
            "outlier_mad": self._outlier_mad,
        }
        write_json(ckpt.routine_result("efficiency", self.axis), result)
        return result

    def run_search(self, ckpt: CheckpointDir, params: dict,
                    a_min: float, v_min: float,
                    initial: float = DEFAULT_ETA_INITIAL_GUESS,
                    bounds: tuple[float, float] = (DEFAULT_ETA_LOW,
                                                     DEFAULT_ETA_HIGH),
                    tol: float = DEFAULT_ETA_TOL,
                    max_iter: int = DEFAULT_ETA_MAX_ITER,
                    replicates: int = DEFAULT_REPLICATES_PER_CANDIDATE,
                    max_replacements: int = DEFAULT_MAX_REPLACEMENTS_PER_CANDIDATE,
                    outlier_mad: float = DEFAULT_OUTLIER_MAD,
                    tol_x_frac: float = DEFAULT_TOL_X_FRAC,
                    min_iter: int = DEFAULT_MIN_ITER,
                    expand_factor: float = DEFAULT_EXPAND_FACTOR,
                    ) -> dict:
        self._ckpt = ckpt
        self._params = dict(params)
        self._a_min = float(a_min)
        self._v_min = float(v_min)
        self._replicates = max(1, int(replicates))
        self._max_replacements = max(0, int(max_replacements))
        self._outlier_mad = float(outlier_mad)
        self._candidate_history = []
        self._cache = ProgressCache.load_or_new(
            ckpt.routine_dir("efficiency", self.axis) / "progress.json")
        self._trial_idx = (len(self._cache.entries)
                           * (self._replicates + self._max_replacements))
        if self._cache.entries:
            self.get_logger().info(
                f"[efficiency][{self.axis}] resuming from cache "
                f"({len(self._cache.entries)} prior aggregated evaluations)"
            )
        self.get_logger().info(
            f"[efficiency][{self.axis}] bracketed search initial={initial} "
            f"bounds={bounds}  tol_y={tol}  tol_x_frac={tol_x_frac}  "
            f"min_iter={min_iter}  max_iter={max_iter}  "
            f"expand_factor={expand_factor}  "
            f"replicates_per_candidate={self._replicates}  "
            f"max_replacements_per_candidate={self._max_replacements}  "
            f"outlier_mad={self._outlier_mad}σ"
        )
        # Seed the bracket: bracket_root_search will expand outward if both
        # seeds happen to land on the same side of the root.
        x0 = max(bounds[0], initial * 0.7)
        x1 = min(bounds[1], initial * 1.3)
        if abs(x1 - x0) < 0.1:
            x1 = min(bounds[1], x0 + 1.0)
        best_x, best_err = bracketed_root_search(
            self._evaluate, x0, x1,
            bounds=bounds, tol_y=tol, tol_x_frac=tol_x_frac,
            max_iter=max_iter, min_iter=min_iter,
            expand_factor=expand_factor,
            on_iter=lambda i, x, e, phase: self.get_logger().info(
                f"[efficiency][{self.axis}] iter {i} [{phase}]: "
                f"eta={x:.3f} err={e:+.4f}"
            ),
        )
        # Push the winner.
        self._apply_candidate(best_x)
        result = {
            "axis": self.axis, "mode": "search",
            "eta": float(best_x),
            "error_signed_final": float(best_err),
            "bounds": list(bounds),
            "params_after": self._params,
            "a_min": float(a_min), "v_min": float(v_min),
            "candidates": self._candidate_history,
            "replicates_per_candidate": self._replicates,
            "max_replacements_per_candidate": self._max_replacements,
            "outlier_mad": self._outlier_mad,
            "tol_x_frac": tol_x_frac,
            "min_iter": min_iter,
        }
        write_json(ckpt.routine_result("efficiency", self.axis), result)
        return result


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--mode", choices=["single", "search"], default="search")
    p.add_argument("--value", type=float, default=None,
                    help="(--mode single) motor_efficiency_factor to evaluate")
    p.add_argument("--initial", type=float, default=DEFAULT_ETA_INITIAL_GUESS,
                    help="Initial guess for the secant search")
    p.add_argument("--a-min", type=float, required=False)
    p.add_argument("--v-min", type=float, default=0.0)
    p.add_argument("--amplitude", type=float, default=None,
                    help="Triangular pulse amplitude (axis-default if unset)")
    p.add_argument("--bounds-low", type=float, default=DEFAULT_ETA_LOW)
    p.add_argument("--bounds-high", type=float, default=DEFAULT_ETA_HIGH)
    p.add_argument("--tol", type=float, default=DEFAULT_ETA_TOL,
                    help="|f(x)| stop threshold (tol_y). Combined with "
                         "--tol-x-frac and --min-iter, all three must be met "
                         "to terminate. "
                         f"Default {DEFAULT_ETA_TOL}.")
    p.add_argument("--tol-x-frac", type=float, default=DEFAULT_TOL_X_FRAC,
                    help="Bracket-width stop threshold as fraction of "
                         "(bounds_high - bounds_low). "
                         f"Default {DEFAULT_TOL_X_FRAC}.")
    p.add_argument("--min-iter", type=int, default=DEFAULT_MIN_ITER,
                    help="Minimum evaluations before any tol-based exit is "
                         "honored. Prevents tol-skin convergence on a single "
                         f"noisy seed. Default {DEFAULT_MIN_ITER}.")
    p.add_argument("--expand-factor", type=float, default=DEFAULT_EXPAND_FACTOR,
                    help="If initial seeds don't bracket the root, expand the "
                         "seed pair outward by this multiplicative step until "
                         f"a bracket is found. Default {DEFAULT_EXPAND_FACTOR}.")
    p.add_argument("--max-iter", type=int, default=DEFAULT_ETA_MAX_ITER)
    p.add_argument("--replicates", type=int,
                    default=DEFAULT_REPLICATES_PER_CANDIDATE,
                    help="Per-candidate replicate count; the search consumes "
                         "the median signed error. Set 1 to restore "
                         f"single-shot. Default {DEFAULT_REPLICATES_PER_CANDIDATE}.")
    p.add_argument("--max-replacements", type=int,
                    default=DEFAULT_MAX_REPLACEMENTS_PER_CANDIDATE,
                    help="Per-candidate outlier-replacement cap. "
                         f"Default {DEFAULT_MAX_REPLACEMENTS_PER_CANDIDATE}.")
    p.add_argument("--outlier-mad", type=float,
                    default=DEFAULT_OUTLIER_MAD,
                    help="MAD-σ multiplier for per-candidate outlier "
                         "detection (lower = stricter). "
                         f"Default {DEFAULT_OUTLIER_MAD}.")
    return p
