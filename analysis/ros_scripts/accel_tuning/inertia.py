"""Rotational-inertia tuning routine (angular axis only).

After motor efficiency has been tuned on the linear axis, the realized
angular acceleration may still not match the commanded value because ``iz``
is unknown. This routine searches ``iz`` so the angular triangular profile
produces the commanded acceleration with the linear ``eta`` held fixed.
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
from .coulomb import axis_friction_indices, compute_coulomb
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
DEFAULT_IZ_LOW = 0.001
DEFAULT_IZ_HIGH = 0.05
DEFAULT_IZ_TOL = 0.05
DEFAULT_IZ_MAX_ITER = 8
DEFAULT_TOL_X_FRAC = 0.05
DEFAULT_MIN_ITER = 4
DEFAULT_EXPAND_FACTOR = 1.5
# Per-candidate noise control (same semantics as viscous / efficiency).
DEFAULT_REPLICATES_PER_CANDIDATE = 3
DEFAULT_MAX_REPLACEMENTS_PER_CANDIDATE = 2
DEFAULT_OUTLIER_MAD = 3.0
# ===================


class InertiaRoutine(BaseTuneNode):
    """Angular-axis secant search on iz."""

    def __init__(self, robot_id: int, amplitude: float, dry_run: bool = False,
                 skip_turnaround: bool = False):
        super().__init__("inertia_tune", robot_id, axis="theta", dry_run=dry_run,
                         skip_turnaround=skip_turnaround)
        self.amplitude = float(amplitude)
        self._ckpt: Optional[CheckpointDir] = None
        self._params: dict = {}
        self._a_min: float = 0.0
        self._v_min: float = 0.0
        self._trial_idx = 0
        self._cache: Optional[ProgressCache] = None
        self._best: Optional[tuple[float, float]] = None  # (iz, signed err)
        # Per-candidate noise control (set by run_search / run_single).
        self._replicates: int = DEFAULT_REPLICATES_PER_CANDIDATE
        self._max_replacements: int = DEFAULT_MAX_REPLACEMENTS_PER_CANDIDATE
        self._outlier_mad: float = DEFAULT_OUTLIER_MAD
        self._candidate_history: list[dict] = []

    def _apply_candidate(self, iz: float) -> dict:
        # iz appears in both inertia and the coulomb formula (angular axis).
        coul_idx, visc_idx = axis_friction_indices("theta")
        new_inertia = list(self._params["PHYS_INERTIA"])
        new_inertia[1] = float(iz)
        eta = self._params["PHYS_MOTOR_MODEL"][1]
        c_visc = self._params["PHYS_FRICTION_MODEL"][visc_idx]
        c_coul = compute_coulomb(eta, iz, self._a_min, c_visc, self._v_min)
        new_friction = list(self._params["PHYS_FRICTION_MODEL"])
        new_friction[coul_idx] = c_coul
        updated = merge_params_dict(self._params, {
            "PHYS_INERTIA": new_inertia,
            "PHYS_FRICTION_MODEL": new_friction,
        })
        self.push_params(updated, only_keys=["PHYS_INERTIA", "PHYS_FRICTION_MODEL"])
        return updated

    def _run_single_trial(self, iz: float, replicate_idx: int,
                          replicates_total: int) -> float:
        """One physical triangular pulse at ``iz``; returns the **secant
        error** ``-(realized - commanded)`` (negated so the secant search
        drives in the correct direction — realized < commanded ⇒ iz too
        small ⇒ need iz larger). Does NOT touch ``self._best`` or the
        progress cache.
        """
        updated = self._apply_candidate(iz)
        candidate = {
            "iz": float(iz),
            "replicate_idx": int(replicate_idx),
            "replicates_total": int(replicates_total),
        }
        best_desc = (f"iz={self._best[0]:.5f}  |err|={abs(self._best[1]):.4f}"
                     if self._best is not None else None)
        log_lines(self.get_logger(), format_trial_start_banner(
            "inertia", "theta", self._trial_idx, candidate, updated,
            best_desc=best_desc,
        ))
        outcome = run_triangular_trial(
            self, self.amplitude, err_fn=accel_match_error,
        )
        record = TrialRecord(
            routine="inertia", axis="theta",
            trial_idx=self._trial_idx,
            candidate=candidate,
            metrics=outcome.metrics,
            accepted=False,
            notes=(f"amplitude={self.amplitude}; "
                   f"replicate {replicate_idx + 1}/{replicates_total}"),
        )
        save_trial(self._ckpt, record, telemetry=self.buffer.to_arrays())  # type: ignore[arg-type]
        best_desc = (f"iz={self._best[0]:.5f}  |err|={abs(self._best[1]):.4f}"
                     if self._best is not None else "(none yet)")
        log_lines(self.get_logger(), format_trial_end_banner(
            "inertia", "theta", self._trial_idx, outcome,
            best_desc=best_desc,
        ))
        self.get_logger().info(
            f"  err_signed={outcome.error_signed:+.4f}  (target 0)"
        )
        self._trial_idx += 1
        self._params = updated
        # Inertia is angular; settling is fine, no manual rotation needed.
        self.wait_for_turnaround(prompt=False)
        return float(-outcome.error_signed)

    def _evaluate(self, iz: float) -> float:
        if self._cache is not None:
            cached = self._cache.find(iz)
            if cached is not None:
                self.get_logger().info(
                    f"[inertia][theta] cache hit iz={iz:.5f} "
                    f"→ err={cached['score']:.4f} (median of "
                    f"{cached.get('extra', {}).get('replicates_total', '?')} "
                    f"replicates)"
                )
                self._params = self._apply_candidate(iz)
                return float(cached["score"])

        self.get_logger().info(
            f"[inertia][theta] EVALUATE iz={iz:.5f}: "
            f"{self._replicates} replicates "
            f"(max {self._max_replacements} outlier replacements, "
            f"MAD threshold {self._outlier_mad}σ)"
        )
        summary = replicated_evaluate(
            candidate_label=f"[inertia][theta] iz={iz:.5f}",
            logger=self.get_logger(),
            run_trial=lambda i, n: self._run_single_trial(iz, i, n),
            replicates=self._replicates,
            max_replacements=self._max_replacements,
            outlier_mad=self._outlier_mad,
        )
        aggregated_err = summary["aggregated"]
        candidate_record = {
            "iz": float(iz),
            "secant_errors": summary["scores"],
            "secant_err_median": aggregated_err,
            "secant_err_mean": summary["mean"],
            "secant_err_std": summary["std"],
            "secant_err_min": summary["min"],
            "secant_err_max": summary["max"],
            "replicates_total": summary["replicates_total"],
            "replacements": summary["replacements"],
            "outlier_threshold_final": summary["outlier_threshold_final"],
        }
        self._candidate_history.append(candidate_record)
        self.get_logger().info(
            f"[inertia][theta] iz={iz:.5f} AGGREGATED: "
            f"err_median={aggregated_err:+.4f}  mean={summary['mean']:+.4f} "
            f"± {summary['std']:.4f}  "
            f"(min={summary['min']:+.4f}, max={summary['max']:+.4f}, "
            f"replacements={summary['replacements_done']}/{self._max_replacements}, "
            f"threshold=±{summary['outlier_threshold_final']:.4f})"
        )
        if self._best is None or abs(aggregated_err) < abs(self._best[1]):
            self._best = (iz, aggregated_err)
        if self._cache is not None:
            self._cache.append(iz, aggregated_err, extra={
                "trial_idx_end": self._trial_idx,
                "replicates_total": summary["replicates_total"],
                "replacements": summary["replacements_done"],
                "raw_secant_errors": summary["scores"],
            })
        return aggregated_err

    def run_single(self, ckpt: CheckpointDir, params: dict,
                    a_min: float, v_min: float, iz_value: float,
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
            ckpt.routine_dir("inertia", "theta") / "progress.json")
        self._trial_idx = (len(self._cache.entries)
                           * (self._replicates + self._max_replacements))
        err = self._evaluate(iz_value)
        result = {
            "axis": "theta", "mode": "single",
            "iz": float(iz_value),
            "error_signed": float(err),
            "params_after": self._params,
            "candidates": self._candidate_history,
            "replicates_per_candidate": self._replicates,
            "max_replacements_per_candidate": self._max_replacements,
            "outlier_mad": self._outlier_mad,
        }
        write_json(ckpt.routine_result("inertia", "theta"), result)
        return result

    def run_search(self, ckpt: CheckpointDir, params: dict,
                    a_min: float, v_min: float,
                    initial: Optional[float] = None,
                    bounds: tuple[float, float] = (DEFAULT_IZ_LOW,
                                                     DEFAULT_IZ_HIGH),
                    tol: float = DEFAULT_IZ_TOL,
                    max_iter: int = DEFAULT_IZ_MAX_ITER,
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
            ckpt.routine_dir("inertia", "theta") / "progress.json")
        self._trial_idx = (len(self._cache.entries)
                           * (self._replicates + self._max_replacements))
        if self._cache.entries:
            self.get_logger().info(
                f"[inertia][theta] resuming from cache "
                f"({len(self._cache.entries)} prior aggregated evaluations)"
            )
        if initial is None:
            initial = float(params["PHYS_INERTIA"][1])
        self.get_logger().info(
            f"[inertia][theta] bracketed search initial={initial} "
            f"bounds={bounds}  tol_y={tol}  tol_x_frac={tol_x_frac}  "
            f"min_iter={min_iter}  max_iter={max_iter}  "
            f"expand_factor={expand_factor}  "
            f"replicates_per_candidate={self._replicates}  "
            f"max_replacements_per_candidate={self._max_replacements}  "
            f"outlier_mad={self._outlier_mad}σ"
        )
        x0 = max(bounds[0], initial * 0.5)
        x1 = min(bounds[1], initial * 1.5)
        if abs(x1 - x0) < (bounds[1] - bounds[0]) * 0.01:
            x1 = min(bounds[1], x0 + (bounds[1] - bounds[0]) * 0.1)
        best_x, best_err = bracketed_root_search(
            self._evaluate, x0, x1,
            bounds=bounds, tol_y=tol, tol_x_frac=tol_x_frac,
            max_iter=max_iter, min_iter=min_iter,
            expand_factor=expand_factor,
            on_iter=lambda i, x, e, phase: self.get_logger().info(
                f"[inertia][theta] iter {i} [{phase}]: iz={x:.5f} err={e:+.4f}"
            ),
        )
        self._apply_candidate(best_x)
        result = {
            "axis": "theta", "mode": "search",
            "iz": float(best_x),
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
        write_json(ckpt.routine_result("inertia", "theta"), result)
        return result


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--mode", choices=["single", "search"], default="search")
    p.add_argument("--value", type=float, default=None,
                    help="(--mode single) iz to evaluate")
    p.add_argument("--initial", type=float, default=None,
                    help="Initial guess for iz (defaults to current value)")
    p.add_argument("--a-min", type=float, required=False)
    p.add_argument("--v-min", type=float, default=0.0)
    p.add_argument("--amplitude", type=float, default=None,
                    help="Triangular angular-pulse amplitude (axis-default if unset)")
    p.add_argument("--bounds-low", type=float, default=DEFAULT_IZ_LOW)
    p.add_argument("--bounds-high", type=float, default=DEFAULT_IZ_HIGH)
    p.add_argument("--tol", type=float, default=DEFAULT_IZ_TOL,
                    help="|f(x)| stop threshold (tol_y). All of --tol, "
                         "--tol-x-frac, --min-iter must be met to terminate. "
                         f"Default {DEFAULT_IZ_TOL}.")
    p.add_argument("--tol-x-frac", type=float, default=DEFAULT_TOL_X_FRAC,
                    help="Bracket-width stop threshold as fraction of "
                         "(bounds_high - bounds_low). "
                         f"Default {DEFAULT_TOL_X_FRAC}.")
    p.add_argument("--min-iter", type=int, default=DEFAULT_MIN_ITER,
                    help="Minimum evaluations before any tol-based exit is "
                         f"honored. Default {DEFAULT_MIN_ITER}.")
    p.add_argument("--expand-factor", type=float, default=DEFAULT_EXPAND_FACTOR,
                    help="Multiplicative seed-expansion step when initial "
                         f"seeds don't bracket. Default {DEFAULT_EXPAND_FACTOR}.")
    p.add_argument("--max-iter", type=int, default=DEFAULT_IZ_MAX_ITER)
    p.add_argument("--replicates", type=int,
                    default=DEFAULT_REPLICATES_PER_CANDIDATE,
                    help="Per-candidate replicate count; search consumes the "
                         "median secant-error. Set 1 to restore single-shot. "
                         f"Default {DEFAULT_REPLICATES_PER_CANDIDATE}.")
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
