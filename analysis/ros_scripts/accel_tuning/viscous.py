"""Viscous-friction tuning routine.

Runs triangular acceleration profiles and searches for the viscous
coefficient that keeps the velocity ramps as straight as possible. On each
candidate, the coulomb coefficient is recomputed from the existing motor
efficiency, inertia, and the ``(a_min, v_min)`` recorded by the coulomb
step so that the friction balance is preserved.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy

from .analysis import SlopeFit, linearity_score
from .base import ANGULAR_AXES, BaseTuneNode
from .checkpoint import CheckpointDir, TrialRecord, save_trial, write_json
from .coulomb import axis_friction_indices, axis_inertia, compute_coulomb
from ._search import (
    format_trial_end_banner,
    format_trial_start_banner,
    golden_section_search,
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
DEFAULT_VISC_LOW_LIN = 0.0
DEFAULT_VISC_HIGH_LIN = 12.0
DEFAULT_VISC_LOW_ANG = 0.0
DEFAULT_VISC_HIGH_ANG = 0.05
DEFAULT_SEARCH_TOL = 0.05
DEFAULT_MAX_ITER = 8
# Per-candidate noise control. EVERY golden-section evaluation runs
# ``replicates`` trials at the same c_visc; the median is returned to the
# search so the bracket isn't poisoned by a single noisy trial. Outliers
# (|score - median| > k * 1.4826 * MAD) get replaced with extra trials up to
# ``max_replacements`` times. Setting replicates=1 + max_replacements=0
# restores the old single-shot behavior. Be aware: total trials per search
# ≈ max_iter * (replicates + average_replacements_used), so each turnaround
# costs you ~3-5x the old search.
DEFAULT_REPLICATES_PER_CANDIDATE = 3
DEFAULT_MAX_REPLACEMENTS_PER_CANDIDATE = 2
DEFAULT_OUTLIER_MAD = 3.0
# ===================


def viscous_bounds(axis: str) -> tuple[float, float]:
    if axis in ANGULAR_AXES:
        return (DEFAULT_VISC_LOW_ANG, DEFAULT_VISC_HIGH_ANG)
    return (DEFAULT_VISC_LOW_LIN, DEFAULT_VISC_HIGH_LIN)


class ViscousRoutine(BaseTuneNode):
    """Triangular-profile search for the viscous coefficient."""

    def __init__(self, robot_id: int, axis: str, amplitude: float,
                 dry_run: bool = False, skip_turnaround: bool = False):
        super().__init__("viscous_tune", robot_id, axis, dry_run=dry_run,
                         skip_turnaround=skip_turnaround)
        self.amplitude = float(amplitude)
        self._ckpt: Optional[CheckpointDir] = None
        self._params: dict = {}
        self._a_min: float = 0.0
        self._v_min: float = 0.0
        self._trial_idx = 0
        self._best: Optional[tuple[float, float]] = None
        self._cache: Optional[ProgressCache] = None
        # Per-candidate noise control (set by run_search / run_single).
        self._replicates: int = DEFAULT_REPLICATES_PER_CANDIDATE
        self._max_replacements: int = DEFAULT_MAX_REPLACEMENTS_PER_CANDIDATE
        self._outlier_mad: float = DEFAULT_OUTLIER_MAD
        # Tracks per-candidate replicate stats; serialised into result.json.
        self._candidate_history: list[dict] = []

    # ------------------------------------------------------------------
    # Trial loop
    # ------------------------------------------------------------------

    def _apply_candidate(self, c_visc: float) -> dict:
        """Update params dict with this c_visc and the consistent c_coul.

        Returns the updated params dict (also pushed to the robot).
        """
        eta = self._params["PHYS_MOTOR_MODEL"][1]
        inertia = axis_inertia(self.axis, self._params)
        c_coul = compute_coulomb(eta, inertia, self._a_min, c_visc, self._v_min)
        coul_idx, visc_idx = axis_friction_indices(self.axis)
        new_friction = list(self._params["PHYS_FRICTION_MODEL"])
        new_friction[coul_idx] = c_coul
        new_friction[visc_idx] = c_visc
        updated = merge_params_dict(self._params, {
            "PHYS_FRICTION_MODEL": new_friction,
        })
        self.push_params(updated, only_keys=["PHYS_FRICTION_MODEL"])
        return updated

    def _run_single_trial(self, c_visc: float, replicate_idx: int,
                          replicates_total: int) -> float:
        """Run one triangular pulse at ``c_visc`` and return its score.

        Pushes params (the firmware may have been updated between trials by
        other clients, so push every time), runs the pulse, saves trial
        record + NPZ, increments ``trial_idx``, and waits for the manual
        turnaround on linear axes. Does NOT update ``self._best`` or the
        progress cache — those are done at the aggregated-evaluation level
        by ``_evaluate``.
        """
        updated = self._apply_candidate(c_visc)
        candidate = {
            "c_visc": float(c_visc),
            "c_coul": float(self._coul_for(updated)),
            "replicate_idx": int(replicate_idx),
            "replicates_total": int(replicates_total),
        }
        best_desc = (f"c_visc={self._best[0]:.4f}  score={self._best[1]:.4f}"
                     if self._best is not None else None)
        log_lines(self.get_logger(), format_trial_start_banner(
            "viscous", self.axis, self._trial_idx, candidate, updated,
            best_desc=best_desc,
        ))
        outcome = run_triangular_trial(self, self.amplitude)
        record = TrialRecord(
            routine="viscous",
            axis=self.axis,
            trial_idx=self._trial_idx,
            candidate=candidate,
            metrics=outcome.metrics,
            accepted=False,
            notes=(f"amplitude={self.amplitude}; "
                   f"replicate {replicate_idx + 1}/{replicates_total}"),
        )
        save_trial(self._ckpt, record, telemetry=self.buffer.to_arrays())  # type: ignore[arg-type]
        best_desc = (f"c_visc={self._best[0]:.4f}  score={self._best[1]:.4f}"
                     if self._best is not None else "(none yet)")
        log_lines(self.get_logger(), format_trial_end_banner(
            "viscous", self.axis, self._trial_idx, outcome,
            best_desc=best_desc,
        ))
        self._trial_idx += 1
        self._params = updated
        # Manual turnaround on linear axes; angular routes through the
        # rotational settle helper (prompt=False).
        if self.axis not in ANGULAR_AXES:
            self.wait_for_turnaround()
        else:
            self.wait_for_turnaround(prompt=False)
        return outcome.score.total

    def _evaluate(self, c_visc: float) -> float:
        """Replicated-trial evaluation called by the golden-section search.

        For each candidate ``c_visc`` the search asks about:

        1. Cache check: if we've previously aggregated a score for this
           ``c_visc`` (within ProgressCache rounding), return it. This is
           the only thing that makes ``--run-id`` resume cheap.
        2. Delegate to ``replicated_evaluate`` for the replicate +
           outlier-replacement loop.
        3. Aggregate to median, push to cache, update running-best, return.

        The aggregated median is what the golden-section sees, so the
        bracket isn't poisoned by a single noisy trial.
        """
        if self._cache is not None:
            cached = self._cache.find(c_visc)
            if cached is not None:
                self.get_logger().info(
                    f"[viscous][{self.axis}] cache hit c_visc={c_visc:.4f} "
                    f"→ score={cached['score']:.4f} (median of "
                    f"{cached.get('extra', {}).get('replicates_total', '?')} "
                    f"replicates)"
                )
                self._params = self._apply_candidate(c_visc)
                return float(cached["score"])

        self.get_logger().info(
            f"[viscous][{self.axis}] EVALUATE c_visc={c_visc:.4f}: "
            f"{self._replicates} replicates "
            f"(max {self._max_replacements} outlier replacements, "
            f"MAD threshold {self._outlier_mad}σ)"
        )

        summary = replicated_evaluate(
            candidate_label=f"[viscous][{self.axis}] c_visc={c_visc:.4f}",
            logger=self.get_logger(),
            run_trial=lambda i, n: self._run_single_trial(c_visc, i, n),
            replicates=self._replicates,
            max_replacements=self._max_replacements,
            outlier_mad=self._outlier_mad,
        )
        aggregated = summary["aggregated"]
        candidate_record = {
            "c_visc": float(c_visc),
            "scores": summary["scores"],
            "score_median": aggregated,
            "score_mean": summary["mean"],
            "score_std": summary["std"],
            "score_min": summary["min"],
            "score_max": summary["max"],
            "replicates_total": summary["replicates_total"],
            "replacements": summary["replacements"],
            "outlier_threshold_final": summary["outlier_threshold_final"],
        }
        self._candidate_history.append(candidate_record)

        self.get_logger().info(
            f"[viscous][{self.axis}] c_visc={c_visc:.4f} AGGREGATED: "
            f"median={aggregated:.4f}  mean={summary['mean']:.4f} "
            f"± {summary['std']:.4f}  "
            f"(min={summary['min']:.4f}, max={summary['max']:.4f}, "
            f"replacements={summary['replacements_done']}/{self._max_replacements}, "
            f"threshold=±{summary['outlier_threshold_final']:.4f})"
        )

        if self._best is None or aggregated < self._best[1]:
            self._best = (c_visc, aggregated)

        # Cache the aggregated score so a resumed GS replays the same
        # deterministic candidate sequence and skips re-evaluation.
        if self._cache is not None:
            self._cache.append(c_visc, aggregated, extra={
                "trial_idx_end": self._trial_idx,
                "replicates_total": summary["replicates_total"],
                "replacements": summary["replacements_done"],
                "raw_scores": summary["scores"],
            })
        return aggregated

    def _coul_for(self, params: dict) -> float:
        coul_idx, _ = axis_friction_indices(self.axis)
        return params["PHYS_FRICTION_MODEL"][coul_idx]

    # ------------------------------------------------------------------
    # Public entry points
    # ------------------------------------------------------------------

    def run_single(self, ckpt: CheckpointDir, params: dict,
                    a_min: float, v_min: float,
                    c_visc_value: float,
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
            ckpt.routine_dir("viscous", self.axis) / "progress.json")
        # Seed trial_idx so re-runs don't overwrite earlier trial NPZs.
        self._trial_idx = len(self._cache.entries) * self._replicates
        aggregated = self._evaluate(c_visc_value)
        result = {
            "axis": self.axis,
            "mode": "single",
            "c_visc": float(c_visc_value),
            "score": float(aggregated),
            "params_after": self._params,
            "candidates": self._candidate_history,
            "replicates_per_candidate": self._replicates,
            "max_replacements_per_candidate": self._max_replacements,
            "outlier_mad": self._outlier_mad,
        }
        write_json(ckpt.routine_result("viscous", self.axis), result)
        return result

    def run_search(self, ckpt: CheckpointDir, params: dict,
                    a_min: float, v_min: float,
                    bounds: Optional[tuple[float, float]] = None,
                    tol: float = DEFAULT_SEARCH_TOL,
                    max_iter: int = DEFAULT_MAX_ITER,
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
            ckpt.routine_dir("viscous", self.axis) / "progress.json")
        # Each cache entry corresponds to one aggregated candidate, which is
        # backed by up to (replicates + max_replacements) physical trials.
        # Seed trial_idx pessimistically so resume can't clobber prior NPZs.
        self._trial_idx = (len(self._cache.entries)
                           * (self._replicates + self._max_replacements))
        if self._cache.entries:
            self.get_logger().info(
                f"[viscous][{self.axis}] resuming from cache "
                f"({len(self._cache.entries)} prior aggregated evaluations)"
            )
        lo, hi = bounds if bounds is not None else viscous_bounds(self.axis)
        self.get_logger().info(
            f"[viscous][{self.axis}] search bounds=[{lo}, {hi}]  "
            f"tol={tol}  max_iter={max_iter}  "
            f"replicates_per_candidate={self._replicates}  "
            f"max_replacements_per_candidate={self._max_replacements}  "
            f"outlier_mad={self._outlier_mad}σ"
        )
        best_x, best_score = golden_section_search(
            self._evaluate, lo, hi, tol=tol, max_iter=max_iter,
            on_iter=lambda i, x, s: self.get_logger().info(
                f"[viscous][{self.axis}] iter {i}: "
                f"best c_visc={x:.4f} score={s:.4f} (median of replicates)"
            ),
        )
        # Push the winning candidate one more time so the firmware is left
        # in the converged-state friction model.
        self._apply_candidate(best_x)

        result = {
            "axis": self.axis,
            "mode": "search",
            "c_visc": float(best_x),
            "score": float(best_score),
            "bounds": [lo, hi],
            "params_after": self._params,
            "a_min": float(a_min),
            "v_min": float(v_min),
            "candidates": self._candidate_history,
            "replicates_per_candidate": self._replicates,
            "max_replacements_per_candidate": self._max_replacements,
            "outlier_mad": self._outlier_mad,
        }
        write_json(ckpt.routine_result("viscous", self.axis), result)
        return result


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--mode", choices=["single", "search"], default="search")
    p.add_argument("--value", type=float, default=None,
                    help="(--mode single) c_visc to evaluate")
    p.add_argument("--a-min", type=float, required=False,
                    help="a_min from prior coulomb step (m/s^2 or rad/s^2)")
    p.add_argument("--v-min", type=float, required=False, default=0.0,
                    help="v_min from prior coulomb step (m/s or rad/s)")
    p.add_argument("--amplitude", type=float, default=None,
                    help="Triangular pulse amplitude (axis-default if unset)")
    p.add_argument("--bounds-low", type=float, default=None)
    p.add_argument("--bounds-high", type=float, default=None)
    p.add_argument("--tol", type=float, default=DEFAULT_SEARCH_TOL)
    p.add_argument("--max-iter", type=int, default=DEFAULT_MAX_ITER)
    p.add_argument("--replicates", type=int,
                    default=DEFAULT_REPLICATES_PER_CANDIDATE,
                    help="Per-candidate replicate count: every golden-section "
                         "evaluation runs this many trials at the same c_visc "
                         "and returns the median to the search. Set 1 to "
                         "restore single-shot behavior. "
                         f"Default {DEFAULT_REPLICATES_PER_CANDIDATE}.")
    p.add_argument("--max-replacements", type=int,
                    default=DEFAULT_MAX_REPLACEMENTS_PER_CANDIDATE,
                    help="Per-candidate outlier-replacement cap. After the "
                         "initial replicates, any trial more than "
                         "outlier-mad·1.4826·MAD from the median may be "
                         "replaced by a fresh trial, up to this many times. "
                         f"Default {DEFAULT_MAX_REPLACEMENTS_PER_CANDIDATE}.")
    p.add_argument("--outlier-mad", type=float,
                    default=DEFAULT_OUTLIER_MAD,
                    help="MAD-σ multiplier for per-candidate outlier "
                         "detection (lower = stricter). "
                         f"Default {DEFAULT_OUTLIER_MAD}.")
    return p
