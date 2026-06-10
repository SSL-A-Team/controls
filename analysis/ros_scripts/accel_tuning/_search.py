"""Shared trial-runner and scalar-search helpers for triangular-profile
tuning routines (viscous, efficiency, inertia).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np
import rclpy

from .analysis import (
    SlopeFit,
    TriangularScore,
    fit_slope,
    triangular_profile_score,
)
from .base import (
    AXIS_TO_COMPONENT,
    BaseTuneNode,
    PHASE_COAST,
    PHASE_PULSE_NEG,
    PHASE_PULSE_POS,
)
from .checkpoint import CheckpointDir, TrialRecord, save_trial
from .triangular import T_PULSE, TriangularPulseRunner, default_pulse_accel


@dataclass
class TrialOutcome:
    pos_fit: Optional[SlopeFit]
    neg_fit: Optional[SlopeFit]
    coast_fit: Optional[SlopeFit]
    commanded_accel: float
    score: TriangularScore
    error_signed: float       # realized - commanded, sign-relevant for secant
    metrics: dict


def run_triangular_trial(node: BaseTuneNode, amplitude: float,
                          err_fn: Optional[Callable[
                              [Optional[SlopeFit], Optional[SlopeFit], float],
                              float]] = None,
                          t_pulse: float = T_PULSE,
                          ) -> TrialOutcome:
    """Execute one triangular trial and return fit/score metrics.

    Caller is responsible for pushing any param updates before calling.
    """
    node.begin_trial()
    runner = TriangularPulseRunner(node, amplitude=amplitude)

    # Hook the runner into the node's ticking. Save/restore on_tick so
    # multiple trials can be run sequentially without leaking state.
    original_on_tick = node.on_tick
    node.on_tick = runner.tick  # type: ignore[assignment]
    try:
        while rclpy.ok() and not runner.done:
            rclpy.spin_once(node, timeout_sec=1.0 / node.PUBLISH_RATE_HZ)
    finally:
        node.on_tick = original_on_tick  # type: ignore[assignment]
        node.publish_off()

    t_pos, v_pos = node.buffer.axis_velocity(PHASE_PULSE_POS, node.axis)
    t_neg, v_neg = node.buffer.axis_velocity(PHASE_PULSE_NEG, node.axis)
    t_coast, v_coast = node.buffer.axis_velocity(PHASE_COAST, node.axis)
    pos_fit = fit_slope(t_pos, v_pos)
    neg_fit = fit_slope(t_neg, v_neg)
    coast_fit = fit_slope(t_coast, v_coast)
    score = triangular_profile_score(pos_fit, neg_fit, coast_fit,
                                       amplitude, t_pulse)
    err = (err_fn(pos_fit, neg_fit, amplitude)
           if err_fn is not None else float("nan"))
    metrics = {
        "commanded_accel": float(amplitude),
        "pos_slope": pos_fit.slope if pos_fit else None,
        "neg_slope": neg_fit.slope if neg_fit else None,
        "coast_slope": coast_fit.slope if coast_fit else None,
        "pos_residual_rms": pos_fit.residual_rms if pos_fit else None,
        "neg_residual_rms": neg_fit.residual_rms if neg_fit else None,
        "coast_residual_rms": coast_fit.residual_rms if coast_fit else None,
        "pos_samples": pos_fit.n_samples if pos_fit else 0,
        "neg_samples": neg_fit.n_samples if neg_fit else 0,
        "coast_samples": coast_fit.n_samples if coast_fit else 0,
        "v_peak_pos": pos_fit.v_end if pos_fit else None,
        "v_peak_neg": neg_fit.v_start if neg_fit else None,
        "peak_v": float(score.peak_v),
        "expected_peak_v": float(score.expected_peak),
        "low_motion": bool(score.low_motion),
        "score_total": (float(score.total)
                        if not math.isinf(score.total) else None),
        "score_residual": (float(score.residual)
                           if not math.isinf(score.residual) else None),
        "score_asymmetry": (float(score.asymmetry)
                            if not math.isinf(score.asymmetry) else None),
        "score_coast_flatness": float(score.coast_flatness),
        "error_signed": (float(err) if (err is not None
                                          and not math.isnan(err)) else None),
    }
    return TrialOutcome(pos_fit=pos_fit, neg_fit=neg_fit, coast_fit=coast_fit,
                         commanded_accel=amplitude,
                         score=score,
                         error_signed=err if err is not None else float("nan"),
                         metrics=metrics)


def format_score_log(routine: str, axis: str, candidate_desc: str,
                      outcome: TrialOutcome) -> str:
    """Human-friendly one-line summary of a trial result.

    Explicitly notes that lower scores are better and flags low-motion
    trials.
    """
    s = outcome.score
    if math.isinf(s.total):
        return (f"[{routine}][{axis}] {candidate_desc}: NO FIT "
                "(missing telemetry on one of the pulse phases)")
    parts = [
        f"score={s.total:.4f} (lower is better)",
        f"residual={s.residual:.4f}",
        f"asym={s.asymmetry:.4f}",
        f"coast_flat={s.coast_flatness:.4f}",
        f"peak_v={s.peak_v:.3f}/{s.expected_peak:.3f}",
    ]
    if s.low_motion:
        parts.append(
            f"LOW_MOTION (peak<{int(100 * 0.25)}% expected, +penalty)"
        )
    return f"[{routine}][{axis}] {candidate_desc}: " + "  ".join(parts)


# --------------------------------------------------------------------------
# Trial banners
# --------------------------------------------------------------------------

_BANNER_WIDTH = 78


def _rule(char: str = "─") -> str:
    return char * _BANNER_WIDTH


def format_trial_start_banner(routine: str, axis: str, trial_idx: int,
                                candidate: dict,
                                pushed_params: dict,
                                best_desc: Optional[str] = None) -> list[str]:
    """Multi-line banner printed before a trial begins.

    Includes the trial number, the candidate parameter(s) the search picked,
    a snapshot of the friction/motor/inertia parameters that were just
    pushed to firmware, and the current best (if any).
    """
    cand_str = ", ".join(
        f"{k}={v:.4f}" if isinstance(v, (int, float)) else f"{k}={v}"
        for k, v in candidate.items()
    )
    fric = pushed_params.get("PHYS_FRICTION_MODEL")
    motor = pushed_params.get("PHYS_MOTOR_MODEL")
    inertia = pushed_params.get("PHYS_INERTIA")
    rows = [_rule("═")]
    rows.append(f"▶ {routine.upper()} · {axis} · TRIAL {trial_idx:04d}")
    rows.append(_rule("─"))
    rows.append(f"  candidate: {cand_str}")
    if fric is not None and len(fric) >= 6:
        rows.append(
            f"  friction:  c_coul=[x:{fric[0]:.4f}, y:{fric[1]:.4f}, "
            f"θ:{fric[2]:.4f}]"
        )
        rows.append(
            f"             c_visc=[x:{fric[3]:.4f}, y:{fric[4]:.4f}, "
            f"θ:{fric[5]:.4f}]"
        )
    if motor is not None:
        rows.append(f"  motor:     Kt={motor[0]:.4f}  η={motor[1]:.4f}")
    if inertia is not None:
        rows.append(f"  inertia:   mass={inertia[0]:.4f}  iz={inertia[1]:.5f}")
    if best_desc is not None:
        rows.append(f"  best so far: {best_desc}")
    rows.append(_rule("─"))
    return rows


def format_trial_end_banner(routine: str, axis: str, trial_idx: int,
                              outcome: TrialOutcome,
                              best_desc: Optional[str] = None) -> list[str]:
    """Compact end-of-trial banner showing the score line + current best."""
    s = outcome.score
    rows = [
        format_score_log(routine, axis,
                          f"TRIAL {trial_idx:04d} result", outcome),
    ]
    if s.low_motion and not math.isinf(s.total):
        rows.append(
            f"  ↑ LOW MOTION penalty (peak {s.peak_v:.3f} < "
            f"{int(100 * 0.25)}% of expected {s.expected_peak:.3f})"
        )
    if best_desc is not None:
        rows.append(f"  best so far: {best_desc}")
    rows.append(_rule("═"))
    return rows


def log_lines(logger, lines: list[str], level: str = "info") -> None:
    """Emit ``lines`` to ``logger`` one at a time at the given level."""
    fn = getattr(logger, level)
    for line in lines:
        fn(line)


# --------------------------------------------------------------------------
# Scalar search algorithms
# --------------------------------------------------------------------------

def golden_section_search(evaluate: Callable[[float], float],
                           low: float, high: float,
                           tol: float = 1e-3, max_iter: int = 12,
                           on_iter: Optional[Callable[
                               [int, float, float], None]] = None,
                           ) -> tuple[float, float]:
    """Minimise ``evaluate`` over ``[low, high]`` via golden-section.

    Returns ``(best_x, best_score)``. ``evaluate`` is assumed expensive (one
    physical trial per call), so this routine evaluates each midpoint at
    most once per iteration.
    """
    phi = (math.sqrt(5.0) - 1.0) / 2.0
    a, b = float(low), float(high)
    c = b - phi * (b - a)
    d = a + phi * (b - a)
    fc = evaluate(c)
    fd = evaluate(d)
    best_x, best_score = (c, fc) if fc < fd else (d, fd)
    if on_iter is not None:
        on_iter(0, best_x, best_score)

    for i in range(1, max_iter + 1):
        if abs(b - a) < tol:
            break
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - phi * (b - a)
            fc = evaluate(c)
        else:
            a, c, fc = c, d, fd
            d = a + phi * (b - a)
            fd = evaluate(d)
        cand_x, cand_score = (c, fc) if fc < fd else (d, fd)
        if cand_score < best_score:
            best_x, best_score = cand_x, cand_score
        if on_iter is not None:
            on_iter(i, best_x, best_score)
    return best_x, best_score


def secant_search(evaluate: Callable[[float], float],
                   x0: float, x1: float,
                   tol: float = 1e-3, max_iter: int = 8,
                   bounds: Optional[tuple[float, float]] = None,
                   on_iter: Optional[Callable[
                       [int, float, float], None]] = None,
                   ) -> tuple[float, float]:
    """Find a root of ``evaluate`` (signed-error) via secant iterations.

    Returns ``(best_x, last_error)``. Falls back to bisection-style steps
    when the secant denominator is tiny or the next point would leave bounds.
    """
    lo, hi = bounds if bounds is not None else (-math.inf, math.inf)
    f0 = evaluate(x0)
    if on_iter is not None:
        on_iter(0, x0, f0)
    f1 = evaluate(x1)
    if on_iter is not None:
        on_iter(1, x1, f1)
    best_x, best_err = (x0, abs(f0))
    if abs(f1) < best_err:
        best_x, best_err = x1, abs(f1)

    for i in range(2, max_iter + 1):
        if abs(f1) < tol:
            break
        denom = f1 - f0
        if abs(denom) < 1e-9:
            x_next = 0.5 * (x0 + x1)
        else:
            x_next = x1 - f1 * (x1 - x0) / denom
        x_next = float(np.clip(x_next, lo, hi))
        f_next = evaluate(x_next)
        if on_iter is not None:
            on_iter(i, x_next, f_next)
        x0, f0 = x1, f1
        x1, f1 = x_next, f_next
        if abs(f1) < best_err:
            best_x, best_err = x1, abs(f1)
    return best_x, best_err



# --------------------------------------------------------------------------
# Scalar search algorithms
# --------------------------------------------------------------------------

def golden_section_search(evaluate: Callable[[float], float],
                           low: float, high: float,
                           tol: float = 1e-3, max_iter: int = 12,
                           on_iter: Optional[Callable[
                               [int, float, float], None]] = None,
                           ) -> tuple[float, float]:
    """Minimise ``evaluate`` over ``[low, high]`` via golden-section.

    Returns ``(best_x, best_score)``. ``evaluate`` is assumed expensive (one
    physical trial per call), so this routine evaluates each midpoint at
    most once per iteration.
    """
    phi = (math.sqrt(5.0) - 1.0) / 2.0
    a, b = float(low), float(high)
    c = b - phi * (b - a)
    d = a + phi * (b - a)
    fc = evaluate(c)
    fd = evaluate(d)
    best_x, best_score = (c, fc) if fc < fd else (d, fd)
    if on_iter is not None:
        on_iter(0, best_x, best_score)

    for i in range(1, max_iter + 1):
        if abs(b - a) < tol:
            break
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - phi * (b - a)
            fc = evaluate(c)
        else:
            a, c, fc = c, d, fd
            d = a + phi * (b - a)
            fd = evaluate(d)
        cand_x, cand_score = (c, fc) if fc < fd else (d, fd)
        if cand_score < best_score:
            best_x, best_score = cand_x, cand_score
        if on_iter is not None:
            on_iter(i, best_x, best_score)
    return best_x, best_score


def secant_search(evaluate: Callable[[float], float],
                   x0: float, x1: float,
                   tol: float = 1e-3, max_iter: int = 8,
                   bounds: Optional[tuple[float, float]] = None,
                   on_iter: Optional[Callable[
                       [int, float, float], None]] = None,
                   ) -> tuple[float, float]:
    """Find a root of ``evaluate`` (signed-error) via secant iterations.

    Returns ``(best_x, last_error)``. Falls back to bisection-style steps
    when the secant denominator is tiny or the next point would leave bounds.
    """
    lo, hi = bounds if bounds is not None else (-math.inf, math.inf)
    f0 = evaluate(x0)
    if on_iter is not None:
        on_iter(0, x0, f0)
    f1 = evaluate(x1)
    if on_iter is not None:
        on_iter(1, x1, f1)
    best_x, best_err = (x0, abs(f0))
    if abs(f1) < best_err:
        best_x, best_err = x1, abs(f1)

    for i in range(2, max_iter + 1):
        if abs(f1) < tol:
            break
        denom = f1 - f0
        if abs(denom) < 1e-9:
            x_next = 0.5 * (x0 + x1)
        else:
            x_next = x1 - f1 * (x1 - x0) / denom
        x_next = float(np.clip(x_next, lo, hi))
        f_next = evaluate(x_next)
        if on_iter is not None:
            on_iter(i, x_next, f_next)
        x0, f0 = x1, f1
        x1, f1 = x_next, f_next
        if abs(f1) < best_err:
            best_x, best_err = x1, abs(f1)
    return best_x, best_err


# --------------------------------------------------------------------------
# Per-candidate replicate evaluation with MAD outlier removal
# --------------------------------------------------------------------------

def replicated_evaluate(
    *,
    candidate_label: str,
    logger,
    run_trial: Callable[[int, int], float],
    replicates: int,
    max_replacements: int,
    outlier_mad: float,
    mad_floor_abs: float = 0.02,
    mad_floor_rel: float = 0.05,
) -> dict:
    """Run a candidate ``replicates`` times and replace outlier scores.

    Used by viscous / efficiency / inertia to characterise per-candidate
    score noise before handing a value back to the enclosing search
    algorithm. The **median** of the replicate set is what the search sees,
    so a single bad trial cannot poison the bracket.

    ``run_trial(replicate_idx, replicates_total)`` is the routine-specific
    callable that runs one physical trial at the same candidate and returns
    its scalar metric (a score for viscous, signed accel-match error for
    efficiency/inertia). ``replicates_total`` advances to the
    initial-plus-current-replacement count so user-facing logs can show
    "replicate 4/5" after the first replacement.

    Outlier rule::

        threshold = max(outlier_mad · 1.4826 · MAD,
                        mad_floor_rel · |median|,
                        mad_floor_abs)

    The floor terms prevent the threshold from collapsing to noise level
    when initial replicates happen to cluster very tightly (without the
    floor, an unusually consistent first batch makes any normal-jitter
    third replicate look like an outlier, and the replacement loop wastes
    trials chasing zero-mean noise).

    Returns::

        {
            "scores": [..],              # final score list after replacements
            "aggregated": float,         # median, what the search consumes
            "mean": float, "std": float,
            "min": float, "max": float,
            "replicates_total": int,     # initial + replacements_done
            "replacements_done": int,
            "replacements": [{"replaced_idx", "old_score", "new_score"}, ...],
            "outlier_threshold_final": float,
        }
    """
    scores: list[float] = []
    for i in range(replicates):
        logger.info(f"{candidate_label} replicate {i + 1}/{replicates}")
        scores.append(float(run_trial(i, replicates)))

    replacements: list[dict] = []
    threshold = 0.0
    while len(replacements) < max_replacements:
        arr = np.asarray(scores, dtype=float)
        med = float(np.median(arr))
        mad = float(np.median(np.abs(arr - med)))
        threshold = max(
            outlier_mad * mad * 1.4826,
            mad_floor_rel * abs(med),
            mad_floor_abs,
        )
        outlier_indices = [i for i, s in enumerate(scores)
                           if abs(s - med) > threshold]
        if not outlier_indices:
            break
        worst_idx = max(outlier_indices,
                        key=lambda i: abs(scores[i] - med))
        old_score = scores[worst_idx]
        logger.info(
            f"{candidate_label} outlier: replicate[{worst_idx}]="
            f"{old_score:.4f} vs median={med:.4f} (threshold=±{threshold:.4f}); "
            f"replacement {len(replacements) + 1}/{max_replacements}"
        )
        replicates_total = replicates + len(replacements) + 1
        new_score = float(run_trial(len(scores) + len(replacements),
                                     replicates_total))
        replacements.append({
            "replaced_idx": int(worst_idx),
            "old_score": float(old_score),
            "new_score": float(new_score),
        })
        scores[worst_idx] = new_score

    final = np.asarray(scores, dtype=float)
    return {
        "scores": [float(s) for s in scores],
        "aggregated": float(np.median(final)),
        "mean": float(np.mean(final)),
        "std": float(np.std(final)),
        "min": float(np.min(final)),
        "max": float(np.max(final)),
        "replicates_total": int(len(scores)),
        "replacements_done": len(replacements),
        "replacements": replacements,
        "outlier_threshold_final": float(threshold),
    }


# --------------------------------------------------------------------------
# Bracketed root search (replaces secant for noisy/exploration-needing axes)
# --------------------------------------------------------------------------

def bracketed_root_search(
    evaluate: Callable[[float], float],
    x0: float,
    x1: float,
    *,
    bounds: tuple[float, float],
    tol_y: float = 0.05,
    tol_x_frac: float = 0.05,
    max_iter: int = 12,
    min_iter: int = 4,
    expand_factor: float = 1.5,
    on_iter: Optional[Callable[[int, float, float, str], None]] = None,
) -> tuple[float, float]:
    """Bracket-and-refine root search for monotonic-ish signed-error f(x).

    Designed to replace the prior plain-secant search in efficiency and
    inertia. Solves three failure modes of the secant:

    1. **No exploration:** Plain secant exited as soon as ``|f(x1)| < tol``,
       so a lucky second seed converged in 2 evaluations with no real
       characterisation of the curve. This routine requires at least
       ``min_iter`` evaluations regardless of tol.
    2. **Unbracketed root:** Plain secant happily extrapolated when both
       seeds had the same sign — the "root" might not exist in the search
       interval. Here we first expand the seed pair outward (clamped by
       ``bounds``) until ``f`` changes sign; only then do we refine. If
       the bracket can't be found within ``max_iter``, we return the best
       |f| seen so the caller has *something*, but the routine flags it.
    3. **One-sided stagnation:** Once bracketed, we use false-position
       with the Illinois modification — it converges as fast as secant in
       the well-behaved case and prevents the slow drift you get from
       plain false-position when one endpoint never moves.

    Stop conditions (all must hold): at least ``min_iter`` evaluations
    AND bracket width ≤ ``tol_x_frac · (bounds[1] - bounds[0])`` AND
    ``|f(midpoint)| ≤ tol_y``. ``max_iter`` is the safety cap.

    The ``on_iter`` callback receives ``(eval_index, x, f, phase)`` where
    ``phase`` is one of ``"seed"``, ``"expand"``, ``"refine"``.

    Returns ``(best_x, best_abs_err)`` — the x with smallest |f| seen across
    all evaluations, not just the final one.
    """
    lo_b, hi_b = bounds
    initial_range = max(hi_b - lo_b, 1e-9)
    tol_x = tol_x_frac * initial_range

    def _track_best(x, fx, best_x, best_err):
        if abs(fx) < best_err:
            return x, abs(fx)
        return best_x, best_err

    f0 = evaluate(x0)
    if on_iter is not None:
        on_iter(0, x0, f0, "seed")
    f1 = evaluate(x1)
    if on_iter is not None:
        on_iter(1, x1, f1, "seed")

    best_x, best_err = _track_best(x0, f0, x0, math.inf)
    best_x, best_err = _track_best(x1, f1, best_x, best_err)

    iter_count = 2

    # ---- Phase 1: bracketing --------------------------------------------
    # Expand outward toward the bound on the side that's likely to contain
    # the root (the side of the smaller |f|). Each expansion step
    # multiplies the gap by expand_factor.
    while f0 * f1 > 0 and iter_count < max_iter:
        if abs(f0) <= abs(f1):
            # f0 is closer to zero; root is more likely beyond x0.
            new_x = max(lo_b, x0 - expand_factor * abs(x1 - x0))
            if new_x == x0:  # hit the bound; can't expand further
                break
            f_new = evaluate(new_x)
            iter_count += 1
            if on_iter is not None:
                on_iter(iter_count - 1, new_x, f_new, "expand")
            best_x, best_err = _track_best(new_x, f_new, best_x, best_err)
            # Slide the pair so x0 < x1 still holds, with new point on the lo end.
            x1, f1 = x0, f0
            x0, f0 = new_x, f_new
        else:
            new_x = min(hi_b, x1 + expand_factor * abs(x1 - x0))
            if new_x == x1:
                break
            f_new = evaluate(new_x)
            iter_count += 1
            if on_iter is not None:
                on_iter(iter_count - 1, new_x, f_new, "expand")
            best_x, best_err = _track_best(new_x, f_new, best_x, best_err)
            x0, f0 = x1, f1
            x1, f1 = new_x, f_new

    bracketed = (f0 * f1 < 0)

    # ---- Phase 2: refinement via false-position (Illinois) ---------------
    side = 0  # +1 = last refinement replaced x1, -1 = replaced x0
    while iter_count < max_iter:
        stop_y = abs(f0) <= tol_y or abs(f1) <= tol_y
        stop_x = bracketed and abs(x1 - x0) <= tol_x
        if iter_count >= min_iter and stop_y and stop_x:
            break

        if bracketed:
            denom = f1 - f0
            if abs(denom) < 1e-12:
                x_new = 0.5 * (x0 + x1)  # bisect
            else:
                x_new = x1 - f1 * (x1 - x0) / denom
            # Guard: keep x_new strictly inside [x0, x1].
            lo_in, hi_in = (x0, x1) if x0 < x1 else (x1, x0)
            x_new = float(np.clip(x_new, lo_in + 1e-9, hi_in - 1e-9))
        else:
            # No bracket; fall back to plain secant clipped to bounds. This
            # is the same behaviour as the old routine, but at least
            # min_iter forces continued exploration.
            denom = f1 - f0
            if abs(denom) < 1e-12:
                x_new = 0.5 * (x0 + x1)
            else:
                x_new = x1 - f1 * (x1 - x0) / denom
            x_new = float(np.clip(x_new, lo_b, hi_b))

        f_new = evaluate(x_new)
        iter_count += 1
        if on_iter is not None:
            on_iter(iter_count - 1, x_new, f_new, "refine")
        best_x, best_err = _track_best(x_new, f_new, best_x, best_err)

        if bracketed:
            # Illinois: shrink the stale endpoint's f-value to avoid
            # one-sided stagnation when false-position keeps replacing the
            # same side multiple iterations in a row.
            if f_new * f1 < 0:
                x0, f0 = x1, f1
                x1, f1 = x_new, f_new
                if side == +1:
                    f0 *= 0.5
                side = +1
            elif f_new * f0 < 0:
                x1, f1 = x_new, f_new
                if side == -1:
                    f0 *= 0.5
                side = -1
            else:
                break  # f_new == 0
        else:
            # Slide the pair, but also re-check bracket status — a fresh
            # secant point might fortuitously land on the other side.
            x0, f0 = x1, f1
            x1, f1 = x_new, f_new
            if f0 * f1 < 0:
                bracketed = True

    return best_x, best_err
