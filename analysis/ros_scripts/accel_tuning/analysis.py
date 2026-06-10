"""Shared analysis helpers for the tuning toolkit.

These functions consume the buffered telemetry from
:class:`accel_tuning.base.TelemetryBuffer` and compute scalar metrics used
by the search routines (linearity score, realized acceleration, etc.).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


# Tunable score weights / thresholds (top of file for easy editing).
ASYMMETRY_WEIGHT = 0.5            # ||slope_+| - |slope_-||
COAST_FLATNESS_WEIGHT = 1.0       # |slope_coast|
LOW_MOTION_FRACTION = 0.25        # peak < 25% of expected ⇒ low-motion penalty
LOW_MOTION_PENALTY = 5.0          # added to score when peak velocity too low


@dataclass
class SlopeFit:
    """Result of a linear fit of velocity vs. time on a phase segment."""

    slope: float            # m/s^2 (or rad/s^2)
    intercept: float        # m/s   (or rad/s)
    n_samples: int
    residual_rms: float     # RMS of velocity - (slope*t + intercept)
    duration: float         # seconds
    v_start: float
    v_end: float


def fit_slope(t: np.ndarray, v: np.ndarray) -> Optional[SlopeFit]:
    """Linear-regression fit of velocity vs. time.

    Returns ``None`` if fewer than 2 samples were provided (no fit possible).
    """
    if t.size < 2:
        return None
    coeffs = np.polyfit(t, v, 1)
    slope = float(coeffs[0])
    intercept = float(coeffs[1])
    fitted = slope * t + intercept
    residual_rms = float(np.sqrt(np.mean((v - fitted) ** 2)))
    return SlopeFit(
        slope=slope,
        intercept=intercept,
        n_samples=int(t.size),
        residual_rms=residual_rms,
        duration=float(t[-1] - t[0]),
        v_start=float(v[0]),
        v_end=float(v[-1]),
    )


@dataclass
class TriangularScore:
    """Decomposed linearity score for a triangular acceleration profile.

    Lower ``total`` is better. The components are kept so the caller can
    inspect/log them; ``total`` is what the search routines minimise.
    """

    residual: float = float("inf")     # sum of ramp residual RMS
    asymmetry: float = float("inf")    # ||slope_+| - |slope_-||
    coast_flatness: float = 0.0        # |slope_coast|
    low_motion: bool = False           # peak |v| < LOW_MOTION_FRACTION * expected
    peak_v: float = 0.0
    expected_peak: float = 0.0
    total: float = float("inf")


def triangular_profile_score(pos_fit: Optional[SlopeFit],
                              neg_fit: Optional[SlopeFit],
                              coast_fit: Optional[SlopeFit],
                              commanded_accel: float,
                              t_pulse: float) -> TriangularScore:
    """Compose the per-trial score for a triangular pulse trial.

    Components (all non-negative, lower is better):

    * ``residual`` — sum of linear-fit residual RMS for the ``+A`` and ``−A``
      ramps. Penalises curvature (the slope should be constant if the
      friction model is correct).
    * ``asymmetry`` — ``||slope_+| − |slope_−||``. The ramps should produce
      equal-magnitude slopes; mismatch means velocity-dependent friction is
      mis-modeled.
    * ``coast_flatness`` — ``|slope_coast|``. During coast we command zero
      acceleration, so the residual slope is the friction model's leftover
      effect on the body. A correctly modeled robot has ``slope_coast ≈ 0``.
    * Low-motion penalty — if the peak local-frame velocity is less than
      ``LOW_MOTION_FRACTION × (commanded_accel × t_pulse)``, add
      ``LOW_MOTION_PENALTY``. This catches degenerate trials where the
      friction model is so strong that the robot barely moves.

    Returns ``TriangularScore(total=+inf, ...)`` if either ramp fit is
    missing (no telemetry samples for that phase).
    """
    expected_peak = abs(commanded_accel) * float(t_pulse)
    if pos_fit is None or neg_fit is None:
        return TriangularScore(expected_peak=expected_peak)

    residual = pos_fit.residual_rms + neg_fit.residual_rms
    asymmetry = abs(abs(pos_fit.slope) - abs(neg_fit.slope))
    coast_flat = abs(coast_fit.slope) if coast_fit is not None else 0.0

    peak_v = max(abs(pos_fit.v_end), abs(neg_fit.v_start))
    low_motion = (expected_peak > 0
                  and peak_v < LOW_MOTION_FRACTION * expected_peak)

    total = (residual
             + ASYMMETRY_WEIGHT * asymmetry
             + COAST_FLATNESS_WEIGHT * coast_flat
             + (LOW_MOTION_PENALTY if low_motion else 0.0))

    return TriangularScore(
        residual=residual,
        asymmetry=asymmetry,
        coast_flatness=coast_flat,
        low_motion=low_motion,
        peak_v=peak_v,
        expected_peak=expected_peak,
        total=total,
    )


def linearity_score(pos_fit: Optional[SlopeFit],
                    neg_fit: Optional[SlopeFit],
                    commanded_accel: float) -> float:
    """Backwards-compatible scalar score (no coast term, no low-motion).

    Kept for callers that still want a simple scalar; new code should use
    :func:`triangular_profile_score` for the full decomposition.
    """
    if pos_fit is None or neg_fit is None:
        return float("inf")
    return (pos_fit.residual_rms + neg_fit.residual_rms
            + ASYMMETRY_WEIGHT * abs(abs(pos_fit.slope) - abs(neg_fit.slope)))


def accel_match_error(pos_fit: Optional[SlopeFit],
                      neg_fit: Optional[SlopeFit],
                      commanded_accel: float) -> float:
    """Signed error: mean(realized_+, -realized_-) - commanded.

    The sign tells the secant search which direction to step:
      * positive → realized > commanded → robot is moving more than expected
        (e.g. for efficiency, need larger ``eta``; for inertia, need larger
        ``iz``).
      * negative → realized < commanded → opposite.

    Returns NaN if either fit is missing.
    """
    if pos_fit is None or neg_fit is None:
        return float("nan")
    realized_mag = 0.5 * (pos_fit.slope + (-neg_fit.slope))
    return realized_mag - commanded_accel
