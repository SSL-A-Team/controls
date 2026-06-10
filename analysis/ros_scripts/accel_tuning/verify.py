"""Verification routine — upload a tuned param set, run N triangular pulses,
score each, and (optionally) plot.

Use this after the coulomb/viscous/efficiency/inertia steps to sanity-check
that the resulting open-loop feed-forward model produces consistent
acceleration profiles. Nothing is searched; nothing is mutated. The routine
just pushes the params, replays the same back-and-forth profile ``N`` times,
and emits per-trial score metrics.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy

from .analysis import accel_match_error
from .base import ANGULAR_AXES, BaseTuneNode, TurnaroundTimeout
from .checkpoint import CheckpointDir, TrialRecord, save_trial, write_json
from ._search import (
    format_trial_end_banner,
    format_trial_start_banner,
    log_lines,
    run_triangular_trial,
)
from .triangular import default_pulse_accel

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
CONTROLS_DIR = next(p for p in SCRIPTS_DIR.parents if p.name == "controls")
sys.path.insert(0, str(CONTROLS_DIR / "analysis"))
from params import PARAM_MAP  # noqa: E402


class VerifyRoutine(BaseTuneNode):
    """Push a tuned param set, run N triangular pulses, score each."""

    def __init__(self, robot_id: int, axis: str, amplitude: float,
                 trials: int, dry_run: bool = False,
                 skip_turnaround: bool = False):
        super().__init__("verify_tune", robot_id, axis, dry_run=dry_run,
                         skip_turnaround=skip_turnaround)
        self.amplitude = float(amplitude)
        self.trials = int(trials)
        self._ckpt: Optional[CheckpointDir] = None
        self._params: dict = {}

    def run(self, ckpt: CheckpointDir, params: dict) -> dict:
        self._ckpt = ckpt
        self._params = dict(params)
        # Push every PARAM_MAP entry we have a value for (so the firmware is
        # in a known state matching the tuned baseline).
        keys = [k for k in PARAM_MAP if k in self._params]
        self.get_logger().info(
            f"[verify][{self.axis}] uploading {len(keys)} param entries to robot"
        )
        self.push_params(self._params, only_keys=keys)
        self.get_logger().info(
            f"[verify][{self.axis}] running {self.trials} triangular trials "
            f"at amplitude={self.amplitude}"
        )

        scores: list[float] = []
        peaks: list[float] = []
        errs: list[float] = []
        best: Optional[tuple[int, float]] = None   # (trial_idx, score)
        aborted = False
        try:
            for i in range(self.trials):
                best_desc = (f"trial {best[0]:04d}  score={best[1]:.4f}"
                             if best is not None else None)
                log_lines(self.get_logger(), format_trial_start_banner(
                    "verify", self.axis, i,
                    {"amplitude": float(self.amplitude)},
                    self._params,
                    best_desc=best_desc,
                ))
                outcome = run_triangular_trial(
                    self, self.amplitude,
                    err_fn=accel_match_error,
                )
                record = TrialRecord(
                    routine="verify", axis=self.axis, trial_idx=i,
                    candidate={"amplitude": float(self.amplitude)},
                    metrics=outcome.metrics, accepted=True,
                    notes="verify pass with tuned params",
                )
                save_trial(self._ckpt, record,
                            telemetry=self.buffer.to_arrays())
                score_t = outcome.score.total
                if not (score_t != score_t) and (best is None or score_t < best[1]):
                    best = (i, float(score_t))
                best_desc = (f"trial {best[0]:04d}  score={best[1]:.4f}"
                             if best is not None else None)
                log_lines(self.get_logger(), format_trial_end_banner(
                    "verify", self.axis, i, outcome,
                    best_desc=best_desc,
                ))
                if not (score_t != score_t):  # not NaN
                    scores.append(float(score_t))
                peaks.append(float(outcome.score.peak_v))
                if outcome.error_signed == outcome.error_signed:  # not NaN
                    errs.append(float(outcome.error_signed))
                # Settle / turnaround between trials.
                if self.axis not in ANGULAR_AXES:
                    self.wait_for_turnaround()
                else:
                    self.wait_for_turnaround(prompt=False)
        except TurnaroundTimeout as exc:
            self.get_logger().error(
                f"verify aborted mid-run: {exc}. "
                "Partial results saved; trial files written so far are valid."
            )
            aborted = True

        # Aggregate scores
        summary: dict = {
            "axis": self.axis,
            "amplitude": float(self.amplitude),
            "n_trials_completed": len(peaks),
            "n_trials_requested": self.trials,
            "aborted": aborted,
            "scores": scores,
            "peak_velocities": peaks,
            "accel_errors": errs,
            "score_mean": float(np.mean(scores)) if scores else None,
            "score_std": float(np.std(scores)) if scores else None,
            "score_min": float(np.min(scores)) if scores else None,
            "score_max": float(np.max(scores)) if scores else None,
            "peak_v_mean": float(np.mean(peaks)) if peaks else None,
            "peak_v_std": float(np.std(peaks)) if peaks else None,
            "accel_err_mean": float(np.mean(errs)) if errs else None,
            "accel_err_std": float(np.std(errs)) if errs else None,
            "params_used": self._params,
        }
        write_json(self._ckpt.routine_result("verify", self.axis), summary)
        if scores:
            self.get_logger().info(
                f"[verify][{self.axis}] DONE: score mean={summary['score_mean']:.4f} "
                f"± {summary['score_std']:.4f}  "
                f"min={summary['score_min']:.4f}  max={summary['score_max']:.4f}  "
                f"(lower is better, over {len(scores)} trials)"
            )
            self.get_logger().info(
                f"[verify][{self.axis}] peak_v mean={summary['peak_v_mean']:.3f} "
                f"± {summary['peak_v_std']:.3f}  "
                f"(expected {abs(self.amplitude) * 0.5:.3f} for A·T_pulse)"
            )
        return summary


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--trials", type=int, default=5,
                    help="Number of back-and-forth triangular trials to run "
                         "(default: 5)")
    p.add_argument("--amplitude", type=float, default=None,
                    help="Triangular pulse amplitude (axis-default if unset)")
    return p
