"""Coulomb-friction tuning routine.

Pulses an acceleration command at increasing amplitude and records the first
amplitude at which the robot sustains motion above the velocity threshold for
the trailing half of the pulse. The user is prompted to give the robot a
small push at the start of each pulse so the measurement reflects the
kinetic-friction balance (smallest accel that keeps a pushed robot moving)
rather than the static-friction breakaway event (which has high
trial-to-trial variance from microcontact, dust, and carpet thread angle).

From that ``(a_min, v_min)`` pair the routine computes the coulomb-friction
coefficient consistent with the currently active motor efficiency, viscous
coefficient, and inertia::

    c_coul = eta * I * a_min - c_visc * v_min

For axes ``x`` and ``y`` ``I`` is the robot mass. For axis ``theta`` ``I``
is the moment of inertia about z (``iz``). Each axis writes its own slot in
the firmware friction model:

* ``x``     → ``c_coul_linear_x``, ``c_visc_linear_x``
* ``y``     → ``c_coul_linear_y``, ``c_visc_linear_y``
* ``theta`` → ``c_coul_angular``,  ``c_visc_angular``

The routine is intended to be run with motor_efficiency=1.0, c_visc=0,
c_coul=0, default inertia at first (baseline coulomb tuning). The same
``compute_coulomb`` formula is reused by later routines (viscous, optimizer)
to keep the friction balance satisfied as those parameters change.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.duration import Duration

from .analysis import fit_slope
from .base import (
    AXIS_TO_COMPONENT,
    ANGULAR_AXES,
    BaseTuneNode,
    LINEAR_AXES,
    PHASE_PULSE,
    PHASE_REST,
)
from .checkpoint import CheckpointDir, TrialRecord, save_trial, write_json

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
CONTROLS_DIR = next(p for p in SCRIPTS_DIR.parents if p.name == "controls")
sys.path.insert(0, str(CONTROLS_DIR / "analysis"))
from params import (  # noqa: E402
    PARAM_MAP,
    default_params_dict,
    merge_params_dict,
    phys_params_to_json_dict,
)
from ateam_controls import default_phys_params  # noqa: E402


# ===== EDIT ME =====
# Defaults for the pulse-step search. Each routine accepts CLI overrides.
DEFAULT_STEP_LIN = 0.05         # m/s^2 per attempt
DEFAULT_STEP_ANG = 0.2          # rad/s^2 per attempt
DEFAULT_PULSE_DURATION = 0.3    # seconds
DEFAULT_REST_DURATION = 1.0     # seconds between pulses
DEFAULT_MAX_ACCEL_LIN = 5.0     # m/s^2 search ceiling
DEFAULT_MAX_ACCEL_ANG = 15.0    # rad/s^2 search ceiling
DEFAULT_VEL_THRESHOLD_WHEEL = 0.5  # rad/s of wheel encoders
DEFAULT_MIN_SAMPLES = 3
# Walk-to-robot delay before the first pulse begins.
START_DELAY_SECONDS = 5
# Push countdown printed between rest and each pulse: "3", "2", "1", PUSH NOW.
PUSH_COUNTDOWN_SECONDS = 3
# ===================


def default_step(axis: str) -> float:
    return DEFAULT_STEP_ANG if axis in ANGULAR_AXES else DEFAULT_STEP_LIN


def default_max_accel(axis: str) -> float:
    return DEFAULT_MAX_ACCEL_ANG if axis in ANGULAR_AXES else DEFAULT_MAX_ACCEL_LIN


def axis_inertia(axis: str, params_dict: dict) -> float:
    """Return the inertia for the coulomb formula on this axis."""
    mass, iz = params_dict["PHYS_INERTIA"]
    return float(iz) if axis in ANGULAR_AXES else float(mass)


# Friction-model array layout (PARAM_MAP / PHYS_FRICTION_MODEL):
#   [0] coulomb_linear_x   [1] coulomb_linear_y   [2] coulomb_angular
#   [3] viscous_linear_x   [4] viscous_linear_y   [5] viscous_angular
_FRICTION_INDICES = {
    "x":     (0, 3),  # (c_coul_index, c_visc_index)
    "y":     (1, 4),
    "theta": (2, 5),
}


def axis_friction_indices(axis: str) -> tuple[int, int]:
    """Return ``(c_coul_index, c_visc_index)`` into PHYS_FRICTION_MODEL."""
    try:
        return _FRICTION_INDICES[axis]
    except KeyError as exc:
        raise ValueError(
            f"axis {axis!r} has no friction-array slot; expected one of "
            f"{list(_FRICTION_INDICES)}"
        ) from exc


def empty_friction_array() -> list[float]:
    """Zero-initialised PHYS_FRICTION_MODEL array (6 entries)."""
    return [0.0] * 6


def compute_coulomb(eta: float, inertia: float, a_min: float,
                    c_visc: float, v_min: float) -> float:
    """``c_coul = eta * I * a_min - c_visc * v_min`` (see module docstring)."""
    return float(eta * inertia * a_min - c_visc * v_min)


class CoulombRoutine(BaseTuneNode):
    """Pulse-and-step search for the minimum-motion acceleration."""

    def __init__(self, robot_id: int, axis: str,
                 step: float, start_accel: float,
                 pulse_duration: float, rest_duration: float,
                 max_accel: float, velocity_threshold: float,
                 min_samples: int, dry_run: bool = False):
        super().__init__("coulomb_tune", robot_id, axis, dry_run=dry_run)
        self.step = float(step)
        self.start_accel = float(start_accel)
        self.pulse_duration = float(pulse_duration)
        self.rest_duration = float(rest_duration)
        self.max_accel = float(max_accel)
        self.velocity_threshold = float(velocity_threshold)
        self.min_samples = int(min_samples)

        self.current_accel = self.start_accel
        self.detection_accel: Optional[float] = None
        self.v_min: Optional[float] = None
        self.done = False

        # pulse/rest state
        self._in_pulse = False
        self._phase_start = self.get_clock().now()
        self._pulse_sustained = True
        self._pulse_samples = 0
        self._wheel_vel_during_pulse: list[float] = []
        # Trial index for checkpoint writes
        self._trial_idx = 0
        # Gate so the on_tick timer (which starts in BaseTuneNode.__init__)
        # doesn't fire pulses before run() finishes its walk-to-robot countdown.
        self._started = False
        # Per-pulse push countdown state. Reset in _finish_pulse so the next
        # rest period prints "3", "2", "1" again before the next pulse.
        self._countdown_start = None  # type: Optional[object]
        self._next_count_to_print = 0

        # Initialise trial buffer baseline.
        self.begin_trial()

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def on_tick(self) -> None:
        if self.done or not self._started:
            self.publish_off()
            return
        elapsed = (self.get_clock().now() - self._phase_start).nanoseconds * 1e-9
        if self._in_pulse:
            if elapsed >= self.pulse_duration:
                self._finish_pulse()
                self.publish_off()
            else:
                self.set_phase(PHASE_PULSE)
                self.publish_accel(self.current_accel)
        else:
            self.set_phase(PHASE_REST)
            self.publish_off()
            if elapsed >= self.rest_duration:
                self._tick_push_countdown()

    def _tick_push_countdown(self) -> None:
        """Drive the 3-2-1 countdown between rest and the next pulse.

        Called from on_tick once the rest period has elapsed. Prints the
        countdown integers at one-second intervals, then starts the pulse
        once the full ``PUSH_COUNTDOWN_SECONDS`` window has elapsed (the
        pulse itself logs ``>>> PUSH NOW``).
        """
        now = self.get_clock().now()
        if self._countdown_start is None:
            self._countdown_start = now
            self._next_count_to_print = PUSH_COUNTDOWN_SECONDS
        countdown_elapsed = (now - self._countdown_start).nanoseconds * 1e-9
        while (self._next_count_to_print > 0
                and countdown_elapsed
                    >= (PUSH_COUNTDOWN_SECONDS - self._next_count_to_print)):
            self.get_logger().info(f"  {self._next_count_to_print}...")
            self._next_count_to_print -= 1
        if countdown_elapsed >= PUSH_COUNTDOWN_SECONDS:
            self._countdown_start = None
            self._start_pulse()

    def _start_pulse(self) -> None:
        if self.current_accel > self.max_accel:
            self.get_logger().warn(
                f"Reached max accel ({self.max_accel}) without sustained motion."
            )
            self.done = True
            return
        self._in_pulse = True
        self._phase_start = self.get_clock().now()
        self._pulse_sustained = True
        self._pulse_samples = 0
        self._wheel_vel_during_pulse = []
        self.get_logger().info(f"Pulse → {self.current_accel:.4f}")
        # Push-sustain measurement: the user manually nudges the robot at the
        # start of each pulse so detection measures kinetic-friction balance
        # (does the commanded accel sustain pushed motion?) instead of static
        # breakaway. Sustain detection logic in _finish_pulse is unchanged.
        self.get_logger().info(">>> PUSH NOW (small forward shove)")

    def _finish_pulse(self) -> None:
        # Score this pulse from the wheel-encoder samples gathered in
        # _on_extended_telem hook below.
        self._in_pulse = False
        self._phase_start = self.get_clock().now()
        # Arm the next push countdown for the upcoming rest → pulse transition.
        self._countdown_start = None
        self._next_count_to_print = 0

        if self._pulse_samples < self.min_samples:
            note = (f"only {self._pulse_samples} telemetry samples "
                    f"(need {self.min_samples}); treating as no motion")
            self.get_logger().warn(f"Pulse at {self.current_accel:.4f}: {note}")
            self._pulse_sustained = False
            self._record_trial(False, mean_vel=0.0, note=note)
            self.current_accel += self.step
            return

        # Sustained-motion criterion: the *trailing half* of the pulse must
        # sit above the threshold (handles motor rise time at the start of
        # the pulse). We also require the mean of the trailing half to clear
        # the threshold so a single noisy spike doesn't qualify.
        n = len(self._wheel_vel_during_pulse)
        tail = self._wheel_vel_during_pulse[n // 2:]
        tail_min = min(tail) if tail else 0.0
        tail_mean = (sum(tail) / len(tail)) if tail else 0.0
        sustained = (tail_min >= self.velocity_threshold
                     and tail_mean >= self.velocity_threshold)
        self._pulse_sustained = sustained
        mean_vel = (float(np.mean(self._wheel_vel_during_pulse))
                    if self._wheel_vel_during_pulse else 0.0)
        note = (f"sustained (tail_min={tail_min:.3f}, tail_mean={tail_mean:.3f})"
                if sustained
                else f"below threshold (tail_min={tail_min:.3f}, "
                     f"tail_mean={tail_mean:.3f}, peak="
                     f"{max(self._wheel_vel_during_pulse):.3f})")

        # Extract v_min from the KF estimate BEFORE _record_trial clears
        # the in-memory telemetry buffer.
        body_vel = self._extract_body_vel()
        self._record_trial(sustained, mean_vel=mean_vel, note=note)
        if sustained:
            self.detection_accel = self.current_accel
            self.v_min = abs(body_vel)
            self.done = True
            self.get_logger().info(
                f">>> MOTION at accel={self.detection_accel:.4f}, "
                f"v_min≈{self.v_min:.4f}  ({note})"
            )
        else:
            self.get_logger().info(
                f"Pulse at {self.current_accel:.4f} not sustained: {note}"
            )
            self.current_accel += self.step

    # ------------------------------------------------------------------
    # Telemetry hook (post-process buffer additions)
    # ------------------------------------------------------------------

    def _on_extended_telem(self, msg) -> None:
        super()._on_extended_telem(msg)
        if not self._in_pulse:
            return
        wheel_vels = [abs(v) for v in self.buffer.samples[-1].wheel_vel]
        if not wheel_vels:
            return
        peak = max(wheel_vels)
        self._wheel_vel_during_pulse.append(peak)
        self._pulse_samples += 1
        # Detection is evaluated in _finish_pulse from the full sample list,
        # not per-sample, so the first few low samples (motor rise time)
        # don't disqualify an otherwise-moving pulse.

    def _extract_body_vel(self) -> float:
        """Mean local-frame body velocity along the active axis during the
        last pulse.

        The KF state is in the global frame; ``BCM_LOCAL_ACCEL`` commands and
        ``v_min`` are local-frame quantities, so for linear axes we rotate
        ``(vx_g, vy_g)`` by ``−θ`` per sample. Uses the trailing half of the
        pulse to skip motor rise-time samples.
        """
        pulse_samples = self.buffer.phase_slice(PHASE_PULSE)
        if not pulse_samples:
            return 0.0
        tail = pulse_samples[len(pulse_samples) // 2:]
        if self.axis == "theta":
            return float(np.mean([s.kf_vel[2] for s in tail]))
        vx_g = np.array([s.kf_vel[0] for s in tail])
        vy_g = np.array([s.kf_vel[1] for s in tail])
        theta = np.array([s.kf_pos[2] for s in tail])
        c, s_ = np.cos(theta), np.sin(theta)
        vx_l = c * vx_g + s_ * vy_g
        vy_l = -s_ * vx_g + c * vy_g
        local = vx_l if self.axis == "x" else vy_l
        return float(np.mean(local))

    # ------------------------------------------------------------------
    # Checkpoint writing
    # ------------------------------------------------------------------

    def _record_trial(self, accepted: bool, mean_vel: float, note: str) -> None:
        if self._ckpt is None:
            return
        record = TrialRecord(
            routine="coulomb",
            axis=self.axis,
            trial_idx=self._trial_idx,
            candidate={"accel": self.current_accel},
            metrics={
                "pulse_samples": self._pulse_samples,
                "mean_wheel_vel": mean_vel,
                "body_axis_vel_kf": float(self._extract_body_vel()),
                "sustained": bool(self._pulse_sustained),
            },
            accepted=accepted,
            notes=note,
        )
        save_trial(self._ckpt, record, telemetry=self.buffer.to_arrays())
        self._trial_idx += 1
        # Reset only the in-buffer record after writing trial telemetry.
        self.begin_trial()
        self._phase_start = self.get_clock().now()

    # ------------------------------------------------------------------
    # Orchestration: drive spin until done
    # ------------------------------------------------------------------

    _ckpt: Optional[CheckpointDir] = None

    def run(self, ckpt: CheckpointDir,
            current_params: Optional[dict] = None) -> dict:
        """Push baseline params, spin until detection, write result.json.

        ``current_params`` is the JSON-style param dict that should be active
        on the robot before the search starts. If None, defaults are used and
        coulomb tuning will be performed with eta=1, c_*=0, default inertia.
        """
        self._ckpt = ckpt
        if current_params is None:
            current_params = default_params_dict()
            current_params["PHYS_MOTOR_MODEL"][1] = 1.0  # eta=1
            current_params["PHYS_FRICTION_MODEL"] = empty_friction_array()

        self.push_params(current_params,
                          only_keys=["PHYS_INERTIA", "PHYS_MOTOR_MODEL",
                                     "PHYS_FRICTION_MODEL"])

        self.get_logger().info(
            f"[coulomb][{self.axis}] start_accel={self.start_accel} "
            f"step={self.step} max={self.max_accel}"
        )

        # Walk-to-robot countdown before the first pulse: keeps ROS spinning
        # (telemetry subs stay live) but on_tick stays gated by _started so
        # no pulse fires until the countdown completes.
        self.get_logger().info(
            f">>> Starting in {START_DELAY_SECONDS}s — walk over to the robot"
        )
        for i in range(START_DELAY_SECONDS, 0, -1):
            self.get_logger().info(f"  {i}...")
            end = self.get_clock().now() + Duration(seconds=1.0)
            while rclpy.ok() and self.get_clock().now() < end:
                rclpy.spin_once(self, timeout_sec=1.0 / self.PUBLISH_RATE_HZ)
        # Re-arm phase_start so the first rest period starts now, not at
        # routine construction time (otherwise the first pulse would skip the
        # rest + countdown).
        self._phase_start = self.get_clock().now()
        self._started = True

        while rclpy.ok() and not self.done:
            rclpy.spin_once(self, timeout_sec=1.0 / self.PUBLISH_RATE_HZ)

        # Compute c_coul under the active param set for this axis's slot.
        eta = current_params["PHYS_MOTOR_MODEL"][1]
        inertia = axis_inertia(self.axis, current_params)
        coul_idx, visc_idx = axis_friction_indices(self.axis)
        c_visc = current_params["PHYS_FRICTION_MODEL"][visc_idx]

        if self.detection_accel is None:
            self.get_logger().warn(
                f"[coulomb][{self.axis}] no motion up to {self.max_accel}"
            )
            result = {
                "axis": self.axis,
                "detected": False,
                "a_min": None, "v_min": None,
                "c_coul": None,
                "params_used": current_params,
            }
        else:
            c_coul = compute_coulomb(eta, inertia, self.detection_accel,
                                     c_visc, self.v_min or 0.0)
            result = {
                "axis": self.axis,
                "detected": True,
                "a_min": float(self.detection_accel),
                "v_min": float(self.v_min or 0.0),
                "eta": float(eta),
                "inertia": float(inertia),
                "c_visc": float(c_visc),
                "c_coul": float(c_coul),
                "params_used": current_params,
            }
            # Update PHYS_FRICTION_MODEL with this axis's coulomb value.
            new_friction = list(current_params["PHYS_FRICTION_MODEL"])
            new_friction[coul_idx] = c_coul
            updated = merge_params_dict(current_params, {
                "PHYS_FRICTION_MODEL": new_friction,
            })
            self.push_params(updated, only_keys=["PHYS_FRICTION_MODEL"])
            result["params_after"] = updated

        write_json(ckpt.routine_result("coulomb", self.axis), result)
        return result


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--step", type=float, default=None,
                    help="Pulse amplitude increment (axis-dependent default)")
    p.add_argument("--start-accel", type=float, default=0.0,
                    help="Initial pulse amplitude (default 0.0)")
    p.add_argument("--pulse-duration", type=float, default=DEFAULT_PULSE_DURATION,
                    help=f"Seconds per pulse (default {DEFAULT_PULSE_DURATION})")
    p.add_argument("--rest-duration", type=float, default=DEFAULT_REST_DURATION,
                    help=f"Seconds between pulses (default {DEFAULT_REST_DURATION})")
    p.add_argument("--max-accel", type=float, default=None,
                    help="Search ceiling (axis-dependent default)")
    p.add_argument("--velocity-threshold", type=float,
                    default=DEFAULT_VEL_THRESHOLD_WHEEL,
                    help="Wheel-encoder velocity threshold (rad/s) for motion")
    p.add_argument("--min-samples", type=int, default=DEFAULT_MIN_SAMPLES,
                    help="Minimum telemetry samples to evaluate a pulse")
    return p
