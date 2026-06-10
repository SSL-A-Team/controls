"""Triangular acceleration profile shared by the viscous/efficiency/inertia
tuning routines.

Profile per trial::

    rest (no command, settle) ─►
    pulse_pos (+A for T_pulse) ─►
    coast (0 for T_coast) ─►
    pulse_neg (-A for T_pulse) ─►
    rest (settle, await turnaround if linear axis)

The amplitude ``A`` is a module-level constant per axis class so it can be
edited in one place. Routines may override ``pulse_accel`` per-trial if they
need to (e.g. inertia tuning can use a smaller value to stay below
``max_vel_angular``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from .base import (
    ANGULAR_AXES,
    BaseTuneNode,
    PHASE_COAST,
    PHASE_PULSE_NEG,
    PHASE_PULSE_POS,
    PHASE_REST,
    PHASE_WAIT_TURN,
)


# ===== EDIT ME =====
# Default triangular-pulse amplitudes per axis class. These are intentionally
# placed near the top of the file for easy tuning.
TRIANGULAR_PULSE_ACCEL_LIN = 1.0     # m/s^2 for linear axes
TRIANGULAR_PULSE_ACCEL_ANG = 3.0     # rad/s^2 for the angular axis
T_PULSE = 0.5                        # seconds per pulse half
T_COAST = 0.5                        # seconds at zero between pulses
T_REST = 0.5                         # seconds at BCM_OFF between trials
# ===================


def default_pulse_accel(axis: str) -> float:
    return (TRIANGULAR_PULSE_ACCEL_ANG if axis in ANGULAR_AXES
            else TRIANGULAR_PULSE_ACCEL_LIN)


@dataclass
class TriangularPhaseTimes:
    pulse_pos_until: float
    coast_until: float
    pulse_neg_until: float
    rest_until: float


def schedule(now: float, t_pulse: float = T_PULSE,
             t_coast: float = T_COAST,
             t_rest: float = T_REST) -> TriangularPhaseTimes:
    """Return absolute phase-end times when starting a pulse at ``now``."""
    p1 = now + t_pulse
    p2 = p1 + t_coast
    p3 = p2 + t_pulse
    p4 = p3 + t_rest
    return TriangularPhaseTimes(p1, p2, p3, p4)


class TriangularPulseRunner:
    """Tick-driven state machine that publishes one triangular profile.

    Subclasses of ``BaseTuneNode`` create one of these per trial and call
    :meth:`tick` from their ``on_tick`` handler. The runner returns ``True``
    once the trial (pulse + coast + neg pulse + rest) is complete.
    """

    def __init__(self, node: BaseTuneNode, amplitude: float,
                 t_pulse: float = T_PULSE,
                 t_coast: float = T_COAST,
                 t_rest: float = T_REST,
                 on_phase_change: Optional[Callable[[str], None]] = None):
        self.node = node
        self.amplitude = float(amplitude)
        self.t_pulse = float(t_pulse)
        self.t_coast = float(t_coast)
        self.t_rest = float(t_rest)
        self.on_phase_change = on_phase_change

        self._times: Optional[TriangularPhaseTimes] = None
        self._done = False

    def _arm(self) -> None:
        self._times = schedule(self.node.trial_elapsed(),
                                self.t_pulse, self.t_coast, self.t_rest)
        self._set_phase(PHASE_PULSE_POS)

    def _set_phase(self, phase: str) -> None:
        prev = self.node.current_phase
        self.node.set_phase(phase)
        if self.on_phase_change is not None and prev != phase:
            self.on_phase_change(phase)

    def tick(self) -> bool:
        """Drive one publisher tick. Returns ``True`` when the trial is done."""
        if self._done:
            self.node.publish_off()
            return True
        if self._times is None:
            self._arm()
        t = self.node.trial_elapsed()
        times = self._times
        assert times is not None

        if t < times.pulse_pos_until:
            self._set_phase(PHASE_PULSE_POS)
            self.node.publish_accel(+self.amplitude)
        elif t < times.coast_until:
            self._set_phase(PHASE_COAST)
            self.node.publish_accel(0.0)
        elif t < times.pulse_neg_until:
            self._set_phase(PHASE_PULSE_NEG)
            self.node.publish_accel(-self.amplitude)
        elif t < times.rest_until:
            self._set_phase(PHASE_REST)
            self.node.publish_off()
        else:
            self._set_phase(PHASE_REST)
            self.node.publish_off()
            self._done = True
            return True
        return False

    @property
    def done(self) -> bool:
        return self._done
