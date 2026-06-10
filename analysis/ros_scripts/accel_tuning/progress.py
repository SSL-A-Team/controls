"""On-disk evaluation cache used to resume searches across runs.

Each routine writes a ``progress.json`` under
``analysis/data/accel_tuning/<run_id>/<routine>/<axis>/`` containing the
list of ``(candidate, score)`` pairs it has already evaluated. When the
same run-id is reused on a later invocation, the search-step ``_evaluate``
serves matching candidates from the cache instead of re-pulsing the robot.

The deterministic search algorithms (golden-section, secant) always request
the same sequence of candidates given the same bounds and initial guesses,
so replaying the search after a partial completion lets the routine pick up
exactly where it left off.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from .checkpoint import write_json
import json


# Two candidate values are considered "the same" if they differ by less than
# this. Tight enough to distinguish iteration midpoints, loose enough to
# survive float-roundoff between runs.
DEFAULT_CACHE_TOL = 1e-6


@dataclass
class ProgressCache:
    """Append-only cache of evaluated candidates for one routine/axis."""

    path: Path
    tol: float = DEFAULT_CACHE_TOL
    entries: list[dict] = field(default_factory=list)

    @classmethod
    def load_or_new(cls, path: Path,
                     tol: float = DEFAULT_CACHE_TOL) -> "ProgressCache":
        path = Path(path)
        if path.exists():
            try:
                entries = json.loads(path.read_text())
                if not isinstance(entries, list):
                    entries = []
            except (json.JSONDecodeError, OSError):
                entries = []
        else:
            entries = []
        return cls(path=path, tol=tol, entries=entries)

    def find(self, candidate: float) -> Optional[dict]:
        for e in self.entries:
            if abs(float(e["candidate"]) - float(candidate)) < self.tol:
                return e
        return None

    def append(self, candidate: float, score: float, extra: Optional[dict] = None) -> None:
        entry: dict = {"candidate": float(candidate), "score": float(score)}
        if extra:
            entry.update(extra)
        # Replace existing entry if present (same candidate evaluated again).
        for i, e in enumerate(self.entries):
            if abs(float(e["candidate"]) - float(candidate)) < self.tol:
                self.entries[i] = entry
                break
        else:
            self.entries.append(entry)
        write_json(self.path, self.entries)
