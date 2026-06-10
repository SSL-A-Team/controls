"""Checkpoint / persistence layer for the acceleration-model tuning toolkit.

Every tuning run gets a unique ``run_id`` (timestamp by default), under
which each routine writes per-axis trial data and a final ``result.json``.

Directory layout::

    analysis/data/accel_tuning/<run_id>/
    ├── run_metadata.json
    ├── baseline_params.json
    ├── <routine>/<axis>/
    │   ├── trial_0000.json
    │   ├── trial_0000.npz
    │   └── result.json
    └── summary.json
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any, Iterable, Optional

import numpy as np

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
CONTROLS_DIR = next(p for p in SCRIPTS_DIR.parents if p.name == "controls")
TUNING_ROOT = CONTROLS_DIR / "analysis" / "data" / "accel_tuning"


def _atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", delete=False, dir=str(path.parent), prefix=f".{path.name}.",
    )
    try:
        tmp.write(text)
        tmp.flush()
        os.fsync(tmp.fileno())
        tmp.close()
        os.replace(tmp.name, path)
    except Exception:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass
        raise


def write_json(path: Path, payload: Any) -> None:
    """Atomically write ``payload`` as pretty-printed JSON."""
    _atomic_write_text(path, json.dumps(payload, indent=2, sort_keys=True) + "\n")


def write_npz(path: Path, **arrays: np.ndarray) -> None:
    """Save NumPy arrays to a ``.npz`` archive (parent dirs created)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **arrays)


@dataclass
class CheckpointDir:
    """Filesystem layout helper for a single tuning run."""

    run_id: str
    root: Path

    @classmethod
    def create(cls, run_id: Optional[str] = None,
               root: Path = TUNING_ROOT) -> "CheckpointDir":
        if run_id is None:
            run_id = time.strftime("%Y%m%d_%H%M%S")
        path = Path(root) / run_id
        path.mkdir(parents=True, exist_ok=True)
        return cls(run_id=run_id, root=path)

    # ---- path builders ----

    def routine_dir(self, routine: str, axis: str) -> Path:
        d = self.root / routine / axis
        d.mkdir(parents=True, exist_ok=True)
        return d

    def trial_paths(self, routine: str, axis: str, trial_idx: int) -> tuple[Path, Path]:
        d = self.routine_dir(routine, axis)
        return (d / f"trial_{trial_idx:04d}.json",
                d / f"trial_{trial_idx:04d}.npz")

    def routine_result(self, routine: str, axis: str) -> Path:
        return self.routine_dir(routine, axis) / "result.json"

    # ---- top-level files ----

    def write_metadata(self, robot_id: int, cli_args: dict) -> None:
        write_json(self.root / "run_metadata.json", {
            "run_id": self.run_id,
            "robot_id": robot_id,
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "cli": cli_args,
            "python_argv": list(sys.argv),
        })

    def write_baseline(self, params_dict: dict) -> None:
        write_json(self.root / "baseline_params.json", params_dict)

    def write_summary(self, summary: dict) -> None:
        write_json(self.root / "summary.json", summary)


@dataclass
class TrialRecord:
    """Per-trial payload that gets serialised to ``trial_XXXX.json``."""

    routine: str
    axis: str
    trial_idx: int
    candidate: dict = field(default_factory=dict)
    metrics: dict = field(default_factory=dict)
    accepted: bool = False
    notes: str = ""
    timestamp: str = field(default_factory=lambda: time.strftime("%Y-%m-%dT%H:%M:%S%z"))

    def as_dict(self) -> dict:
        return asdict(self)


def save_trial(ckpt: CheckpointDir, record: TrialRecord,
               telemetry: Optional[dict] = None) -> tuple[Path, Optional[Path]]:
    """Persist a trial's JSON + (optional) NPZ telemetry archive."""
    json_path, npz_path = ckpt.trial_paths(record.routine, record.axis,
                                            record.trial_idx)
    write_json(json_path, record.as_dict())
    if telemetry:
        write_npz(npz_path, **telemetry)
        return json_path, npz_path
    return json_path, None
