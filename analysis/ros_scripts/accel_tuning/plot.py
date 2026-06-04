"""Optional plotting hook — feeds tuning-trial telemetry into
``analysis.visualization.body.plot_telem`` so a routine's output can be
visualised with the same 3×3 grid used for ros-bag telemetry.

Each per-trial NPZ already stores the firmware-style slash-keyed arrays
expected by ``plot_telem`` (see ``base.TelemetryBuffer.to_arrays``); this
module just concatenates all trials in a routine/axis dir and offsets
``timestamp_us`` so each trial appears on its own time window in the plot.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Iterable, Optional

import numpy as np

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
CONTROLS_DIR = next(p for p in SCRIPTS_DIR.parents if p.name == "controls")
sys.path.insert(0, str(CONTROLS_DIR / "analysis"))


def _trial_npz_paths(ckpt_root: Path, routine: str, axis: str) -> list[Path]:
    d = Path(ckpt_root) / routine / axis
    if not d.is_dir():
        return []
    return sorted(d.glob("trial_*.npz"))


def assemble_run_telemetry(ckpt_root: Path, routine: str,
                            axis: str,
                            gap_us: int = 500_000) -> Optional[dict]:
    """Concatenate every trial NPZ in ``routine/axis`` into one dict.

    Returns a telemetry dict compatible with ``plot_telem``. A ``gap_us``
    silent gap is inserted between trials so the time axis shows discrete
    trial windows.
    """
    paths = _trial_npz_paths(ckpt_root, routine, axis)
    if not paths:
        return None
    arrays: dict[str, list[np.ndarray]] = {}
    t_offset_us = 0
    for p in paths:
        npz = np.load(p, allow_pickle=False)
        # Shift this trial's timestamp_us to fit after the previous one.
        ts = npz["timestamp_us"].astype(np.int64) + int(t_offset_us)
        # End-of-trial timestamp + gap is the next trial's offset.
        if ts.size:
            t_offset_us = int(ts[-1]) + int(gap_us)
        for k in npz.files:
            v = npz[k]
            if k == "timestamp_us":
                v = ts
            arrays.setdefault(k, []).append(v)

    merged: dict[str, np.ndarray] = {}
    for k, parts in arrays.items():
        try:
            merged[k] = np.concatenate(parts, axis=0)
        except ValueError:
            # Heterogenous shapes (e.g. empty trial) — drop the first if zero.
            non_empty = [p for p in parts if p.shape[0] != 0]
            merged[k] = (np.concatenate(non_empty, axis=0)
                          if non_empty else parts[0])
    return merged


def show_run_plot(ckpt_root: Path, routine: str, axis: str,
                   title_suffix: str = "",
                   per_trial: bool = True) -> None:
    """Open the ``plot_telem`` 3×3 grid for trials in ``routine/axis``.

    When ``per_trial=True`` (the default), each trial NPZ is opened in its
    own matplotlib window with a title naming the trial number, the
    candidate parameter, and the routine. When ``per_trial=False``, all
    trials are concatenated into a single window with gaps between them.
    """
    paths = _trial_npz_paths(ckpt_root, routine, axis)
    if not paths:
        print(f"[plot] no trials found at {ckpt_root}/{routine}/{axis}/")
        return

    # Lazy imports so matplotlib only loads when the user asks for plots.
    import json
    import matplotlib.pyplot as plt
    sys.path.insert(0, str(CONTROLS_DIR / "analysis" / "visualization"))
    from body import plot_telem  # noqa: E402

    if per_trial:
        for path in paths:
            telemetry = _single_trial_telemetry(path)
            if telemetry is None:
                continue
            json_path = path.with_suffix(".json")
            trial_idx = _trial_index(path)
            cand_str = ""
            score_str = ""
            try:
                with open(json_path) as f:
                    rec = json.load(f)
                cand = rec.get("candidate") or {}
                if cand:
                    cand_str = " · " + ", ".join(
                        f"{k}={v:.4f}" if isinstance(v, (int, float))
                        else f"{k}={v}"
                        for k, v in cand.items())
                metrics = rec.get("metrics") or {}
                score_total = metrics.get("score_total")
                if score_total is not None:
                    score_str = f" · score={float(score_total):.4f}"
                    if metrics.get("low_motion"):
                        score_str += " LOW_MOTION"
                err = metrics.get("error_signed")
                if err is not None:
                    score_str += f" · err={float(err):.4f}"
            except (OSError, json.JSONDecodeError):
                pass
            fig, axs, t, state_est = plot_telem(telemetry, param_json=None)
            title = (f"accel_tuning · {routine} · {axis} · "
                     f"trial {trial_idx:04d}{cand_str}{score_str}")
            if title_suffix:
                title += f" · {title_suffix}"
            fig.suptitle(title, fontsize=12)
        plt.show()
        return

    telemetry = assemble_run_telemetry(ckpt_root, routine, axis)
    if telemetry is None:
        return
    fig, axs, t, state_est = plot_telem(telemetry, param_json=None)
    title = f"accel_tuning · {routine} · {axis} (all trials)"
    if title_suffix:
        title += f" · {title_suffix}"
    fig.suptitle(title, fontsize=12)
    plt.show()


def _single_trial_telemetry(path: Path) -> Optional[dict]:
    """Load one trial NPZ as a telemetry dict compatible with ``plot_telem``."""
    npz = np.load(path, allow_pickle=False)
    out: dict[str, np.ndarray] = {}
    for k in npz.files:
        out[k] = npz[k]
    if out.get("timestamp_us") is None or out["timestamp_us"].size == 0:
        return None
    return out


def _trial_index(path: Path) -> int:
    stem = path.stem  # 'trial_0007'
    try:
        return int(stem.rsplit("_", 1)[-1])
    except ValueError:
        return -1
