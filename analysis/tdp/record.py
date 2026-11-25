"""Record extended telemetry during a named preset of tether-safe test moves, for the
offline fit figures (fig_fits.py). Uses the same moves and recorder as the
calibration scripts, but each preset goes to its own file (wheel_slip.py writes its
linear and rotation recordings to the same --out, so the linear one is lost).

Presets:
  fit-moves      forward, sideways, diagonal 0.2 m pairs  (efficiency/friction fit)
  slip-linear    sideways, forward 0.2 m pairs            (wheel slip fit)
  slip-rotation  one fast theta ramp                      (slip check for rotation, expect ~0)
  latency        two fast theta ramps                     (vision latency)

    python analysis/tdp/record.py fit-moves --home=0,0,3.14 --heading-ref 3.14 \
        --out analysis/data/tdp/fit_moves_r2.npz
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "calibration"))
from common import add_common_args, move, record_during, test_cmd  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("preset", choices=("fit-moves", "slip-linear", "slip-rotation", "latency"))
    add_common_args(ap)
    args = ap.parse_args()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)

    ramp = test_cmd("theta_ramp_test", args, "--w", 1.8, "--ramp", 0.5, "--hold-w", 0.2)
    cmds, duration = {
        "fit-moves": ([move(args, 0.2, 0, 0), move(args, 0, 0.2, 0), move(args, 0.15, 0.15, 0)], 40),
        "slip-linear": ([move(args, 0, 0.2, 0), move(args, 0.2, 0, 0)], 28),
        "slip-rotation": ([ramp], 16),
        "latency": ([ramp, ramp], 32),
    }[args.preset]
    t, _ = record_during(args, cmds, duration)
    print(f"recorded {len(t)} rows to {args.out}")


if __name__ == "__main__":
    main()
