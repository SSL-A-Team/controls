"""A/B sweep of radio parameters on the calibration move set, logged to JSON for figures.

Like calibration/common.py::validate, but:
* every move's metrics and log file are saved (--out JSON), so fig_sweep.py can plot
  means with error bars;
* candidates are interleaved and shuffled per repeat, so slow drift (battery, carpet,
  motor temperature) spreads over all candidates instead of biasing the last one;
* several parameters can change per candidate (e.g. both current caps);
* the robot's original values are read first and restored at the end.

Candidate syntax:  LABEL:PID=v1,v2,...[;PID=v1,...]   (an empty spec = leave as is)

    # wheel slip gain (param 19)
    python analysis/tdp/sweep.py --home=0,0,3.14 --heading-ref 3.14 --repeats 3 \
        --out analysis/data/tdp/slip_sweep_r2.json \
        --candidate 0:19=0 --candidate 0.8:19=0.8 --candidate 1.2:19=1.2 --candidate 1.6:19=1.6

    # current caps: FF limit (18) and wheel loop total cap (17 = kP, kI, anti-jitter, max A)
    python analysis/tdp/sweep.py --home=0,0,3.14 --heading-ref 3.14 --repeats 3 \
        --out analysis/data/tdp/caps_sweep_r2.json \
        --candidate 2.7/3.0:18=2.7;17=400,0,1.5,3.0 \
        --candidate 3.5/4.0:18=3.5;17=400,0,1.5,4.0
"""

from __future__ import annotations

import argparse
import json
import os
import random
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "calibration"))
import tdp_common as tc  # noqa: E402
from common import CONTROLS_ROOT, get_param, move_tracking, set_param, test_cmd  # noqa: E402

DEFAULT_MOVES = ("0.2,0,0", "0,0.2,0", "0.15,0.15,90")


def parse_candidate(s):
    label, _, spec = s.partition(":")
    params = {}
    for part in filter(None, spec.split(";")):
        pid, _, vals = part.partition("=")
        params[int(pid)] = vals
    return label, params


def metrics(npz):
    xy, hd, reach = move_tracking(npz)
    a, tripped = tc.load_small_move(npz)
    moving = np.linalg.norm(a[:, tc.SM["vel_u"]][:, :2], axis=1) > 0.02
    slip = np.abs(tc.slip_residual(a[moving, tc.SM["wv"]])).max(axis=1) if moving.any() else np.array([np.nan])
    cur = np.abs(a[:, tc.SM["cur"]])
    return dict(xy_rms_mm=xy, heading_rms_deg=hd, reach_5mm_s=reach,
                slip_p95_mps=float(np.nanpercentile(slip, 95)), max_current_a=float(cur.max()),
                rms_current_a=float(np.sqrt((cur ** 2).mean())), tripped=tripped)


def run_move(args, dx, dy, dth):
    data = CONTROLS_ROOT / "analysis/data/small_move"
    before = set(data.glob("*.npz")) if data.exists() else set()
    cmd = test_cmd("small_move_test", args, "--dx", dx, "--dy", dy, "--dtheta-deg", dth,
                   "--amax", args.amax, "--alphamax", args.alphamax, "--vmax", args.vmax, "--wmax", 3.0,
                   "--cycles", args.cycles, "--hold", 1.5)
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=180, cwd=CONTROLS_ROOT)
    new = sorted(set(data.glob("*.npz")) - before)
    if not new or len(np.load(new[-1], allow_pickle=True)["data"]) < 50:
        return None, "no telemetry (bridge running? extended telemetry DIP switch on?)"
    if "TRIP" in res.stdout + res.stderr:
        trip = [l for l in (res.stdout + res.stderr).splitlines() if "TRIP" in l]
        return str(new[-1]), trip[-1].strip()
    return str(new[-1]), None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--home", required=True)
    ap.add_argument("--heading-ref", type=float, required=True)
    ap.add_argument("--candidate", action="append", required=True, help="LABEL:PID=vals[;PID=vals]")
    ap.add_argument("--moves", nargs="*", default=list(DEFAULT_MOVES), help="'dx,dy,dtheta_deg' out-and-back moves")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--cycles", type=int, default=1, help="out-and-back cycles per move")
    ap.add_argument("--amax", type=float, default=3.0)
    ap.add_argument("--alphamax", type=float, default=10.0)
    ap.add_argument("--vmax", type=float, default=3.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--note", default="", help="free text saved in the JSON (robot state, carpet, battery...)")
    ap.add_argument("--out", required=True, help="results JSON (appended to if it exists)")
    args = ap.parse_args()

    cands = [parse_candidate(c) for c in args.candidate]
    pids = sorted({p for _, ps in cands for p in ps})
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    doc = json.loads(out.read_text()) if out.exists() else {"runs": []}
    doc.setdefault("meta", []).append(dict(started=time.strftime("%Y-%m-%d %H:%M:%S"), robot=args.robot,
                                           argv=sys.argv[1:], note=args.note))

    originals = {}
    for pid in pids:
        originals[pid] = get_param(args.robot, pid)
        print(f"param {pid} on robot {args.robot}: {originals[pid]}")
        if originals[pid] is None:
            raise SystemExit(f"could not read param {pid}; is the radio bridge up?")

    rng = random.Random(args.seed)
    try:
        for rep in range(args.repeats):
            order = cands[:]
            rng.shuffle(order)
            for label, params in order:
                ok = all(set_param(args.robot, pid, vals) for pid, vals in params.items())
                if not ok:
                    print(f"[rep {rep}] {label}: could not set {params}; skipped")
                    continue
                for mv in args.moves:
                    dx, dy, dth = (float(v) for v in mv.split(","))
                    t_start = time.time()
                    f, err = run_move(args, dx, dy, dth)
                    # t_start/t_end (unix s) locate this move in a bag.sh recording
                    run = dict(label=label, params=params, rep=rep, move=mv, file=f, error=err,
                               robot=args.robot, time=time.strftime("%H:%M:%S"),
                               t_start=t_start, t_end=time.time())
                    if f and not err:
                        run.update(metrics(f))
                        print(f"[rep {rep}] {label:>10s} move {mv:>14s}: XY {run['xy_rms_mm']:5.1f} mm, "
                              f"hdg {run['heading_rms_deg']:4.2f} deg, slip p95 {run['slip_p95_mps']:.2f} m/s, "
                              f"I max {run['max_current_a']:.2f} A")
                    else:
                        print(f"[rep {rep}] {label:>10s} move {mv:>14s}: {err}")
                    doc["runs"].append(run)
                    out.write_text(json.dumps(doc, indent=1))  # save as we go
    finally:
        for pid, vals in originals.items():
            ok = set_param(args.robot, pid, ",".join(f"{v:g}" for v in vals))
            print(f"restored param {pid} = {vals}" + ("" if ok else "  ** FAILED: reboot the robot **"))
    print(f"saved {out}")


if __name__ == "__main__":
    main()
