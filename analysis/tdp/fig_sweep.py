"""Plot sweep.py results: a metric per candidate, mean +/- std over repeats, one series
per JSON file (e.g. one per robot).

Each repeat's value is the mean over that repeat's moves (failed/tripped moves are
dropped and counted), so the error bar is repeat-to-repeat variation.

    python analysis/tdp/fig_sweep.py analysis/data/tdp/slip_sweep_r*.json \
        --metric xy_rms_mm --xlabel "slip gain K ((rad/s)/A)" --name sweep_slip
    python analysis/tdp/fig_sweep.py analysis/data/tdp/caps_sweep_r2.json \
        --metric reach_5mm_s slip_p95_mps --xlabel "FF / total current cap (A)" --name sweep_caps

Metrics: xy_rms_mm, heading_rms_deg, reach_5mm_s, slip_p95_mps, max_current_a, rms_current_a
"""

from __future__ import annotations

import argparse
import os
import sys
from collections import OrderedDict, defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import tdp_common as tc  # noqa: E402

YLABEL = dict(xy_rms_mm="XY tracking RMS (mm)", heading_rms_deg="heading RMS (deg)",
              reach_5mm_s="time to 5 mm (s)", slip_p95_mps="slip p95 (m/s)",
              max_current_a="max wheel current (A)", rms_current_a="RMS wheel current (A)")


def value(run, metric):
    v = run.get(metric)
    if isinstance(v, list):  # reach_5mm_s: one per target change
        v = np.nanmean(v) if len(v) else np.nan
    return np.nan if v is None else float(v)


def summarize(doc, metric, move_filter):
    per = OrderedDict()
    failed = defaultdict(int)
    for r in doc["runs"]:
        if move_filter and r["move"] not in move_filter:
            continue
        if r.get("error"):
            failed[r["label"]] += 1
            continue
        per.setdefault(r["label"], defaultdict(list))[r["rep"]].append(value(r, metric))
    out = OrderedDict()
    for label, reps in per.items():
        means = np.array([np.nanmean(v) for v in reps.values()])
        out[label] = (np.nanmean(means), np.nanstd(means, ddof=1) if len(means) > 1 else 0.0, len(means),
                      failed[label])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("json", nargs="+")
    ap.add_argument("--metric", nargs="+", default=["xy_rms_mm"])
    ap.add_argument("--moves", nargs="*", default=[], help="only these moves ('dx,dy,dth' as in the JSON)")
    ap.add_argument("--labels", nargs="*", default=[], help="series names (default: file stem)")
    ap.add_argument("--xlabel", default="candidate")
    ap.add_argument("--name", default="sweep")
    ap.add_argument("--out", default="analysis/tdp/figures")
    args = ap.parse_args()
    plt = tc.lncs_style()
    docs = [tc.load_json(p) for p in args.json]
    names = args.labels or [Path(p).stem for p in args.json]

    n = len(args.metric)
    fig, axs = plt.subplots(1, n, figsize=(tc.LNCS_WIDTH_IN * (0.5 if n == 1 else 1.0), 1.9), squeeze=False)
    for ax, metric in zip(axs[0], args.metric):
        for si, (doc, name) in enumerate(zip(docs, names)):
            s = summarize(doc, metric, args.moves)
            labels = list(s)
            x = np.arange(len(labels)) + (si - (len(docs) - 1) / 2) * 0.12
            m = [s[l][0] for l in labels]
            e = [s[l][1] for l in labels]
            ax.errorbar(x, m, yerr=e, fmt="o-", ms=3, capsize=2, label=name)
            ax.set_xticks(np.arange(len(labels)), labels)
            for l in labels:
                mu, sd, nrep, nfail = s[l]
                print(f"{name:20s} {metric:15s} {l:>10s}: {mu:7.3f} +/- {sd:.3f} (reps {nrep}, failed moves {nfail})")
        ax.set_xlabel(args.xlabel)
        ax.set_ylabel(YLABEL.get(metric, metric))
    if len(docs) > 1:
        axs[0, 0].legend()
    tc.save(fig, args.out, args.name)


if __name__ == "__main__":
    main()
