"""Overlay XY paths (trajectory vs estimate) of small_move_test logs, e.g. the
current-budget ablation (per-wheel clipping firmware vs direction-preserving).

    python analysis/tdp/fig_paths.py \
        "per-wheel clip=analysis/data/small_move/small_move_20270110_101500.npz" \
        "budget=analysis/data/small_move/small_move_20270110_103000.npz" --name ablation_paths
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import tdp_common as tc  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="'label=path.npz'")
    ap.add_argument("--name", default="paths")
    ap.add_argument("--out", default="analysis/tdp/figures")
    args = ap.parse_args()
    plt = tc.lncs_style()
    fig, axs = plt.subplots(1, 2, figsize=(tc.LNCS_WIDTH_IN, 2.2), gridspec_kw=dict(width_ratios=[1, 1.4]))
    traj_drawn = False
    for item in args.logs:
        label, _, path = item.partition("=")
        a, tripped = tc.load_small_move(path)
        ok = ~np.isnan(a[:, tc.SM["target"].start])
        a = a[ok]
        home = a[0, tc.SM["target"]]
        pos = (a[:, tc.SM["pos"]][:, :2] - home[:2]) * 1e3
        traj = (a[:, tc.SM["traj"]][:, :2] - home[:2]) * 1e3
        if not traj_drawn:
            axs[0].plot(traj[:, 0], traj[:, 1], "k--", lw=0.8, label="trajectory")
            traj_drawn = True
        line, = axs[0].plot(pos[:, 0], pos[:, 1], label=label)
        t = a[:, tc.SM["t"]] - a[0, tc.SM["t"]]
        err = np.hypot(*(pos - traj).T)
        axs[1].plot(t, err, color=line.get_color(), label=label)
        print(f"{label}: XY error rms {np.sqrt((err ** 2).mean()):.1f} mm, max {err.max():.1f} mm, tripped={tripped}")
    axs[0].set_aspect("equal", adjustable="datalim")
    axs[0].set_xlabel("x from home (mm)")
    axs[0].set_ylabel("y from home (mm)")
    axs[0].legend()
    axs[1].set_xlabel("time (s)")
    axs[1].set_ylabel("|trajectory - estimate| (mm)")
    tc.save(fig, args.out, args.name)


if __name__ == "__main__":
    main()
