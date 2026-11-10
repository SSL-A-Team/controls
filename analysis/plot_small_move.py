"""Plot small_move_test runs: pose vs target/trajectory and wheel currents per axis.

    python analysis/plot_small_move.py out.png TAG=run.npz [TAG=run.npz ...]
"""
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load(path):
    rows = [np.asarray(r, float) for r in np.load(path, allow_pickle=True)["data"]]
    a = np.array([r[:34] for r in rows])
    t = a[:, 0] - a[0, 0]
    ok = ~np.isnan(a[:, 25])
    return t[ok], a[ok]


def main():
    out, runs = sys.argv[1], [arg.split("=", 1) for arg in sys.argv[2:]]
    fig, axes = plt.subplots(4, len(runs), figsize=(4.2 * len(runs), 10), squeeze=False, sharey="row")
    for c, (tag, path) in enumerate(runs):
        t, a = load(path)
        ref = a[0, 25:28]
        for r, (lbl, k, sc) in enumerate((("x [mm]", 0, 1e3), ("y [mm]", 1, 1e3), ("theta [deg]", 2, 180 / math.pi))):
            wrap = (lambda v: np.remainder(v - ref[k] + math.pi, 2 * math.pi) - math.pi) if k == 2 else (lambda v: v - ref[k])
            ax = axes[r, c]
            ax.plot(t, wrap(a[:, 25 + k]) * sc, "k--", lw=1, label="target")
            ax.plot(t, wrap(a[:, 16 + k]) * sc, "C1", lw=1, label="trajectory")
            ax.plot(t, wrap(a[:, 1 + k]) * sc, "C0", lw=1.3, label="estimate")
            ax.grid(alpha=.3)
            if c == 0:
                ax.set_ylabel(lbl)
        for i in range(4):
            axes[3, c].plot(t, a[:, 8 + i], lw=.8, label=("FL", "BL", "BR", "FR")[i])
        axes[3, c].grid(alpha=.3)
        axes[3, c].set_xlabel("t [s]")
        axes[0, c].set_title(tag)
    axes[3, 0].set_ylabel("wheel current [A]")
    axes[0, 0].legend(fontsize=7)
    axes[3, 0].legend(fontsize=7, ncol=4)
    fig.tight_layout()
    fig.savefig(out, dpi=90)


if __name__ == "__main__":
    main()
