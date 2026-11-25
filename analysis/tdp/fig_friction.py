"""Friction vs travel direction from breakaway (friction_profile.py) and constant-speed
(plateau_profile.py) runs, raw and normalized by the roller-speed factor k(dir).

If the omni-roller friction model holds, the normalized points are flat across
direction: that is the figure's claim. Motor-side force comes straight from the
logged wheel current setpoints (F = W * I * Kt / r), so it does not depend on the
efficiency/friction parameters the robot was running.

    python analysis/tdp/fig_friction.py \
        --breakaway analysis/data/friction_profile/*.npz \
        --plateau analysis/data/plateau_profile/*.npz
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import tdp_common as tc  # noqa: E402


def collect(paths, kind):
    """-> rows (kind, dir_deg or 'w', force N (linear, along motion) or torque N m)."""
    rows = []
    for p in paths:
        res = np.load(p, allow_pickle=True)["results"]
        for r in res:
            r = dict(r)
            if kind == "breakaway" and r.get("breakaway_accel") is None:
                continue  # never broke away before --max-accel
            tok, sign = str(r["dir"]), float(r["sign"])
            I = np.asarray(r["currents"] if kind == "breakaway" else r["I"], float)
            F = tc.currents_to_local_force(I)
            if tok == "w":
                rows.append((kind, "w", abs(F[2]), r.get("speed")))
            elif not tok.endswith("w"):
                th = math.radians(float(tok))
                u = sign * np.array([math.cos(th), math.sin(th)])
                rows.append((kind, float(tok) % 180.0, float(F[:2] @ u), r.get("speed")))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--breakaway", nargs="*", default=[])
    ap.add_argument("--plateau", nargs="*", default=[])
    ap.add_argument("--out", default="analysis/tdp/figures")
    args = ap.parse_args()
    rows = collect(args.breakaway, "breakaway") + collect(args.plateau, "kinetic")
    if not rows:
        raise SystemExit("no results in the given files")
    plt = tc.lncs_style()

    fig, axs = plt.subplots(1, 2, figsize=(tc.LNCS_WIDTH_IN, 1.9), sharex=True)
    grid = np.linspace(0, 180, 181)
    kg = np.array([tc.k_dir((math.cos(math.radians(a)), math.sin(math.radians(a)))) for a in grid])
    norm_all = []
    for kind, mk in (("breakaway", "o"), ("kinetic", "s")):
        lin = [(d, f) for k, d, f, _ in rows if k == kind and d != "w"]
        if not lin:
            continue
        d = np.array([x[0] for x in lin])
        f = np.array([x[1] for x in lin])
        k = np.array([tc.k_dir((math.cos(math.radians(a)), math.sin(math.radians(a)))) for a in d])
        jitter = 1.5 if kind == "kinetic" else -1.5
        axs[0].plot(d + jitter, f, mk, ms=3, alpha=0.7, label=kind)
        axs[1].plot(d + jitter, f / k, mk, ms=3, alpha=0.7, label=kind)
        norm_all += list(f / k)
        for a in sorted(set(d)):
            sel = d == a
            print(f"{kind:9s} dir {a:5.0f} deg: F {f[sel].mean():.2f} +/- {f[sel].std():.2f} N "
                  f"(n={sel.sum()}), F/k {np.mean(f[sel] / k[sel]):.2f} N")
    c = float(np.median(norm_all))
    axs[0].plot(grid, c * kg, "k--", lw=0.8, label=f"{c:.1f} N x k(dir)")
    axs[1].axhline(c, color="k", lw=0.8, ls="--")
    axs[0].set_ylabel("motor-side force (N)")
    axs[1].set_ylabel("force / k(dir) (N)")
    for ax in axs:
        ax.set_xlabel("direction (deg, 0 = fwd, 90 = side)")
        ax.set_xticks(range(0, 181, 45))
    for ax in axs:  # from 0, so the spread reads relative to the force itself
        ax.set_ylim(0, ax.get_ylim()[1] * 1.05)
    fig.legend(*axs[0].get_legend_handles_labels(), loc="outside lower center", ncol=3)
    axs[0].set_title("raw")
    axs[1].set_title("normalized by roller speed")
    tc.save(fig, args.out, "friction_direction")
    print(f"normalized median {c:.2f} N; spread (std/mean) raw "
          f"{np.std([r[2] for r in rows if r[1] != 'w']) / np.mean([r[2] for r in rows if r[1] != 'w']):.2f} "
          f"-> normalized {np.std(norm_all) / np.mean(norm_all):.2f}")
    w = [(k, f) for k, d, f, _ in rows if d == "w"]
    for kind in ("breakaway", "kinetic"):
        v = [f for k, f in w if k == kind]
        if v:
            print(f"rotation {kind}: {np.mean(v):.3f} +/- {np.std(v):.3f} N m (n={len(v)})")


if __name__ == "__main__":
    main()
