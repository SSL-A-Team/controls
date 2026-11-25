"""Figures from the identification fits, using recordings from record.py (offline).

  efficiency  measured m*a vs model eff*F_motor - k(dir)*C*dir   (fit-moves recording)
  slip        excess wheel speed vs linear wheel current, per wheel (slip-linear [+ slip-rotation])
  latency     vision-vs-integrated-heading alignment residual vs time shift (latency recording)

The math mirrors calibration/efficiency_friction.py, wheel_slip.py and
vision_latency.py; their printed numbers and these figures should agree.

    python analysis/tdp/fig_fits.py efficiency analysis/data/tdp/fit_moves_r2.npz
    python analysis/tdp/fig_fits.py slip analysis/data/tdp/slip_linear_r2.npz --rotation analysis/data/tdp/slip_rotation_r2.npz
    python analysis/tdp/fig_fits.py latency analysis/data/tdp/latency_r2.npz
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "calibration"))
import tdp_common as tc  # noqa: E402
from common import M, R, load, wheels  # noqa: E402
from vision_latency import firmware_default  # noqa: E402

try:
    from scipy.signal import savgol_filter
except ImportError:  # same filter in numpy (edges: nearest padding instead of scipy's interp)
    def savgol_filter(x, window, order, deriv=0, delta=1.0):
        h = window // 2
        V = np.vander(np.arange(-h, h + 1), order + 1, increasing=True)
        c = np.linalg.pinv(V)[deriv] * math.factorial(deriv) / delta ** deriv
        xp = np.pad(x, h, mode="edge")
        return np.convolve(xp, c[::-1], mode="valid")


def k_dir(dx, dy):
    D = M[:2].T
    roller = lambda x, y: np.abs(x * D[:, 1] - y * D[:, 0]).sum()
    return roller(dx, dy) / roller(0.0, 1.0)


def fig_efficiency(plt, path, mass, out):
    t, g = load(path)
    T = np.arange(t[0], t[-1], 0.01)
    x, y = np.interp(T, t, g("x")), np.interp(T, t, g("y"))
    th = np.interp(T, t, np.unwrap(g("th")))
    sg = lambda s, k: savgol_filter(s, 15, 3, deriv=k, delta=0.01)
    vx, vy, ax_, ay = sg(x, 1), sg(y, 1), sg(x, 2), sg(y, 2)
    c, s = np.cos(th), np.sin(th)
    v_l = np.stack([c * vx + s * vy, -s * vx + c * vy], 1)
    a_l = np.stack([c * ax_ + s * ay, -s * ax_ + c * ay], 1)
    I = np.stack([np.interp(T, t, col) for col in wheels(g, "i_sp").T], 1)
    F = (tc.KT / R) * I @ M.T
    speed = np.linalg.norm(v_l, axis=1)
    sel = np.where((speed > 0.1) & (np.abs(np.interp(T, t, g("gyro"))) < 1.0))[0][::2]
    rows, rhs, axis_of, dir_of = [], [], [], []
    for k in sel:
        u = v_l[k] / speed[k]
        kd = k_dir(*u)
        for j in range(2):
            rows.append([F[k, j], -kd * u[j]])
            rhs.append(mass * a_l[k, j])
            axis_of.append(j)
            dir_of.append(math.degrees(math.atan2(abs(u[1]), abs(u[0]))))
    A, b = np.array(rows), np.array(rhs)
    (eff, coul), *_ = np.linalg.lstsq(A, b, rcond=None)
    pred = A @ [eff, coul]
    resid = np.sqrt(np.mean((pred - b) ** 2))
    print(f"efficiency_xy {eff:.3f}, coulomb_xy {coul:.2f} N, residual {resid:.2f} N over {len(sel)} samples")

    fig, ax = plt.subplots(figsize=(tc.LNCS_WIDTH_IN * 0.55, tc.LNCS_WIDTH_IN * 0.5))
    sc = ax.scatter(pred, b, c=np.array(dir_of), s=3, cmap="viridis", vmin=0, vmax=90, alpha=0.6)
    lim = [min(pred.min(), b.min()), max(pred.max(), b.max())]
    ax.plot(lim, lim, "k--", lw=0.8)
    ax.set_xlabel(r"model $\eta F_{motor} - k(\hat v) C \hat v$ (N)")
    ax.set_ylabel(r"measured $m\,a$ (N)")
    ax.set_title(rf"$\eta$ = {eff:.2f}, C = {coul:.1f} N, rms {resid:.2f} N")
    fig.colorbar(sc, ax=ax, label="travel dir. (deg from fwd)")
    tc.save(fig, out, "fit_efficiency")


def slip_data(path, rotation):
    _, g = load(path)
    W = wheels(g, "enc")
    kv = np.stack([g("vx_l"), g("vy_l"), g("gyro")], 1)
    excess = W - kv @ M / R
    I = wheels(g, "i_sp")
    Fb = I @ M.T
    lin = np.stack([Fb[:, 0], Fb[:, 1], np.zeros(len(Fb))], 1) @ np.linalg.pinv(M).T
    sel = np.abs(g("gyro")) > 0.3 if rotation else np.hypot(g("vx_l"), g("vy_l")) > 0.05
    if sel.sum() < 30:
        raise SystemExit(f"{path}: only {sel.sum()} samples {'rotating' if rotation else 'moving'}; "
                         f"is this the {'slip-rotation' if rotation else 'slip-linear'} recording?")
    return ((I - lin) if rotation else lin)[sel], excess[sel]


def fig_slip(plt, path, rot_path, out):
    fig, axs = plt.subplots(1, 2 if rot_path else 1, figsize=(tc.LNCS_WIDTH_IN, 2.0), squeeze=False)
    for ax, p, rot in ((axs[0, 0], path, False), *(((axs[0, 1], rot_path, True),) if rot_path else ())):
        x, e = slip_data(p, rot)
        ks = []
        for i in range(4):
            k, b = np.polyfit(x[:, i], e[:, i], 1)
            ks.append(k)
            ax.scatter(x[::3, i], e[::3, i], s=2, alpha=0.35, label=f"{tc.WHEEL_NAMES[i]} K={k:.2f}")
            xx = np.linspace(x[:, i].min(), x[:, i].max(), 2)
            ax.plot(xx, k * xx + b, lw=0.8)
        print(f"{'rotation' if rot else 'linear'} slip K per wheel {np.round(ks, 2)}, mean {np.mean(ks):.2f} (rad/s)/A")
        ax.set_xlabel(f"{'rotation' if rot else 'linear'} part of wheel current (A)")
        ax.set_title(f"{'rotation' if rot else 'linear'}: mean K = {np.mean(ks):.2f} (rad/s)/A")
        ax.legend(markerscale=4, loc="upper left")
    axs[0, 0].set_ylabel("wheel speed - kinematic (rad/s)")
    tc.save(fig, out, "fit_slip")


def fig_latency(plt, path, min_latency_ms, ekf_delay_ms, out):
    t, g = load(path)
    v = (g("vis_upd") > 0) & (g("vis_old") == 0)
    tcap = t[v] - ekf_delay_ms * 1e-3
    vth = np.unwrap(g("vis_th")[v])
    gyro = g("gyro")
    moving = np.interp(tcap, t, np.convolve(np.abs(gyro), np.ones(30) / 30, "same")) > 0.15
    enc_rate = (R * wheels(g, "enc") @ np.linalg.pinv(M))[:, 2]
    shifts = np.arange(-0.15, 0.15, 0.001)
    fig, ax = plt.subplots(figsize=(tc.LNCS_WIDTH_IN * 0.55, tc.LNCS_WIDTH_IN * 0.42))
    best = []
    for name, rate in (("gyro", gyro), ("encoders", enc_rate)):
        heading = np.concatenate([[0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * np.diff(t))])
        res = []
        for d in shifts:
            h = np.interp(tcap + d, t, heading)
            A = np.stack([h, np.ones_like(h), tcap - tcap[0]], 1)[moving]
            sol = np.linalg.lstsq(A, vth[moving], rcond=None)[0]
            res.append(np.sqrt(np.mean((A @ sol - vth[moving]) ** 2)))
        res = np.degrees(res)
        i = int(np.argmin(res))
        best.append(shifts[i])
        ax.plot(min_latency_ms - shifts * 1e3, res, label=f"{name}: {min_latency_ms - shifts[i] * 1e3:.0f} ms")
    rec = min_latency_ms - np.mean(best) * 1e3
    ax.axvline(rec, color="k", lw=0.6, ls=":")
    ax.set_xlabel("assumed vision latency (ms)")
    ax.set_ylabel("heading residual (deg)")
    ax.set_title(f"vision latency {rec:.0f} ms")
    ax.legend()
    print(f"recommended latency {rec:.0f} ms (robot was running {min_latency_ms:.0f} ms)")
    tc.save(fig, out, "fit_latency")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kind", choices=("efficiency", "slip", "latency"))
    ap.add_argument("recording")
    ap.add_argument("--rotation", help="slip: slip-rotation recording for the second panel")
    ap.add_argument("--mass", type=float, default=tc.MASS)
    ap.add_argument("--min-latency-ms", type=float, default=firmware_default("DEFAULT_VISION_MIN_LATENCY_US"),
                    help="latency: DEFAULT_VISION_MIN_LATENCY_US the robot was running (ms; default: defaults.rs)")
    ap.add_argument("--ekf-delay-ms", type=float, default=firmware_default("DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US"),
                    help="latency: DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US the robot was running (ms; default: defaults.rs)")
    ap.add_argument("--out", default="analysis/tdp/figures")
    args = ap.parse_args()
    plt = tc.lncs_style()
    if args.kind == "efficiency":
        fig_efficiency(plt, args.recording, args.mass, args.out)
    elif args.kind == "slip":
        fig_slip(plt, args.recording, args.rotation, args.out)
    else:
        fig_latency(plt, args.recording, args.min_latency_ms, args.ekf_delay_ms, args.out)


if __name__ == "__main__":
    main()
