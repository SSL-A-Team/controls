"""Decode and analyze HFCAP rows (1 kHz control-loop capture) from a defmt RTT log.

Firmware: build control-board with `--features hf-capture` (src/tasks/hf_capture.rs).

    python analysis/hf_capture_decode.py /tmp/radiodbg/rtt.log [--out cap.npz] [--plot]
"""

from __future__ import annotations

import argparse
import re

import numpy as np

NAMES = (["tick", "gyro", "x", "y", "th"]
         + [f"wv{i}" for i in range(4)] + [f"sp{i}" for i in range(4)]
         + [f"iff{i}" for i in range(4)] + [f"itot{i}" for i in range(4)]
         + ["ux", "uy", "uw"])
SCALE = np.array([1, 1e3, 1e4, 1e4, 1e4] + [100] * 8 + [1e3] * 8 + [1e3, 1e3, 1e3], dtype=float)
ROW = re.compile(r"HFCAP (\d+) \[([-\d, ]+)\]")


def load(path: str) -> np.ndarray:
    """Return the last complete capture in the log as an (N, 24) float array in SI units."""
    caps, cur = [], {}
    with open(path, errors="replace") as f:
        for line in f:
            m = ROW.search(line)
            if not m:
                continue
            i = int(m.group(1))
            if i == 0 and cur:
                caps.append(cur)
                cur = {}
            cur[i] = [int(v) for v in m.group(2).split(",")]
    if cur:
        caps.append(cur)
    if not caps:
        raise SystemExit("no HFCAP rows")
    cap = caps[-1]
    n = max(cap) + 1
    missing = n - len(cap)
    a = np.full((n, len(NAMES)), np.nan)
    for i, r in cap.items():
        a[i] = r
    if missing:
        print(f"warning: {missing} rows missing (RTT overflow); interpolating")
        for c in range(a.shape[1]):
            bad = np.isnan(a[:, c])
            a[bad, c] = np.interp(np.flatnonzero(bad), np.flatnonzero(~bad), a[~bad, c])
    return a / SCALE


def psd(x: np.ndarray, fs: float = 1000.0, nseg: int = 256):
    """Welch PSD with a Hann window (no scipy dependency)."""
    x = x - x.mean()
    step = nseg // 2
    w = np.hanning(nseg)
    segs = [x[i:i + nseg] * w for i in range(0, len(x) - nseg + 1, step)]
    p = np.mean([np.abs(np.fft.rfft(s)) ** 2 for s in segs], axis=0) / (fs * (w ** 2).sum())
    p[1:-1] *= 2
    return np.fft.rfftfreq(nseg, 1 / fs), p


def band_rms(f, p, lo, hi):
    m = (f >= lo) & (f < hi)
    return float(np.sqrt(np.trapz(p[m], f[m]))) if m.any() else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--out", default=None)
    ap.add_argument("--start", type=float, default=0.0, help="analysis window start (s)")
    ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()
    a = load(args.log)
    if args.out:
        np.savez(args.out, data=a, names=np.array(NAMES))
    t = np.arange(len(a)) * 1e-3
    sel = (t >= args.start) & (t < (args.end if args.end else t[-1] + 1))
    s = a[sel]
    bands = [(0, 2), (2, 8), (8, 20), (20, 60), (60, 200), (200, 500)]
    print("band RMS per signal [" + " ".join(f"{lo}-{hi}Hz" for lo, hi in bands) + "]  peak(>2Hz)")
    for name in ["gyro", "wv0", "wv1", "wv2", "wv3", "itot0", "itot1", "itot2", "itot3",
                 "iff0", "iff1", "iff2", "iff3", "sp0", "uw", "ux", "uy"]:
        x = s[:, NAMES.index(name)]
        f, p = psd(x)
        pk = f[2:][np.argmax(p[2:])]
        print(f"{name:6s} " + " ".join(f"{band_rms(f, p, lo, hi):8.3f}" for lo, hi in bands) + f"   {pk:6.1f}")
    if args.plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(4, 1, sharex=True, figsize=(12, 10))
        ax[0].plot(t, a[:, 1], label="gyro"); ax[0].plot(t, a[:, 23], label="u_w"); ax[0].legend()
        for i in range(4):
            ax[1].plot(t, a[:, 5 + i], label=f"wv{i}")
            ax[2].plot(t, a[:, 17 + i], label=f"itot{i}")
        ax[3].plot(t, a[:, 2] - a[0, 2], label="x"); ax[3].plot(t, a[:, 3] - a[0, 3], label="y")
        ax[3].plot(t, a[:, 4] - a[0, 4], label="th")
        for x in ax:
            x.legend(fontsize=7)
        plt.savefig(args.out.replace(".npz", ".png") if args.out else "/tmp/hfcap.png", dpi=110)


if __name__ == "__main__":
    main()
