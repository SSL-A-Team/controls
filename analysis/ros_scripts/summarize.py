"""Generic summary of a small_move_test-family log (.npz with `data` rows).

Row layout (small_move_test.on_telem): 0 t, 1-3 KF pose, 4-6 KF vel (robot-local),
7 gyro, 8-11 wheel current setpoints, 12-15 wheel vel, 16-18 traj pos, 19-21 vel_u,
22-24 accel_u_fric_comp, 25-27 target (pose, NaN when velocity mode), 28 vision_update,
29 body_control_mode, 30-33 wheel vel setpoints. vel_test-family files also carry
`vel_target` (commanded [vx, vy, w], NaN when not in velocity mode).

    python analysis/ros_scripts/summarize.py FILE [--plot out.png]
"""

import argparse
import math

import numpy as np


def load(path):
    d = np.load(path, allow_pickle=True)
    a = np.array([np.asarray(r, float)[:34] for r in d["data"]])
    vt = d["vel_target"][: len(a)] if "vel_target" in d.files else np.full((len(a), 3), np.nan)
    return a, vt, str(d["tripped"]) if "tripped" in d.files else ""


def wrap(x):
    return np.remainder(x + math.pi, 2 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--plot", default=None)
    args = ap.parse_args()
    a, vt, tripped = load(args.file)
    t = a[:, 0] - a[0, 0]
    active = a[:, 29] != 0
    print(f"{args.file.split('/')[-1]}: {t[-1]:.1f}s, tripped: {tripped or 'no'}")
    if not active.any():
        print("  never active")
        return
    pos_mode = active & (a[:, 29] == 10)
    vel_mode = active & ~np.isnan(vt[:, 0])
    I = a[:, 8:12]
    dI = np.abs(np.diff(I, axis=0)).mean(1)
    print(f"  max |I| {np.abs(I[active]).max():.2f} A, max |gyro| {np.abs(a[active, 7]).max():.2f}, "
          f"max |wheel v| {np.abs(a[active, 12:16]).max():.1f}, xy range x[{a[active,1].min():.3f},{a[active,1].max():.3f}] "
          f"y[{a[active,2].min():.3f},{a[active,2].max():.3f}]")
    print(f"  current chatter mean|dI| per sample {dI[active[1:]].mean():.3f} A; "
          f"accel_fc mean|step| xy {np.abs(np.diff(a[active, 22:24], axis=0)).mean():.3f} "
          f"w {np.abs(np.diff(a[active, 24])).mean():.2f}")
    if pos_mode.any():
        exy = 1e3 * np.hypot(a[pos_mode, 16] - a[pos_mode, 1], a[pos_mode, 17] - a[pos_mode, 2])
        eth = np.degrees(np.abs(wrap(a[pos_mode, 18] - a[pos_mode, 3])))
        gxy = 1e3 * np.hypot(a[pos_mode, 25] - a[pos_mode, 1], a[pos_mode, 26] - a[pos_mode, 2])
        gth = np.degrees(np.abs(wrap(a[pos_mode, 27] - a[pos_mode, 3])))
        # settled = trajectory at the target
        settled = pos_mode & (np.hypot(a[:, 16] - a[:, 25], a[:, 17] - a[:, 26]) < 1e-3) & \
            (np.abs(wrap(a[:, 18] - a[:, 27])) < 1e-3)
        print(f"  POS traj-vs-est: xy rms {np.sqrt((exy**2).mean()):.1f} max {exy.max():.0f} mm, "
              f"th rms {np.sqrt((eth**2).mean()):.2f} max {eth.max():.1f} deg")
        if settled.sum() > 20:
            sxy = 1e3 * np.hypot(a[settled, 25] - a[settled, 1], a[settled, 26] - a[settled, 2])
            sth = np.degrees(np.abs(wrap(a[settled, 27] - a[settled, 3])))
            print(f"  POS settled ({settled.sum()} samples): target err xy median {np.median(sxy):.1f} "
                  f"p95 {np.percentile(sxy, 95):.1f} mm, th median {np.median(sth):.2f} p95 {np.percentile(sth, 95):.2f} deg, "
                  f"I std {I[settled].std(0).mean():.2f}")
    if vel_mode.any():
        vl = np.stack([a[:, 4], a[:, 5], a[:, 7]], 1)  # KF velocity is already robot-local
        e = vl[vel_mode] - vt[vel_mode]
        print(f"  VEL cmd-vs-actual(local, gyro) rms: vx {np.sqrt((e[:,0]**2).mean()):.3f} vy {np.sqrt((e[:,1]**2).mean()):.3f} "
              f"w {np.sqrt((e[:,2]**2).mean()):.3f}; max |w err| {np.abs(e[:,2]).max():.2f}")
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(5, 1, figsize=(12, 13), sharex=True)
        ax[0].plot(t, a[:, 25], ".", ms=2, label="tgt x"); ax[0].plot(t, a[:, 16], label="traj x"); ax[0].plot(t, a[:, 1], label="est x")
        ax[0].plot(t, a[:, 26], ".", ms=2, label="tgt y"); ax[0].plot(t, a[:, 17], label="traj y"); ax[0].plot(t, a[:, 2], label="est y")
        ax[0].legend(ncol=3, fontsize=7); ax[0].set_ylabel("m")
        ax[1].plot(t, a[:, 27], ".", ms=2, label="tgt th"); ax[1].plot(t, a[:, 18], label="traj th"); ax[1].plot(t, a[:, 3], label="est th")
        ax[1].legend(fontsize=7); ax[1].set_ylabel("rad")
        ax[2].plot(t, vt[:, 2], "--", label="cmd w (vel mode)"); ax[2].plot(t, a[:, 21], label="vel_u w"); ax[2].plot(t, a[:, 7], label="gyro")
        ax[2].plot(t, a[:, 19], label="vel_u x"); ax[2].plot(t, a[:, 20], label="vel_u y"); ax[2].legend(ncol=5, fontsize=7)
        ax[3].plot(t, a[:, 22:25]); ax[3].legend(["ax", "ay", "aw"], fontsize=7); ax[3].set_ylabel("accel_u_fc")
        ax[4].plot(t, I); ax[4].set_ylabel("I sp (A)")
        for x in ax:
            x.grid()
        plt.tight_layout()
        plt.savefig(args.plot, dpi=60)


if __name__ == "__main__":
    main()
