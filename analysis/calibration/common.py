"""Shared helpers for the calibration scripts: record extended telemetry while running
the ros_scripts motion tests, load recordings, robot geometry."""

from __future__ import annotations

import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROS_SCRIPTS = Path(__file__).resolve().parents[1] / "ros_scripts"
CONTROLS_ROOT = ROS_SCRIPTS.parents[1]  # the tests save to analysis/data/... relative to here

# Robot geometry (ateam-controls defaults.rs): wheel angles, wheel distance, wheel radius
ALPHA, BETA, L, R = math.pi / 6, math.pi / 4, 0.0814, 0.030
# Wheel forces -> local [Fx, Fy, Tz]; M.T / R maps a local twist to wheel speeds (rad/s)
M = np.array([[-math.cos(ALPHA), -math.cos(BETA), math.cos(BETA), math.cos(ALPHA)],
              [math.sin(ALPHA), -math.sin(BETA), -math.sin(BETA), math.sin(ALPHA)],
              [L, L, L, L]])


def add_common_args(ap):
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--home", type=str, required=True,
                    help="absolute home pose 'x,y,theta' (inside the tether box)")
    ap.add_argument("--heading-ref", type=float, required=True,
                    help="absolute heading (rad) the tests' +/-95 deg limit is measured from")
    ap.add_argument("--out", type=str, default="/tmp/calib.npz", help="telemetry recording path")


def test_cmd(script, args, *extra):
    """Command line for a ros_scripts motion test (they carry the tether safety monitor)."""
    return [sys.executable, str(ROS_SCRIPTS / f"{script}.py"), "-r", str(args.robot),
            f"--home={args.home}", "--heading-ref", str(args.heading_ref), *map(str, extra)]


def move(args, dx, dy, dtheta_deg=0.0, amax=3.0):
    """Out-and-back position move from home (small_move_test)."""
    return test_cmd("small_move_test", args, "--dx", dx, "--dy", dy, "--dtheta-deg", dtheta_deg,
                    "--amax", amax, "--alphamax", 10, "--vmax", 3.0, "--wmax", 3.0,
                    "--cycles", 1, "--hold", 1.5)


def record_during(args, cmds, duration):
    """Record ExtendedTelemetry to args.out while running each test command in turn."""
    rec = subprocess.Popen([sys.executable, str(ROS_SCRIPTS / "telem_record.py"), "-r", str(args.robot),
                            "--duration", str(duration), "--out", args.out],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)
    for cmd in cmds:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=120, cwd=CONTROLS_ROOT)
        trip = [l for l in (res.stdout + res.stderr).splitlines() if "TRIP" in l or "too large" in l]
        if trip:
            print("  test stopped:", trip[-1].strip())
    rec.wait(timeout=duration + 30)
    return load(args.out)


def load(path):
    """Load a telem_record npz sorted by robot time, duplicates removed.
    Returns (t_robot, getter) where getter(name) returns that column."""
    d = np.load(path, allow_pickle=True)
    cols = list(d["columns"])
    a = np.array(d["data"], float)
    if len(a) == 0:
        raise SystemExit(f"{path}: no telemetry recorded (is the extended telemetry DIP switch on?)")
    t = a[:, cols.index("t_robot")]
    o = np.argsort(t)
    a, t = a[o], t[o]
    t, i = np.unique(t, return_index=True)
    a = a[i]
    return t, (lambda k: a[:, cols.index(k)])


def wheels(g, field):
    return np.stack([g(f"{field}{i}") for i in range(4)], 1)


def set_param(robot, pid, values):
    """Set a firmware radio parameter (live, until the next reboot). values: 'a,b,...'"""
    for _ in range(10):
        r = subprocess.run(["ros2", "service", "call", "/set_firmware_param", "ateam_msgs/srv/SetFirmwareParameter",
                            f"{{robot_id: {robot}, parameter_id: {pid}, data: [{values}]}}"],
                           capture_output=True, text=True, timeout=30)
        if "success=True" in r.stdout:
            return True
        time.sleep(1.0)
    return False


def get_param(robot, pid):
    for _ in range(10):
        r = subprocess.run(["ros2", "service", "call", "/get_firmware_param", "ateam_msgs/srv/GetFirmwareParameter",
                            f"{{robot_id: {robot}, parameter_id: {pid}}}"],
                           capture_output=True, text=True, timeout=30)
        if "success=True" in r.stdout:
            inside = r.stdout.split("data=[", 1)[1].split("]", 1)[0]
            return [float(x) for x in inside.split(",") if x.strip()]
        time.sleep(1.0)
    return None


def move_tracking(npz):
    """Trajectory-vs-estimate tracking of a small_move_test log while moving:
    (XY rms mm, heading rms deg, time to within 5 mm of each target (s) list)."""
    a = np.array([np.asarray(r, float)[:39] for r in np.load(npz, allow_pickle=True)["data"]])
    t = a[:, 38]
    ok = ~np.isnan(a[:, 25])
    mv = ok & (np.linalg.norm(a[:, 19:21], axis=1) > 0.02)
    e = np.hypot(a[mv, 16] - a[mv, 1], a[mv, 17] - a[mv, 2]) * 1e3
    h = np.degrees(np.remainder(a[mv, 18] - a[mv, 3] + np.pi, 2 * np.pi) - np.pi)
    reach = []
    ch = np.where(ok[1:] & ok[:-1] & np.any(np.abs(np.diff(a[:, 25:28], axis=0)) > 1e-6, axis=1))[0] + 1
    for c in ch:
        q = (t >= t[c]) & (t < t[c] + 2.5)
        d = np.hypot(a[q, 1] - a[c, 25], a[q, 2] - a[c, 26])
        reach.append(float(t[q][np.argmax(d < 0.005)] - t[c]) if (d < 0.005).any() else float("nan"))
    return float(np.sqrt((e ** 2).mean())), float(np.sqrt((h ** 2).mean())), reach


MOVE_SET = ((0.2, 0.0, 0.0), (0.0, 0.2, 0.0), (0.15, 0.15, 90.0))


def run_move_set(args, repeats=1):
    """Runs the forward / sideways / diagonal+90 move pairs; returns mean tracking
    (XY rms mm, heading rms deg, mean time-to-5 mm s) over them."""
    data = CONTROLS_ROOT / "analysis/data/small_move"
    xy, hd, reach = [], [], []
    for _ in range(repeats):
        for dx, dy, dth in MOVE_SET:
            before = set(data.glob("*.npz")) if data.exists() else set()
            res = subprocess.run(move(args, dx, dy, dth), capture_output=True, text=True, timeout=120,
                                 cwd=CONTROLS_ROOT)
            new = sorted(set(data.glob("*.npz")) - before)
            empty = not new or len(np.load(new[-1], allow_pickle=True)["data"]) < 50
            if "TRIP" in res.stdout + res.stderr or empty:
                print(f"    move ({dx}, {dy}, {dth}) tripped or got no telemetry (bridge running?); skipped")
                continue
            x, h, r = move_tracking(new[-1])
            xy.append(x)
            hd.append(h)
            reach += r
    if not xy:
        return float("nan"), float("nan"), float("nan")
    return float(np.mean(xy)), float(np.mean(hd)), float(np.nanmean(reach))


def validate(args, pid, candidates, label):
    """A/B the candidate values of radio parameter `pid` on the move set."""
    print(f"\nvalidating {label} on the move set (lower is better):")
    for c in candidates:
        if not set_param(args.robot, pid, c):
            print(f"  {c}: could not set parameter {pid}")
            continue
        xy, hd, reach = run_move_set(args, repeats=args.repeats)
        print(f"  {label} = {c:24s}: XY track rms {xy:5.1f} mm, heading {hd:4.2f} deg, reach 5 mm in {reach:.2f} s")
    print("  (reflash or reboot the robot to restore the compiled values)")
