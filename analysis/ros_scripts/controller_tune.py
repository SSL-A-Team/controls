#!/usr/bin/env python3
"""
Controller tuning one-shot pipeline.

Single trial that:

  1. Uploads robot firmware parameters from a JSON file (in-process via
     ``params.upload_params_dict``).
  2. Starts a ``ros2 bag record`` on the robot's telemetry + command topics.
  3. Runs ``signal_input.py`` to drive a parameterized signal (pulse,
     sinusoid, step, chirp) for ``--duration`` seconds.
  4. Stops the bag, converts it to NPZ via ``telem_bag2np.py``, and
     launches ``telem_visualize.py`` (blocks until the plot window closes).

Bags and NPZs are auto-indexed under ``analysis/data/{bags,telemetry}/`` so
repeated runs don't overwrite each other.

Example::

    python ros_scripts/controller_tune.py \\
        --robot-id 2 --axis x --signal sinusoid \\
        --amplitude 0.3 --sine-frequency 1.5 --duration 8.0
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from ateam_msgs.srv import SetFirmwareParameter

SCRIPTS_DIR = Path(__file__).resolve().parent
CONTROLS_DIR = next(p for p in SCRIPTS_DIR.parents if p.name == "controls")
ANALYSIS_DIR = CONTROLS_DIR / "analysis"

# analysis/ is a flat (non-package) python dir; make it importable so we can
# pull in params.upload_params_dict the same way other tuning scripts do.
sys.path.insert(0, str(ANALYSIS_DIR))
from params import upload_params_dict  # noqa: E402

SIGNAL_INPUT_SCRIPT = SCRIPTS_DIR / "signal_input.py"
TELEM_BAG2NP_SCRIPT = SCRIPTS_DIR / "telem_bag2np.py"
TELEM_VISUALIZE_SCRIPT = ANALYSIS_DIR / "telem_visualize.py"


# ---------------------------------------------------------------- pipeline steps

def upload_params(robot_id: int, param_json_path: Path) -> bool:
    """Upload firmware params in-process. Returns True on full success."""
    if not param_json_path.is_file():
        print(f"[upload] ERROR: param JSON not found: {param_json_path}")
        return False
    with open(param_json_path) as f:
        params = json.load(f)

    rclpy.init()
    node = rclpy.create_node("controller_tune_param_uploader")
    client = node.create_client(SetFirmwareParameter, "/set_firmware_param")
    try:
        print(f"[upload] waiting for /set_firmware_param service...")
        if not client.wait_for_service(timeout_sec=5.0):
            print("[upload] ERROR: /set_firmware_param service not available")
            return False
        print(f"[upload] uploading params for robot {robot_id} from {param_json_path}")
        ok = upload_params_dict(node, client, robot_id, params, logger=print)
        print(f"[upload] {'OK' if ok else 'FAILED'}")
        return ok
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def pick_run_index(bags_dir: Path, telem_dir: Path,
                   bag_prefix: str, npz_prefix: str) -> int:
    """Find the next index N such that neither bag nor npz exists yet."""
    bags_dir.mkdir(parents=True, exist_ok=True)
    telem_dir.mkdir(parents=True, exist_ok=True)
    n = 0
    while ((bags_dir / f"{bag_prefix}_{n}").exists()
           or (telem_dir / f"{npz_prefix}_{n}.npz").exists()):
        n += 1
    return n


def start_bag(bag_path: Path, topics: list[str], robot_id: int) -> subprocess.Popen:
    """Start ros2 bag record in its own process group so we can SIGINT it.

    Writes a per-topic QoS override file so the bag recorder subscribes to the
    motion command topic with BEST_EFFORT. Without this, rosbag2 defaults to
    RELIABLE which is incompatible with signal_input.py's BEST_EFFORT
    publisher (matched to the radio_bridge subscriber), and the bag would
    silently miss every command message.
    """
    qos_path = bag_path.parent / f"{bag_path.name}_qos.yaml"
    qos_path.parent.mkdir(parents=True, exist_ok=True)
    qos_path.write_text(
        f"/robot_motion_commands/robot{robot_id}:\n"
        f"  reliability: best_effort\n"
        f"  history: keep_last\n"
        f"  depth: 100\n"
        f"  durability: volatile\n"
    )
    cmd = [
        "ros2", "bag", "record",
        "-o", str(bag_path),
        "-s", "mcap",
        "--qos-profile-overrides-path", str(qos_path),
        *topics,
    ]
    print(f"[bag] starting recording -> {bag_path}")
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    # Give the recorder a moment to subscribe before the signal starts.
    time.sleep(2.0)
    return proc


def stop_bag(proc: subprocess.Popen) -> None:
    """Gracefully stop ros2 bag record with SIGINT to its process group."""
    if proc.poll() is not None:
        return
    print("[bag] stopping recording...")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=10)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        print("[bag] forcing kill")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=5)
    print("[bag] stopped")


def run_signal_input(args: argparse.Namespace) -> int:
    """Run signal_input.py in the foreground (blocks)."""
    cmd = [
        sys.executable, str(SIGNAL_INPUT_SCRIPT),
        "--robot-id", str(args.robot_id),
        "--team-color", args.team_color,
        "--control-mode", args.control_mode,
        "--axis", args.axis,
        "--signal", args.signal,
        "--amplitude", str(args.amplitude),
        "--pulse-frequency", str(args.pulse_frequency),
        "--pulse-width", str(args.pulse_width),
        "--sine-frequency", str(args.sine_frequency),
        "--chirp-start-freq", str(args.chirp_start_freq),
        "--chirp-end-freq", str(args.chirp_end_freq),
        "--chirp-duration", str(args.chirp_duration),
        "--step-time", str(args.step_time),
        "--warmup-duration", str(args.warmup_duration),
        "--duration", str(args.duration),
        "--rate-hz", str(args.rate_hz),
        "--limit-vel-linear", str(args.limit_vel_linear),
        "--limit-vel-angular", str(args.limit_vel_angular),
        "--limit-acc-linear", str(args.limit_acc_linear),
        "--limit-acc-angular", str(args.limit_acc_angular),
    ]
    print(f"[signal] running for {args.duration}s "
          f"(+{args.warmup_duration}s warmup)")
    timeout = args.warmup_duration + args.duration + 30.0
    return subprocess.run(cmd, timeout=timeout).returncode


def convert_bag(bag_path: Path, robot_id: int, npz_path: Path) -> bool:
    cmd = [
        sys.executable, str(TELEM_BAG2NP_SCRIPT),
        "--bag", str(bag_path),
        "--robot", str(robot_id),
        "--output", str(npz_path),
    ]
    print(f"[convert] {bag_path} -> {npz_path}")
    return subprocess.run(cmd).returncode == 0


def visualize(npz_path: Path, param_json_path: Path | None) -> int:
    cmd = [sys.executable, str(TELEM_VISUALIZE_SCRIPT),
           "--telemetry", str(npz_path)]
    if param_json_path is not None:
        cmd += ["--param-json", str(param_json_path)]
    print(f"[viz] launching telem_visualize.py (close window to exit)")
    return subprocess.run(cmd).returncode


# ------------------------------------------------------------------------ CLI

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    # Pipeline / IO
    p.add_argument("--param-json", type=str,
                   default=str(ANALYSIS_DIR / "data" / "robot_params.json"))
    p.add_argument("--output-dir", type=str,
                   default=str(ANALYSIS_DIR / "data"),
                   help="Parent dir; bags/ and telemetry/ subdirs are used")
    p.add_argument("--skip-upload", action="store_true")
    p.add_argument("--skip-viz", action="store_true")

    # Robot
    p.add_argument("--robot-id", type=int, default=2)
    p.add_argument("--team-color", type=str, default="blue",
                   choices=["blue", "yellow"])

    # Signal (passed through to signal_input.py)
    p.add_argument("--control-mode", type=str, default="position",
                   choices=["position", "velocity"])
    p.add_argument("--axis", type=str, default="x",
                   choices=["x", "y", "theta"])
    p.add_argument("--signal", type=str, default="sinusoid",
                   choices=["pulse", "sinusoid", "step", "chirp"])
    p.add_argument("--amplitude", type=float, default=0.2)
    p.add_argument("--pulse-frequency", type=float, default=1.0)
    p.add_argument("--pulse-width", type=float, default=0.2)
    p.add_argument("--sine-frequency", type=float, default=1.0)
    p.add_argument("--chirp-start-freq", type=float, default=0.1)
    p.add_argument("--chirp-end-freq", type=float, default=5.0)
    p.add_argument("--chirp-duration", type=float, default=30.0)
    p.add_argument("--step-time", type=float, default=1.0)
    p.add_argument("--warmup-duration", type=float, default=2.0)
    p.add_argument("--duration", type=float, default=5.0,
                   help="signal-phase duration in seconds; must be > 0 "
                        "for a one-shot trial")
    p.add_argument("--rate-hz", type=float, default=100.0)
    p.add_argument("--limit-vel-linear", type=float, default=0.0)
    p.add_argument("--limit-vel-angular", type=float, default=0.0)
    p.add_argument("--limit-acc-linear", type=float, default=0.0)
    p.add_argument("--limit-acc-angular", type=float, default=0.0)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if args.duration <= 0:
        print("ERROR: --duration must be > 0 for a one-shot trial")
        return 2

    output_dir = Path(args.output_dir)
    bags_dir = output_dir / "bags"
    telem_dir = output_dir / "telemetry"
    param_json_path = Path(args.param_json)

    # Step 1: upload params
    if not args.skip_upload:
        if not upload_params(args.robot_id, param_json_path):
            print("[abort] parameter upload failed")
            return 1
    else:
        print("[upload] skipped")

    # Step 2: pick output paths
    idx = pick_run_index(bags_dir, telem_dir,
                         bag_prefix="signal", npz_prefix="signal")
    bag_path = bags_dir / f"signal_{idx}"
    npz_path = telem_dir / f"signal_{idx}.npz"
    print(f"[run] index={idx} bag={bag_path} npz={npz_path}")

    # Step 3: start bag recording
    topics = [
        f"/robot_feedback/extended/robot{args.robot_id}",
        f"/robot_feedback/basic/robot{args.robot_id}",
        f"/robot_motion_commands/robot{args.robot_id}",
    ]
    bag_proc = start_bag(bag_path, topics, args.robot_id)

    signal_rc = 1
    try:
        # Step 4: run the signal generator
        signal_rc = run_signal_input(args)
        if signal_rc != 0:
            print(f"[signal] WARN: signal_input.py exited with {signal_rc}")
    finally:
        # Step 5: always stop the bag
        stop_bag(bag_proc)

    # Step 6: convert
    if not convert_bag(bag_path, args.robot_id, npz_path):
        print("[convert] FAILED — skipping visualization")
        return 1

    # Step 7: visualize (blocking)
    if args.skip_viz:
        print("[viz] skipped")
    else:
        visualize(npz_path,
                  param_json_path if param_json_path.is_file() else None)

    return 0 if signal_rc == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
