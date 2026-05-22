#!/usr/bin/env python3
"""
Interactive parameter tuning loop.

Workflow per iteration:
  1. Prompt to edit the param JSON file
  2. Upload all parameters to the robot via ros2 service calls
  3. Start ROS bag recording
  4. Run motion_input_node for one fn_period (5 s) then auto-exit
  5. Stop ROS bag recording
  6. Convert bag to npz via telem_bag2np
  7. Visualize with telem_visualize (blocks until plot window closed)
  8. Prompt to continue or quit
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parent
CONTROLS_DIR = next(p for p in SCRIPTS_DIR.parents if p.name == "controls")
WS_ROOT = next(p for p in CONTROLS_DIR.parents if (p / "install").is_dir())


def upload_params(robot_id: int, param_json_path: str):
    """Upload parameters by calling upload_params.py as a subprocess."""
    cmd = [
        sys.executable,
        str(SCRIPTS_DIR / "upload_params.py"),
        "--robot-id", str(robot_id),
        "--param-json", param_json_path,
    ]
    result = subprocess.run(cmd)
    return result.returncode == 0


def start_bag_recording(bag_path: str, topics: list = None) -> subprocess.Popen:
    """Start ros2 bag record in the background. Returns the Popen handle."""
    cmd = ["ros2", "bag", "record", "-o", bag_path, "-s", "mcap"]
    if topics:
        cmd += ["--topics"] + topics
    else:
        cmd.append("--all")
    print(f"Starting bag recording → {bag_path}")
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    time.sleep(2)  # give it a moment to initialize
    return proc


def stop_bag_recording(proc: subprocess.Popen):
    """Gracefully stop the bag recorder with SIGINT."""
    print("Stopping bag recording...")
    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    proc.wait(timeout=10)
    print("Bag recording stopped.\n")


def run_motion_input_node(robot_id: int, amp: float = 0.2, dimension: str = "x", fn_type: str = "oscillate", freq: float = 1.0, width: float = 0.5, duration: float = 5.0, param_json: str = None):
    """Run the motion_input_node for the given duration, then it auto-exits."""
    cmd = [
        "ros2", "run", "ateam_motion_input", "motion_input_node",
        "--ros-args",
        "-p", f"robot_id:={robot_id}",
        "-p", f"amp:={amp}",
        "-p", f"dimension:='{dimension}'",
        "-p", f"fn_type:='{fn_type}'",
        "-p", f"freq:={freq}",
        "-p", f"width:={width}",
        "-p", f"duration:={duration}",
    ]
    if param_json:
        cmd.extend(["-p", f"param_json:={param_json}"])
    if duration > 0:
        print(f"Running motion_input_node for {duration}s on robot {robot_id} (fn={fn_type}, dim={dimension}, amp={amp})...")
        subprocess.run(cmd, timeout=duration + 10)
    else:
        print(f"Running motion_input_node indefinitely on robot {robot_id} (fn={fn_type}, dim={dimension}, amp={amp}). Ctrl+C to stop.")
        proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        try:
            proc.wait()
        except KeyboardInterrupt:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=10)
    print("motion_input_node finished.\n")


def convert_bag_to_npz(bag_path: str, robot_id: int, output_path: str):
    """Run telem_bag2np.py to convert bag to npz."""
    cmd = [
        sys.executable,
        str(SCRIPTS_DIR / "telem_bag2np.py"),
        "--bag", bag_path,
        "--robot", str(robot_id),
        "--output", output_path,
    ]
    print(f"Converting bag to npz → {output_path}")
    subprocess.run(cmd, check=True)
    print()


def visualize(npz_path: str, param_json_path: str = None) -> list:
    """Launch telem_visualize.py in the background.
    Returns the list of Popen handles so they can be killed later."""
    cmd = [
        sys.executable,
        str(CONTROLS_DIR / "analysis" / "telem_visualize.py"),
        "--telemetry", npz_path,
    ]
    if param_json_path:
        cmd += ["--param-json", param_json_path]

    print("Launching visualization...")
    procs = [subprocess.Popen(cmd)]
    return procs


def kill_visualizations(procs: list):
    """Terminate any running visualization processes."""
    for proc in procs:
        if proc.poll() is None:
            proc.terminate()
    for proc in procs:
        proc.wait(timeout=5)
    print("Previous visualizations closed.")


def main():
    parser = argparse.ArgumentParser(
        description="Interactive parameter tuning loop",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--param-json", type=str, default=str(CONTROLS_DIR / "analysis" / "data" / "robot_params.json"),
        help="Path to the JSON file with RobotModel parameters",
    )
    parser.add_argument(
        "--robot-id", type=int, default=2,
        help="Robot ID (default: 2)",
    )
    parser.add_argument(
        "--amp", type=float, default=0.2,
        help="Amplitude of oscillation (default: 0.2)",
    )
    parser.add_argument(
        "--dimension", type=str, default="x", choices=["x", "y", "xy", "theta"],
        help="Dimension to oscillate: x, y, xy, or theta (default: x)",
    )
    parser.add_argument(
        "--fn-type", type=str, default="oscillate", choices=["pulse", "oscillate", "step", "bangbang_pose", "bangbang_accel", "square"],
        help="Control function type (default: oscillate)",
    )
    parser.add_argument(
        "--width", type=float, default=0.5,
        help="Pulse width in seconds, used with --fn-type pulse (default: 0.5)",
    )
    parser.add_argument(
        "--freq", type=float, default=1.0,
        help="Frequency in Hz (default: 1.0)",
    )
    parser.add_argument(
        "--duration", type=float, default=5.0,
        help="Duration in seconds to run motion_input_node (default: 5.0)",
    )
    parser.add_argument(
        "--output-dir", type=str, default=str(CONTROLS_DIR / "analysis" / "data"),
        help="Directory to store bags and npz files",
    )
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    # Determine starting iteration from existing files in output dir
    existing = [
        f.name for f in Path(args.output_dir).iterdir()
        if f.name.startswith("tuning_iter_")
    ]
    existing_iters = []
    for name in existing:
        # Extract number from tuning_iter_N or tuning_iter_N.npz
        base = name.replace(".npz", "")
        parts = base.split("_")
        try:
            existing_iters.append(int(parts[-1]))
        except ValueError:
            pass
    iteration = max(existing_iters, default=0)
    viz_procs = []

    try:
        while True:
            iteration += 1
            print("=" * 60)
            print(f"  ITERATION {iteration}")
            print("=" * 60)

            input(f"Edit {args.param_json} if desired, then press Enter to continue...")

            # Close previous visualization windows
            if viz_procs:
                kill_visualizations(viz_procs)
                viz_procs = []

            # 1. Upload params to robot
            if not upload_params(args.robot_id, args.param_json):
                print("Skipping iteration due to parameter upload failure.")
                continue

            # 2. Start bag recording
            bag_path = os.path.join(args.output_dir, f"tuning_iter_{iteration}")
            bag_topics = [
                f"/robot_feedback/extended/robot{args.robot_id}",
                f"/robot_feedback/basic/robot{args.robot_id}",
                f"/robot_motion_commands/robot{args.robot_id}",
            ]
            print("Starting bag recording...")
            bag_proc = start_bag_recording(bag_path, topics=bag_topics)
            # print("Waiting 2 seconds...")
            # time.sleep(2)  # ensure bag is recording before starting the test

            try:
                # 3. Run motion_input_node
                run_motion_input_node(args.robot_id, amp=args.amp, dimension=args.dimension, fn_type=args.fn_type, freq=args.freq, width=args.width, duration=args.duration, param_json=args.param_json)
            finally:
                # 4. Stop bag recording
                stop_bag_recording(bag_proc)

            # 5. Convert bag to npz
            npz_path = os.path.join(args.output_dir, f"tuning_iter_{iteration}.npz")
            convert_bag_to_npz(bag_path, args.robot_id, npz_path)

            # 6. Visualize
            viz_procs = visualize(npz_path, param_json_path=args.param_json)

            # # 7. Continue?
            # answer = input("Run another iteration? [Y/n] ").strip().lower()
            # if answer in ("n", "no"):
            #     kill_visualizations(viz_procs)
            #     print("Done.")
            #     break
    finally:
        pass


if __name__ == "__main__":
    main()
