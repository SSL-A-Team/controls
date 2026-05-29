#!/usr/bin/env python3
"""Upload all parameters from a JSON file to a robot via ROS2 service calls."""

import argparse
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from ateam_msgs.srv import SetFirmwareParameter

SCRIPTS_DIR = Path(__file__).resolve().parent
CONTROLS_DIR = next(p for p in SCRIPTS_DIR.parents if p.name == "controls")

# Import PARAM_MAP from analysis/params.py
sys.path.insert(0, str(CONTROLS_DIR / "analysis"))
from params import PARAM_MAP


def set_firmware_param(node: Node, client, robot_id: int, param_id: int, data: list):
    """Send a single parameter to the robot via the SetFirmwareParameter service."""
    req = SetFirmwareParameter.Request()
    req.robot_id = robot_id
    req.parameter_id = param_id
    req.data = [float(v) for v in data]

    for attempt in range(3):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result() is None:
            print(f"  WARNING: param_id={param_id} — service call timed out")
            return False
        resp = future.result()
        if resp.success:
            print(f"  param_id={param_id}  data={data}  OK")
            return True
        print(f"  WARNING: param_id={param_id} — {resp.reason} (attempt {attempt + 1}/3)")

    print(f"  ERROR: param_id={param_id} — failed after 3 attempts")
    return False


def upload_params(robot_id: int, param_json_path: str):
    """Read the JSON param file and upload every parameter to the robot."""
    with open(param_json_path) as f:
        params = json.load(f)

    node = rclpy.create_node("param_uploader")
    client = node.create_client(SetFirmwareParameter, "/set_firmware_param")

    print(f"Waiting for /set_firmware_param service...")
    if not client.wait_for_service(timeout_sec=5.0):
        print("ERROR: /set_firmware_param service not available")
        node.destroy_node()
        return False

    print(f"Uploading parameters to robot {robot_id} from {param_json_path}")
    all_ok = True
    for name, info in PARAM_MAP.items():
        param_id = info["param_id"]
        if param_id is None:
            continue  # skip unimplemented parameters
        if name in params:
            value = params[name]
            data = [float(v) for v in value] if isinstance(value, list) else [float(value)]
            if not set_firmware_param(node, client, robot_id, param_id, data):
                all_ok = False

    node.destroy_node()
    if all_ok:
        print("Parameter upload complete.\n")
    else:
        print("ERROR: Parameter upload had failures.\n")
    return all_ok


def main():
    parser = argparse.ArgumentParser(description="Upload firmware parameters to a robot")
    parser.add_argument(
        "-p", "--param-json", type=str,
        default=str(CONTROLS_DIR / "analysis" / "data" / "robot_params.json"),
        help="Path to the JSON file with robot parameters",
    )
    parser.add_argument(
        "-r", "--robot-id", type=int, default=2,
        help="Robot ID (default: 2)",
    )
    args = parser.parse_args()

    rclpy.init()
    try:
        success = upload_params(args.robot_id, args.param_json)
    finally:
        rclpy.shutdown()

    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
