import argparse
import re
import numpy as np
from collections import defaultdict
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


CONTROLS_REPO_PATH = next(p for p in Path(__file__).resolve().parents if p.name == "controls")
MSG_TYPE = "ateam_radio_msgs/msg/ExtendedTelemetry"
EXTENDED_TOPIC_RE = re.compile(r"^/robot_feedback/extended/robot(\d+)$")


def available_robots(bag_path):
    """Return a sorted list of robot numbers with extended-telemetry data.

    Reads the bag's ``metadata.yaml`` and returns the robot numbers whose
    ``/robot_feedback/extended/robot<N>`` topic has ``message_count > 0``.
    """
    metadata_path = Path(bag_path) / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"No metadata.yaml found in bag: {bag_path}")

    import yaml

    with open(metadata_path) as f:
        metadata = yaml.safe_load(f)

    info = metadata["rosbag2_bagfile_information"]
    robots = []
    for entry in info["topics_with_message_count"]:
        name = entry["topic_metadata"]["name"]
        match = EXTENDED_TOPIC_RE.match(name)
        if match and entry["message_count"] > 0:
            robots.append(int(match.group(1)))
    return sorted(robots)


def infer_robot(bag_path):
    """Infer which robot to use from the bag's available telemetry streams.

    Errors if none are available. If multiple are available, warns with the
    list of available robots and returns the lowest-index one.
    """
    robots = available_robots(bag_path)
    if not robots:
        raise SystemExit(
            f"ERROR: no extended-telemetry streams with data found in bag: {bag_path}"
        )
    if len(robots) > 1:
        print(
            f"WARNING: multiple extended-telemetry streams available: robots {robots}. "
            f"Continuing with robot {robots[0]}."
        )
    return robots[0]


def load_extended_telemetry_bag(bag_path, topic, msg_type):
    reader = SequentialReader()

    storage_options = StorageOptions(
        uri=bag_path,
        storage_id="mcap",
    )

    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader.open(storage_options, converter_options)

    # Only deserialize the target topic. Without this the reader pulls every
    # message from every topic (vision, joystick, all 16 robots, ...) off disk
    # just to discard them, which dominates the cost on long bags.
    reader.set_filter(StorageFilter(topics=[topic]))

    msg_cls = get_message(msg_type)

    telem = defaultdict(list)
    def add_msg_to_telem(parent_telem_key, ros_msg):
        for field_name, field_type in ros_msg.get_fields_and_field_types().items():
            if parent_telem_key:
                telem_key = f"{parent_telem_key}/{field_name}"
            else:
                telem_key = field_name
            value = getattr(ros_msg, field_name)
            if hasattr(value, "get_fields_and_field_types"):
                add_msg_to_telem(telem_key, value)
            else:
                telem[telem_key].append(value)

    # ---------- read bag ----------
    start_t = None
    i = 0
    while reader.has_next():
        topic_name, data, ros_t = reader.read_next()

        if topic_name != topic:
            continue

        if start_t is None:
            start_t = ros_t

        ros_t = ros_t - start_t

        msg = deserialize_message(data, msg_cls)

        telem["ros_t"].append(ros_t * 1e-9)  # nanoseconds -> seconds

        add_msg_to_telem("", msg)

        i += 1

    # Special postprocess of timestamp_us fields
    if "timestamp_us_hi" in telem and "timestamp_us_lo" in telem:
        for hi, lo in zip(telem["timestamp_us_hi"], telem["timestamp_us_lo"]):
            timestamp_us = (hi << 32) | lo
            telem["timestamp_us"].append(timestamp_us)

    # ---------- convert to NumPy ----------
    telem_np = {k: np.asarray(v, dtype=np.float32) for k, v in telem.items()}

    return telem_np


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert ROS2 bag to numpy archive")
    parser.add_argument("-b", "--bag", type=str, required=True, help="Path to the ROS2 bag directory")
    parser.add_argument("-r", "--robot", type=int, default=None,
                        help="Robot number (default: inferred from the bag)")
    parser.add_argument("-o", "--output", type=str, default="telemetry.npz", help="Output path for the NPZ file (default: telemetry.npz)")
    args = parser.parse_args()

    robot = args.robot if args.robot is not None else infer_robot(args.bag)
    topic = f"/robot_feedback/extended/robot{robot}"
    print("Loading telemetry from bag...")
    telem = load_extended_telemetry_bag(args.bag, topic, MSG_TYPE)
    print("Frames loaded:", telem["ros_t"].shape[0])
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.output, **telem)
    print(f"Saved to {args.output}")
