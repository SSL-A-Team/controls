import argparse
import numpy as np
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


CONTROLS_REPO_PATH = next(p for p in Path(__file__).resolve().parents if p.name == "controls")
MSG_TYPE = "ateam_radio_msgs/msg/ExtendedTelemetry"


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

    msg_cls = get_message(msg_type)

    telem = {}
    def add_msg_to_telem(parent_telem_key, ros_msg):
        for field_name, field_type in ros_msg.get_fields_and_field_types().items():
            if parent_telem_key:
                telem_key = f"{parent_telem_key}__{field_name}"
            else:
                telem_key = field_name
            value = getattr(ros_msg, field_name)
            if hasattr(value, "get_fields_and_field_types"):
                add_msg_to_telem(telem_key, value)
            else:
                telem[telem_key] = telem.get(telem_key, []) + [value]

    # ---------- read bag ----------
    start_t = None
    i = 0
    while reader.has_next():
        topic_name, data, ros_t = reader.read_next()

        if start_t is None:
            start_t = ros_t

        ros_t = ros_t - start_t

        if topic_name != topic:
            continue

        msg = deserialize_message(data, msg_cls)

        telem["ros_t"] = telem.get("ros_t", []) + [ros_t * 1e-9]  # nanoseconds -> seconds
        
        add_msg_to_telem("", msg)

        i += 1

    # Special postprocess of timestamp_us fields
    if "timestamp_us_hi" in telem and "timestamp_us_lo" in telem:
        for hi, lo in zip(telem["timestamp_us_hi"], telem["timestamp_us_lo"]):
            timestamp_us = (hi << 32) | lo
            telem["timestamp_us"] = telem.get("timestamp_us", []) + [timestamp_us]

    # ---------- convert to NumPy ----------
    telem_np = {k: np.asarray(v, dtype=np.float32) for k, v in telem.items()}

    return telem_np


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert ROS2 bag to numpy archive")
    parser.add_argument("--bag", type=str, required=True, help="Path to the ROS2 bag directory")
    parser.add_argument("--robot", type=int, default=2, help="Robot number (default: 2)")
    parser.add_argument("--output", type=str, default="telemetry.npz", help="Output path for the NPZ file (default: telemetry.npz)")
    args = parser.parse_args()
    
    topic = f"/robot_feedback/extended/robot{args.robot}"
    telem = load_extended_telemetry_bag(args.bag, topic, MSG_TYPE)
    print("Frames loaded:", telem["ros_t"].shape[0])
    np.savez(args.output, **telem)
    print(f"Saved to {args.output}")
