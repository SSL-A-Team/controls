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

    # ---------- storage ----------
    telem = {
        "t": [],
        "imu_gyro": [],
        "imu_accel": [],
        "wheel_velocity_fl": [],
        "wheel_velocity_bl": [],
        "wheel_velocity_br": [],
        "wheel_velocity_fr": [],
        "body_cmd": [],
        "kf_body_pose_estimate": [],
        "kf_body_twist_estimate": [],
        "body_twist_u": [],
        "body_accel_u": [],
        "wheel_velocity_u": [],
        "wheel_torque_u": [],
    }

    # ---------- read bag ----------
    while reader.has_next():
        topic_name, data, t = reader.read_next()

        if topic_name != topic:
            continue

        msg = deserialize_message(data, msg_cls)

        telem["t"].append(t * 1e-9)  # nanoseconds → seconds

        telem["imu_gyro"].append(msg.imu_gyro)
        telem["imu_accel"].append(msg.imu_accel)

        telem["wheel_velocity_fl"].append(msg.front_left_motor.vel_enc_estimate)
        telem["wheel_velocity_bl"].append(msg.back_left_motor.vel_enc_estimate)
        telem["wheel_velocity_br"].append(msg.back_right_motor.vel_enc_estimate)
        telem["wheel_velocity_fr"].append(msg.front_right_motor.vel_enc_estimate)

        telem["body_cmd"].append(msg.body_cmd)
        telem["kf_body_pose_estimate"].append(msg.kf_body_pose_estimate)
        telem["kf_body_twist_estimate"].append(msg.kf_body_twist_estimate)
        telem["body_twist_u"].append(msg.body_twist_u)
        telem["body_accel_u"].append(msg.body_accel_u)

        telem["wheel_velocity_u"].append(msg.wheel_velocity_u)
        telem["wheel_torque_u"].append(msg.wheel_torque_u)


    # ---------- convert to NumPy ----------
    telem_np = {}
    telem_np["t"] = np.asarray(telem["t"], dtype=np.float64)
    telem_np["imu_gyro"] = np.asarray(telem["imu_gyro"], dtype=np.float32)
    telem_np["imu_accel"] = np.asarray(telem["imu_accel"], dtype=np.float32)
    telem_np["wheel_velocity_fl"] = np.asarray(telem["wheel_velocity_fl"], dtype=np.float32)
    telem_np["wheel_velocity_bl"] = np.asarray(telem["wheel_velocity_bl"], dtype=np.float32)
    telem_np["wheel_velocity_br"] = np.asarray(telem["wheel_velocity_br"], dtype=np.float32)
    telem_np["wheel_velocity_fr"] = np.asarray(telem["wheel_velocity_fr"], dtype=np.float32)
    telem_np["body_cmd"] = np.asarray(telem["body_cmd"], dtype=np.float32)
    telem_np["kf_body_pose_estimate"] = np.asarray(telem["kf_body_pose_estimate"], dtype=np.float32)
    telem_np["kf_body_twist_estimate"] = np.asarray(telem["kf_body_twist_estimate"], dtype=np.float32)
    telem_np["body_twist_u"] = np.asarray(telem["body_twist_u"], dtype=np.float32)
    telem_np["body_accel_u"] = np.asarray(telem["body_accel_u"], dtype=np.float32)
    telem_np["wheel_velocity_u"] = np.asarray(telem["wheel_velocity_u"], dtype=np.float32)
    telem_np["wheel_torque_u"] = np.asarray(telem["wheel_torque_u"], dtype=np.float32)

    return telem_np


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert ROS2 bag to numpy archive")
    parser.add_argument("--bag", type=str, required=True, help="Path to the ROS2 bag directory")
    parser.add_argument("--robot", type=int, default=2, help="Robot number (default: 2)")
    parser.add_argument("--output", type=str, default="telemetry.npz", help="Output path for the NPZ file (default: telemetry.npz)")
    args = parser.parse_args()
    
    topic = f"/robot_feedback/extended/robot{args.robot}"
    telem = load_extended_telemetry_bag(args.bag, topic, MSG_TYPE)
    print("Frames loaded:", telem["t"].shape[0])
    np.savez(args.output, **telem)
    print(f"Saved to {args.output}")
