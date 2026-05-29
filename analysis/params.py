"""Robot parameter definitions.

This file is the source of truth for mapping the robot_params.json
format to both:
  1. ctypes structs (for visualization computations via the Rust FFI)
  2. firmware parameter IDs (for uploading to the robot via ROS)

JSON keys map to grouped parameter vectors matching the firmware ParameterName
enum. Each PARAM_MAP entry describes one JSON key:
  - param_id:  firmware ParameterName enum value (for robot upload)
  - fields:    list of (array_index, ctypes_struct_name, ctypes_field_name) tuples
               describing how to populate the ctypes structs from the JSON array
"""

import ctypes
import json

from ateam_controls import (
    TrajectoryParams,
    default_kf_params,
    default_phys_params,
    default_traj_params,
    robot_model_new,
)


# ---------------------------------------------------------------------------
# Parameter map: JSON key → (param_id, fields)
#
# param_id: firmware ParameterName enum value (for robot upload)
# fields: list of (array_index, struct_name, field_name) tuples.
# struct_name is one of: "traj", "kf", "phys" (or None if not used in viz).
# ---------------------------------------------------------------------------

PARAM_MAP = {
    "KF_PROCESS_STD": {
        "param_id": 0,
        "fields": [
            (0, "kf", "process_noise_std_pos_linear"),
            (1, "kf", "process_noise_std_pos_angular"),
            (2, "kf", "process_noise_std_vel_linear"),
            (3, "kf", "process_noise_std_vel_angular"),
        ],
    },
    "KF_MEASUREMENT_STD": {
        "param_id": 1,
        "fields": [
            (0, "kf", "measurement_noise_std_vision_pos_linear"),
            (1, "kf", "measurement_noise_std_vision_pos_angular"),
            (2, "kf", "measurement_noise_std_encoder_vel_angular"),
            (3, "kf", "measurement_noise_std_gyro_vel_angular"),
        ],
    },
    "KF_MAX_STATE": {
        "param_id": 2,
        "fields": [
            (0, "kf", "max_pos_linear"),
            (1, "kf", "max_pos_angular"),
            (2, "kf", "max_vel_linear"),
            (3, "kf", "max_vel_angular"),
        ],
    },
    "PHYS_WHEEL": {
        "param_id": 3,
        "fields": [
            (0, "phys", "alpha"),
            (1, "phys", "beta"),
            (2, "phys", "l"),
            (3, "phys", "r"),
        ],
    },
    "PHYS_INERTIA": {
        "param_id": 4,
        "fields": [
            (0, "phys", "mass"),
            (1, "phys", "iz"),
        ],
    },
    "PHYS_MOTOR_MODEL": {
        "param_id": 5,
        "fields": [
            (0, "phys", "motor_torque_constant"),
            (1, "phys", "motor_efficiency_factor"),
        ],
    },
    # Parameters used only for upload (no ctypes struct mapping):
    "PHYS_FRICTION_MODEL": {
        "param_id": 6,
        "fields": [],
    },
    "COULOMB_COMP_ACCEL_DEADZONE": {
        "param_id": 7,
        "fields": [],
    },
    "POSE_CONTROL_GAIN": {
        "param_id": 8,
        "fields": [],
    },
    "TRAJ_RECOMPUTE_ERROR": {
        "param_id": 9,
        "fields": [],
    },
    "POSE_FB_PIDII_LINEAR": {
        "param_id": 10,
        "fields": [],
    },
    "POSE_FB_PIDII_ANGULAR": {
        "param_id": 11,
        "fields": [],
    },
    "TWIST_FB_PIDII_LINEAR": {
        "param_id": 12,
        "fields": [],
    },
    "TWIST_FB_PIDII_ANGULAR": {
        "param_id": 13,
        "fields": [],
    },
}


# ---------------------------------------------------------------------------
# JSON → ctypes struct loading (for visualization)
# ---------------------------------------------------------------------------

def _load_struct(struct, struct_name, data):
    """Populate a ctypes struct from JSON data using PARAM_MAP."""
    for key, info in PARAM_MAP.items():
        if key not in data:
            continue
        for idx, sname, field in info["fields"]:
            if sname == struct_name:
                setattr(struct, field, float(data[key][idx]))
    return struct


def load_traj_params(path: str = None) -> TrajectoryParams:
    """Load TrajectoryParams from a robot_params.json file (or defaults)."""
    traj = default_traj_params()
    if path is not None:
        with open(path) as f:
            _load_struct(traj, "traj", json.load(f))
    return traj


def load_robot_model(path: str = None):
    """Create a RobotModel from a robot_params.json file (or defaults).

    Returns an opaque ctypes.c_void_p handle. The caller must call
    robot_model_free() when done.
    """
    kf = default_kf_params()
    phys = default_phys_params()
    if path is not None:
        with open(path) as f:
            data = json.load(f)
        _load_struct(kf, "kf", data)
        _load_struct(phys, "phys", data)
    return robot_model_new(ctypes.c_float(0.001), kf, phys)
