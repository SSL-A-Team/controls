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
    KalmanFilterParams,
    RobotPhysicalParams,
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
    "PHYS_FRICTION_MODEL": {
        "param_id": 6,
        "fields": [
            (0, "phys", "coulomb_friction_coefficient_linear_x"),
            (1, "phys", "coulomb_friction_coefficient_linear_y"),
            (2, "phys", "coulomb_friction_coefficient_angular"),
            (3, "phys", "viscous_friction_coefficient_linear_x"),
            (4, "phys", "viscous_friction_coefficient_linear_y"),
            (5, "phys", "viscous_friction_coefficient_angular"),
        ],
    },
    # Parameters used only for upload (no ctypes struct mapping):
    "FRICTION_COMP_GATING": {
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


# ---------------------------------------------------------------------------
# ctypes struct → JSON dict (reverse of _load_struct)
# ---------------------------------------------------------------------------

def _struct_to_dict(struct, struct_name: str) -> dict:
    """Walk PARAM_MAP and emit a JSON dict for one struct's contributions.

    Only PARAM_MAP entries whose ``fields`` reference ``struct_name`` are
    populated. Each emitted value is a list of floats, indexed by
    ``array_index`` from PARAM_MAP.
    """
    out: dict[str, list[float]] = {}
    for key, info in PARAM_MAP.items():
        contributors = [(idx, field) for idx, sname, field in info["fields"]
                        if sname == struct_name]
        if not contributors:
            continue
        size = max(idx for idx, _ in contributors) + 1
        arr = [0.0] * size
        for idx, field in contributors:
            arr[idx] = float(getattr(struct, field))
        out[key] = arr
    return out


def phys_params_to_json_dict(phys: RobotPhysicalParams) -> dict:
    """Return a JSON-compatible dict of all PARAM_MAP entries sourced from phys."""
    return _struct_to_dict(phys, "phys")


def kf_params_to_json_dict(kf: KalmanFilterParams) -> dict:
    """Return a JSON-compatible dict of all PARAM_MAP entries sourced from kf."""
    return _struct_to_dict(kf, "kf")


def default_params_dict() -> dict:
    """Full parameter dict built from the Rust library defaults.

    Includes everything PARAM_MAP knows how to materialize from
    KalmanFilterParams and RobotPhysicalParams. Entries that have no
    ctypes-field mapping (e.g. controller gains) are omitted; callers
    should merge them in from a baseline JSON if needed.
    """
    return {
        **kf_params_to_json_dict(default_kf_params()),
        **phys_params_to_json_dict(default_phys_params()),
    }


def merge_params_dict(base: dict, override: dict) -> dict:
    """Return a shallow merge: override entries replace base entries by key."""
    merged = dict(base)
    merged.update(override)
    return merged


# ---------------------------------------------------------------------------
# Single-request firmware param push (shared by upload_params and the
# accel-tuning toolkit).
# ---------------------------------------------------------------------------

def _send_set_firmware_param(node, client, robot_id: int, param_id: int,
                              data: list, retries: int = 3,
                              timeout_sec: float = 5.0, logger=print) -> bool:
    """Call /set_firmware_param once with retries; returns True on success.

    ``node`` and ``client`` must already be created/initialised. ``logger``
    receives status strings (defaults to ``print`` to match the legacy
    upload_params.py behaviour; pass a Node.get_logger().info-style callable
    when used from inside another node).
    """
    import rclpy
    # Lazy import so this module stays importable in non-ROS contexts.
    from ateam_msgs.srv import SetFirmwareParameter

    req = SetFirmwareParameter.Request()
    req.robot_id = robot_id
    req.parameter_id = param_id
    req.data = [float(v) for v in data]

    for attempt in range(retries):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if future.result() is None:
            logger(f"  WARNING: param_id={param_id} — service call timed out")
            return False
        resp = future.result()
        if resp.success:
            logger(f"  param_id={param_id}  data={data}  OK")
            return True
        logger(f"  WARNING: param_id={param_id} — {resp.reason} "
               f"(attempt {attempt + 1}/{retries})")
    logger(f"  ERROR: param_id={param_id} — failed after {retries} attempts")
    return False


def upload_params_dict(node, client, robot_id: int, params: dict,
                       only_keys=None, logger=print) -> bool:
    """Upload PARAM_MAP entries from an in-memory dict via /set_firmware_param.

    Args:
        node: live rclpy Node to drive service-call spinning.
        client: pre-created Service client for SetFirmwareParameter.
        robot_id: target robot ID.
        params: dict matching the robot_params.json schema (PARAM_MAP keys).
        only_keys: optional iterable of PARAM_MAP keys to restrict the upload
            to. If None, every PARAM_MAP entry present in ``params`` with a
            non-None ``param_id`` is uploaded.
        logger: status-message callable (default ``print``).

    Returns True iff every attempted upload succeeded.
    """
    only = set(only_keys) if only_keys is not None else None
    all_ok = True
    for name, info in PARAM_MAP.items():
        if only is not None and name not in only:
            continue
        param_id = info["param_id"]
        if param_id is None:
            continue
        if name not in params:
            continue
        value = params[name]
        data = [float(v) for v in value] if isinstance(value, list) else [float(value)]
        if not _send_set_firmware_param(node, client, robot_id, param_id, data,
                                         logger=logger):
            all_ok = False
    return all_ok
