"""
Python interface to Rust controls library via ctypes FFI.

Loads the shared library (libateam_controls_c.so) built from the
ateam-controls-c crate and exposes trajectory and state-measurement
functions without subprocess overhead.
"""

from pathlib import Path
import ctypes
import json
import subprocess
import numpy as np
import pandas as pd


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

CONTROLS_REPO_PATH = next(p for p in Path(__file__).resolve().parents if p.name == "controls")
_LIB_PATH = CONTROLS_REPO_PATH / "target" / "release" / "libateam_controls_c.so"

# ---------------------------------------------------------------------------
# Error codes (must match ateam_controls.h)
# ---------------------------------------------------------------------------

ATEAM_CONTROLS_OK             =  0
ATEAM_CONTROLS_INVALID_INPUT  = -1
ATEAM_CONTROLS_SINGULAR       = -2
ATEAM_CONTROLS_NO_SOLUTION    = -3
ATEAM_CONTROLS_INVALID_TIME   = -4
ATEAM_CONTROLS_EXCEEDS_LIMIT  = -5

_ERROR_NAMES = {
    ATEAM_CONTROLS_INVALID_INPUT: "INVALID_INPUT",
    ATEAM_CONTROLS_SINGULAR:      "SINGULAR_MATRIX",
    ATEAM_CONTROLS_NO_SOLUTION:   "NO_SOLUTION",
    ATEAM_CONTROLS_INVALID_TIME:  "INVALID_TIME",
    ATEAM_CONTROLS_EXCEEDS_LIMIT: "EXCEEDS_LIMIT",
}


class ControlsError(RuntimeError):
    """Raised when the Rust controls library returns a non-zero error code."""
    def __init__(self, rc: int, context: str = ""):
        name = _ERROR_NAMES.get(rc, f"UNKNOWN({rc})")
        msg = f"ateam_controls error {rc} ({name})"
        if context:
            msg += f" in {context}"
        super().__init__(msg)
        self.rc = rc


def _check_rc(rc: int, context: str = ""):
    """Raise ControlsError if rc != 0."""
    if rc != ATEAM_CONTROLS_OK:
        raise ControlsError(rc, context)

# ---------------------------------------------------------------------------
# ctypes struct definitions (must match ateam_controls.h)
# ---------------------------------------------------------------------------

class Vector3C(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]

class Vector6C(ctypes.Structure):
    _fields_ = [("data", ctypes.c_float * 6)]

class Vector8C(ctypes.Structure):
    _fields_ = [("data", ctypes.c_float * 8)]

class Matrix3x4C(ctypes.Structure):
    _fields_ = [("data", ctypes.c_float * 12)]

class KalmanFilterParams(ctypes.Structure):
    _fields_ = [
        ("process_noise_std_pos_linear", ctypes.c_float),
        ("process_noise_std_pos_angular", ctypes.c_float),
        ("process_noise_std_vel_linear", ctypes.c_float),
        ("process_noise_std_vel_angular", ctypes.c_float),
        ("measurement_noise_std_vision_pos_linear", ctypes.c_float),
        ("measurement_noise_std_vision_pos_angular", ctypes.c_float),
        ("measurement_noise_std_encoder_vel_angular", ctypes.c_float),
        ("measurement_noise_std_gyro_vel_angular", ctypes.c_float),
        ("max_pos_linear", ctypes.c_float),
        ("max_pos_angular", ctypes.c_float),
        ("max_vel_linear", ctypes.c_float),
        ("max_vel_angular", ctypes.c_float),
    ]

class RobotPhysicalParams(ctypes.Structure):
    _fields_ = [
        ("alpha", ctypes.c_float),
        ("beta", ctypes.c_float),
        ("l", ctypes.c_float),
        ("r", ctypes.c_float),
        ("mass", ctypes.c_float),
        ("iz", ctypes.c_float),
        ("motor_torque_constant", ctypes.c_float),
        ("motor_efficiency_factor", ctypes.c_float),
    ]

class TrajectoryParams(ctypes.Structure):
    _fields_ = [
        ("allowable_error_pos_linear", ctypes.c_float),
        ("allowable_error_pos_angular", ctypes.c_float),
        ("allowable_error_vel_linear", ctypes.c_float),
        ("allowable_error_vel_angular", ctypes.c_float),
        ("max_vel_linear", ctypes.c_float),
        ("max_vel_angular", ctypes.c_float),
        ("max_accel_linear", ctypes.c_float),
        ("max_accel_angular", ctypes.c_float),
    ]

class BangBangTraj1D(ctypes.Structure):
    _fields_ = [
        ("sdd1", ctypes.c_float),
        ("sdd2", ctypes.c_float),
        ("sdd3", ctypes.c_float),
        ("t1", ctypes.c_float),
        ("t2", ctypes.c_float),
        ("t3", ctypes.c_float),
        ("t4", ctypes.c_float),
    ]

class BangBangTraj3D(ctypes.Structure):
    _fields_ = [
        ("x", BangBangTraj1D),
        ("y", BangBangTraj1D),
        ("z", BangBangTraj1D),
    ]


# ---------------------------------------------------------------------------
# JSON key → ctypes field mapping (matches Rust serde renames)
#
# The robot_params.json now stores grouped parameter vectors matching the
# firmware ParameterName enum.  Each entry below is:
#   (json_key, array_index, ctypes_field_name)
# ---------------------------------------------------------------------------

_TRAJ_JSON_MAP = [
    # TRAJ_MAX [MAX_VEL_LINEAR, MAX_VEL_ANGULAR, MAX_ACCEL_LINEAR, MAX_ACCEL_ANGULAR]
    ("TRAJ_MAX", 0, "max_vel_linear"),
    ("TRAJ_MAX", 1, "max_vel_angular"),
    ("TRAJ_MAX", 2, "max_accel_linear"),
    ("TRAJ_MAX", 3, "max_accel_angular"),
]

_KF_JSON_MAP = [
    # KF_PROCESS_STD [POS_LINEAR, POS_ANGULAR, VEL_LINEAR, VEL_ANGULAR]
    ("KF_PROCESS_STD", 0, "process_noise_std_pos_linear"),
    ("KF_PROCESS_STD", 1, "process_noise_std_pos_angular"),
    ("KF_PROCESS_STD", 2, "process_noise_std_vel_linear"),
    ("KF_PROCESS_STD", 3, "process_noise_std_vel_angular"),
    # KF_MEASUREMENT_STD [VISION_LINEAR, VISION_ANGULAR, ENCODER_ANGULAR, GYRO_ANGULAR]
    ("KF_MEASUREMENT_STD", 0, "measurement_noise_std_vision_pos_linear"),
    ("KF_MEASUREMENT_STD", 1, "measurement_noise_std_vision_pos_angular"),
    ("KF_MEASUREMENT_STD", 2, "measurement_noise_std_encoder_vel_angular"),
    ("KF_MEASUREMENT_STD", 3, "measurement_noise_std_gyro_vel_angular"),
    # KF_MAX_STATE [POS_LINEAR, POS_ANGULAR, VEL_LINEAR, VEL_ANGULAR]
    ("KF_MAX_STATE", 0, "max_pos_linear"),
    ("KF_MAX_STATE", 1, "max_pos_angular"),
    ("KF_MAX_STATE", 2, "max_vel_linear"),
    ("KF_MAX_STATE", 3, "max_vel_angular"),
]

_PHYS_JSON_MAP = [
    # PHYS_WHEEL [ALPHA, BETA, DISTANCE, RADIUS]
    ("PHYS_WHEEL", 0, "alpha"),
    ("PHYS_WHEEL", 1, "beta"),
    ("PHYS_WHEEL", 2, "l"),
    ("PHYS_WHEEL", 3, "r"),
    # PHYS_INERTIA [BODY_MASS, BODY_MOMENT_Z]
    ("PHYS_INERTIA", 0, "mass"),
    ("PHYS_INERTIA", 1, "iz"),
    # PHYS_MOTOR_MODEL [TORQUE_CONSTANT, EFFICIENCY_FACTOR]
    ("PHYS_MOTOR_MODEL", 0, "motor_torque_constant"),
    ("PHYS_MOTOR_MODEL", 1, "motor_efficiency_factor"),
]


# ---------------------------------------------------------------------------
# Library loading & function signatures
# ---------------------------------------------------------------------------

_lib = None

def _get_lib():
    """Load the shared library, caching the handle."""
    global _lib
    if _lib is None:
        if not _LIB_PATH.exists():
            raise FileNotFoundError(
                f"Shared library not found at {_LIB_PATH}. "
                "Run compile_controls() first."
            )
        _lib = ctypes.CDLL(str(_LIB_PATH))
        _setup_signatures(_lib)
    return _lib


def _setup_signatures(lib):
    """Declare argument/return types for every FFI function we use."""
    # Trajectory
    lib.ateam_controls_traj_params_default.argtypes = []
    lib.ateam_controls_traj_params_default.restype = TrajectoryParams

    lib.ateam_controls_traj_from_target_pose.argtypes = [
        Vector6C, Vector3C, TrajectoryParams, ctypes.POINTER(BangBangTraj3D)]
    lib.ateam_controls_traj_from_target_pose.restype = ctypes.c_int32

    lib.ateam_controls_traj_end_time.argtypes = [BangBangTraj3D]
    lib.ateam_controls_traj_end_time.restype = ctypes.c_float

    lib.ateam_controls_traj_state_at.argtypes = [
        BangBangTraj3D, Vector6C, ctypes.c_float, ctypes.c_float,
        ctypes.POINTER(Vector6C)]
    lib.ateam_controls_traj_state_at.restype = ctypes.c_int32

    lib.ateam_controls_traj_accel_at.argtypes = [
        BangBangTraj3D, ctypes.c_float, ctypes.POINTER(Vector3C)]
    lib.ateam_controls_traj_accel_at.restype = ctypes.c_int32

    # RobotModel
    lib.ateam_controls_robot_model_new.argtypes = [
        ctypes.c_float, KalmanFilterParams, RobotPhysicalParams,
        ctypes.POINTER(ctypes.c_void_p)]
    lib.ateam_controls_robot_model_new.restype = ctypes.c_int32

    lib.ateam_controls_robot_model_new_default.argtypes = [
        ctypes.c_float, ctypes.POINTER(ctypes.c_void_p)]
    lib.ateam_controls_robot_model_new_default.restype = ctypes.c_int32

    lib.ateam_controls_robot_model_free.argtypes = [ctypes.c_void_p]
    lib.ateam_controls_robot_model_free.restype = None

    lib.ateam_controls_robot_model_transform_wheel2twist.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.ateam_controls_robot_model_transform_wheel2twist.restype = Matrix3x4C


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_traj_params_from_json(params: TrajectoryParams, path: str) -> TrajectoryParams:
    """Load TrajectoryParams from a JSON file using the grouped parameter vectors."""
    with open(path) as f:
        data = json.load(f)
    for json_key, idx, field in _TRAJ_JSON_MAP:
        if json_key in data:
            setattr(params, field, float(data[json_key][idx]))
    return params


def _load_robot_model_from_json(lib, path: str):
    """Create a RobotModel from a JSON file using the grouped parameter vectors."""
    with open(path) as f:
        data = json.load(f)
    kf = KalmanFilterParams()
    for json_key, idx, field in _KF_JSON_MAP:
        if json_key in data:
            setattr(kf, field, float(data[json_key][idx]))
    phys = RobotPhysicalParams()
    for json_key, idx, field in _PHYS_JSON_MAP:
        if json_key in data:
            setattr(phys, field, float(data[json_key][idx]))
    model = ctypes.c_void_p()
    rc = lib.ateam_controls_robot_model_new(
        ctypes.c_float(0.001), kf, phys, ctypes.byref(model))
    _check_rc(rc, "robot_model_new")
    return model


# ---------------------------------------------------------------------------
# Compilation helpers
# ---------------------------------------------------------------------------

def compile_controls():
    """Build the shared library (and binaries) in release mode."""
    build_cmd = ["cargo", "build", "--release", "--workspace"]
    subprocess.run(build_cmd, cwd=CONTROLS_REPO_PATH, check=True)
    print("Built controls workspace (release).")


# ---------------------------------------------------------------------------
# Runtime wrappers
# ---------------------------------------------------------------------------

def compute_trajectory(init_state: list, target_pose: list, param_json: str = None,
                       t_start: float = None, t_end: float = None) -> pd.DataFrame:
    """Compute an optimal bang-bang trajectory via ctypes FFI.

    :param init_state: [x, y, theta, xd, yd, thetad]
    :type init_state: list
    :param target_pose: [x, y, theta]
    :type target_pose: list
    :param param_json: Optional path to a JSON file containing trajectory parameters
    :type param_json: str or None
    :param t_start: Optional start of the sampling window (trajectory-local time).
        Defaults to 0.
    :type t_start: float or None
    :param t_end: Optional end of the sampling window (trajectory-local time).
        Clamped to the trajectory end time.
    :type t_end: float or None
    :return: DataFrame with columns: time, x, y, theta, xd, yd, thetad, xdd, ydd, thetadd
    :rtype: pd.DataFrame
    """
    if len(init_state) != 6:
        raise ValueError("init_state must have 6 elements: [x, y, theta, xd, yd, thetad]")
    if len(target_pose) != 3:
        raise ValueError("target_pose must have 3 elements: [x, y, theta]")

    lib = _get_lib()

    # Build ctypes structs
    state_c = Vector6C(data=(ctypes.c_float * 6)(*[float(v) for v in init_state]))
    target_c = Vector3C(x=float(target_pose[0]), y=float(target_pose[1]), z=float(target_pose[2]))

    params = lib.ateam_controls_traj_params_default()
    if param_json is not None:
        params = _load_traj_params_from_json(params, param_json)

    # Compute trajectory
    traj = BangBangTraj3D()
    rc = lib.ateam_controls_traj_from_target_pose(
        state_c, target_c, params, ctypes.byref(traj))
    _check_rc(rc, "traj_from_target_pose")
    end_time = lib.ateam_controls_traj_end_time(traj)

    # Clamp sampling window
    sample_start = max(0.0, t_start) if t_start is not None else 0.0
    sample_end = min(end_time, t_end) if t_end is not None else end_time
    if sample_start > sample_end:
        columns = ["time", "x", "y", "theta", "xd", "yd", "thetad", "xdd", "ydd", "thetadd"]
        return pd.DataFrame(columns=columns)

    # Sample at 1 ms resolution
    time_resolution = 0.001
    times = np.arange(sample_start, sample_end + time_resolution * 0.5, time_resolution)

    n_steps = len(times)
    rows = np.empty((n_steps, 10), dtype=np.float32)
    st = Vector6C()
    ac = Vector3C()
    for i, t in enumerate(times):
        rc = lib.ateam_controls_traj_state_at(
            traj, state_c, ctypes.c_float(0.0), ctypes.c_float(t),
            ctypes.byref(st))
        _check_rc(rc, "traj_state_at")
        rc = lib.ateam_controls_traj_accel_at(
            traj, ctypes.c_float(t), ctypes.byref(ac))
        _check_rc(rc, "traj_accel_at")
        rows[i] = [t, st.data[0], st.data[1], st.data[2],
                      st.data[3], st.data[4], st.data[5],
                      ac.x, ac.y, ac.z]

    columns = ["time", "x", "y", "theta", "xd", "yd", "thetad", "xdd", "ydd", "thetadd"]
    return pd.DataFrame(rows, columns=columns)


def compute_state_measurement(state: list, sensor_readings: list, param_json: str = None) -> pd.DataFrame:
    """Compute state-space measurements from raw sensor readings via ctypes FFI.

    :param state: [x, y, theta, xd, yd, thetad]
    :type state: list
    :param sensor_readings: [vision_x, vision_y, vision_theta, encoder_vfl, encoder_vbl, encoder_vbr, encoder_vfr, gyro_thetad]
    :type sensor_readings: list
    :param param_json: Optional path to a JSON file containing RobotModel parameters
    :type param_json: str or None
    :return: DataFrame with columns: vision_x_meas, vision_y_meas, vision_theta_meas, encoder_xd_meas, encoder_yd_meas, encoder_thetad_meas, gyro_thetad_meas
    :rtype: pd.DataFrame
    """
    if len(state) != 6:
        raise ValueError("state must have 6 elements: [x, y, theta, xd, yd, thetad]")
    if len(sensor_readings) != 8:
        raise ValueError(
            "sensor_readings must have 8 elements: "
            "[vision_x, vision_y, vision_theta, encoder_vfl, encoder_vbl, encoder_vbr, encoder_vfr, gyro_thetad]"
        )

    lib = _get_lib()

    theta = float(state[2])

    # Create RobotModel
    if param_json is not None:
        model = _load_robot_model_from_json(lib, param_json)
    else:
        model = ctypes.c_void_p()
        rc = lib.ateam_controls_robot_model_new_default(
            ctypes.c_float(0.001), ctypes.byref(model))
        _check_rc(rc, "robot_model_new_default")

    try:
        # Get wheel2twist transform (3x4 matrix, column-major)
        mat_c = lib.ateam_controls_robot_model_transform_wheel2twist(model, ctypes.c_float(theta))
        mat = np.array(list(mat_c.data), dtype=np.float32).reshape((3, 4), order='F')

        # Encoder readings (4 wheel velocities)
        encoders = np.array(sensor_readings[3:7], dtype=np.float32)

        # Multiply: twist = mat @ encoders  (3x4 @ 4x1 = 3x1)
        twist = mat @ encoders

        # Assemble 7-element measurement vector
        meas = np.array([
            sensor_readings[0],   # vision_x
            sensor_readings[1],   # vision_y
            sensor_readings[2],   # vision_theta
            twist[0],             # encoder_xd
            twist[1],             # encoder_yd
            twist[2],             # encoder_thetad
            sensor_readings[7],   # gyro_thetad
        ], dtype=np.float64)
    finally:
        lib.ateam_controls_robot_model_free(model)

    columns = [
        "vision_x_meas", "vision_y_meas", "vision_theta_meas",
        "encoder_xd_meas", "encoder_yd_meas", "encoder_thetad_meas",
        "gyro_thetad_meas",
    ]
    return pd.DataFrame([meas], columns=columns)
