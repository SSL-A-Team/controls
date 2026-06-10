# =============================================================================
# AUTO-GENERATED from ateam_controls.h — DO NOT EDIT
#
# This file is a thin ctypes wrapper around the public C API of ateam_controls.
#
# Source: ateam-controls-c/include/ateam_controls/ateam_controls.h
# =============================================================================

import ctypes
from pathlib import Path

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

CONTROLS_REPO_PATH = next(
    p for p in Path(__file__).resolve().parents
    if p.name == "controls" and (p / "Cargo.toml").exists()
)
_LIB_PATH = CONTROLS_REPO_PATH / "target" / "release" / "libateam_controls_c.so"

# ---------------------------------------------------------------------------
# Error codes
# ---------------------------------------------------------------------------

ATEAM_CONTROLS_OK                        =   0
ATEAM_CONTROLS_INVALID_INPUT             =  -1
ATEAM_CONTROLS_SINGULAR                  =  -2
ATEAM_CONTROLS_NO_SOLUTION               =  -3
ATEAM_CONTROLS_INVALID_TIME              =  -4
ATEAM_CONTROLS_EXCEEDS_LIMIT             =  -5

_ERROR_NAMES = {
     -1: "INVALID_INPUT",
     -2: "SINGULAR",
     -3: "NO_SOLUTION",
     -4: "INVALID_TIME",
     -5: "EXCEEDS_LIMIT",
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
# ctypes struct definitions
# ---------------------------------------------------------------------------

class Vector3C(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
    ]

class Vector4C(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("w", ctypes.c_float),
    ]

class Vector6C(ctypes.Structure):
    _fields_ = [
        ("data", ctypes.c_float * 6),
    ]

class Vector8C(ctypes.Structure):
    _fields_ = [
        ("data", ctypes.c_float * 8),
    ]

class Matrix3C(ctypes.Structure):
    _fields_ = [
        ("data", ctypes.c_float * 9),
    ]

class Matrix3x4C(ctypes.Structure):
    _fields_ = [
        ("data", ctypes.c_float * 12),
    ]

class Matrix4x3C(ctypes.Structure):
    _fields_ = [
        ("data", ctypes.c_float * 12),
    ]

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
        ("coulomb_friction_coefficient_linear_x", ctypes.c_float),
        ("coulomb_friction_coefficient_linear_y", ctypes.c_float),
        ("coulomb_friction_coefficient_angular", ctypes.c_float),
        ("viscous_friction_coefficient_linear_x", ctypes.c_float),
        ("viscous_friction_coefficient_linear_y", ctypes.c_float),
        ("viscous_friction_coefficient_angular", ctypes.c_float),
    ]

class TrajectoryParams(ctypes.Structure):
    _fields_ = [
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
# Library loading
# ---------------------------------------------------------------------------

_lib_handle = None

def _lib():
    """Load the shared library, caching the handle."""
    global _lib_handle
    if _lib_handle is None:
        if not _LIB_PATH.exists():
            raise FileNotFoundError(
                f"Shared library not found at {_LIB_PATH}"
            )
        _lib_handle = ctypes.CDLL(str(_LIB_PATH))
        _setup_signatures(_lib_handle)
    return _lib_handle


def _setup_signatures(lib):
    """Declare argument/return types for every FFI function."""
    # Default parameter constructors
    lib.ateam_controls_default_kf_params.argtypes = []
    lib.ateam_controls_default_kf_params.restype = KalmanFilterParams
    lib.ateam_controls_default_phys_params.argtypes = []
    lib.ateam_controls_default_phys_params.restype = RobotPhysicalParams
    lib.ateam_controls_default_traj_params.argtypes = []
    lib.ateam_controls_default_traj_params.restype = TrajectoryParams
    # RobotModel
    lib.ateam_controls_robot_model_new.argtypes = [ctypes.c_float, KalmanFilterParams, RobotPhysicalParams, ctypes.POINTER(ctypes.c_void_p)]
    lib.ateam_controls_robot_model_new.restype = ctypes.c_int32
    lib.ateam_controls_robot_model_free.argtypes = [ctypes.c_void_p]
    lib.ateam_controls_robot_model_free.restype = None
    lib.ateam_controls_robot_model_update_kf_params.argtypes = [ctypes.c_void_p, KalmanFilterParams]
    lib.ateam_controls_robot_model_update_kf_params.restype = None
    lib.ateam_controls_robot_model_update_physical_params.argtypes = [ctypes.c_void_p, RobotPhysicalParams]
    lib.ateam_controls_robot_model_update_physical_params.restype = ctypes.c_int32
    lib.ateam_controls_robot_model_kf_predict.argtypes = [ctypes.c_void_p, Vector3C]
    lib.ateam_controls_robot_model_kf_predict.restype = None
    lib.ateam_controls_robot_model_kf_update.argtypes = [ctypes.c_void_p, Vector8C, ctypes.c_bool, ctypes.c_bool, ctypes.c_bool]
    lib.ateam_controls_robot_model_kf_update.restype = ctypes.c_int32
    lib.ateam_controls_robot_model_get_state.argtypes = [ctypes.c_void_p]
    lib.ateam_controls_robot_model_get_state.restype = Vector6C
    lib.ateam_controls_robot_model_set_state.argtypes = [ctypes.c_void_p, Vector6C]
    lib.ateam_controls_robot_model_set_state.restype = None
    lib.ateam_controls_robot_model_transform_wheel2twist.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.ateam_controls_robot_model_transform_wheel2twist.restype = Matrix3x4C
    lib.ateam_controls_robot_model_transform_twist2wheel.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.ateam_controls_robot_model_transform_twist2wheel.restype = Matrix4x3C
    lib.ateam_controls_robot_model_transform_wheel2accel.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.ateam_controls_robot_model_transform_wheel2accel.restype = Matrix3x4C
    lib.ateam_controls_robot_model_transform_accel2wheel.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.ateam_controls_robot_model_transform_accel2wheel.restype = Matrix4x3C
    lib.ateam_controls_robot_model_torques_to_currents.argtypes = [ctypes.c_void_p, Vector4C]
    lib.ateam_controls_robot_model_torques_to_currents.restype = Vector4C
    lib.ateam_controls_traj_from_target_pose.argtypes = [Vector6C, Vector3C, TrajectoryParams, ctypes.POINTER(BangBangTraj3D)]
    lib.ateam_controls_traj_from_target_pose.restype = ctypes.c_int32
    lib.ateam_controls_traj_from_target_twist.argtypes = [Vector3C, Vector3C, TrajectoryParams, ctypes.POINTER(BangBangTraj3D)]
    lib.ateam_controls_traj_from_target_twist.restype = ctypes.c_int32
    lib.ateam_controls_traj_1d_time_shift.argtypes = [ctypes.POINTER(BangBangTraj1D), ctypes.c_float]
    lib.ateam_controls_traj_1d_time_shift.restype = None
    lib.ateam_controls_traj_3d_time_shift.argtypes = [ctypes.POINTER(BangBangTraj3D), ctypes.c_float]
    lib.ateam_controls_traj_3d_time_shift.restype = None
    lib.ateam_controls_traj_end_time.argtypes = [BangBangTraj3D]
    lib.ateam_controls_traj_end_time.restype = ctypes.c_float
    lib.ateam_controls_traj_state_at.argtypes = [BangBangTraj3D, Vector6C, ctypes.c_float, ctypes.c_float, ctypes.POINTER(Vector6C)]
    lib.ateam_controls_traj_state_at.restype = ctypes.c_int32
    lib.ateam_controls_traj_accel_at.argtypes = [BangBangTraj3D, ctypes.c_float, ctypes.POINTER(Vector3C)]
    lib.ateam_controls_traj_accel_at.restype = ctypes.c_int32


# ---------------------------------------------------------------------------
# Wrapper functions (1:1 mapping of the C API)
# ---------------------------------------------------------------------------

def default_kf_params():
    return _lib().ateam_controls_default_kf_params()

def default_phys_params():
    return _lib().ateam_controls_default_phys_params()

def default_traj_params():
    return _lib().ateam_controls_default_traj_params()

def robot_model_new(kf_dt, kf_params, phys_params):
    _out = ctypes.c_void_p()
    _rc = _lib().ateam_controls_robot_model_new(kf_dt, kf_params, phys_params, ctypes.byref(_out))
    _check_rc(_rc, 'ateam_controls_robot_model_new')
    return _out

def robot_model_free(model):
    _lib().ateam_controls_robot_model_free(model)

def robot_model_update_kf_params(model, kf_params):
    _lib().ateam_controls_robot_model_update_kf_params(model, kf_params)

def robot_model_update_physical_params(model, phys_params):
    _rc = _lib().ateam_controls_robot_model_update_physical_params(model, phys_params)
    _check_rc(_rc, 'ateam_controls_robot_model_update_physical_params')

def robot_model_kf_predict(model, accel):
    _lib().ateam_controls_robot_model_kf_predict(model, accel)

def robot_model_kf_update(model, measurement, mask_vision, mask_encoder, mask_gyro):
    _rc = _lib().ateam_controls_robot_model_kf_update(model, measurement, mask_vision, mask_encoder, mask_gyro)
    _check_rc(_rc, 'ateam_controls_robot_model_kf_update')

def robot_model_get_state(model):
    return _lib().ateam_controls_robot_model_get_state(model)

def robot_model_set_state(model, state):
    _lib().ateam_controls_robot_model_set_state(model, state)

def robot_model_transform_wheel2twist(model, theta):
    return _lib().ateam_controls_robot_model_transform_wheel2twist(model, theta)

def robot_model_transform_twist2wheel(model, theta):
    return _lib().ateam_controls_robot_model_transform_twist2wheel(model, theta)

def robot_model_transform_wheel2accel(model, theta):
    return _lib().ateam_controls_robot_model_transform_wheel2accel(model, theta)

def robot_model_transform_accel2wheel(model, theta):
    return _lib().ateam_controls_robot_model_transform_accel2wheel(model, theta)

def robot_model_torques_to_currents(model, torques):
    return _lib().ateam_controls_robot_model_torques_to_currents(model, torques)

def traj_from_target_pose(init_state, target_pose, params):
    _out = BangBangTraj3D()
    _rc = _lib().ateam_controls_traj_from_target_pose(init_state, target_pose, params, ctypes.byref(_out))
    _check_rc(_rc, 'ateam_controls_traj_from_target_pose')
    return _out

def traj_from_target_twist(init_twist, target_twist, params):
    _out = BangBangTraj3D()
    _rc = _lib().ateam_controls_traj_from_target_twist(init_twist, target_twist, params, ctypes.byref(_out))
    _check_rc(_rc, 'ateam_controls_traj_from_target_twist')
    return _out

def traj_1d_time_shift(traj, dt):
    _lib().ateam_controls_traj_1d_time_shift(traj, dt)

def traj_3d_time_shift(traj, dt):
    _lib().ateam_controls_traj_3d_time_shift(traj, dt)

def traj_end_time(traj):
    return _lib().ateam_controls_traj_end_time(traj)

def traj_state_at(traj, current_state, current_time, t):
    _out = Vector6C()
    _rc = _lib().ateam_controls_traj_state_at(traj, current_state, current_time, t, ctypes.byref(_out))
    _check_rc(_rc, 'ateam_controls_traj_state_at')
    return _out

def traj_accel_at(traj, t):
    _out = Vector3C()
    _rc = _lib().ateam_controls_traj_accel_at(traj, t, ctypes.byref(_out))
    _check_rc(_rc, 'ateam_controls_traj_accel_at')
    return _out
