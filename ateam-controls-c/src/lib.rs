use ateam_controls;
use ateam_controls::ctypes::*;
use ateam_controls::bangbang_trajectory::*;
use ateam_controls::robot_model::*;

// ============================================================================
// Utility
// ============================================================================

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_add(left: u64, right: u64) -> u64 {
    ateam_controls::add(left, right)
}

// ============================================================================
// RobotModel (opaque pointer)
// ============================================================================

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_robot_model_new(
    kf_dt: f32,
    kf_params: KalmanFilterParams,
    phys_params: RobotPhysicalParams,
) -> *mut RobotModel {
    Box::into_raw(Box::new(RobotModel::new(kf_dt, kf_params, phys_params)))
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_robot_model_new_default(kf_dt: f32) -> *mut RobotModel {
    Box::into_raw(Box::new(RobotModel::new_from_default_params(kf_dt)))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_robot_model_free(model: *mut RobotModel) {
    if !model.is_null() {
        unsafe { drop(Box::from_raw(model)); }
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_robot_model_update_kf_params(
    model: *mut RobotModel,
    kf_params: KalmanFilterParams,
) {
    unsafe { (*model).update_kf_params(kf_params); }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_robot_model_update_physical_params(
    model: *mut RobotModel,
    phys_params: RobotPhysicalParams,
) {
    unsafe { (*model).update_physical_params(phys_params); }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_robot_model_kf_predict(
    model: *mut RobotModel,
    accel: Vector3C,
) {
    unsafe { (*model).kf_predict(accel.into()); }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_robot_model_kf_update(
    model: *mut RobotModel,
    measurement: Vector8C,
    mask_vision: bool,
    mask_encoder: bool,
    mask_gyro: bool,
) {
    unsafe { (*model).kf_update(measurement.into(), mask_vision, mask_encoder, mask_gyro); }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_robot_model_get_state(
    model: *const RobotModel,
) -> Vector6C {
    unsafe { (*model).x.into() }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_robot_model_set_state(
    model: *mut RobotModel,
    state: Vector6C,
) {
    unsafe { (*model).x = state.into(); }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_robot_model_transform_wheel2twist(
    model: *const RobotModel,
    theta: f32,
) -> Matrix3x4C {
    unsafe { (*model).transform_wheel2twist(theta).into() }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_robot_model_transform_twist2wheel(
    model: *const RobotModel,
    theta: f32,
) -> Matrix4x3C {
    unsafe { (*model).transform_twist2wheel(theta).into() }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_robot_model_transform_wheel2accel(
    model: *const RobotModel,
    theta: f32,
) -> Matrix3x4C {
    unsafe { (*model).transform_wheel2accel(theta).into() }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_robot_model_transform_accel2wheel(
    model: *const RobotModel,
    theta: f32,
) -> Matrix4x3C {
    unsafe { (*model).transform_accel2wheel(theta).into() }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_robot_model_torques_to_currents(
    model: *const RobotModel,
    torques: Vector4C,
) -> Vector4C {
    unsafe { (*model).torques_to_currents(torques.into()).into() }
}

// ============================================================================
// BangBang Trajectory
// ============================================================================

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_traj_params_default() -> TrajectoryParams {
    TrajectoryParams::default()
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_traj_from_target_pose(
    init_state: Vector6C,
    target_pose: Vector3C,
    params: TrajectoryParams,
) -> BangBangTraj3D {
    BangBangTraj3D::from_target_pose(init_state.into(), target_pose.into(), params)
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_traj_from_target_twist(
    init_twist: Vector3C,
    target_twist: Vector3C,
    params: TrajectoryParams,
) -> BangBangTraj3D {
    BangBangTraj3D::from_target_twist(init_twist.into(), target_twist.into(), params)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_traj_1d_time_shift(traj: *mut BangBangTraj1D, dt: f32) {
    unsafe { (*traj).time_shift(dt); }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_traj_3d_time_shift(traj: *mut BangBangTraj3D, dt: f32) {
    unsafe { (*traj).time_shift(dt); }
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_traj_end_time(traj: BangBangTraj3D) -> f32 {
    traj.end_time()
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_traj_state_at(
    traj: BangBangTraj3D,
    current_state: Vector6C,
    current_time: f32,
    t: f32,
) -> Vector6C {
    traj.state_at(current_state.into(), current_time, t).into()
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_traj_accel_at(traj: BangBangTraj3D, t: f32) -> Vector3C {
    traj.accel_at(t).into()
}
