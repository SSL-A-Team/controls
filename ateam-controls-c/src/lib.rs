use ateam_controls::ctypes::*;
use ateam_controls::trajectory::*;
use ateam_controls::bangbang_trajectory::*;
use ateam_controls::pivot_trajectory::*;
use ateam_controls::robot_model::*;

// ============================================================================
// Default parameter constructors
// ============================================================================

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_default_kf_params() -> KalmanFilterParams {
    KalmanFilterParams::default()
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_default_phys_params() -> RobotPhysicalParams {
    RobotPhysicalParams::default()
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_default_traj_params() -> TrajectoryParams {
    TrajectoryParams::default()
}

// ============================================================================
// RobotModel (opaque pointer)
// ============================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_robot_model_new(
    kf_dt: f32,
    kf_params: KalmanFilterParams,
    phys_params: RobotPhysicalParams,
    out: *mut *mut RobotModel,
) -> i32 {
    match RobotModel::new(kf_dt, kf_params, phys_params) {
        Ok(model) => {
            unsafe { *out = Box::into_raw(Box::new(model)); }
            0
        }
        Err(e) => e as i32,
    }
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
) -> i32 {
    match unsafe { (*model).update_physical_params(phys_params) } {
        Ok(()) => 0,
        Err(e) => e as i32,
    }
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
) -> i32 {
    match unsafe { (*model).kf_update(measurement.into(), mask_vision, mask_encoder, mask_gyro) } {
        Ok(()) => 0,
        Err(e) => e as i32,
    }
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
pub unsafe extern "C" fn ateam_controls_traj_from_target_pose(
    init_state: Vector6C,
    target_pose: Vector3C,
    params: TrajectoryParams,
    out: *mut BangBangTraj3D,
) -> i32 {
    match BangBangTraj3D::from_target_pose(init_state.into(), target_pose.into(), params) {
        Ok(traj) => {
            unsafe { *out = traj; }
            0
        }
        Err(e) => e as i32,
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_traj_from_target_twist(
    init_state: Vector6C,
    target_twist: Vector3C,
    params: TrajectoryParams,
    out: *mut BangBangTraj3D,
) -> i32 {
    match BangBangTraj3D::from_target_twist(init_state.into(), target_twist.into(), params) {
        Ok(traj) => {
            unsafe { *out = traj; }
            0
        }
        Err(e) => e as i32,
    }
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
pub unsafe extern "C" fn ateam_controls_traj_state_at(
    traj: BangBangTraj3D,
    t: f32,
    out: *mut Vector6C,
) -> i32 {
    match traj.state_at(t) {
        Ok(state) => {
            unsafe { *out = state.into(); }
            0
        }
        Err(e) => e as i32,
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_traj_accel_at(
    traj: BangBangTraj3D,
    t: f32,
    out: *mut Vector3C,
) -> i32 {
    match traj.accel_at(t) {
        Ok(accel) => {
            unsafe { *out = accel.into(); }
            0
        }
        Err(e) => e as i32,
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_traj_tick(traj: *mut BangBangTraj3D, dt: f32) {
    unsafe { (*traj).tick(dt); }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_traj_sample(
    traj: BangBangTraj3D,
    state_out: *mut Vector6C,
    accel_out: *mut Vector3C,
) {
    unsafe {
        let (state, accel) = traj.sample();
        *state_out = state.into();
        *accel_out = accel.into();
    }
}

// ============================================================================
// Pivot Trajectory
// ============================================================================

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_default_pivot_params() -> PivotParams {
    PivotParams::default()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_pivot_traj_new(
    init_state: Vector6C,
    target_heading: f32,
    params: PivotParams,
    out: *mut PivotTrajectory,
) -> i32 {
    match PivotTrajectory::from_target_heading(init_state.into(), target_heading, params) {
        Ok(traj) => {
            unsafe { *out = traj; }
            0
        }
        Err(e) => e as i32,
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_pivot_traj_time_shift(traj: *mut PivotTrajectory, dt: f32) {
    unsafe { (*traj).time_shift(dt); }
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_pivot_traj_end_time(traj: PivotTrajectory) -> f32 {
    traj.end_time()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_pivot_traj_state_at(
    traj: PivotTrajectory,
    t: f32,
    out: *mut Vector6C,
) -> i32 {
    match traj.state_at(t) {
        Ok(state) => {
            unsafe { *out = state.into(); }
            0
        }
        Err(e) => e as i32,
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_pivot_traj_accel_at(
    traj: PivotTrajectory,
    t: f32,
    out: *mut Vector3C,
) -> i32 {
    match traj.accel_at(t) {
        Ok(accel) => {
            unsafe { *out = accel.into(); }
            0
        }
        Err(e) => e as i32,
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_pivot_traj_tick(traj: *mut PivotTrajectory, dt: f32) {
    unsafe { (*traj).tick(dt); }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ateam_controls_pivot_traj_sample(
    traj: PivotTrajectory,
    state_out: *mut Vector6C,
    accel_out: *mut Vector3C,
) {
    unsafe {
        let (state, accel) = traj.sample();
        *state_out = state.into();
        *accel_out = accel.into();
    }
}
