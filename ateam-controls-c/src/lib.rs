use ateam_controls;
use ateam_controls::ctypes::*;
use ateam_controls::bangbang_trajectory::*;
use ateam_controls::robot_model::*;


#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_add(left: u64, right: u64) -> u64 {
  ateam_controls::add(left, right)
}

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_transform_frame_global2robot_pose(robot_pose: Vector3C, pose: Vector3C) -> Vector3C {
//   transform_frame_global2robot_pose(robot_pose.into(), pose.into()).into()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_transform_frame_robot2global_pose(robot_pose: Vector3C, pose: Vector3C) -> Vector3C {
//   transform_frame_robot2global_pose(robot_pose.into(), pose.into()).into()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_transform_frame_global2robot_twist(robot_pose: Vector3C, twist: Vector3C) -> Vector3C {
//   transform_frame_global2robot_twist(robot_pose.into(), twist.into()).into()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_transform_frame_robot2global_twist(robot_pose: Vector3C, twist: Vector3C) -> Vector3C {
//   transform_frame_robot2global_twist(robot_pose.into(), twist.into()).into()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_transform_frame_global2robot_accel(robot_pose: Vector3C, accel: Vector3C) -> Vector3C {
//   transform_frame_global2robot_accel(robot_pose.into(), accel.into()).into()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_transform_frame_robot2global_accel(robot_pose: Vector3C, accel: Vector3C) -> Vector3C {
//   transform_frame_robot2global_accel(robot_pose.into(), accel.into()).into()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_robot_model_new_from_constants() -> RobotModelC {
//   RobotModel::new_from_constants().into()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_robot_model_wheel_velocities_to_twist(robot_model: RobotModelC, wheel_velocities: Vector4C) -> Vector3C {
//   RobotModel::from(robot_model).wheel_velocities_to_twist(wheel_velocities.into()).into()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_robot_model_twist_to_wheel_velocities(robot_model: RobotModelC, twist: Vector3C) -> Vector4C {
//   RobotModel::from(robot_model).twist_to_wheel_velocities(twist.into()).into()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_robot_model_wheel_torques_to_accel(robot_model: RobotModelC, torques: Vector4C) -> Vector3C {
//   RobotModel::from(robot_model).wheel_torques_to_accel(torques.into()).into()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_robot_model_accel_to_wheel_torques(robot_model: RobotModelC, accel: Vector3C) -> Vector4C {
//   RobotModel::from(robot_model).accel_to_wheel_torques(accel.into()).into()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_bangbang_traj_1d_time_shift(traj: *mut BangBangTraj1D, dt: f32) {
//   unsafe {(&mut *traj).time_shift(dt)};
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_bangbang_traj_3d_time_shift(traj: *mut BangBangTraj3D, dt: f32) {
//   unsafe {(&mut *traj).time_shift(dt)};
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_bangbang_traj_3d_get_end_time(traj: BangBangTraj3D) -> f32 {
//   traj.get_end_time()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_compute_optimal_bangbang_traj_3d(init_state: RigidBodyStateC, target_pose: Vector3C) -> BangBangTraj3D {
//   compute_optimal_bangbang_traj_3d(init_state.into(), target_pose.into())
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_compute_bangbang_traj_3d_state_at_t(traj: BangBangTraj3D, current_state: RigidBodyStateC, current_time: f32, t: f32) -> RigidBodyStateC {
//   compute_bangbang_traj_3d_state_at_t(traj, current_state.into(), current_time, t).into()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_compute_bangbang_traj_3d_accel_at_t(traj: BangBangTraj3D, t: f32) -> Vector3C {
//   compute_bangbang_traj_3d_accel_at_t(traj, t).into()
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn ateam_controls_compute_bangbang_traj_1d_accel_at_t(traj: BangBangTraj1D, t: f32) -> f32 {
//   compute_bangbang_traj_1d_accel_at_t(traj, t)
// }
