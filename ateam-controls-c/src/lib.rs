use ateam_controls;
use ateam_controls::geometry::*;
use ateam_controls::bangbang_trajectory::*;

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_add(left: u64, right: u64) -> u64 {
  ateam_controls::add(left, right)
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_compute_optimal_bangbang_traj_3d(init_state: RigidBodyState, target_pose: Pose) -> BangBangTraj3D {
  compute_optimal_bangbang_traj_3d(init_state, target_pose)
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_compute_bangbang_traj_3d_state_at_t(traj: BangBangTraj3D, current_state: RigidBodyState, current_time: f32, t: f32) -> RigidBodyState {
  compute_bangbang_traj_3d_state_at_t(traj, current_state, current_time, t)
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_compute_bangbang_traj_3d_accel_at_t(traj: BangBangTraj3D, t: f32) -> Accel {
  compute_bangbang_traj_3d_accel_at_t(traj, t)
}
