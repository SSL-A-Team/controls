use ateam_controls;

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_add(left: u64, right: u64) -> u64 {
  ateam_controls::add(left, right)
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_compute_bangbang_traj_3d_state_at_t(traj: ateam_controls::bangbang_trajectory::BangBangTraj3D, current_state: ateam_controls::geometry::RigidBodyState, current_time: f64, t: f64) -> ateam_controls::geometry::RigidBodyState {
  ateam_controls::bangbang_trajectory::compute_bangbang_traj_3d_state_at_t(traj, current_state, current_time, t)
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_compute_optimal_bangbang_traj_3d(init_state: ateam_controls::geometry::RigidBodyState, target_state: ateam_controls::geometry::RigidBodyState) -> ateam_controls::bangbang_trajectory::BangBangTraj3D {
  ateam_controls::bangbang_trajectory::compute_optimal_bangbang_traj_3d(init_state, target_state)
}