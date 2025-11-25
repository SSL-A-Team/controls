use ateam_controls;

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_add(left: u64, right: u64) -> u64 {
  ateam_controls::add(left, right)
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_compute_bang_bang_traj_3d_state_at_t(traj: ateam_controls::bang_bang_trajectory::BangBangTraj3D, current_state: ateam_controls::GlobalState, current_time: f64, t: f64) -> ateam_controls::GlobalState {
  ateam_controls::bang_bang_trajectory::compute_bang_bang_traj_3d_state_at_t(traj, current_state, current_time, t)
}

#[unsafe(no_mangle)]
pub extern "C" fn ateam_controls_compute_optimal_bang_bang_traj_3d(init_state: ateam_controls::GlobalState, target_state: ateam_controls::GlobalState) -> ateam_controls::bang_bang_trajectory::BangBangTraj3D {
  ateam_controls::bang_bang_trajectory::compute_optimal_bang_bang_traj_3d(init_state, target_state)
}