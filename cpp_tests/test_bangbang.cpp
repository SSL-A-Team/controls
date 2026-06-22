#include <gtest/gtest.h>
#include <ateam_controls/ateam_controls.h>
#include <cmath>

// ============================================================================
// Binding smoke tests – verify C FFI round-trips work correctly
// ============================================================================

static TrajectoryParams_t test_traj_params() {
  TrajectoryParams_t p = {};
  p.max_vel_linear = 3.0f;
  p.max_vel_angular = 3.0f * static_cast<float>(M_PI);
  p.max_accel_linear = 2.0f;
  p.max_accel_angular = 2.0f * static_cast<float>(M_PI);
  return p;
}

static Vector6C_t make_state(float x, float y, float z, float dx, float dy, float dz) {
  Vector6C_t s = {};
  s.data[0] = x; s.data[1] = y; s.data[2] = z;
  s.data[3] = dx; s.data[4] = dy; s.data[5] = dz;
  return s;
}

TEST(BangBangBindings, FromTargetPoseRoundTrip) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 0.0f, 0.0f};
  TrajectoryParams_t params = test_traj_params();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  EXPECT_GT(end, 0.0f);
}

TEST(BangBangBindings, FromTargetTwistRoundTrip) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target_twist = {0.3f, 0.0f, 0.0f};
  TrajectoryParams_t params = test_traj_params();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_twist(init, target_twist, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  EXPECT_GT(end, 0.0f);
}

TEST(BangBangBindings, StateAtRoundTrip) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 0.0f, 0.0f};
  TrajectoryParams_t params = test_traj_params();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  Vector6C_t final_state = {};
  ASSERT_EQ(ateam_controls_traj_state_at(traj, end, &final_state), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(final_state.data[0], target.x, 1e-3f);
}

TEST(BangBangBindings, AccelAtRoundTrip) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 0.0f, 0.0f};
  TrajectoryParams_t params = test_traj_params();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  Vector3C_t accel = {};
  ASSERT_EQ(ateam_controls_traj_accel_at(traj, 0.0f, &accel), ATEAM_CONTROLS_OK);
  EXPECT_GT(std::fabs(accel.x), 0.0f);
}

TEST(BangBangBindings, TimeShift) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 0.0f, 0.0f};
  TrajectoryParams_t params = test_traj_params();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float initial_end = ateam_controls_traj_end_time(traj);
  ateam_controls_traj_3d_time_shift(&traj, 2.5f);
  float shifted_end = ateam_controls_traj_end_time(traj);
  EXPECT_NEAR(shifted_end - initial_end, 2.5f, 1e-6f);
}

TEST(BangBangBindings, SampleMatchesStateAtZero) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 0.0f, 0.0f};
  TrajectoryParams_t params = test_traj_params();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  Vector6C_t state = {};
  Vector3C_t accel = {};
  ateam_controls_traj_sample(traj, &state, &accel);
  Vector6C_t state_ref = {};
  ASSERT_EQ(ateam_controls_traj_state_at(traj, 0.0f, &state_ref), ATEAM_CONTROLS_OK);
  Vector3C_t accel_ref = {};
  ASSERT_EQ(ateam_controls_traj_accel_at(traj, 0.0f, &accel_ref), ATEAM_CONTROLS_OK);
  for (int i = 0; i < 6; ++i) EXPECT_NEAR(state.data[i], state_ref.data[i], 1e-5f);
  EXPECT_NEAR(accel.x, accel_ref.x, 1e-5f);
  EXPECT_NEAR(accel.y, accel_ref.y, 1e-5f);
  EXPECT_NEAR(accel.z, accel_ref.z, 1e-5f);
}

TEST(BangBangBindings, TickAdvancesToTarget) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 0.0f, 0.0f};
  TrajectoryParams_t params = test_traj_params();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  const float dt = 0.001f;
  int steps = static_cast<int>(end / dt) + 10;
  for (int i = 0; i < steps; ++i) ateam_controls_traj_tick(&traj, dt);
  Vector6C_t state = {};
  Vector3C_t accel = {};
  ateam_controls_traj_sample(traj, &state, &accel);
  EXPECT_NEAR(state.data[0], target.x, 1e-2f);
  // Once the trajectory has fully elapsed, no acceleration should be commanded.
  EXPECT_NEAR(accel.x, 0.0f, 1e-3f);
}
