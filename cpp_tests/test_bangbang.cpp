#include <gtest/gtest.h>
#include <ateam_controls/ateam_controls.h>
#include <cmath>

// Helper to build a zero-initialized Vector6C_t
static Vector6C_t make_state(float x, float y, float z, float dx, float dy, float dz) {
  Vector6C_t s = {};
  s.data[0] = x; s.data[1] = y; s.data[2] = z;
  s.data[3] = dx; s.data[4] = dy; s.data[5] = dz;
  return s;
}

// ============================================================================
// Default trajectory params
// ============================================================================

TEST(BangBang, DefaultParamsNonZero) {
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  EXPECT_GT(params.allowable_error_pos_linear, 0.0f);
  EXPECT_GT(params.allowable_error_pos_angular, 0.0f);
  EXPECT_GT(params.allowable_error_vel_linear, 0.0f);
  EXPECT_GT(params.allowable_error_vel_angular, 0.0f);
  EXPECT_GT(params.max_vel_linear, 0.0f);
  EXPECT_GT(params.max_vel_angular, 0.0f);
  EXPECT_GT(params.max_accel_linear, 0.0f);
  EXPECT_GT(params.max_accel_angular, 0.0f);
}

// ============================================================================
// from_target_pose – single axis
// ============================================================================

TEST(BangBang, PoseX) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 0.0f, 0.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  EXPECT_GT(end, 0.0f);
  Vector6C_t final_state = {};
  ASSERT_EQ(ateam_controls_traj_state_at(traj, init, 0.0f, end, &final_state), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(final_state.data[0], target.x, 1e-3f);
  EXPECT_NEAR(final_state.data[1], target.y, 1e-3f);
  EXPECT_NEAR(final_state.data[2], target.z, 1e-3f);
}

TEST(BangBang, PoseY) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {0.0f, 1.0f, 0.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  Vector6C_t final_state = {};
  ASSERT_EQ(ateam_controls_traj_state_at(traj, init, 0.0f, end, &final_state), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(final_state.data[0], target.x, 1e-3f);
  EXPECT_NEAR(final_state.data[1], target.y, 1e-3f);
  EXPECT_NEAR(final_state.data[2], target.z, 1e-3f);
}

TEST(BangBang, PoseZ) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {0.0f, 0.0f, 1.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  Vector6C_t final_state = {};
  ASSERT_EQ(ateam_controls_traj_state_at(traj, init, 0.0f, end, &final_state), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(final_state.data[0], target.x, 1e-3f);
  EXPECT_NEAR(final_state.data[1], target.y, 1e-3f);
  EXPECT_NEAR(final_state.data[2], target.z, 1e-3f);
}

// ============================================================================
// from_target_pose – combined axes
// ============================================================================

TEST(BangBang, PoseXYZ) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 1.0f, 1.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  Vector6C_t final_state = {};
  ASSERT_EQ(ateam_controls_traj_state_at(traj, init, 0.0f, end, &final_state), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(final_state.data[0], target.x, 1e-3f);
  EXPECT_NEAR(final_state.data[1], target.y, 1e-3f);
  EXPECT_NEAR(final_state.data[2], target.z, 1e-3f);
}

// Negative target
TEST(BangBang, PoseNegative) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {-2.0f, -1.5f, -0.5f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  Vector6C_t final_state = {};
  ASSERT_EQ(ateam_controls_traj_state_at(traj, init, 0.0f, end, &final_state), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(final_state.data[0], target.x, 1e-3f);
  EXPECT_NEAR(final_state.data[1], target.y, 1e-3f);
  EXPECT_NEAR(final_state.data[2], target.z, 1e-3f);
}

// ============================================================================
// Final velocity should be ~zero at end of pose trajectory
// ============================================================================

TEST(BangBang, PoseFinalVelocityZero) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 0.5f, 0.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  Vector6C_t final_state = {};
  ASSERT_EQ(ateam_controls_traj_state_at(traj, init, 0.0f, end, &final_state), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(final_state.data[3], 0.0f, 1e-3f);
  EXPECT_NEAR(final_state.data[4], 0.0f, 1e-3f);
  EXPECT_NEAR(final_state.data[5], 0.0f, 1e-3f);
}

// ============================================================================
// Acceleration symmetry: first phase accel == -last phase accel (from rest)
// ============================================================================

TEST(BangBang, AccelSymmetry) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 0.0f, 1.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(traj.x.sdd1, -traj.x.sdd3, 1e-6f);
  EXPECT_NEAR(traj.z.sdd1, -traj.z.sdd3, 1e-6f);
}

// ============================================================================
// Accel at time – before start is the first accel, after end is zero
// ============================================================================

TEST(BangBang, AccelAtTimeAfterEnd) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 0.0f, 0.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  Vector3C_t accel = {};
  ASSERT_EQ(ateam_controls_traj_accel_at(traj, end + 1.0f, &accel), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(accel.x, 0.0f, 1e-6f);
  EXPECT_NEAR(accel.y, 0.0f, 1e-6f);
  EXPECT_NEAR(accel.z, 0.0f, 1e-6f);
}

TEST(BangBang, AccelAtTimeStart) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 0.0f, 0.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  Vector3C_t accel = {};
  ASSERT_EQ(ateam_controls_traj_accel_at(traj, 0.0f, &accel), ATEAM_CONTROLS_OK);
  // Should be non-zero in x direction
  EXPECT_GT(std::fabs(accel.x), 0.0f);
}

// ============================================================================
// 3D time shift
// ============================================================================

TEST(BangBang, TimeShift3D) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 0.0f, 0.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float shift = 2.5f;
  float initial_end = ateam_controls_traj_end_time(traj);
  ateam_controls_traj_3d_time_shift(&traj, shift);
  float shifted_end = ateam_controls_traj_end_time(traj);
  EXPECT_NEAR(shifted_end - initial_end, shift, 1e-6f);
}

// ============================================================================
// 1D time shift
// ============================================================================

TEST(BangBang, TimeShift1D) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {1.0f, 0.5f, 0.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float orig_t1 = traj.x.t1;
  float shift = 3.0f;
  ateam_controls_traj_1d_time_shift(&traj.x, shift);
  EXPECT_NEAR(traj.x.t1 - orig_t1, shift, 1e-6f);
  EXPECT_NEAR(traj.x.t2 - traj.x.t1, (traj.x.t2 - shift) - (traj.x.t1 - shift), 1e-6f);
}

// ============================================================================
// Already-at-target yields zero-length trajectory
// ============================================================================

TEST(BangBang, AlreadyAtTarget) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {0.0f, 0.0f, 0.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  EXPECT_NEAR(end, 0.0f, 1e-6f);
}

// ============================================================================
// from_target_twist
// ============================================================================

TEST(BangBang, TwistTarget) {
  Vector3C_t init_twist = {0.0f, 0.0f, 0.0f};
  Vector3C_t target_twist = {0.3f, 0.0f, 0.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_twist(init_twist, target_twist, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  EXPECT_GT(end, 0.0f);
  // Acceleration in x should be positive to reach positive twist
  EXPECT_GT(traj.x.sdd1, 0.0f);
}

TEST(BangBang, TwistAlreadyAtTarget) {
  Vector3C_t twist = {0.0f, 0.0f, 0.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_twist(twist, twist, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  EXPECT_NEAR(end, 0.0f, 1e-6f);
}

// ============================================================================
// State at intermediate time – position should be between start and end
// ============================================================================

TEST(BangBang, StateAtMidpoint) {
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  Vector3C_t target = {2.0f, 0.0f, 0.0f};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  float mid = end / 2.0f;
  Vector6C_t mid_state = {};
  ASSERT_EQ(ateam_controls_traj_state_at(traj, init, 0.0f, mid, &mid_state), ATEAM_CONTROLS_OK);
  // x position should be between 0 and target
  EXPECT_GT(mid_state.data[0], 0.0f);
  EXPECT_LT(mid_state.data[0], target.x);
}

// ============================================================================
// Theta trajectory wraps through ±π instead of going through 0
// ============================================================================

TEST(BangBang, ThetaWrapsAroundPi) {
  // Start near +π, target near -π.  Shortest path is ~0.48 rad through ±π,
  // not ~5.6 rad through 0.
  const float start_theta = 2.8f;
  const float target_theta = -2.8f;
  Vector6C_t init = make_state(0, 0, start_theta, 0, 0, 0);
  Vector3C_t target = {0.0f, 0.0f, target_theta};
  TrajectoryParams_t params = ateam_controls_traj_params_default();
  BangBangTraj3D_t traj = {};
  ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_traj_end_time(traj);
  EXPECT_GT(end, 0.0f);

  // Sample theta at many points along the trajectory and verify it never
  // passes through 0 (i.e. |theta| should stay above π/2 at all times).
  const int N = 50;
  for (int i = 0; i <= N; ++i) {
    float t = end * static_cast<float>(i) / static_cast<float>(N);
    Vector6C_t st = {};
    ASSERT_EQ(ateam_controls_traj_state_at(traj, init, 0.0f, t, &st), ATEAM_CONTROLS_OK);
    float theta = st.data[2];
    EXPECT_GT(std::fabs(theta), static_cast<float>(M_PI) / 2.0f)
        << "theta crossed through 0 at sample " << i << " (t=" << t << ", theta=" << theta << ")";
  }
}

// ============================================================================
// Theta stays within [-π, π] throughout the trajectory
// ============================================================================

TEST(BangBang, ThetaStaysWithinPiBounds) {
  // Test several start/target combinations that exercise different quadrants
  struct Case { float start; float target; };
  Case cases[] = {
    { 0.0f,  2.5f},
    { 0.0f, -2.5f},
    { 2.8f, -2.8f},   // wraps through +π
    {-2.8f,  2.8f},   // wraps through -π
    { 1.0f, -1.0f},
    { 3.0f,  0.5f},
    {-3.0f, -0.5f},
  };

  TrajectoryParams_t params = ateam_controls_traj_params_default();
  const int N = 100;

  for (const auto& c : cases) {
    Vector6C_t init = make_state(0, 0, c.start, 0, 0, 0);
    Vector3C_t target = {0.0f, 0.0f, c.target};
    BangBangTraj3D_t traj = {};
    ASSERT_EQ(ateam_controls_traj_from_target_pose(init, target, params, &traj), ATEAM_CONTROLS_OK);
    float end = ateam_controls_traj_end_time(traj);

    for (int i = 0; i <= N; ++i) {
      float t = end * static_cast<float>(i) / static_cast<float>(N);
      Vector6C_t st = {};
      ASSERT_EQ(ateam_controls_traj_state_at(traj, init, 0.0f, t, &st), ATEAM_CONTROLS_OK);
      float theta = st.data[2];
      EXPECT_LE(theta, static_cast<float>(M_PI) + 1e-3f)
          << "theta > π: start=" << c.start << " target=" << c.target
          << " t=" << t << " theta=" << theta;
      EXPECT_GE(theta, -static_cast<float>(M_PI) - 1e-3f)
          << "theta < -π: start=" << c.start << " target=" << c.target
          << " t=" << t << " theta=" << theta;
    }
  }
}
