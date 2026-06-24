#include <gtest/gtest.h>
#include <ateam_controls/ateam_controls.h>
#include <cmath>

// ============================================================================
// Pivot trajectory binding smoke tests
// ============================================================================

static const float PI = static_cast<float>(M_PI);

static PivotParams_t test_pivot_params() {
  PivotParams_t p = {};
  p.max_vel_angular = 2.0f * PI;
  p.max_accel_angular = 4.0f * PI;
  p.orbit_radius = 0.0215f + 0.090f;  // DEFAULT_BALL_RADIUS + DEFAULT_ROBOT_RADIUS
  p.inset_angle = 0.0f;
  return p;
}

static Vector6C_t make_state(float x, float y, float z, float dx, float dy, float dz) {
  Vector6C_t s = {};
  s.data[0] = x; s.data[1] = y; s.data[2] = z;
  s.data[3] = dx; s.data[4] = dy; s.data[5] = dz;
  return s;
}

static Vector6C_t init_state_at(float x, float y, float heading) {
  return make_state(x, y, heading, 0.0f, 0.0f, 0.0f);
}

TEST(PivotTrajBindings, DefaultParams) {
  PivotParams_t p = ateam_controls_default_pivot_params();
  EXPECT_GT(p.max_vel_angular, 0.0f);
  EXPECT_GT(p.max_accel_angular, 0.0f);
  EXPECT_GT(p.orbit_radius, 0.0f);
}

TEST(PivotTrajBindings, NewSucceeds) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = init_state_at(0.0f, 0.0f, 0.0f);
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_from_target_heading(init, PI / 2.0f, params, &traj), ATEAM_CONTROLS_OK);
  EXPECT_GT(ateam_controls_pivot_traj_end_time(traj), 0.0f);
}

TEST(PivotTrajBindings, HeadingReachesTarget) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = init_state_at(0.0f, 0.0f, 0.0f);
  float target = PI / 2.0f;
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_from_target_heading(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_pivot_traj_end_time(traj);
  Vector6C_t final_state = {};
  ASSERT_EQ(ateam_controls_pivot_traj_state_at(traj, end, &final_state), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(final_state.data[2], target, 1e-3f);
}

TEST(PivotTrajBindings, StateAtMaintainsOrbitRadius) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = init_state_at(1.0f, 0.5f, 0.3f);
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_from_target_heading(init, 0.3f + PI / 2.0f, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_pivot_traj_end_time(traj);
  int n = 10;
  for (int i = 0; i <= n; ++i) {
    float t = end * static_cast<float>(i) / static_cast<float>(n);
    Vector6C_t state = {};
    ASSERT_EQ(ateam_controls_pivot_traj_state_at(traj, t, &state), ATEAM_CONTROLS_OK);
    float dx = state.data[0] - traj.center_x;
    float dy = state.data[1] - traj.center_y;
    float dist = std::sqrt(dx * dx + dy * dy);
    EXPECT_NEAR(dist, traj.orbit_radius, 1e-4f) << "at t=" << t;
  }
}

TEST(PivotTrajBindings, AccelAtRoundTrip) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = init_state_at(0.0f, 0.0f, 0.0f);
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_from_target_heading(init, PI / 2.0f, params, &traj), ATEAM_CONTROLS_OK);
  Vector3C_t accel = {};
  ASSERT_EQ(ateam_controls_pivot_traj_accel_at(traj, 0.0f, &accel), ATEAM_CONTROLS_OK);
}

TEST(PivotTrajBindings, TimeShift) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = init_state_at(0.0f, 0.0f, 0.0f);
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_from_target_heading(init, PI / 2.0f, params, &traj), ATEAM_CONTROLS_OK);
  float initial_end = ateam_controls_pivot_traj_end_time(traj);
  float shift = 1.5f;
  ateam_controls_pivot_traj_time_shift(&traj, shift);
  EXPECT_NEAR(ateam_controls_pivot_traj_end_time(traj) - initial_end, shift, 1e-6f);
}

TEST(PivotTrajBindings, ZeroDisplacementHasZeroEndTime) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = init_state_at(0.0f, 0.0f, 1.0f);
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_from_target_heading(init, 1.0f, params, &traj), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(ateam_controls_pivot_traj_end_time(traj), 0.0f, 1e-6f);
}

TEST(PivotTrajBindings, HeadingOffsetConstant) {
  PivotParams_t params = test_pivot_params();
  params.inset_angle = PI / 5.0f;
  Vector6C_t init = init_state_at(0.2f, 0.1f, -0.6f);
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_from_target_heading(init, 1.2f, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_pivot_traj_end_time(traj);
  int n = 10;
  for (int i = 0; i <= n; ++i) {
    float t = end * static_cast<float>(i) / static_cast<float>(n);
    Vector6C_t state = {};
    ASSERT_EQ(ateam_controls_pivot_traj_state_at(traj, t, &state), ATEAM_CONTROLS_OK);
    // Orbit angle from center to robot; heading must stay a fixed offset ahead.
    float phi = std::atan2(state.data[1] - traj.center_y, state.data[0] - traj.center_x);
    float offset = std::remainder(state.data[2] - phi, 2.0f * PI);
    EXPECT_NEAR(std::remainder(offset - traj.heading_offset, 2.0f * PI), 0.0f, 1e-3f) << "at t=" << t;
  }
}

TEST(PivotTrajBindings, ComputedInsetOverridesProvided) {
  PivotParams_t params = test_pivot_params();
  params.inset_angle = 999.0f;          // must be ignored
  params.compute_inset_angle = true;
  Vector6C_t init = init_state_at(0.0f, 0.0f, 0.0f);
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_from_target_heading(init, 0.5f, params, &traj), ATEAM_CONTROLS_OK);
  // Forward + small CCW target => left circle => offset = pi/2 + psi, psi in [0, pi/2].
  EXPECT_GT(traj.heading_offset, PI / 2.0f - 1e-3f);
  EXPECT_LE(traj.heading_offset, PI + 1e-3f);
}

TEST(PivotTrajBindings, BackwardDirection) {
  PivotParams_t params = test_pivot_params();
  params.inset_angle = PI / 6.0f;
  params.direction = PIVOT_DIRECTION_BACKWARD;
  Vector6C_t init = init_state_at(0.0f, 0.0f, 0.0f);
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_from_target_heading(init, PI / 2.0f, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_pivot_traj_end_time(traj);
  Vector6C_t s = {};
  ASSERT_EQ(ateam_controls_pivot_traj_state_at(traj, end * 0.5f, &s), ATEAM_CONTROLS_OK);
  // Backward: velocity vector is obtuse with the heading.
  float vel_dir = std::atan2(s.data[4], s.data[3]);
  float angle = std::fabs(std::remainder(s.data[2] - vel_dir, 2.0f * PI));
  EXPECT_GT(angle, PI / 2.0f);
}

TEST(PivotTrajBindings, FromTargetPointFacesPoint) {
  PivotParams_t params = test_pivot_params();
  params.inset_angle = 0.3f;
  Vector6C_t init = init_state_at(0.0f, 0.0f, 0.3f);
  const float tx = 2.0f, ty = 1.0f;
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_from_target_point(init, tx, ty, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_pivot_traj_end_time(traj);
  Vector6C_t final_state = {};
  ASSERT_EQ(ateam_controls_pivot_traj_state_at(traj, end, &final_state), ATEAM_CONTROLS_OK);
  float bearing = std::atan2(ty - final_state.data[1], tx - final_state.data[0]);
  EXPECT_NEAR(std::remainder(final_state.data[2] - bearing, 2.0f * PI), 0.0f, 1e-2f);
}

TEST(PivotTrajBindings, FromTargetPointRejectsInteriorPoint) {
  PivotParams_t params = test_pivot_params();
  params.inset_angle = 0.3f;
  Vector6C_t init = init_state_at(0.0f, 0.0f, 0.7f);
  PivotTrajectory_t traj = {};
  // The robot sits on both candidate circles, so facing its own position is rejected.
  EXPECT_EQ(ateam_controls_pivot_traj_from_target_point(init, 0.0f, 0.0f, params, &traj),
            ATEAM_CONTROLS_INVALID_INPUT);
}

TEST(PivotTrajBindings, SampleMatchesStateAtZero) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = init_state_at(1.0f, 0.5f, 0.3f);
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_from_target_heading(init, 0.3f + PI / 2.0f, params, &traj), ATEAM_CONTROLS_OK);
  Vector6C_t state = {};
  Vector3C_t accel = {};
  ateam_controls_pivot_traj_sample(traj, &state, &accel);
  Vector6C_t state_ref = {};
  ASSERT_EQ(ateam_controls_pivot_traj_state_at(traj, 0.0f, &state_ref), ATEAM_CONTROLS_OK);
  Vector3C_t accel_ref = {};
  ASSERT_EQ(ateam_controls_pivot_traj_accel_at(traj, 0.0f, &accel_ref), ATEAM_CONTROLS_OK);
  for (int i = 0; i < 6; ++i) EXPECT_NEAR(state.data[i], state_ref.data[i], 1e-5f);
  EXPECT_NEAR(accel.x, accel_ref.x, 1e-5f);
  EXPECT_NEAR(accel.y, accel_ref.y, 1e-5f);
  EXPECT_NEAR(accel.z, accel_ref.z, 1e-5f);
}

TEST(PivotTrajBindings, TickAdvancesToTarget) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = init_state_at(0.0f, 0.0f, 0.0f);
  float target = PI / 2.0f;
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_from_target_heading(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_pivot_traj_end_time(traj);
  const float dt = 0.001f;
  int steps = static_cast<int>(end / dt) + 10;
  for (int i = 0; i < steps; ++i) ateam_controls_pivot_traj_tick(&traj, dt);
  Vector6C_t state = {};
  Vector3C_t accel = {};
  ateam_controls_pivot_traj_sample(traj, &state, &accel);
  EXPECT_NEAR(state.data[2], target, 1e-2f);
}

TEST(PivotTrajBindings, InvalidParamsRejected) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  PivotTrajectory_t traj = {};
  params.max_vel_angular = 0.0f;
  EXPECT_NE(ateam_controls_pivot_traj_from_target_heading(init, 1.0f, params, &traj), ATEAM_CONTROLS_OK);
}
