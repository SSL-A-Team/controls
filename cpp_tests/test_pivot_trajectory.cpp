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
  ASSERT_EQ(ateam_controls_pivot_traj_new(init, PI / 2.0f, params, &traj), ATEAM_CONTROLS_OK);
  EXPECT_GT(ateam_controls_pivot_traj_end_time(traj), 0.0f);
}

TEST(PivotTrajBindings, HeadingReachesTarget) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = init_state_at(0.0f, 0.0f, 0.0f);
  float target = PI / 2.0f;
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_new(init, target, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_pivot_traj_end_time(traj);
  Vector6C_t final_state = {};
  ASSERT_EQ(ateam_controls_pivot_traj_state_at(traj, end, &final_state), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(final_state.data[2], target, 1e-3f);
}

TEST(PivotTrajBindings, StateAtMaintainsOrbitRadius) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = init_state_at(1.0f, 0.5f, 0.3f);
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_new(init, 0.3f + PI / 2.0f, params, &traj), ATEAM_CONTROLS_OK);
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
  ASSERT_EQ(ateam_controls_pivot_traj_new(init, PI / 2.0f, params, &traj), ATEAM_CONTROLS_OK);
  Vector3C_t accel = {};
  ASSERT_EQ(ateam_controls_pivot_traj_accel_at(traj, 0.0f, &accel), ATEAM_CONTROLS_OK);
}

TEST(PivotTrajBindings, TimeShift) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = init_state_at(0.0f, 0.0f, 0.0f);
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_new(init, PI / 2.0f, params, &traj), ATEAM_CONTROLS_OK);
  float initial_end = ateam_controls_pivot_traj_end_time(traj);
  float shift = 1.5f;
  ateam_controls_pivot_traj_time_shift(&traj, shift);
  EXPECT_NEAR(ateam_controls_pivot_traj_end_time(traj) - initial_end, shift, 1e-6f);
}

TEST(PivotTrajBindings, ZeroDisplacementHasZeroEndTime) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = init_state_at(0.0f, 0.0f, 1.0f);
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_new(init, 1.0f, params, &traj), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(ateam_controls_pivot_traj_end_time(traj), 0.0f, 1e-6f);
}

TEST(PivotTrajBindings, InsetAngleMaintained) {
  PivotParams_t params = test_pivot_params();
  params.inset_angle = PI / 5.0f;
  Vector6C_t init = init_state_at(0.2f, 0.1f, -0.6f);
  PivotTrajectory_t traj = {};
  ASSERT_EQ(ateam_controls_pivot_traj_new(init, 1.2f, params, &traj), ATEAM_CONTROLS_OK);
  float end = ateam_controls_pivot_traj_end_time(traj);
  float d = (traj.orbit_target - traj.orbit_start) >= 0.0f ? 1.0f : -1.0f;
  int n = 10;
  for (int i = 0; i <= n; ++i) {
    float t = end * static_cast<float>(i) / static_cast<float>(n);
    Vector6C_t state = {};
    ASSERT_EQ(ateam_controls_pivot_traj_state_at(traj, t, &state), ATEAM_CONTROLS_OK);
    float phi = std::atan2(state.data[1] - traj.center_y, state.data[0] - traj.center_x);
    // Radius line pointing from the robot toward the orbit center.
    float toward_center = phi + PI;
    // Inset is the signed angle from that radius line to the heading, positive
    // toward the direction of travel.
    float measured_inset = -d * std::remainder(state.data[2] - toward_center, 2.0f * PI);
    EXPECT_NEAR(std::remainder(measured_inset - params.inset_angle, 2.0f * PI), 0.0f, 1e-3f) << "at t=" << t;
  }
}

TEST(PivotTrajBindings, InvalidParamsRejected) {
  PivotParams_t params = test_pivot_params();
  Vector6C_t init = make_state(0, 0, 0, 0, 0, 0);
  PivotTrajectory_t traj = {};
  params.max_vel_angular = 0.0f;
  EXPECT_NE(ateam_controls_pivot_traj_new(init, 1.0f, params, &traj), ATEAM_CONTROLS_OK);
}
