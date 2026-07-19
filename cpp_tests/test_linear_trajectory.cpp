#include <gtest/gtest.h>
#include <ateam_controls/ateam_controls.h>
#include <cmath>

// ============================================================================
// Linear (line-following) trajectory binding smoke tests
// ============================================================================

static const float PI = static_cast<float>(M_PI);

static LinearParams_t test_linear_params() {
  LinearParams_t p = {};
  p.max_vel_colinear = 3.0f;
  p.max_vel_perp = 3.0f;
  p.max_vel_angular = 3.0f * PI;
  p.max_accel_perp = 2.0f;
  p.max_accel_colinear = 2.0f;
  p.max_accel_angular = 2.0f * PI;
  p.colinear_start_thresh_linear = 0.1f;
  return p;
}

static Vector6C_t make_state(float x, float y, float z, float dx, float dy, float dz) {
  Vector6C_t s = {};
  s.data[0] = x; s.data[1] = y; s.data[2] = z;
  s.data[3] = dx; s.data[4] = dy; s.data[5] = dz;
  return s;
}

static Vector2C_t make_vec2(float x, float y) {
  Vector2C_t v = {};
  v.x = x; v.y = y;
  return v;
}

// Signed perpendicular distance of a state from the line through start with
// unit direction (dx, dy). Perp direction is the +90 deg rotation (-dy, dx).
static float perp_dist(const Vector6C_t& s, const Vector2C_t& start, float dx, float dy) {
  float rx = s.data[0] - start.x;
  float ry = s.data[1] - start.y;
  return rx * (-dy) + ry * (dx);
}

static float colinear_vel(const Vector6C_t& s, float dx, float dy) {
  return s.data[3] * dx + s.data[4] * dy;
}

TEST(LinearTrajBindings, DefaultParams) {
  LinearParams_t p = ateam_controls_default_linear_params();
  EXPECT_GT(p.max_vel_colinear, 0.0f);
  EXPECT_GT(p.max_accel_colinear, 0.0f);
  EXPECT_GT(p.colinear_start_thresh_linear, 0.0f);
}

TEST(LinearTrajBindings, FromLineSucceeds) {
  LinearParams_t params = test_linear_params();
  Vector6C_t init = make_state(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  LinearTrajectory_t traj = {};
  ASSERT_EQ(
      ateam_controls_linear_traj_from_line(
          init, 0.0f, make_vec2(0.0f, 0.0f), make_vec2(1.0f, 0.0f), 1.5f, params, &traj),
      ATEAM_CONTROLS_OK);
  EXPECT_GT(ateam_controls_linear_traj_end_time(traj), 0.0f);
}

TEST(LinearTrajBindings, DegenerateDirectionRejected) {
  LinearParams_t params = test_linear_params();
  Vector6C_t init = make_state(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  LinearTrajectory_t traj = {};
  EXPECT_EQ(
      ateam_controls_linear_traj_from_line(
          init, 0.0f, make_vec2(0.0f, 0.0f), make_vec2(0.0f, 0.0f), 1.5f, params, &traj),
      ATEAM_CONTROLS_INVALID_INPUT);
}

TEST(LinearTrajBindings, ReachesLineHeadingAndSpeed) {
  LinearParams_t params = test_linear_params();
  Vector2C_t start = make_vec2(0.0f, 0.0f);
  float dx = 1.0f, dy = 0.0f;
  float line_vel = 1.5f;
  Vector6C_t init = make_state(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  LinearTrajectory_t traj = {};
  ASSERT_EQ(
      ateam_controls_linear_traj_from_line(
          init, 0.0f, start, make_vec2(dx, dy), line_vel, params, &traj),
      ATEAM_CONTROLS_OK);

  float end = ateam_controls_linear_traj_end_time(traj);
  Vector6C_t final_state = {};
  ASSERT_EQ(ateam_controls_linear_traj_state_at(traj, end, &final_state), ATEAM_CONTROLS_OK);
  EXPECT_NEAR(perp_dist(final_state, start, dx, dy), 0.0f, 1e-2f);
  EXPECT_NEAR(final_state.data[2], 0.0f, 1e-2f);
  EXPECT_NEAR(colinear_vel(final_state, dx, dy), line_vel, 1e-2f);
}

TEST(LinearTrajBindings, AccelAndSampleDefined) {
  LinearParams_t params = test_linear_params();
  Vector6C_t init = make_state(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  LinearTrajectory_t traj = {};
  ASSERT_EQ(
      ateam_controls_linear_traj_from_line(
          init, 0.0f, make_vec2(0.0f, 0.0f), make_vec2(1.0f, 0.0f), 1.5f, params, &traj),
      ATEAM_CONTROLS_OK);

  Vector3C_t accel = {};
  ASSERT_EQ(ateam_controls_linear_traj_accel_at(traj, 0.0f, &accel), ATEAM_CONTROLS_OK);

  Vector6C_t state = {};
  Vector3C_t sample_accel = {};
  ateam_controls_linear_traj_sample(traj, &state, &sample_accel);
  EXPECT_FLOAT_EQ(state.data[1], 1.0f);
}

TEST(LinearTrajBindings, TickAdvancesState) {
  LinearParams_t params = test_linear_params();
  Vector6C_t init = make_state(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  LinearTrajectory_t traj = {};
  ASSERT_EQ(
      ateam_controls_linear_traj_from_line(
          init, 0.0f, make_vec2(0.0f, 0.0f), make_vec2(1.0f, 0.0f), 1.5f, params, &traj),
      ATEAM_CONTROLS_OK);

  Vector6C_t predicted = {};
  ASSERT_EQ(ateam_controls_linear_traj_state_at(traj, 0.01f, &predicted), ATEAM_CONTROLS_OK);
  ateam_controls_linear_traj_tick(&traj, 0.01f);
  Vector6C_t sampled = {};
  Vector3C_t accel = {};
  ateam_controls_linear_traj_sample(traj, &sampled, &accel);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(sampled.data[i], predicted.data[i], 1e-4f);
  }
}

// ----------------------------------------------------------------------------
// from_point (face a global point) binding smoke tests
// ----------------------------------------------------------------------------

TEST(LinearTrajBindings, FromPointSucceeds) {
  LinearParams_t params = test_linear_params();
  Vector6C_t init = make_state(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  LinearTrajectory_t traj = {};
  ASSERT_EQ(
      ateam_controls_linear_traj_from_point(
          init, 2.0f, 2.0f, make_vec2(0.0f, 0.0f), make_vec2(1.0f, 0.0f), 1.5f, params, &traj),
      ATEAM_CONTROLS_OK);
  EXPECT_EQ(traj.heading_mode, LINEAR_HEADING_MODE_FACE_POINT);
  EXPECT_GT(ateam_controls_linear_traj_end_time(traj), 0.0f);
}

TEST(LinearTrajBindings, FromPointRejectsStartOnPoint) {
  LinearParams_t params = test_linear_params();
  Vector6C_t init = make_state(1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  LinearTrajectory_t traj = {};
  EXPECT_EQ(
      ateam_controls_linear_traj_from_point(
          init, 1.0f, 1.0f, make_vec2(0.0f, 0.0f), make_vec2(1.0f, 0.0f), 1.0f, params, &traj),
      ATEAM_CONTROLS_INVALID_INPUT);
}

TEST(LinearTrajBindings, FromPointFacesPointInSteadyState) {
  LinearParams_t params = test_linear_params();
  Vector2C_t start = make_vec2(0.0f, 0.0f);
  float dx = 1.0f, dy = 0.0f;
  float px = 5.0f, py = 3.0f;
  float line_vel = 1.0f;
  Vector6C_t init = make_state(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  LinearTrajectory_t traj = {};
  ASSERT_EQ(
      ateam_controls_linear_traj_from_point(
          init, px, py, start, make_vec2(dx, dy), line_vel, params, &traj),
      ATEAM_CONTROLS_OK);

  float end = ateam_controls_linear_traj_end_time(traj);
  Vector6C_t st = {};
  ASSERT_EQ(ateam_controls_linear_traj_state_at(traj, end + 1.0f, &st), ATEAM_CONTROLS_OK);
  // On the line and at target colinear speed.
  EXPECT_NEAR(perp_dist(st, start, dx, dy), 0.0f, 1e-2f);
  EXPECT_NEAR(colinear_vel(st, dx, dy), line_vel, 1e-2f);
  // Heading points at the target point.
  float los = std::atan2(py - st.data[1], px - st.data[0]);
  float err = std::atan2(std::sin(st.data[2] - los), std::cos(st.data[2] - los));
  EXPECT_NEAR(err, 0.0f, 3e-2f);
}

TEST(LinearTrajBindings, FromPointAccelAndSampleDefined) {
  LinearParams_t params = test_linear_params();
  Vector6C_t init = make_state(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  LinearTrajectory_t traj = {};
  ASSERT_EQ(
      ateam_controls_linear_traj_from_point(
          init, 2.0f, 2.0f, make_vec2(0.0f, 0.0f), make_vec2(1.0f, 0.0f), 1.5f, params, &traj),
      ATEAM_CONTROLS_OK);
  Vector3C_t accel = {};
  ASSERT_EQ(ateam_controls_linear_traj_accel_at(traj, 0.0f, &accel), ATEAM_CONTROLS_OK);
  Vector6C_t state = {};
  Vector3C_t sample_accel = {};
  ateam_controls_linear_traj_sample(traj, &state, &sample_accel);
  EXPECT_FLOAT_EQ(state.data[1], 1.0f);
}
