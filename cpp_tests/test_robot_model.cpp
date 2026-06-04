#include <gtest/gtest.h>
#include <ateam_controls/ateam_controls.h>
#include <cmath>

// ============================================================================
// Test helpers – construct params explicitly
// ============================================================================

static KalmanFilterParams_t test_kf_params() {
  KalmanFilterParams_t kf = {};
  kf.process_noise_std_pos_linear = 0.01f;
  kf.process_noise_std_pos_angular = 0.05f;
  kf.process_noise_std_vel_linear = 0.02f;
  kf.process_noise_std_vel_angular = 0.1f;
  kf.measurement_noise_std_vision_pos_linear = 1.0f;
  kf.measurement_noise_std_vision_pos_angular = 3.14f;
  kf.measurement_noise_std_encoder_vel_angular = 50.0f;
  kf.measurement_noise_std_gyro_vel_angular = 0.015f;
  kf.max_pos_linear = 64.0f;
  kf.max_pos_angular = 3.14f;
  kf.max_vel_linear = 3.0f;
  kf.max_vel_angular = 3.0f * static_cast<float>(M_PI);
  return kf;
}

static RobotPhysicalParams_t test_phys_params() {
  RobotPhysicalParams_t phys = {};
  phys.alpha = static_cast<float>(M_PI / 6.0);
  phys.beta = static_cast<float>(M_PI / 4.0);
  phys.l = 0.0814f;
  phys.r = 0.030f;
  phys.mass = 2.7f;
  phys.iz = 0.008f;
  phys.motor_torque_constant = 0.0335f;
  phys.motor_efficiency_factor = 13.0f;
  phys.coulomb_friction_coefficient_linear_x = 2.058f;
  phys.coulomb_friction_coefficient_linear_y = 2.058f;
  phys.coulomb_friction_coefficient_angular = 0.05f;
  phys.viscous_friction_coefficient_linear_x = 5.0f;
  phys.viscous_friction_coefficient_linear_y = 5.0f;
  phys.viscous_friction_coefficient_angular = 0.0063f;
  return phys;
}

static int32_t create_test_model(float kf_dt, RobotModel_t** out) {
  return ateam_controls_robot_model_new(kf_dt, test_kf_params(), test_phys_params(), out);
}

// ============================================================================
// Binding smoke tests – verify C FFI round-trips work correctly
// ============================================================================

TEST(RobotModelBindings, NewAndFree) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(0.01f, &model), ATEAM_CONTROLS_OK);
  ASSERT_NE(model, nullptr);
  ateam_controls_robot_model_free(model);
}

TEST(RobotModelBindings, NewWithCustomParams) {
  KalmanFilterParams_t kf = {};
  kf.process_noise_std_pos_linear = 0.01f;
  kf.process_noise_std_pos_angular = 0.01f;
  kf.process_noise_std_vel_linear = 0.1f;
  kf.process_noise_std_vel_angular = 0.1f;
  kf.measurement_noise_std_vision_pos_linear = 0.01f;
  kf.measurement_noise_std_vision_pos_angular = 0.01f;
  kf.measurement_noise_std_encoder_vel_angular = 0.1f;
  kf.measurement_noise_std_gyro_vel_angular = 0.1f;
  kf.max_pos_linear = 64.0f;
  kf.max_pos_angular = 6.2832f;
  kf.max_vel_linear = 4.0f;
  kf.max_vel_angular = 3.0f * static_cast<float>(M_PI);

  RobotPhysicalParams_t phys = {};
  phys.alpha = 0.7854f;
  phys.beta = 0.7854f;
  phys.l = 0.08f;
  phys.r = 0.025f;
  phys.mass = 2.5f;
  phys.iz = 0.01f;
  phys.motor_torque_constant = 0.02f;
  phys.motor_efficiency_factor = 0.85f;

  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new(0.01f, kf, phys, &model), ATEAM_CONTROLS_OK);
  ASSERT_NE(model, nullptr);
  ateam_controls_robot_model_free(model);
}

TEST(RobotModelBindings, FreeNull) {
  ateam_controls_robot_model_free(nullptr);
}

TEST(RobotModelBindings, SetAndGetStateRoundTrip) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(0.01f, &model), ATEAM_CONTROLS_OK);
  Vector6C_t state = {};
  state.data[0] = 1.0f;
  state.data[1] = 2.0f;
  state.data[2] = 0.5f;
  state.data[3] = 0.1f;
  state.data[4] = -0.2f;
  state.data[5] = 0.3f;
  ateam_controls_robot_model_set_state(model, state);
  Vector6C_t out = ateam_controls_robot_model_get_state(model);
  for (int i = 0; i < 6; i++) {
    EXPECT_NEAR(out.data[i], state.data[i], 1e-9f);
  }
  ateam_controls_robot_model_free(model);
}

TEST(RobotModelBindings, UpdateKfParams) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(0.01f, &model), ATEAM_CONTROLS_OK);
  KalmanFilterParams_t kf = test_kf_params();
  kf.process_noise_std_pos_linear = 0.05f;
  ateam_controls_robot_model_update_kf_params(model, kf);
  Vector6C_t state = ateam_controls_robot_model_get_state(model);
  (void)state;
  ateam_controls_robot_model_free(model);
}

TEST(RobotModelBindings, UpdatePhysicalParams) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(0.01f, &model), ATEAM_CONTROLS_OK);
  RobotPhysicalParams_t phys = test_phys_params();
  phys.mass = 3.0f;
  ASSERT_EQ(ateam_controls_robot_model_update_physical_params(model, phys), ATEAM_CONTROLS_OK);
  Matrix3x4C_t w2t = ateam_controls_robot_model_transform_wheel2twist(model, 0.0f);
  (void)w2t;
  ateam_controls_robot_model_free(model);
}

TEST(RobotModelBindings, TransformReturnsSomething) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(0.01f, &model), ATEAM_CONTROLS_OK);
  Matrix3x4C_t w2t = ateam_controls_robot_model_transform_wheel2twist(model, 0.0f);
  Matrix4x3C_t t2w = ateam_controls_robot_model_transform_twist2wheel(model, 0.0f);
  Matrix3x4C_t w2a = ateam_controls_robot_model_transform_wheel2accel(model, 0.0f);
  Matrix4x3C_t a2w = ateam_controls_robot_model_transform_accel2wheel(model, 0.0f);
  // Just verify the binding returned without crashing and has non-zero data
  bool any_nonzero = false;
  for (int i = 0; i < 12; i++) {
    if (w2t.data[i] != 0.0f) any_nonzero = true;
  }
  EXPECT_TRUE(any_nonzero);
  (void)t2w; (void)w2a; (void)a2w;
  ateam_controls_robot_model_free(model);
}

TEST(RobotModelBindings, KfPredictAndUpdate) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(0.01f, &model), ATEAM_CONTROLS_OK);
  Vector3C_t accel = {0.0f, 0.0f, 0.0f};
  ateam_controls_robot_model_kf_predict(model, accel);
  Vector8C_t meas = {};
  ASSERT_EQ(ateam_controls_robot_model_kf_update(model, meas, false, false, false), ATEAM_CONTROLS_OK);
  ateam_controls_robot_model_free(model);
}

TEST(RobotModelBindings, TorquesToCurrents) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(0.01f, &model), ATEAM_CONTROLS_OK);
  Vector4C_t torques = {1.0f, 1.0f, 1.0f, 1.0f};
  Vector4C_t currents = ateam_controls_robot_model_torques_to_currents(model, torques);
  EXPECT_GT(currents.x, 0.0f);
  ateam_controls_robot_model_free(model);
}
