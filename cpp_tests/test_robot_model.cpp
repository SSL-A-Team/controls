#include <gtest/gtest.h>
#include <ateam_controls/ateam_controls.h>
#include <cmath>

// ============================================================================
// Test helpers – construct params explicitly
// ============================================================================

static RobotPhysicalParams_t test_phys_params() {
  RobotPhysicalParams_t phys = {};
  phys.alpha_rad = static_cast<float>(M_PI / 6.0);
  phys.beta_rad = static_cast<float>(M_PI / 4.0);
  phys.l_m = 0.0814f;
  phys.r_m = 0.030f;
  phys.mass_kg = 2.7f;
  phys.iz_kgm2 = 0.008f;
  phys.motor_torque_constant_nmpa = 0.0335f;
  phys.coulomb_xy_n = 2.65f;
  phys.viscous_xy_nspm = 0.5f;
  phys.coulomb_w_nm = 0.097f;
  phys.viscous_w_nmsprad = 0.002f;
  phys.efficiency_xy = 0.72f;
  phys.efficiency_w = 0.6f;
  return phys;
}

static int32_t create_test_model(RobotModel_t** out) {
  return ateam_controls_robot_model_new(test_phys_params(), out);
}

// ============================================================================
// Binding smoke tests – verify C FFI round-trips work correctly
// ============================================================================

TEST(RobotModelBindings, NewAndFree) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(&model), ATEAM_CONTROLS_OK);
  ASSERT_NE(model, nullptr);
  ateam_controls_robot_model_free(model);
}

TEST(RobotModelBindings, NewWithCustomParams) {
  RobotPhysicalParams_t phys = {};
  phys.alpha_rad = 0.7854f;
  phys.beta_rad = 0.7854f;
  phys.l_m = 0.08f;
  phys.r_m = 0.025f;
  phys.mass_kg = 2.5f;
  phys.iz_kgm2 = 0.01f;
  phys.motor_torque_constant_nmpa = 0.02f;
  phys.efficiency_xy = 1.0f;
  phys.efficiency_w = 1.0f;

  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new(phys, &model), ATEAM_CONTROLS_OK);
  ASSERT_NE(model, nullptr);
  ateam_controls_robot_model_free(model);
}

TEST(RobotModelBindings, FreeNull) {
  ateam_controls_robot_model_free(nullptr);
}

TEST(RobotModelBindings, UpdatePhysicalParams) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(&model), ATEAM_CONTROLS_OK);
  RobotPhysicalParams_t phys = test_phys_params();
  phys.mass_kg = 3.0f;
  ASSERT_EQ(ateam_controls_robot_model_update_params(model, phys), ATEAM_CONTROLS_OK);
  Matrix3x4C_t w2t = ateam_controls_robot_model_transform_wheel2vel_global(model, 0.0f);
  (void)w2t;
  ateam_controls_robot_model_free(model);
}

TEST(RobotModelBindings, TransformReturnsSomething) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(&model), ATEAM_CONTROLS_OK);
  Matrix3x4C_t w2t = ateam_controls_robot_model_transform_wheel2vel_global(model, 0.0f);
  Matrix4x3C_t t2w = ateam_controls_robot_model_transform_vel2wheel_global(model, 0.0f);
  Matrix3x4C_t w2a = ateam_controls_robot_model_transform_wheel2accel_global(model, 0.0f);
  Matrix4x3C_t a2w = ateam_controls_robot_model_transform_accel2wheel_global(model, 0.0f);
  Matrix3x4C_t lw2t = ateam_controls_robot_model_transform_wheel2vel_local(model);
  Matrix4x3C_t lt2w = ateam_controls_robot_model_transform_vel2wheel_local(model);
  Matrix3x4C_t lw2a = ateam_controls_robot_model_transform_wheel2accel_local(model);
  Matrix4x3C_t la2w = ateam_controls_robot_model_transform_accel2wheel_local(model);
  // Just verify the binding returned without crashing and has non-zero data
  bool any_nonzero = false;
  for (int i = 0; i < 12; i++) {
    if (w2t.data[i] != 0.0f) any_nonzero = true;
  }
  EXPECT_TRUE(any_nonzero);
  (void)t2w; (void)w2a; (void)a2w;
  (void)lw2t; (void)lt2w; (void)lw2a; (void)la2w;
  ateam_controls_robot_model_free(model);
}

TEST(RobotModelBindings, WheelToLocalVel) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(&model), ATEAM_CONTROLS_OK);
  // Pure rotation: every wheel spins the same way at the same rate
  Vector4C_t encoder = {10.0f, 10.0f, 10.0f, 10.0f};
  Vector3C_t vel = ateam_controls_robot_model_wheel_to_local_vel(model, encoder);
  EXPECT_NEAR(vel.x, 0.0f, 1e-4f);
  EXPECT_NEAR(vel.y, 0.0f, 1e-4f);
  EXPECT_GT(vel.z, 0.0f);
  // Pure rotation is frame independent
  Vector3C_t gvel = ateam_controls_robot_model_wheel_to_global_vel(model, encoder, 1.0f);
  EXPECT_NEAR(gvel.x, 0.0f, 1e-4f);
  EXPECT_NEAR(gvel.y, 0.0f, 1e-4f);
  EXPECT_NEAR(gvel.z, vel.z, 1e-5f);
  ateam_controls_robot_model_free(model);
}

TEST(RobotModelBindings, TorquesToCurrents) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(&model), ATEAM_CONTROLS_OK);
  Vector4C_t torques = {1.0f, 1.0f, 1.0f, 1.0f};
  Vector4C_t currents = ateam_controls_robot_model_torques_to_currents(model, torques);
  EXPECT_GT(currents.x, 0.0f);
  ateam_controls_robot_model_free(model);
}
