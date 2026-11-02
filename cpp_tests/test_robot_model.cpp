#include <gtest/gtest.h>
#include <ateam_controls/ateam_controls.h>
#include <cmath>

// ============================================================================
// Test helpers – construct params explicitly
// ============================================================================

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
  phys.alpha = 0.7854f;
  phys.beta = 0.7854f;
  phys.l = 0.08f;
  phys.r = 0.025f;
  phys.mass = 2.5f;
  phys.iz = 0.01f;
  phys.motor_torque_constant = 0.02f;
  phys.motor_efficiency_factor = 0.85f;

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
  phys.mass = 3.0f;
  ASSERT_EQ(ateam_controls_robot_model_update_physical_params(model, phys), ATEAM_CONTROLS_OK);
  Matrix3x4C_t w2t = ateam_controls_robot_model_transform_wheel2twist(model, 0.0f);
  (void)w2t;
  ateam_controls_robot_model_free(model);
}

TEST(RobotModelBindings, TransformReturnsSomething) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(&model), ATEAM_CONTROLS_OK);
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

TEST(RobotModelBindings, TorquesToCurrents) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(create_test_model(&model), ATEAM_CONTROLS_OK);
  Vector4C_t torques = {1.0f, 1.0f, 1.0f, 1.0f};
  Vector4C_t currents = ateam_controls_robot_model_torques_to_currents(model, torques);
  EXPECT_GT(currents.x, 0.0f);
  ateam_controls_robot_model_free(model);
}
