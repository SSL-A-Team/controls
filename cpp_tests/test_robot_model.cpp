#include <gtest/gtest.h>
#include <ateam_controls/ateam_controls.h>
#include <cmath>

// ============================================================================
// RobotModel lifecycle
// ============================================================================

TEST(RobotModel, NewDefaultAndFree) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
  ASSERT_NE(model, nullptr);
  ateam_controls_robot_model_free(model);
}

TEST(RobotModel, NewWithParamsAndFree) {
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
  kf.max_vel_angular = 20.0f;

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

TEST(RobotModel, FreeNull) {
  // Should not crash
  ateam_controls_robot_model_free(nullptr);
}

// ============================================================================
// Get / Set state
// ============================================================================

TEST(RobotModel, GetStateInitiallyZero) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
  Vector6C_t state = ateam_controls_robot_model_get_state(model);
  for (int i = 0; i < 6; i++) {
    EXPECT_NEAR(state.data[i], 0.0f, 1e-9f);
  }
  ateam_controls_robot_model_free(model);
}

TEST(RobotModel, SetAndGetState) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
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

// ============================================================================
// Transform matrices – wheel2twist and twist2wheel are pseudo-inverses
// ============================================================================

TEST(RobotModel, Wheel2TwistAndBack) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
  float theta = 0.0f;
  Matrix3x4C_t w2t = ateam_controls_robot_model_transform_wheel2twist(model, theta);
  Matrix4x3C_t t2w = ateam_controls_robot_model_transform_twist2wheel(model, theta);

  // Multiply w2t * t2w -> should approximate 3x3 identity
  float result[9] = {};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        // w2t is column-major 3x4, t2w is column-major 4x3
        float a = w2t.data[k * 3 + i]; // w2t(i, k)
        float b = t2w.data[j * 4 + k]; // t2w(k, j)
        sum += a * b;
      }
      result[j * 3 + i] = sum;
    }
  }
  // Check ~identity
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_NEAR(result[j * 3 + i], expected, 1e-4f)
        << "w2t * t2w [" << i << "," << j << "]";
    }
  }
  ateam_controls_robot_model_free(model);
}

// ============================================================================
// Transform matrices – wheel2accel and accel2wheel are pseudo-inverses
// ============================================================================

TEST(RobotModel, Wheel2AccelAndBack) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
  float theta = 0.0f;
  Matrix3x4C_t w2a = ateam_controls_robot_model_transform_wheel2accel(model, theta);
  Matrix4x3C_t a2w = ateam_controls_robot_model_transform_accel2wheel(model, theta);

  // Multiply w2a * a2w -> should approximate 3x3 identity
  float result[9] = {};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        float a = w2a.data[k * 3 + i]; // w2a(i, k)
        float b = a2w.data[j * 4 + k]; // a2w(k, j)
        sum += a * b;
      }
      result[j * 3 + i] = sum;
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_NEAR(result[j * 3 + i], expected, 1e-4f)
        << "w2a * a2w [" << i << "," << j << "]";
    }
  }
  ateam_controls_robot_model_free(model);
}

// ============================================================================
// Transform consistency at different angles
// ============================================================================

TEST(RobotModel, TransformConsistencyAtAngle) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
  float theta = static_cast<float>(M_PI / 4.0);

  Matrix3x4C_t w2t = ateam_controls_robot_model_transform_wheel2twist(model, theta);
  Matrix4x3C_t t2w = ateam_controls_robot_model_transform_twist2wheel(model, theta);

  // w2t * t2w should still approximate identity
  float result[9] = {};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        sum += w2t.data[k * 3 + i] * t2w.data[j * 4 + k];
      }
      result[j * 3 + i] = sum;
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_NEAR(result[j * 3 + i], expected, 1e-3f)
        << "w2t * t2w at pi/4 [" << i << "," << j << "]";
    }
  }
  ateam_controls_robot_model_free(model);
}

// ============================================================================
// Torques to currents – should scale by motor constants
// ============================================================================

TEST(RobotModel, TorquesToCurrents) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
  Vector4C_t torques = {1.0f, 1.0f, 1.0f, 1.0f};
  Vector4C_t currents = ateam_controls_robot_model_torques_to_currents(model, torques);

  // currents = torques / (Kt * efficiency)
  // All torques equal => all currents equal and positive
  EXPECT_GT(currents.x, 0.0f);
  EXPECT_NEAR(currents.x, currents.y, 1e-6f);
  EXPECT_NEAR(currents.x, currents.z, 1e-6f);
  EXPECT_NEAR(currents.x, currents.w, 1e-6f);

  // Zero torque => zero current
  Vector4C_t zero_torques = {0.0f, 0.0f, 0.0f, 0.0f};
  Vector4C_t zero_currents = ateam_controls_robot_model_torques_to_currents(model, zero_torques);
  EXPECT_NEAR(zero_currents.x, 0.0f, 1e-9f);
  EXPECT_NEAR(zero_currents.y, 0.0f, 1e-9f);
  EXPECT_NEAR(zero_currents.z, 0.0f, 1e-9f);
  EXPECT_NEAR(zero_currents.w, 0.0f, 1e-9f);

  ateam_controls_robot_model_free(model);
}

TEST(RobotModel, TorquesToCurrentsLinearity) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
  Vector4C_t t1 = {1.0f, 0.0f, 0.0f, 0.0f};
  Vector4C_t t2 = {2.0f, 0.0f, 0.0f, 0.0f};
  Vector4C_t c1 = ateam_controls_robot_model_torques_to_currents(model, t1);
  Vector4C_t c2 = ateam_controls_robot_model_torques_to_currents(model, t2);

  // Doubling torque doubles current
  EXPECT_NEAR(c2.x, 2.0f * c1.x, 1e-6f);
  ateam_controls_robot_model_free(model);
}

// ============================================================================
// KF predict – state changes with accel input
// ============================================================================

TEST(RobotModel, KfPredictUpdatesState) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
  // Set known state
  Vector6C_t state = {};
  state.data[0] = 1.0f;  // x pos
  state.data[3] = 0.5f;  // x vel
  ateam_controls_robot_model_set_state(model, state);

  // Predict with zero accel – position should change by vel*dt
  Vector3C_t accel = {0.0f, 0.0f, 0.0f};
  ateam_controls_robot_model_kf_predict(model, accel);

  Vector6C_t new_state = ateam_controls_robot_model_get_state(model);
  // x_new ≈ x + dx*dt = 1.0 + 0.5*0.01 = 1.005
  EXPECT_NEAR(new_state.data[0], 1.005f, 1e-4f);
  // velocity unchanged with zero accel
  EXPECT_NEAR(new_state.data[3], 0.5f, 1e-4f);

  ateam_controls_robot_model_free(model);
}

TEST(RobotModel, KfPredictWithAccel) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
  Vector6C_t state = {};
  ateam_controls_robot_model_set_state(model, state);

  Vector3C_t accel = {1.0f, 0.0f, 0.0f};
  ateam_controls_robot_model_kf_predict(model, accel);

  Vector6C_t new_state = ateam_controls_robot_model_get_state(model);
  // vel_x should be accel * dt = 1.0 * 0.01 = 0.01
  EXPECT_NEAR(new_state.data[3], 0.01f, 1e-4f);

  ateam_controls_robot_model_free(model);
}

// ============================================================================
// KF update – state moves toward measurement
// ============================================================================

TEST(RobotModel, KfUpdateMovesTowardMeasurement) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
  // State at origin
  Vector6C_t state = {};
  ateam_controls_robot_model_set_state(model, state);

  // Run a predict step first (required for meaningful P matrix)
  Vector3C_t accel = {0.0f, 0.0f, 0.0f};
  ateam_controls_robot_model_kf_predict(model, accel);

  // Measurement says position is at (1, 0, 0), with vision unmasked
  Vector8C_t measurement = {};
  measurement.data[0] = 1.0f;  // vision x
  // Unmask vision, mask encoder and gyro
  ASSERT_EQ(ateam_controls_robot_model_kf_update(model, measurement, false, true, true), ATEAM_CONTROLS_OK);

  Vector6C_t updated = ateam_controls_robot_model_get_state(model);
  // x should have moved toward 1.0
  EXPECT_GT(updated.data[0], 0.0f);

  ateam_controls_robot_model_free(model);
}

// ============================================================================
// Update params – should not crash
// ============================================================================

TEST(RobotModel, UpdateKfParams) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
  KalmanFilterParams_t kf = {};
  kf.process_noise_std_pos_linear = 0.02f;
  kf.process_noise_std_pos_angular = 0.02f;
  kf.process_noise_std_vel_linear = 0.2f;
  kf.process_noise_std_vel_angular = 0.2f;
  kf.measurement_noise_std_vision_pos_linear = 0.02f;
  kf.measurement_noise_std_vision_pos_angular = 0.02f;
  kf.measurement_noise_std_encoder_vel_angular = 0.2f;
  kf.measurement_noise_std_gyro_vel_angular = 0.2f;
  kf.max_pos_linear = 64.0f;
  kf.max_pos_angular = 6.2832f;
  kf.max_vel_linear = 4.0f;
  kf.max_vel_angular = 20.0f;
  ateam_controls_robot_model_update_kf_params(model, kf);
  // Verify model still works after param update
  Vector6C_t state = ateam_controls_robot_model_get_state(model);
  (void)state;
  ateam_controls_robot_model_free(model);
}

TEST(RobotModel, UpdatePhysicalParams) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
  RobotPhysicalParams_t phys = {};
  phys.alpha = 0.7854f;
  phys.beta = 0.7854f;
  phys.l = 0.08f;
  phys.r = 0.03f;
  phys.mass = 3.0f;
  phys.iz = 0.015f;
  phys.motor_torque_constant = 0.025f;
  phys.motor_efficiency_factor = 0.9f;
  ASSERT_EQ(ateam_controls_robot_model_update_physical_params(model, phys), ATEAM_CONTROLS_OK);
  // Transforms should still work after updating physical params
  Matrix3x4C_t w2t = ateam_controls_robot_model_transform_wheel2twist(model, 0.0f);
  (void)w2t;
  ateam_controls_robot_model_free(model);
}

// ============================================================================
// Multiple predict-update cycles – smoke test
// ============================================================================

TEST(RobotModel, MultiplePredictUpdateCycles) {
  RobotModel_t* model = nullptr;
  ASSERT_EQ(ateam_controls_robot_model_new_default(0.01f, &model), ATEAM_CONTROLS_OK);
  Vector3C_t accel = {0.0f, 0.0f, 0.0f};
  Vector8C_t meas = {};
  meas.data[0] = 1.0f;
  meas.data[1] = 0.5f;

  for (int i = 0; i < 100; i++) {
    ateam_controls_robot_model_kf_predict(model, accel);
    ASSERT_EQ(ateam_controls_robot_model_kf_update(model, meas, false, true, true), ATEAM_CONTROLS_OK);
  }

  Vector6C_t state = ateam_controls_robot_model_get_state(model);
  // After many updates with vision measurement at (1, 0.5, 0), state should converge
  EXPECT_NEAR(state.data[0], 1.0f, 0.1f);
  EXPECT_NEAR(state.data[1], 0.5f, 0.1f);

  ateam_controls_robot_model_free(model);
}
