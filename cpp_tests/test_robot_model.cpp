#include <gtest/gtest.h>
#include <ateam_controls/ateam_controls.h>
#include <cmath>

// Test global to robot pose transform at origin with no rotation
TEST(RobotModel, TransformGlobal2RobotPoseAtOrigin) {
  Vector3C_t robot_pose = {0.0f, 0.0f, 0.0f};
  Vector3C_t global_pose = {1.0f, 0.0f, 0.0f};
  Vector3C_t robot_frame_pose = ateam_controls_transform_frame_global2robot_pose(robot_pose, global_pose);
  EXPECT_NEAR(robot_frame_pose.x, 1.0f, 1e-6f);
  EXPECT_NEAR(robot_frame_pose.y, 0.0f, 1e-6f);
  EXPECT_NEAR(robot_frame_pose.z, 0.0f, 1e-6f);
}

// Test robot to global pose transform at origin with no rotation
TEST(RobotModel, TransformRobot2GlobalPoseAtOrigin) {
  Vector3C_t robot_pose = {0.0f, 0.0f, 0.0f};
  Vector3C_t robot_frame_pose = {1.0f, 0.0f, 0.0f};
  Vector3C_t global_pose = ateam_controls_transform_frame_robot2global_pose(robot_pose, robot_frame_pose);
  EXPECT_NEAR(global_pose.x, 1.0f, 1e-6f);
  EXPECT_NEAR(global_pose.y, 0.0f, 1e-6f);
  EXPECT_NEAR(global_pose.z, 0.0f, 1e-6f);
}

// Test that pose transforms are inverses of each other
TEST(RobotModel, PoseTransformsAreInverses) {
  Vector3C_t robot_pose = {1.0f, 2.0f, 0.0f};
  Vector3C_t global_pose = {3.0f, 4.0f, 0.0f};
  
  Vector3C_t to_robot = ateam_controls_transform_frame_global2robot_pose(robot_pose, global_pose);
  Vector3C_t back_to_global = ateam_controls_transform_frame_robot2global_pose(robot_pose, to_robot);
  
  EXPECT_NEAR(back_to_global.x, global_pose.x, 1e-5f);
  EXPECT_NEAR(back_to_global.y, global_pose.y, 1e-5f);
  EXPECT_NEAR(back_to_global.z, global_pose.z, 1e-5f);
}

// Test global to robot twist transform at origin with no rotation
TEST(RobotModel, TransformGlobal2RobotTwistNoRotation) {
  Vector3C_t robot_pose = {0.0f, 0.0f, 0.0f};
  Vector3C_t global_twist = {1.0f, 0.0f, 0.0f};
  Vector3C_t robot_twist = ateam_controls_transform_frame_global2robot_twist(robot_pose, global_twist);
  EXPECT_NEAR(robot_twist.x, 1.0f, 1e-6f);
  EXPECT_NEAR(robot_twist.y, 0.0f, 1e-6f);
  EXPECT_NEAR(robot_twist.z, 0.0f, 1e-6f);
}

// Test robot to global twist transform at origin with no rotation
TEST(RobotModel, TransformRobot2GlobalTwistNoRotation) {
  Vector3C_t robot_pose = {0.0f, 0.0f, 0.0f};
  Vector3C_t robot_twist = {1.0f, 0.0f, 0.0f};
  Vector3C_t global_twist = ateam_controls_transform_frame_robot2global_twist(robot_pose, robot_twist);
  EXPECT_NEAR(global_twist.x, 1.0f, 1e-6f);
  EXPECT_NEAR(global_twist.y, 0.0f, 1e-6f);
  EXPECT_NEAR(global_twist.z, 0.0f, 1e-6f);
}

// Test that twist transforms are inverses of each other
TEST(RobotModel, TwistTransformsAreInverses) {
  Vector3C_t robot_pose = {0.0f, 0.0f, 0.0f};
  Vector3C_t global_twist = {1.0f, 2.0f, 0.0f};
  
  Vector3C_t to_robot = ateam_controls_transform_frame_global2robot_twist(robot_pose, global_twist);
  Vector3C_t back_to_global = ateam_controls_transform_frame_robot2global_twist(robot_pose, to_robot);
  
  EXPECT_NEAR(back_to_global.x, global_twist.x, 1e-5f);
  EXPECT_NEAR(back_to_global.y, global_twist.y, 1e-5f);
  EXPECT_NEAR(back_to_global.z, global_twist.z, 1e-5f);
}

// Test global to robot accel transform at origin with no rotation
TEST(RobotModel, TransformGlobal2RobotAccelNoRotation) {
  Vector3C_t robot_pose = {0.0f, 0.0f, 0.0f};
  Vector3C_t global_accel = {1.0f, 0.0f, 0.0f};
  Vector3C_t robot_accel = ateam_controls_transform_frame_global2robot_accel(robot_pose, global_accel);
  EXPECT_NEAR(robot_accel.x, 1.0f, 1e-6f);
  EXPECT_NEAR(robot_accel.y, 0.0f, 1e-6f);
  EXPECT_NEAR(robot_accel.z, 0.0f, 1e-6f);
}

// Test robot to global accel transform at origin with no rotation
TEST(RobotModel, TransformRobot2GlobalAccelNoRotation) {
  Vector3C_t robot_pose = {0.0f, 0.0f, 0.0f};
  Vector3C_t robot_accel = {1.0f, 0.0f, 0.0f};
  Vector3C_t global_accel = ateam_controls_transform_frame_robot2global_accel(robot_pose, robot_accel);
  EXPECT_NEAR(global_accel.x, 1.0f, 1e-6f);
  EXPECT_NEAR(global_accel.y, 0.0f, 1e-6f);
  EXPECT_NEAR(global_accel.z, 0.0f, 1e-6f);
}

// Test that accel transforms are inverses of each other
TEST(RobotModel, AccelTransformsAreInverses) {
  Vector3C_t robot_pose = {0.0f, 0.0f, 0.0f};
  Vector3C_t global_accel = {1.0f, 2.0f, 0.0f};
  
  Vector3C_t to_robot = ateam_controls_transform_frame_global2robot_accel(robot_pose, global_accel);
  Vector3C_t back_to_global = ateam_controls_transform_frame_robot2global_accel(robot_pose, to_robot);
  
  EXPECT_NEAR(back_to_global.x, global_accel.x, 1e-5f);
  EXPECT_NEAR(back_to_global.y, global_accel.y, 1e-5f);
  EXPECT_NEAR(back_to_global.z, global_accel.z, 1e-5f);
}

// Test robot model initialization
TEST(RobotModel, RobotModelNewFromConstants) {
  RobotModelC_t robot_model = ateam_controls_robot_model_new_from_constants();
  EXPECT_GT(robot_model.wheel_radius, 0.0f);
}

// Test wheel velocities to twist conversion and back
TEST(RobotModel, WheelVelocitiesToTwistAndBack) {
  RobotModelC_t robot_model = ateam_controls_robot_model_new_from_constants();
  Vector3C_t twist = {1.0f, 0.0f, 0.0f};
  
  Vector4C wheel_velocities = ateam_controls_robot_model_twist_to_wheel_velocities(robot_model, twist);
  Vector3C_t twist_back = ateam_controls_robot_model_wheel_velocities_to_twist(robot_model, wheel_velocities);
  
  EXPECT_NEAR(twist_back.x, twist.x, 1e-4f);
  EXPECT_NEAR(twist_back.y, twist.y, 1e-4f);
  EXPECT_NEAR(twist_back.z, twist.z, 1e-4f);
}

// Test accel to wheel torques conversion and back
TEST(RobotModel, AccelToWheelTorquesAndBack) {
  RobotModelC_t robot_model = ateam_controls_robot_model_new_from_constants();
  Vector3C_t accel = {1.0f, 0.0f, 0.0f};
  
  Vector4C torques = ateam_controls_robot_model_accel_to_wheel_torques(robot_model, accel);
  Vector3C_t accel_back = ateam_controls_robot_model_wheel_torques_to_accel(robot_model, torques);
  
  EXPECT_NEAR(accel_back.x, accel.x, 1e-4f);
  EXPECT_NEAR(accel_back.y, accel.y, 1e-4f);
  EXPECT_NEAR(accel_back.z, accel.z, 1e-4f);
}
