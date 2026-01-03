// #include <gtest/gtest.h>
// #include <ateam_controls/ateam_controls.h>

// TEST(AteamControls, BangBangX) {
//   RigidBodyStateC_t init_state = {
//     .pose = {0.0f, 0.0f, 0.0f},
//     .twist = {0.0f, 0.0f, 0.0f}
//   };
//   Vector3C_t target_pose = {
//     .x = 1.0f,
//     .y = 0.0f,
//     .z = 0.0f
//   };
//   BangBangTraj3D_t traj = ateam_controls_compute_optimal_bangbang_traj_3d(init_state, target_pose);
//   float end_time = ateam_controls_bangbang_traj_3d_get_end_time(traj);
//   RigidBodyStateC_t final_state = ateam_controls_compute_bangbang_traj_3d_state_at_t(traj, init_state, 0.0f, end_time);
//   EXPECT_NEAR(final_state.pose.x, target_pose.x, 1e-6f);
//   EXPECT_NEAR(final_state.pose.y, target_pose.y, 1e-6f);
//   EXPECT_NEAR(final_state.pose.z, target_pose.z, 1e-6f);
// }

// // Test Y direction bang-bang trajectory achieves the target pose
// TEST(AteamControls, BangBangY) {
//   RigidBodyStateC_t init_state = {
//     .pose = {0.0f, 0.0f, 0.0f},
//     .twist = {0.0f, 0.0f, 0.0f}
//   };
//   Vector3C_t target_pose = {
//     .x = 0.0f,
//     .y = 1.0f,
//     .z = 0.0f
//   };
//   BangBangTraj3D_t traj = ateam_controls_compute_optimal_bangbang_traj_3d(init_state, target_pose);
//   float end_time = ateam_controls_bangbang_traj_3d_get_end_time(traj);
//   RigidBodyStateC_t final_state = ateam_controls_compute_bangbang_traj_3d_state_at_t(traj, init_state, 0.0f, end_time);
//   EXPECT_NEAR(final_state.pose.x, target_pose.x, 1e-6f);
//   EXPECT_NEAR(final_state.pose.y, target_pose.y, 1e-6f);
//   EXPECT_NEAR(final_state.pose.z, target_pose.z, 1e-6f);
// }

// // Test Z direction bang-bang trajectory meets the target pose
// TEST(AteamControls, BangBangZ) {
//   RigidBodyStateC_t init_state = {
//     .pose = {0.0f, 0.0f, 0.0f},
//     .twist = {0.0f, 0.0f, 0.0f}
//   };
//   Vector3C_t target_pose = {
//     .x = 0.0f,
//     .y = 0.0f,
//     .z = 1.0f
//   };
//   BangBangTraj3D_t traj = ateam_controls_compute_optimal_bangbang_traj_3d(init_state, target_pose);
//   float end_time = ateam_controls_bangbang_traj_3d_get_end_time(traj);
//   RigidBodyStateC_t final_state = ateam_controls_compute_bangbang_traj_3d_state_at_t(traj, init_state, 0.0f, end_time);
//   EXPECT_NEAR(final_state.pose.x, target_pose.x, 1e-6f);
//   EXPECT_NEAR(final_state.pose.y, target_pose.y, 1e-6f);
//   EXPECT_NEAR(final_state.pose.z, target_pose.z, 1e-6f);
// }

// // Test symmetric XYZ bang-bang trajectory meets the target pose
// TEST(AteamControls, BangBangXYZ) {
//   RigidBodyStateC_t init_state = {
//     .pose = {0.0f, 0.0f, 0.0f},
//     .twist = {0.0f, 0.0f, 0.0f}
//   };
//   Vector3C_t target_pose = {
//     .x = 1.0f,
//     .y = 1.0f,
//     .z = 1.0f
//   };
//   BangBangTraj3D_t traj = ateam_controls_compute_optimal_bangbang_traj_3d(init_state, target_pose);
//   float end_time = ateam_controls_bangbang_traj_3d_get_end_time(traj);
//   RigidBodyStateC_t final_state = ateam_controls_compute_bangbang_traj_3d_state_at_t(traj, init_state, 0.0f, end_time);
//   EXPECT_NEAR(final_state.pose.x, target_pose.x, 1e-6f);
//   EXPECT_NEAR(final_state.pose.y, target_pose.y, 1e-6f);
//   EXPECT_NEAR(final_state.pose.z, target_pose.z, 1e-6f);
// }

// // Test that the initial and final accelerations are equal and opposite
// TEST(AteamControls, BangBangAcceleration) {
//   RigidBodyStateC_t init_state = {
//     .pose = {0.0f, 0.0f, 0.0f},
//     .twist = {0.0f, 0.0f, 0.0f}
//   };
//   Vector3C_t target_pose = {
//     .x = 1.0f,
//     .y = 0.0f,
//     .z = 1.0f
//   };
//   BangBangTraj3D_t traj = ateam_controls_compute_optimal_bangbang_traj_3d(init_state, target_pose);
//   EXPECT_NEAR(traj.x_traj.sdd1, -traj.x_traj.sdd3, 1e-6f);
// }

// // Test that time shifting a trajectory works
// TEST(AteamControls, BangBangTimeShift) {
//   RigidBodyStateC_t init_state = {
//     .pose = {0.0f, 0.0f, 0.0f},
//     .twist = {0.0f, 0.0f, 0.0f}
//   };
//   Vector3C_t target_pose = {
//     .x = 1.0f,
//     .y = 0.0f,
//     .z = 0.0f
//   };
//   BangBangTraj3D_t traj = ateam_controls_compute_optimal_bangbang_traj_3d(init_state, target_pose);
//   float shift_amount = 1.0f;
//   float initial_end_time = ateam_controls_bangbang_traj_3d_get_end_time(traj);
//   ateam_controls_bangbang_traj_3d_time_shift(&traj, shift_amount);
//   float shifted_end_time = ateam_controls_bangbang_traj_3d_get_end_time(traj);
//   EXPECT_NEAR(shifted_end_time - initial_end_time, shift_amount, 1e-6f);
// }
