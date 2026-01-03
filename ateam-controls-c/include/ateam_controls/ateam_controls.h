#ifndef ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__
#define ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__

#include <stdint.h>

extern "C" {

// typedef struct Vector3C {
//     float x;
//     float y;
//     float z;
// } Vector3C_t;

// typedef struct Vector4C {
//     float x;
//     float y;
//     float z;
//     float w;
// } Vector4C_t;

// typedef struct Matrix3C {
//     float data[9];
// } Matrix3C_t;

// typedef struct Matrix3x4C {
//     float data[12];
// } Matrix3x4C_t;

// typedef struct Matrix4x3C {
//     float data[12];
// } Matrix4x3C_t;

// typedef struct RigidBodyStateC {
//     Vector3C pose;
//     Vector3C twist;
// } RigidBodyStateC_t;

// typedef struct RobotModelC {
//     Matrix3x4C wheel_transform_mat;
//     Matrix4x3C wheel_transform_mat_inv;
//     float wheel_radius;
//     Matrix3C body_inirtia;
//     Matrix3C body_inirtia_inv;
// } RobotModelC_t;

// typedef struct BangBangTraj1D {
//     float sdd1;
//     float sdd2;
//     float sdd3;
//     float t1;
//     float t2;
//     float t3;
//     float t4;
// } BangBangTraj1D_t;

// typedef struct BangBangTraj3D {
//     BangBangTraj1D_t x_traj;
//     BangBangTraj1D_t y_traj;
//     BangBangTraj1D_t z_traj;
// } BangBangTraj3D_t;

uint64_t ateam_controls_add(uint64_t left, uint64_t right);
// Vector3C_t ateam_controls_transform_frame_global2robot_pose(Vector3C_t robot_pose, Vector3C_t pose);
// Vector3C_t ateam_controls_transform_frame_robot2global_pose(Vector3C_t robot_pose, Vector3C_t pose);
// Vector3C_t ateam_controls_transform_frame_global2robot_twist(Vector3C_t robot_pose, Vector3C_t twist);
// Vector3C_t ateam_controls_transform_frame_robot2global_twist(Vector3C_t robot_pose, Vector3C_t twist);
// Vector3C_t ateam_controls_transform_frame_global2robot_accel(Vector3C_t robot_pose, Vector3C_t accel);
// Vector3C_t ateam_controls_transform_frame_robot2global_accel(Vector3C_t robot_pose, Vector3C_t accel);
// RobotModelC_t ateam_controls_robot_model_new_from_constants();
// Vector3C_t ateam_controls_robot_model_wheel_velocities_to_twist(RobotModelC_t robot_model, Vector4C wheel_velocities);
// Vector4C ateam_controls_robot_model_twist_to_wheel_velocities(RobotModelC_t robot_model, Vector3C_t twist);
// Vector3C_t ateam_controls_robot_model_wheel_torques_to_accel(RobotModelC_t robot_model, Vector4C torques);
// Vector4C ateam_controls_robot_model_accel_to_wheel_torques(RobotModelC_t robot_model, Vector3C_t accel);
// void ateam_controls_bangbang_traj_1d_time_shift(BangBangTraj1D_t* traj, float dt);
// void ateam_controls_bangbang_traj_3d_time_shift(BangBangTraj3D_t* traj, float dt);
// float ateam_controls_bangbang_traj_3d_get_end_time(BangBangTraj3D_t traj);
// BangBangTraj3D_t ateam_controls_compute_optimal_bangbang_traj_3d(RigidBodyStateC_t init_state, Vector3C_t target_pose);
// RigidBodyStateC_t ateam_controls_compute_bangbang_traj_3d_state_at_t(BangBangTraj3D_t traj, RigidBodyStateC_t current_state, float current_time, float t);
// Vector3C_t ateam_controls_compute_bangbang_traj_3d_accel_at_t(BangBangTraj3D_t traj, float t);
// float ateam_controls_compute_bangbang_traj_1d_accel_at_t(BangBangTraj1D_t traj, float t);

}

#endif  // ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__
