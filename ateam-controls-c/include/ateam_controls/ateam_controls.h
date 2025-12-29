#ifndef ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__
#define ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Types
// ============================================================================

typedef struct Vector3C {
    float x;
    float y;
    float z;
} Vector3C_t;

typedef struct Vector4C {
    float x;
    float y;
    float z;
    float w;
} Vector4C_t;

typedef struct Vector6C {
    float data[6];
} Vector6C_t;

typedef struct Vector8C {
    float data[8];
} Vector8C_t;

typedef struct Matrix3C {
    float data[9];
} Matrix3C_t;

typedef struct Matrix3x4C {
    float data[12];
} Matrix3x4C_t;

typedef struct Matrix4x3C {
    float data[12];
} Matrix4x3C_t;

typedef struct KalmanFilterParams {
    float process_noise_std_pos_linear;
    float process_noise_std_pos_angular;
    float process_noise_std_vel_linear;
    float process_noise_std_vel_angular;
    float measurement_noise_std_vision_pos_linear;
    float measurement_noise_std_vision_pos_angular;
    float measurement_noise_std_encoder_vel_angular;
    float measurement_noise_std_gyro_vel_angular;
    float max_pos_linear;
    float max_pos_angular;
    float max_vel_linear;
    float max_vel_angular;
} KalmanFilterParams_t;

typedef struct RobotPhysicalParams {
    float alpha;
    float beta;
    float l;
    float r;
    float mass;
    float iz;
    float motor_torque_constant;
    float motor_efficiency_factor;
} RobotPhysicalParams_t;

// Opaque handle to the full RobotModel (includes KF state, physical matrices, etc.)
typedef struct RobotModel RobotModel_t;

typedef struct TrajectoryParams {
    float allowable_error_pos_linear;
    float allowable_error_pos_angular;
    float allowable_error_vel_linear;
    float allowable_error_vel_angular;
    float max_vel_linear;
    float max_vel_angular;
    float max_accel_linear;
    float max_accel_angular;
} TrajectoryParams_t;

typedef struct BangBangTraj1D {
    float sdd1;
    float sdd2;
    float sdd3;
    float t1;
    float t2;
    float t3;
    float t4;
} BangBangTraj1D_t;

typedef struct BangBangTraj3D {
    BangBangTraj1D_t x;
    BangBangTraj1D_t y;
    BangBangTraj1D_t z;
} BangBangTraj3D_t;

// ============================================================================
// Utility
// ============================================================================

uint64_t ateam_controls_add(uint64_t left, uint64_t right);

// ============================================================================
// RobotModel
// ============================================================================

RobotModel_t* ateam_controls_robot_model_new(
    float kf_dt,
    KalmanFilterParams_t kf_params,
    RobotPhysicalParams_t phys_params);
RobotModel_t* ateam_controls_robot_model_new_default(float kf_dt);
void ateam_controls_robot_model_free(RobotModel_t* model);

void ateam_controls_robot_model_update_kf_params(
    RobotModel_t* model,
    KalmanFilterParams_t kf_params);
void ateam_controls_robot_model_update_physical_params(
    RobotModel_t* model,
    RobotPhysicalParams_t phys_params);

void ateam_controls_robot_model_kf_predict(RobotModel_t* model, Vector3C_t accel);
void ateam_controls_robot_model_kf_update(
    RobotModel_t* model,
    Vector8C_t measurement,
    bool mask_vision,
    bool mask_encoder,
    bool mask_gyro);

Vector6C_t ateam_controls_robot_model_get_state(const RobotModel_t* model);
void ateam_controls_robot_model_set_state(RobotModel_t* model, Vector6C_t state);

Matrix3x4C_t ateam_controls_robot_model_transform_wheel2twist(
    const RobotModel_t* model, float theta);
Matrix4x3C_t ateam_controls_robot_model_transform_twist2wheel(
    const RobotModel_t* model, float theta);
Matrix3x4C_t ateam_controls_robot_model_transform_wheel2accel(
    const RobotModel_t* model, float theta);
Matrix4x3C_t ateam_controls_robot_model_transform_accel2wheel(
    const RobotModel_t* model, float theta);

Vector4C_t ateam_controls_robot_model_torques_to_currents(
    const RobotModel_t* model, Vector4C_t torques);

// ============================================================================
// BangBang Trajectory
// ============================================================================

TrajectoryParams_t ateam_controls_traj_params_default(void);

BangBangTraj3D_t ateam_controls_traj_from_target_pose(
    Vector6C_t init_state,
    Vector3C_t target_pose,
    TrajectoryParams_t params);
BangBangTraj3D_t ateam_controls_traj_from_target_twist(
    Vector3C_t init_twist,
    Vector3C_t target_twist,
    TrajectoryParams_t params);

void ateam_controls_traj_1d_time_shift(BangBangTraj1D_t* traj, float dt);
void ateam_controls_traj_3d_time_shift(BangBangTraj3D_t* traj, float dt);
float ateam_controls_traj_end_time(BangBangTraj3D_t traj);

Vector6C_t ateam_controls_traj_state_at(
    BangBangTraj3D_t traj,
    Vector6C_t current_state,
    float current_time,
    float t);
Vector3C_t ateam_controls_traj_accel_at(BangBangTraj3D_t traj, float t);

#ifdef __cplusplus
}
#endif

#endif  // ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__
