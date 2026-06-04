#ifndef ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__
#define ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Error codes
// ============================================================================

#define ATEAM_CONTROLS_OK              0
#define ATEAM_CONTROLS_INVALID_INPUT  -1
#define ATEAM_CONTROLS_SINGULAR       -2
#define ATEAM_CONTROLS_NO_SOLUTION    -3
#define ATEAM_CONTROLS_INVALID_TIME   -4
#define ATEAM_CONTROLS_EXCEEDS_LIMIT  -5

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
    float coulomb_friction_coefficient_linear_x;
    float coulomb_friction_coefficient_linear_y;
    float coulomb_friction_coefficient_angular;
    float viscous_friction_coefficient_linear_x;
    float viscous_friction_coefficient_linear_y;
    float viscous_friction_coefficient_angular;
} RobotPhysicalParams_t;

// Opaque handle to the full RobotModel (includes KF state, physical matrices, etc.)
typedef struct RobotModel RobotModel_t;

typedef struct TrajectoryParams {
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
// Default parameter constructors
// ============================================================================

KalmanFilterParams_t ateam_controls_default_kf_params(void);
RobotPhysicalParams_t ateam_controls_default_phys_params(void);
TrajectoryParams_t ateam_controls_default_traj_params(void);

// ============================================================================
// RobotModel
// ============================================================================

int32_t ateam_controls_robot_model_new(
    float kf_dt,
    KalmanFilterParams_t kf_params,
    RobotPhysicalParams_t phys_params,
    RobotModel_t** out);
void ateam_controls_robot_model_free(RobotModel_t* model);

void ateam_controls_robot_model_update_kf_params(
    RobotModel_t* model,
    KalmanFilterParams_t kf_params);
int32_t ateam_controls_robot_model_update_physical_params(
    RobotModel_t* model,
    RobotPhysicalParams_t phys_params);

void ateam_controls_robot_model_kf_predict(RobotModel_t* model, Vector3C_t accel);
int32_t ateam_controls_robot_model_kf_update(
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

int32_t ateam_controls_traj_from_target_pose(
    Vector6C_t init_state,
    Vector3C_t target_pose,
    TrajectoryParams_t params,
    BangBangTraj3D_t* out);
int32_t ateam_controls_traj_from_target_twist(
    Vector3C_t init_twist,
    Vector3C_t target_twist,
    TrajectoryParams_t params,
    BangBangTraj3D_t* out);

void ateam_controls_traj_1d_time_shift(BangBangTraj1D_t* traj, float dt);
void ateam_controls_traj_3d_time_shift(BangBangTraj3D_t* traj, float dt);
float ateam_controls_traj_end_time(BangBangTraj3D_t traj);

int32_t ateam_controls_traj_state_at(
    BangBangTraj3D_t traj,
    Vector6C_t current_state,
    float current_time,
    float t,
    Vector6C_t* out);
int32_t ateam_controls_traj_accel_at(BangBangTraj3D_t traj, float t, Vector3C_t* out);

#ifdef __cplusplus
}
#endif

#endif  // ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__
