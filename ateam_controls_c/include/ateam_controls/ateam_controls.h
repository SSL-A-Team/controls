#ifndef ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__
#define ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__

#include <stdint.h>

extern "C" {

typedef struct Vector3 {
    double x;
    double y;
    double z;
} Vecotr3_t;

typedef struct Vector4 {
    double x;
    double y;
    double z;
    double w;
} Vector4_t;

typedef struct Pose {
    Vector3 position;
    Vector4 orientation;  // Quaternion
} Pose_t;

typedef struct Twist {
    Vector3 linear;
    Vector3 angular;
} Twist_t;

typedef struct Accel {
    Vector3 linear;
    Vector3 angular;
} Accel_t;

typedef struct Wrench {
    Vector3 force;
    Vector3 torque;
} Wrnech_t;

typedef struct RigidBodyState {
    Pose pose;
    Twist twist;
} RigidBodyState_t;

typedef struct WheelTorques {
    double torque_fl;
    double torque_bl;
    double torque_br;
    double torque_fr;
} WheelTorques_t;

typedef struct WheelVelocities {
    double velocity_fl;
    double velocity_bl;
    double velocity_br;
    double velocity_fr;
} WheelVelocities_t;

typedef struct BangBangTraj1D {
    double sdd1;
    double sdd2;
    double sdd3;
    double t1;
    double t2;
    double t3;
    double t4;
} BangBangTraj1D_t;

typedef struct BangBangTraj3D {
    BangBangTraj1D_t x_traj;
    BangBangTraj1D_t y_traj;
    BangBangTraj1D_t z_traj;
} BangBangTraj3D_t;

uint64_t ateam_controls_add(uint64_t left, uint64_t right);
BangBangTraj3D_t ateam_controls_compute_optimal_bangbang_traj_3d(RigidBodyState init_state, RigidBodyState target_state);
RigidBodyState_t ateam_controls_compute_bangbang_traj_3d_state_at_t(BangBangTraj3D_t traj, RigidBodyState_t current_state, double current_time, double t);

}

#endif  // ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__
