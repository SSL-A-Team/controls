#ifndef ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__
#define ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__

#include <stdint.h>

extern "C" {

typedef struct GlobalState {
    double x;
    double y;
    double z;
    double xd;
    double yd;
    double zd;
} GlobalState_t;

typedef struct GlobalControl2Order {
    double xdd;
    double ydd;
    double zdd;
} GlobalControl2Order_t;

typedef struct GlobalControl1Order {
    double xd;
    double yd;
    double zd;
} GlobalControl1Order_t;

typedef struct WheelTorques {
    double torque_fl;
    double torque_bl;
    double torque_br;
    double torque_fr;
} WheelTorques_t;

typedef struct WheelVelocities {
    double velocity_fl;
    double velocity_bl;
    double veloci;
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
BangBangTraj3D_t ateam_controls_compute_optimal_bang_bang_traj_3d(GlobalState_t init_state, GlobalState_t target_state);
GlobalState_t ateam_controls_compute_bang_bang_traj_3d_state_at_t(BangBangTraj3D_t traj, GlobalState_t current_state, double current_time, double t);


}

#endif  // ATEAM_CONTROLS_C__ATEAM_CONTROLS_H__
