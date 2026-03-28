use ateam_controls::{Matrix6f, Matrix6x3f, Vector3f, Vector4f, Vector6f, wrap_angle};
use ateam_controls::robot_model::RobotModel;
use ateam_controls::bangbang_trajectory::{BangBangTraj3D, TrajectoryParams};
use nalgebra::matrix;
use libm::{cosf, sinf};
use core::f32::consts::PI;
use core::net;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let control_dt = 0.001;
    let mut model = RobotModel::new_from_default_params(control_dt)?;

    // let global_accel_cmd = Vector3f::new(1., 0., 0.);
    // let wheel_torques = model.transform_accel2wheel(0.) * global_accel_cmd;
    let wheel_currents = Vector4f::new(-0.17, -0.135, 0.135, 0.17);
    let wheel_torques = wheel_currents * model.physical_params.motor_torque_constant;

    let traction_forces = wheel_torques / model.r;

    let fl_force_vec = Vector3f::new(-cosf(PI / 6.), sinf(PI / 6.), 1.) * traction_forces.x;
    let bl_force_vec = Vector3f::new(-cosf(PI / 4.), -sinf(PI / 4.), 1.) * traction_forces.y;
    let br_force_vec = Vector3f::new(cosf(PI / 4.), -sinf(PI / 4.), 1.) * traction_forces.z;
    let fr_force_vec = Vector3f::new(cosf(PI / 6.), sinf(PI / 6.), 1.) * traction_forces.w;

    let net_force = fl_force_vec + bl_force_vec + br_force_vec + fr_force_vec;
    let net_accel = net_force / model.i[(0, 0)];

    println!("wheel torques: {}", wheel_torques);
    println!("net force: {}", net_force);
    println!("net accel: {}", net_accel);

    Ok(())
}