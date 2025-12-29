use ateam_controls::{Matrix6f, Matrix6x3f, RigidBodyState, Vector3f, Vector4f, Vector6f};
use ateam_controls::robot_model::{RobotModel};
use ateam_controls::bangbang_trajectory::{compute_optimal_bangbang_traj_3d, compute_bangbang_traj_3d_accel_at_t};
use ateam_controls::robot_physical_params::*;
use ateam_controls::trajectory_params::*;
use core::f32::consts::PI;

fn main() {
    // Timing constants
    // let control_dt = 0.02;     // 50 Hz controller
    // let sim_dt = 0.001; // 1000 Hz physics engine
    let control_dt = 0.001;     // 1 KHz controller
    let sim_dt = 0.00001; // 100 KHz physics engine
    let steps_per_control_cycle = (control_dt / sim_dt) as usize;
    let mut total_sim_time = 0.0;

    let mut model = RobotModel::new_from_constants(control_dt);

    let mut sim_state = Vector6f::new(
        0.5, 0.5, 0.0,
        1.0, -1.0, 0.0,
    );
    let sim_a = Matrix6f::new(
        1., 0., 0., sim_dt, 0., 0.,
        0., 1., 0., 0., sim_dt, 0.,
        0., 0., 1., 0., 0., sim_dt,
        0., 0., 0., 1., 0., 0.,
        0., 0., 0., 0., 1., 0.,
        0., 0., 0., 0., 0., 1.,
    );
    let sim_b = Matrix6x3f::new(
        0., 0., 0.,
        0., 0., 0.,
        0., 0., 0.,
        sim_dt, 0., 0.,
        0., sim_dt, 0.,
        0., 0., sim_dt,
    );

    let target_pose = Vector3f::new(0.5, 1.5, PI / 2.0);
    
    let mut control_u = Vector3f::zeros();
    let mut wheel_torques = Vector4f::zeros();
    
    // Print CSV header for the Python visualizer
    println!("time,x,y,theta,vx,vy,vtheta,ax,ay,atheta,wv1,wv2,wv3,wv4,wt1,wt2,wt3,wt4");

    while total_sim_time < 5.0 {
        // --- PHYSICS SIMULATION ---
        for _ in 0..steps_per_control_cycle {
            // Apply torques to find actual acceleration
            let sim_accel = model.transform_wheel2accel(sim_state.z) * wheel_torques;

            // Update sim
            sim_state = sim_a * sim_state + sim_b * sim_accel;

            // Calculate wheel velocities for telemetry
            let wheel_velocities = model.transform_twist2wheel(sim_state.z) * Vector3f::new(sim_state[3], sim_state[4], sim_state[5]);

            // Log data at simulation frequency for smooth video
            println!(
                "{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
                total_sim_time,
                sim_state[0], sim_state[1], sim_state[2],
                sim_state[3], sim_state[4], sim_state[5],
                sim_accel.x, sim_accel.y, sim_accel.z,
                wheel_velocities.x, wheel_velocities.y, wheel_velocities.z, wheel_velocities.w,
                wheel_torques.x, wheel_torques.y, wheel_torques.z, wheel_torques.w
            );

            total_sim_time += sim_dt;

            // Check if arrived
            let sim_pose = Vector3f::new(sim_state[0], sim_state[1], sim_state[2]);
            let sim_twist = Vector3f::new(sim_state[3], sim_state[4], sim_state[5]);
            if (sim_pose - target_pose).norm() < ALLOWABLE_ERROR_POS 
               && sim_twist.norm() < ALLOWABLE_ERROR_VEL {
                return;
            }
        }

        // Simulate sensor readings
        model.update_h_transform(sim_state.z, false, false, false);
        let meas = model.h * sim_state;

        // --- ON-ROBOT PROCESSING ---
        model.kf_predict(control_u);
        model.kf_update(meas, false, false, false);

        // 1. Generate new trajectory from current state
        let traj = compute_optimal_bangbang_traj_3d(model.x, target_pose);
        
        // 2. Determine desired global acceleration at this instant
        control_u = compute_bangbang_traj_3d_accel_at_t(traj, 0.0);
        
        // 3. Convert to output wheel torques
        wheel_torques = model.transform_accel2wheel(model.x[2]) * control_u;

        // println!(
        //     "{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
        //     total_sim_time,
        //     model.x[0], model.x[1], model.x[2],
        //     model.x[3], model.x[4], model.x[5],
        // );

        // if total_sim_time >= 0.005 {
        //     return
        // }
    }
}