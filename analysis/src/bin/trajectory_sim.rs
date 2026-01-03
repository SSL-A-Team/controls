use ateam_controls::{RigidBodyState, Vector3f};
use ateam_controls::robot_model::{RobotModel, transform_frame_global2robot_accel, transform_frame_robot2global_accel, transform_frame_global2robot_twist};
use ateam_controls::bangbang_trajectory::{compute_optimal_bangbang_traj_3d, compute_bangbang_traj_3d_accel_at_t};
use ateam_controls::robot_physical_params::*;
use ateam_controls::trajectory_params::*;
use core::f32::consts::PI;

fn main() {
    let model = RobotModel::new(
        WHEEL_ANGLE_ALPHA, 
        WHEEL_ANGLE_BETA, 
        WHEEL_DISTANCE, 
        WHEEL_RADIUS,
        BODY_MASS,
        BODY_MOMENT_Z,
    );

    let mut current_state = RigidBodyState {
        pose: Vector3f::new(0.5, 0.5, 0.0),
        twist: Vector3f::new(1.0, -1.0, 0.0),
    };

    let target_pose = Vector3f::new(0.5, 1.5, PI / 2.0);
    
    // Timing constants
    // let control_period = 0.02;     // 50 Hz controller
    // let simulation_period = 0.001; // 1000 Hz physics engine
    let control_period = 0.001;     // 1 KHz controller
    let simulation_period = 0.00001; // 100 KHz physics engine
    let steps_per_control_cycle = (control_period / simulation_period) as usize;

    let mut total_sim_time = 0.0;
    
    // Print CSV header for the Python visualizer
    println!("time,x,y,theta,vx,vy,vtheta,ax,ay,atheta,wv1,wv2,wv3,wv4,wt1,wt2,wt3,wt4");

    while total_sim_time < 5.0 {

        // --- ON-ROBOT PROCESSING ---
        // 1. Generate new trajectory from current state
        let traj = compute_optimal_bangbang_traj_3d(current_state, target_pose);
        
        // 2. Determine desired global acceleration at this instant
        let desired_global_accel = compute_bangbang_traj_3d_accel_at_t(traj, 0.0);
        
        // 3. Convert to robot frame and calculate wheel torques
        let robot_accel_cmd = transform_frame_global2robot_accel(current_state.pose, desired_global_accel);
        
        let wheel_torques = model.accel_to_wheel_torques(robot_accel_cmd);

        // --- PHYSICS SIMULATION ---
        for _ in 0..steps_per_control_cycle {
            // Apply torques to find actual acceleration
            let robot_accel = model.wheel_torques_to_accel(wheel_torques);
            
            let global_accel = transform_frame_robot2global_accel(current_state.pose, robot_accel);

            // Integrate
            current_state.pose += current_state.twist * simulation_period + 0.5 * global_accel * simulation_period * simulation_period;  // dx = v*t + 0.5*a*t^2
            current_state.twist += global_accel * simulation_period;  // dv = a*t

            // Calculate wheel velocities for telemetry
            let robot_twist = transform_frame_global2robot_twist(current_state.pose, current_state.twist);
            let wheel_velocities = model.twist_to_wheel_velocities(robot_twist);

            // Log data at simulation frequency for smooth video
            println!(
                "{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
                total_sim_time,
                current_state.pose.x, current_state.pose.y, current_state.pose.z,
                current_state.twist.x, current_state.twist.y, current_state.twist.z,
                global_accel.x, global_accel.y, global_accel.z,
                wheel_velocities.x, wheel_velocities.y, wheel_velocities.z, wheel_velocities.w,
                wheel_torques.x, wheel_torques.y, wheel_torques.z, wheel_torques.w
            );

            total_sim_time += simulation_period;

            // Check if arrived
            if (current_state.pose - target_pose).norm() < ALLOWABLE_ERROR_POS 
               && current_state.twist.norm() < ALLOWABLE_ERROR_VEL {
                return;
            }
        }
    }
}