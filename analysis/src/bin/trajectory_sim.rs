use ateam_controls::{Matrix6f, Matrix6x3f, Vector3f, Vector4f, Vector6f, wrap_angle};
use ateam_controls::robot_model::RobotModel;
use ateam_controls::bangbang_trajectory::{BangBangTraj3D, TrajectoryParams};

fn main() {
    // Timing constants
    // let control_dt = 0.02;     // 50 Hz controller
    // let sim_dt = 0.001; // 1000 Hz physics engine
    let control_dt = 0.001;     // 1 KHz controller
    let sim_dt = 0.00001; // 100 KHz physics engine
    let steps_per_control_cycle = (control_dt / sim_dt) as usize;
    let mut total_sim_time = 0.0;

    let mut model = RobotModel::new_from_default_params(control_dt);

    let mut sim_state = Vector6f::new(
        0.0, 0.0, 3.0,
        0.0, 0.0, 0.0,
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

    let target_pose = Vector3f::new(0.0, 0.0, -3.0);
    
    let mut global_accel_cmd = Vector3f::zeros();
    let mut wheel_torques = Vector4f::zeros();

    // Print CSV header for the Python visualizer
    println!("time,x,y,theta,xd,yd,thetad,xdd,ydd,thetadd,wv1,wv2,wv3,wv4,wt1,wt2,wt3,wt4");

    while total_sim_time < 5.0 {
        // --- PHYSICS SIMULATION ---
        for _ in 0..steps_per_control_cycle {
            // Apply torques to find actual acceleration
            let sim_accel = model.transform_wheel2accel(sim_state.z) * wheel_torques;

            // Update sim
            sim_state = sim_a * sim_state + sim_b * sim_accel;
            sim_state[2] = wrap_angle(sim_state[2]);

            // Calculate wheel velocities for telemetry
            let sim_twist: Vector3f = sim_state.fixed_rows::<3>(3).into();
            let wheel_velocities = model.transform_twist2wheel(sim_state.z) * sim_twist;

            total_sim_time += sim_dt;

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
        }

        // Simulate sensor readings
        model.update_h_transform(sim_state.z, false, false, false);
        let meas = model.h * sim_state;

        // ---------------- ON-ROBOT PROCESSING ------------------
        // Update state estimate
        model.kf_predict(global_accel_cmd);
        model.kf_update(meas, false, false, false);
        let state_estimate = model.x;

        // Generate new trajectory from current state
        let traj = BangBangTraj3D::from_target_pose(state_estimate, target_pose, TrajectoryParams::default());
        
        // Determine desired global acceleration at this instant
        global_accel_cmd = traj.accel_at(0.0);
        
        // Check if controller determined robot is close enough to target
        if global_accel_cmd.x == 0.0 && global_accel_cmd.y == 0.0 && global_accel_cmd.z == 0.0 {
            // End the sim
            break;
        }

        // 3. Convert to output wheel torques
        wheel_torques = model.transform_accel2wheel(state_estimate.z) * global_accel_cmd;
    }
}