use ateam_controls::bangbang_trajectory::{BangBangTraj1D, BangBangTraj3D, compute_bangbang_traj_3d_accel_at_t, compute_bangbang_traj_3d_state_at_t, compute_optimal_bangbang_traj_3d};
use ateam_controls::geometry::{Accel, Pose, RigidBodyState, Twist, Vector3};
use ateam_controls::trajectory_params::{ALLOWABLE_ERROR_POS, ALLOWABLE_ERROR_VEL};

use std::collections::LinkedList;
use plotters::prelude::*;
// use std::{fs::File, io::Write};
// use serde::{Serialize, Deserialize};


fn next_state(mut current_state: RigidBodyState, current_control: Accel, dt: f64) -> RigidBodyState {
    if current_state.pose.position.x.abs() <= ALLOWABLE_ERROR_POS && current_state.twist.linear.x.abs() <= ALLOWABLE_ERROR_VEL {
        current_state.twist.linear.x = 0.0;
    }
    if current_state.pose.position.y.abs() <= ALLOWABLE_ERROR_POS && current_state.twist.linear.y.abs() <= ALLOWABLE_ERROR_VEL {
        current_state.twist.linear.y = 0.0;
    }
    if current_state.pose.to_xy_yaw().z.abs() <= ALLOWABLE_ERROR_POS && current_state.twist.angular.z.abs() <= ALLOWABLE_ERROR_VEL {
        current_state.twist.angular.z = 0.0;
    }
    let next_x = current_state.pose.position.x + current_state.twist.linear.x * dt + 0.5 * current_control.linear.x * dt * dt;
    let next_y = current_state.pose.position.y + current_state.twist.linear.y * dt + 0.5 * current_control.linear.y * dt * dt;
    let next_z = current_state.pose.to_xy_yaw().z + current_state.twist.angular.z * dt + 0.5 * current_control.angular.z * dt * dt;
    let next_xd = current_state.twist.linear.x + current_control.linear.x * dt;
    let next_yd = current_state.twist.linear.y + current_control.linear.y * dt;
    let next_zd = current_state.twist.angular.z + current_control.angular.z * dt;
    let next_pose = Pose::from_xy_yaw(next_x, next_y, next_z);
    let next_twist = Twist {
        linear: Vector3 { x: next_xd, y: next_yd, z: 0.0 },
        angular: Vector3 { x: 0.0, y: 0.0, z: next_zd }
    };
    RigidBodyState { pose: next_pose, twist: next_twist }
}

fn run_simulation(init_state: RigidBodyState, dt: f64) -> (LinkedList<RigidBodyState>, LinkedList<Accel>) {
    let mut states: LinkedList<RigidBodyState> = LinkedList::new();
    let mut controls: LinkedList<Accel> = LinkedList::new();
    let mut current_state = init_state;
    let mut current_control = Accel::default();
    let mut t = 0.0;
    while 
        current_state.pose.position.x.abs() > ALLOWABLE_ERROR_POS ||
        current_state.pose.position.y.abs() > ALLOWABLE_ERROR_POS ||
        current_state.pose.to_xy_yaw().z.abs() > ALLOWABLE_ERROR_POS {
        // if not the first iteration, update the state using previous iteration accelerations
        if t != 0.0 {
            current_state = next_state(current_state, current_control, dt);
        }
        // push the current state to the list of states
        states.push_back(current_state);
        // compute optimal trajectory
        let target = RigidBodyState::default();
        let mut traj = compute_optimal_bangbang_traj_3d(current_state, target);
        traj.time_shift(t);
        // set the accelerations for current time to next time
        current_control = compute_bangbang_traj_3d_accel_at_t(traj, t);
        controls.push_back(current_control);
        t += dt;
    }
    // return list of states
    (states, controls)
}

fn plot_simulation(states: LinkedList<RigidBodyState>, controls: LinkedList<Accel>, dt: f64) -> Result<(), Box<dyn std::error::Error>> {
    // Collect data for plotting
    let mut time = 0.0;
    let mut x_vals = Vec::new();
    let mut xd_vals = Vec::new();
    let mut xdd_vals = Vec::new();
    let mut y_vals = Vec::new();
    let mut yd_vals = Vec::new();
    let mut ydd_vals = Vec::new();
    let mut z_vals = Vec::new();
    let mut zd_vals = Vec::new();
    let mut zdd_vals = Vec::new();
    let mut times = Vec::new();

    for state in &states {
        x_vals.push(state.pose.position.x);
        xd_vals.push(state.twist.linear.x);
        y_vals.push(state.pose.position.y);
        yd_vals.push(state.twist.linear.y);
        z_vals.push(state.pose.to_xy_yaw().z);
        zd_vals.push(state.twist.linear.z);
        times.push(time);
        time += dt;
    }

    for control in &controls {
        xdd_vals.push(control.linear.x);
        ydd_vals.push(control.linear.y);
        zdd_vals.push(control.angular.z);
    }

    // Create a 1920x1440 image (3 rows x 3 columns of 640x480 plots)
    let root = BitMapBackend::new("output.png", (1920, 1440)).into_drawing_area();
    root.fill(&WHITE)?;

    let areas = root.split_evenly((3, 3)); // 3 rows, 3 columns

    // Helper to plot a single variable
    let plot_var = |area: DrawingArea<_, _>, caption: &str, data: &Vec<f64>| -> Result<(), Box<dyn std::error::Error>> {
        let min = data.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = data.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let mut chart = ChartBuilder::on(&area)
            .caption(caption, ("sans-serif", 30))
            .margin(10)
            .x_label_area_size(30)
            .y_label_area_size(40)
            .build_cartesian_2d(0f64..(times.last().copied().unwrap_or(0.0)), min..max)?;
        chart.configure_mesh().draw()?;
        chart.draw_series(LineSeries::new(
            times.iter().cloned().zip(data.iter().cloned()),
            &RED,
        ))?;
        Ok(())
    };

    // Row 0: x position, x velocity, x acceleration
    plot_var(areas[0].clone(), "x position vs time", &x_vals)?;
    plot_var(areas[1].clone(), "x velocity vs time", &xd_vals)?;
    plot_var(areas[2].clone(), "x acceleration vs time", &xdd_vals)?;

    // Row 1: y position, y velocity, y acceleration
    plot_var(areas[3].clone(), "y position vs time", &y_vals)?;
    plot_var(areas[4].clone(), "y velocity vs time", &yd_vals)?;
    plot_var(areas[5].clone(), "y acceleration vs time", &ydd_vals)?;

    // Row 2: z position, z velocity, z acceleration
    plot_var(areas[6].clone(), "z position vs time", &z_vals)?;
    plot_var(areas[7].clone(), "z velocity vs time", &zd_vals)?;
    plot_var(areas[8].clone(), "z acceleration vs time", &zdd_vals)?;

    Ok(())
}

// fn save_states(states: &LinkedList<GlobalState>, path: &str) -> std::io::Result<()> {
//     // Convert the LinkedList into a Vec so serde_json can serialize it easily
//     let vec: Vec<_> = states.iter().collect();

//     // Serialize to JSON
//     let json = serde_json::to_string_pretty(&vec).expect("Failed to serialize");

//     // Write to file
//     let mut file = File::create(path)?;
//     file.write_all(json.as_bytes())?;
//     Ok(())
// }

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let init_state = RigidBodyState {
        pose: Pose::from_xy_yaw(1.0, 1.0, 1.0),
        twist: Twist::default(),
    };
    // let init_state = GlobalState {
    //     x: 2.0,
    //     y: 2.0,
    //     z: -3.0,
    //     xd: 1.0,
    //     yd: -1.0,
    //     zd: 1.0,
    // };
    // let init_state = GlobalState {
    //     x: 10.0,
    //     y: 5.0,
    //     z: 1.0,
    //     xd: 1.0,
    //     yd: -1.0,
    //     zd: 0.0,
    // };
    // let init_state = GlobalState {
    //     x: -1.0,
    //     y: 0.0,
    //     z: 0.0,
    //     xd: 0.0,
    //     yd: 0.0,
    //     zd: 0.0,
    // };

    let dt = 0.01;

    // Write the computed velocities to a file for playback to real robot
    let target_state = RigidBodyState::default();
    let traj = compute_optimal_bangbang_traj_3d(init_state, target_state);
    let mut states = LinkedList::<RigidBodyState>::new();
    let mut controls = LinkedList::<Accel>::new();
    let mut t = 0.0;
    while t < traj.x.t4 ||
          t < traj.y.t4 ||
          t < traj.z.t4 {
        states.push_back(
            compute_bangbang_traj_3d_state_at_t(traj, init_state, 0.0, t)
        );
        controls.push_back(
            compute_bangbang_traj_3d_accel_at_t(traj, t)
        );
        t += dt;
    }

    // save_states(&states, "./states.json")?;

    // Run a "simulation" that steps through the states and recomputes the optimal trajectory at each step
    // let (states, controls) = run_simulation(init_state, dt);
    plot_simulation(states, controls, dt)?;

    Ok(())
}
