use ateam_controls::bang_bang_trajectory::{BangBangTraj1D, compute_optimal_bang_bang_traj_3d, compute_bang_bang_traj_3d_state_at_t};
use ateam_controls::{GlobalState, GlobalControl2Order};
use ateam_controls::trajectory_params::{ALLOWABLE_ERROR_POS, ALLOWABLE_ERROR_VEL};

use std::collections::LinkedList;
use plotters::prelude::*;
// use std::{fs::File, io::Write};
// use serde::{Serialize, Deserialize};


fn set_acceleration(traj: BangBangTraj1D, current_time: f64) -> f64 {
    if current_time >= traj.t3 {
        return traj.sdd3;
    }
    if current_time >= traj.t2 {
        return traj.sdd2;
    }
    if current_time >= traj.t1 {
        // // First phase of the trajectory
        // if traj.t2 - traj.t1 <= TRAJ_PHASE_1_TIME_PRECISION {
        //     return 0.0;  // Don't start phase 1 if there is barely time left in phase 1
        // }
        return traj.sdd1;
    }
    panic!("Tried to use a trajectory that hasn't started yet!")
}

fn next_state(mut current_state: GlobalState, current_control: GlobalControl2Order, dt: f64) -> GlobalState{
    let mut next_state = GlobalState::default();
    if current_state.x.abs() <= ALLOWABLE_ERROR_POS && current_state.xd.abs() <= ALLOWABLE_ERROR_VEL {
        current_state.xd = 0.0;
    }
    if current_state.y.abs() <= ALLOWABLE_ERROR_POS && current_state.yd.abs() <= ALLOWABLE_ERROR_VEL {
        current_state.yd = 0.0;
    }
    if current_state.z.abs() <= ALLOWABLE_ERROR_POS && current_state.zd.abs() <= ALLOWABLE_ERROR_VEL {
        current_state.zd = 0.0;
    }
    next_state.x = current_state.x + current_state.xd * dt + 0.5 * current_control.xdd * dt * dt;
    next_state.y = current_state.y + current_state.yd * dt + 0.5 * current_control.ydd * dt * dt;
    next_state.z = current_state.z + current_state.zd * dt + 0.5 * current_control.zdd * dt * dt;
    next_state.xd = current_state.xd + current_control.xdd * dt;
    next_state.yd = current_state.yd + current_control.ydd * dt;
    next_state.zd = current_state.zd + current_control.zdd * dt;
    next_state
}

fn run_simulation(init_state: GlobalState, dt: f64) -> (LinkedList<GlobalState>, LinkedList<GlobalControl2Order>) {
    let mut states: LinkedList<GlobalState> = LinkedList::new();
    let mut controls: LinkedList<GlobalControl2Order> = LinkedList::new();
    let mut current_state = init_state;
    let mut current_control = GlobalControl2Order {xdd: 0.0, ydd: 0.0, zdd: 0.0};
    let mut t = 0.0;
    while 
        current_state.x.abs() > ALLOWABLE_ERROR_POS ||
        current_state.y.abs() > ALLOWABLE_ERROR_POS ||
        current_state.z.abs() > ALLOWABLE_ERROR_POS {
        // if not the first iteration, update the state using previous iteration accelerations
        if t != 0.0 {
            current_state = next_state(current_state, current_control, dt);
        }
        // push the current state to the list of states
        states.push_back(current_state);
        // compute optimal trajectory
        let target = GlobalState::default();
        let mut traj = compute_optimal_bang_bang_traj_3d(current_state, target);
        traj.time_shift(t);
        // set the accelerations for current time to next time
        current_control = GlobalControl2Order {
            xdd: set_acceleration(traj.x_traj, t),
            ydd: set_acceleration(traj.y_traj, t),
            zdd: set_acceleration(traj.z_traj, t),
        };
        controls.push_back(current_control);
        t += dt;
    }
    // return list of states
    (states, controls)
}

fn plot_simulation(states: LinkedList<GlobalState>, controls: LinkedList<GlobalControl2Order>, dt: f64) -> Result<(), Box<dyn std::error::Error>> {
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
        x_vals.push(state.x);
        xd_vals.push(state.xd);
        y_vals.push(state.y);
        yd_vals.push(state.yd);
        z_vals.push(state.z);
        zd_vals.push(state.zd);
        times.push(time);
        time += dt;
    }

    for control in &controls {
        xdd_vals.push(control.xdd);
        ydd_vals.push(control.ydd);
        zdd_vals.push(control.zdd);
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
    let init_state = GlobalState {
        x: 1.0,
        y: 1.0,
        z: 1.0,
        xd: 0.0,
        yd: 0.0,
        zd: 0.0,
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
    let target_state = GlobalState::default();
    let traj = compute_optimal_bang_bang_traj_3d(init_state, target_state);
    let mut states = LinkedList::<GlobalState>::new();
    let mut controls = LinkedList::<GlobalControl2Order>::new();
    let mut t = 0.0;
    while t < traj.x_traj.t4 ||
          t < traj.y_traj.t4 ||
          t < traj.z_traj.t4 {
        states.push_back(
            compute_bang_bang_traj_3d_state_at_t(traj, init_state, 0.0, t)
        );
        controls.push_back(GlobalControl2Order {
            xdd: set_acceleration(traj.x_traj, t),
            ydd: set_acceleration(traj.y_traj, t),
            zdd: set_acceleration(traj.z_traj, t),
        });
        t += dt;
    }

    // save_states(&states, "./states.json")?;

    // Run a "simulation" that steps through the states and recomputes the optimal trajectory at each step
    // let (states, controls) = run_simulation(init_state, dt);
    plot_simulation(states, controls, dt)?;

    Ok(())
}
