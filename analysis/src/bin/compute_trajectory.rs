use std::io;
use ateam_controls::{Vector3f, Vector6f, bangbang_trajectory::{compute_bangbang_traj_3d_accel_at_t, compute_bangbang_traj_3d_state_at_t, compute_optimal_bangbang_traj_3d}};

fn parse_vector3f(line: &str) -> Result<Vector3f, Box<dyn std::error::Error>> {
    let parts: Vec<&str> = line.trim().split_whitespace().collect();
    if parts.len() != 3 {
        return Err("Expected 3 values for Vector3f".into());
    }
    let x: f32 = parts[0].parse()?;
    let y: f32 = parts[1].parse()?;
    let z: f32 = parts[2].parse()?;
    Ok(Vector3f::new(x, y, z))
}

fn parse_vector6f(line: &str) -> Result<Vector6f, Box<dyn std::error::Error>> {
    let parts: Vec<&str> = line.trim().split_whitespace().collect();
    if parts.len() != 6 {
        return Err("Expected 6 values for Vector6f".into());
    }
    let x: f32 = parts[0].parse()?;
    let y: f32 = parts[1].parse()?;
    let z: f32 = parts[2].parse()?;
    let vx: f32 = parts[3].parse()?;
    let vy: f32 = parts[4].parse()?;
    let vz: f32 = parts[5].parse()?;
    Ok(Vector6f::new(x, y, z, vx, vy, vz))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Read init_state from stdin
    let mut init_state_input = String::new();
    io::stdin().read_line(&mut init_state_input)?;
    let init_state = parse_vector6f(&init_state_input)?;

    // Read target_pose from stdin
    let mut target_pose_input = String::new();
    io::stdin().read_line(&mut target_pose_input)?;
    let target_pose = parse_vector3f(&target_pose_input)?;

    // Compute optimal trajectory
    let traj = compute_optimal_bangbang_traj_3d(init_state, target_pose);
    let end_time = traj.get_end_time();
    let time_resolution = 0.001;

    // Print CSV header
    println!("time,x,y,theta,vx,vy,w,ax,ay,atheta");

    // Iterate through trajectory and output state and control at each time step
    let mut t = 0.0;
    while t <= end_time {
        // Get state at time t
        let state = compute_bangbang_traj_3d_state_at_t(traj, init_state, 0.0, t);
        
        // Get acceleration at time t
        let accel = compute_bangbang_traj_3d_accel_at_t(traj, t);

        // Output in CSV format: t, x, y, theta, vx, vy, w, ax, ay, atheta
        println!(
            "{:.4},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            t,
            state[0], state[1], state[2],  // x, y, theta
            state[3], state[4], state[5],  // vx, vy, w
            accel[0], accel[1], accel[2]   // ax, ay, atheta
        );

        t += time_resolution;
    }

    Ok(())
}