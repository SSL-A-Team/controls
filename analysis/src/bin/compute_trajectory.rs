use std::io;
use std::fs;
use ateam_controls::{Vector3f, Vector6f, bangbang_trajectory::{BangBangTraj3D, TrajectoryParams}};
use clap::Parser;
use serde::Deserialize;

#[derive(Parser)]
#[command(about = "Compute optimal bang-bang trajectory")]
struct Cli {
    /// Path to a JSON file containing parameters (trajectory params are loaded from TRAJ_* keys)
    #[arg(long = "param-json")]
    param_json: Option<String>,
}

#[derive(Deserialize)]
struct ParamFile {
    #[serde(flatten)]
    traj_params: TrajectoryParams,
}

fn load_trajectory_params(cli: &Cli) -> Result<TrajectoryParams, Box<dyn std::error::Error>> {
    match &cli.param_json {
        Some(path) => {
            let contents = fs::read_to_string(path)?;
            let params: ParamFile = serde_json::from_str(&contents)?;
            Ok(params.traj_params)
        }
        None => Ok(TrajectoryParams::default()),
    }
}

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
    let theta: f32 = parts[2].parse()?;
    let xd: f32 = parts[3].parse()?;
    let yd: f32 = parts[4].parse()?;
    let thetad: f32 = parts[5].parse()?;
    Ok(Vector6f::new(x, y, theta, xd, yd, thetad))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    let traj_params = load_trajectory_params(&cli)?;

    // Read init_state from stdin
    let mut init_state_input = String::new();
    io::stdin().read_line(&mut init_state_input)?;
    let init_state = parse_vector6f(&init_state_input)?;

    // Read target_pose from stdin
    let mut target_pose_input = String::new();
    io::stdin().read_line(&mut target_pose_input)?;
    let target_pose = parse_vector3f(&target_pose_input)?;

    // Compute optimal trajectory
    let traj = BangBangTraj3D::from_target_pose(init_state, target_pose, traj_params)?;
    let end_time = traj.end_time();
    let time_resolution = 0.001;

    // Print CSV header
    println!("time,x,y,theta,xd,yd,thetad,xdd,ydd,thetadd");

    // Iterate through trajectory and output state and control at each time step
    let mut t = 0.0;
    while t <= end_time {
        // Get state at time t
        let state = traj.state_at(init_state, 0.0, t)?;
        
        // Get acceleration at time t
        let accel = traj.accel_at(t)?;

        // Output in CSV format: t, x, y, theta, xd, yd, thetad xdd, ydd, thetadd
        println!(
            "{:.4},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            t,
            state[0], state[1], state[2],  // x, y, theta
            state[3], state[4], state[5],  // xd, yd, w
            accel[0], accel[1], accel[2]   // xdd, ydd, thetadd
        );

        t += time_resolution;
    }

    Ok(())
}