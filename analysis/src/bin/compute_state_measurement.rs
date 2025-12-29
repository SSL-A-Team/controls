use std::io;
use std::fs;
use ateam_controls::{Vector6f, Vector8f, robot_model::{RobotModel, KalmanFilterParams, RobotPhysicalParams}};
use clap::Parser;
use nalgebra::vector;
use serde::Deserialize;

#[derive(Parser)]
#[command(about = "Compute state measurement from sensor readings")]
struct Cli {
    /// Path to a JSON file containing RobotModel parameters
    #[arg(long = "param-json")]
    param_json: Option<String>,
}

#[derive(Deserialize)]
struct RobotModelParams {
    #[serde(flatten)]
    kf_params: KalmanFilterParams,
    #[serde(flatten)]
    physical_params: RobotPhysicalParams,
}

fn load_robot_model(cli: &Cli) -> Result<RobotModel, Box<dyn std::error::Error>> {
    match &cli.param_json {
        Some(path) => {
            let contents = fs::read_to_string(path)?;
            let params: RobotModelParams = serde_json::from_str(&contents)?;
            Ok(RobotModel::new(0.001, params.kf_params, params.physical_params))
        }
        None => Ok(RobotModel::new_from_default_params(0.001)),
    }
}

fn parse_state(line: &str) -> Result<Vector6f, Box<dyn std::error::Error>> {
    let parts: Vec<&str> = line.trim().split_whitespace().collect();
    if parts.len() != 6 {
        return Err("Expected 6 values for Vector6f".into());
    }
    let x: f32 = parts[0].parse()?;
    let y: f32 = parts[1].parse()?;
    let z: f32 = parts[2].parse()?;
    let xd: f32 = parts[3].parse()?;
    let yd: f32 = parts[4].parse()?;
    let vz: f32 = parts[5].parse()?;
    Ok(Vector6f::new(x, y, z, xd, yd, vz))
}

fn parse_sensor_readings(line: &str) -> Result<Vector8f, Box<dyn std::error::Error>> {
    let parts: Vec<&str> = line.trim().split_whitespace().collect();
    if parts.len() != 8 {
        return Err("Expected 8 sensor readings".into());
    }
    let vision_x: f32 = parts[0].parse()?;
    let vision_y: f32 = parts[1].parse()?;
    let vision_theta: f32 = parts[2].parse()?;
    let encoder_vfl: f32 = parts[3].parse()?;
    let encoder_vbl: f32 = parts[4].parse()?;
    let encoder_vbr: f32 = parts[5].parse()?;
    let encoder_vfr: f32 = parts[6].parse()?;
    let gyro_thetad: f32 = parts[7].parse()?;
    let sensors: Vector8f = vector![vision_x, vision_y, vision_theta, encoder_vfl, encoder_vbl, encoder_vbr, encoder_vfr, gyro_thetad];
    Ok(sensors)
}

fn state_from_sensors(robot_model: &RobotModel, sensors: Vector8f, state: Vector6f) -> nalgebra::SVector<f32, 7> {
    let encoders = sensors.fixed_rows::<4>(3);
    let vision_pose_meas = sensors.fixed_rows::<3>(0);
    let gyro_thetad_meas = sensors.fixed_rows::<1>(7);
    let encoders_twist_meas = robot_model.transform_wheel2twist(state.z) * encoders;
    vector![
        vision_pose_meas[0],
        vision_pose_meas[1],
        vision_pose_meas[2],
        encoders_twist_meas[0],
        encoders_twist_meas[1],
        encoders_twist_meas[2],
        gyro_thetad_meas[0]
    ]
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    // Read state from stdin
    let mut state_input = String::new();
    io::stdin().read_line(&mut state_input)?;
    let state = parse_state(&state_input)?;

    // Read sensor readings from stdin
    let mut sensor_input = String::new();
    io::stdin().read_line(&mut sensor_input)?;
    let sensor_readings = parse_sensor_readings(&sensor_input)?;

    let mut robot_model = load_robot_model(&cli)?;
    robot_model.update_h_transform(state.z, false, false, false);

    // Print CSV header
    // println!("vision_x_est,vision_y_est,vision_theta_est,encoder_vfl_est,encoder_vbl_est,encoder_vbr_est,encoder_vfr_est,gyro_thetad_est,vision_x_meas,vision_y_meas,vision_theta_meas,encoder_xd_meas,encoder_yd_meas,encoder_thetad_meas,gyro_thetad_meas");
    println!("vision_x_meas,vision_y_meas,vision_theta_meas,encoder_xd_meas,encoder_yd_meas,encoder_thetad_meas,gyro_thetad_meas");

    // let sensors_from_state = robot_model.h * state;
    let state_measurement = state_from_sensors(&robot_model, sensor_readings, state);

    // Output in CSV format
    println!("{}",
        state_measurement.iter()
            .map(|s| format!("{}", s))
            .collect::<Vec<_>>()
            .join(",")
    );

    Ok(())
}
