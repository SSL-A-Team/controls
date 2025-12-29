use ateam_controls::robot_model::*;
use ateam_controls::bangbang_trajectory::*;
use ateam_controls::*;

fn main() {
    let mut sim_state = Vector6f::new(1., 0., 0., 0., 0., 0.);
    let target_pose = Vector3f::new(2., 0., 0.);
    let mut robot = RobotModel::new_from_constants(0.001);

    let mut u = Vector3f::zeros();
    let mut wheel_torques = Vector4f::zeros();

    for i in 0..2000 {
        // Update sim
        let sim_accel = robot.transform_wheel2accel(sim_state[2]) * wheel_torques;
        sim_state = robot.a * sim_state + robot.b * sim_accel;
        robot.update_h_transform(sim_state[2], false, false, false);
        let meas = robot.h * sim_state;

        // Update robot
        robot.kf_predict(u);
        robot.kf_update(meas, false, false, false);

        // Control decision
        let traj = compute_optimal_bangbang_traj_3d(robot.x, target_pose);
        u = compute_bangbang_traj_3d_accel_at_t(traj, 0.);
        wheel_torques = robot.transform_accel2wheel(robot.x[2]) * u;

        println!("sim state  : {}", sim_state);
        println!("robot state: {}", robot.x);
    }
}