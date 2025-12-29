// use crate::{Matrix3f, Matrix6f, Matrix6x3f, Matrix8f, Matrix8x6f, Vector3f, Vector6f, Vector8f, z_rotation_mat};
// use crate::robot_model::{RobotModel, transform_frame_global2robot_twist};
// use libm::{cosf, sinf};
// use nalgebra::vector;


// pub struct ExtendedKalmanFilter {
//     pub state_estimate: Vector6f,
//     pub covariance_estimate: Matrix6f,
//     model: RobotModel,
//     dt: f32,
//     A_mat: Matrix6f,
//     B_mat: Matrix6x3f,
//     Q_mat: Matrix6f,
//     R_mat: Matrix8f,
// }

// impl ExtendedKalmanFilter {
//     pub fn new(model: RobotModel, dt: f32) -> ExtendedKalmanFilter {
//         let A_mat = Matrix6f::new(
//             1., 0., 0., dt, 0., 0.,
//             0., 1., 0., 0., dt, 0.,
//             0., 0., 1., 0., 0., dt,
//             0., 0., 0., 1., 0., 0.,
//             0., 0., 0., 0., 1., 0.,
//             0., 0., 0., 0., 0., 1.,
//         );
//         let B_mat = Matrix6x3f::new(
//             0., 0., 0.,
//             0., 0., 0.,
//             0., 0., 0.,
//             dt, 0., 0.,
//             0., dt, 0.,
//             0., 0., dt,
//         );
//         let Q_mat = Matrix6f::identity();
//         let R_mat = Matrix8f::identity();
//         ExtendedKalmanFilter {
//             state_estimate: Vector6f::zeros(),
//             covariance_estimate: Matrix6f::zeros(),
//             model: model,
//             dt,
//             A_mat,
//             B_mat,
//             Q_mat,
//             R_mat
//         }
//     }

//     pub fn predict(&mut self, u: Vector3f) {
//         self.state_estimate = self.A_mat * self.state_estimate + self.B_mat * u;
//         // self.state_estimate[0] = self.state_estimate[0] + self.dt * self.state_estimate[3];
//         // self.state_estimate[1] = self.state_estimate[1] + self.dt * self.state_estimate[4];
//         // self.state_estimate[2] = self.state_estimate[2] + self.dt * self.state_estimate[5];
//         // self.state_estimate[3] = self.state_estimate[3] + self.dt * u[0];
//         // self.state_estimate[4] = self.state_estimate[4] + self.dt * u[1];
//         // self.state_estimate[5] = self.state_estimate[5] + self.dt * u[2];
//         self.covariance_estimate = self.A_mat * self.covariance_estimate * self.A_mat.transpose() + self.Q_mat;
//     }

//     pub fn update(&mut self, z: Vector8f) {
//         let global_pose = Vector3f::new(
//             self.state_estimate[0],
//             self.state_estimate[1],
//             self.state_estimate[2],
//         );
//         let global_twist = Vector3f::new(
//             self.state_estimate[3],
//             self.state_estimate[4],
//             self.state_estimate[5],
//         );
//         // let robot_twist= transform_frame_global2robot_twist(global_pose, global_twist);
//         // let wheel_velocities = self.model.twist_to_wheel_velocities(robot_twist);
//         let global_body_twist_to_wheel_vel_mat = self.model.wheel_transform_mat.transpose() * z_rotation_mat(- self.state_estimate[2]) / self.model.wheel_radius;
//         let wheel_velocities = global_body_twist_to_wheel_vel_mat * global_twist;
//         let h_x: Vector8f = vector![
//             self.state_estimate[0],  // x
//             self.state_estimate[1],  // y
//             self.state_estimate[2],  // theta
//             self.state_estimate[5],  // dtheta/dt
//             wheel_velocities[0],
//             wheel_velocities[1],
//             wheel_velocities[2],
//             wheel_velocities[3],
//         ];
//         let residual = z - h_x;

//         let M = global_body_twist_to_wheel_vel_mat;
//         let H = Matrix8x6f::from_row_slice(&[
//             1., 0., 0., 0., 0., 0.,
//             0., 1., 0., 0., 0., 0.,
//             0., 0., 1., 0., 0., 0., 
//             0., 0., 0., 0., 0., 1.,
//             0., 0., 0., M[(0, 0)], M[(0, 1)], M[(0, 2)],
//             0., 0., 0., M[(1, 0)], M[(1, 1)], M[(1, 2)],
//             0., 0., 0., M[(2, 0)], M[(2, 1)], M[(2, 2)],
//             0., 0., 0., M[(3, 0)], M[(3, 1)], M[(3, 2)],
//         ]);

//         let kalman_gain = self.covariance_estimate * H.transpose() * (H * self.covariance_estimate * H.transpose() + self.R_mat).try_inverse().unwrap();
//         self.state_estimate = self.state_estimate + kalman_gain * residual;
//         self.covariance_estimate = (Matrix6f::identity() - kalman_gain * H) * self.covariance_estimate;
//     }
// }