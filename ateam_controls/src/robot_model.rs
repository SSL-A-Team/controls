use crate::{GlobalState, WheelTorques, WheelVelocities, GlobalControl1Order, GlobalControl2Order};
use nalgebra::{Matrix3, Matrix3x4, Matrix4x3, Vector3};
use libm::{sin, cos};


struct RobotModel {
    wheel_conversion_mat: Matrix3x4<f64>,
    wheel_conversion_mat_inv: Matrix4x3<f64>,
}

impl RobotModel {
    pub fn new_from_alpha_beta_l(alpha: f64, beta: f64, l: f64) -> RobotModel {
        let mat = Matrix3x4::new(
            -cos(alpha), -cos(beta),  cos(beta), cos(alpha),
             sin(alpha), -sin(beta), -sin(beta), sin(alpha),
             l         ,  l        ,  l        , l
        );
        let mat_inv = pinv_3x4(mat);
        RobotModel {wheel_conversion_mat: mat, wheel_conversion_mat_inv: mat_inv}
    }

    // TODO update to global_control_1order_to_wheel_velocities
    // z = 0 for local robot frame
    pub fn global_state_to_wheel_velocities(&self, current_state: GlobalState) -> WheelVelocities {
        let z = current_state.z;
        let rotation_mat = Matrix3::<f64>::new(
            cos(z), -sin(z), 0.0,
            sin(z),  cos(z), 0.0,
            0.0   ,  0.0   , 1.0,
        );
        let cartesian_velocities = Vector3::<f64>::new(
            current_state.xd, current_state.yd, current_state.zd,
        );
        let wheel_velocities = (rotation_mat * self.wheel_conversion_mat).transpose() * cartesian_velocities;
        WheelVelocities::from_vec(&wheel_velocities)
    }

    // z = 0 for local robot frame
    pub fn wheel_velocities_to_global_control_1order(&self, wheel_velocities: WheelVelocities, z: f64) -> GlobalControl1Order {
        todo!();
    }

    // z = 0 for local robot frame
    pub fn wheel_torques_to_global_control_2order(&self, torques: WheelTorques, z: f64) -> GlobalControl2Order {
        let rotation_mat = Matrix3::<f64>::new(
            cos(z),-sin(z), 0.0,
            sin(z), cos(z), 0.0,
            0.0   , 0.0   , 1.0,
        );
        let wheel_torques = torques.to_vec();
        let accelerations = rotation_mat * self.wheel_conversion_mat * wheel_torques;
        GlobalControl2Order::from_vec(&accelerations)
    }

    // z = 0 for local robot frame
    pub fn global_control_2order_to_wheel_torques(&self, global_control: GlobalControl2Order, z: f64) -> WheelTorques {
        let rotation_mat = Matrix3::<f64>::new(
            cos(-z), sin(z) , 0.0,
            sin(-z), cos(-z), 0.0,
            0.0    , 0.0    , 1.0,
        );
        let control = global_control.to_vec();
        let torques = self.wheel_conversion_mat_inv * rotation_mat * control;
        WheelTorques::from_vec(&torques)
    }
}

fn pinv_3x4(a: Matrix3x4<f64>) -> Matrix4x3<f64> {
    let a_t = a.clone().transpose();
    let a_at = a * a_t.clone();
    let a_at_inv = a_at.try_inverse().unwrap();
    let a_inv = a_t * a_at_inv;
    a_inv
}