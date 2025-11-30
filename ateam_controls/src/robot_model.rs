use nalgebra::{Matrix3, Matrix3x4, Matrix4x3};
use libm::{sin, cos};
use crate::geometry::{Accel, Twist, Vector3};


#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct WheelTorques {
    fl: f64,
    bl: f64,
    br: f64,
    fr: f64,
}

impl From<nalgebra::Vector4<f64>> for WheelTorques {
    fn from(v: nalgebra::Vector4<f64>) -> Self {
        WheelTorques {fl: v.x, bl: v.y, br: v.z, fr: v.w}
    }
}

impl From<WheelTorques> for nalgebra::Vector4<f64> {
    fn from(torques: WheelTorques) -> Self {
        Self::new(torques.fl, torques.bl, torques.br, torques.fr)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct WheelVelocities {
    fl: f64,
    bl: f64,
    br: f64,
    fr: f64,
}

impl From<nalgebra::Vector4<f64>> for WheelVelocities {
    fn from(v: nalgebra::Vector4<f64>) -> Self {
        WheelVelocities {fl: v.x, bl: v.y, br: v.z, fr: v.w}
    }
}

impl From<WheelVelocities> for nalgebra::Vector4<f64> {
    fn from(velocities: WheelVelocities) -> Self {
        Self::new(velocities.fl, velocities.bl, velocities.br, velocities.fr)
    }
}

pub struct RobotModel {
    wheel_transform_mat: Matrix3x4<f64>,
    wheel_transform_mat_inv: Matrix4x3<f64>,
}

impl RobotModel {
    pub fn new_from_alpha_beta_l(alpha: f64, beta: f64, l: f64) -> RobotModel {
        let mat = Matrix3x4::new(
            -cos(alpha), -cos(beta),  cos(beta), cos(alpha),
             sin(alpha), -sin(beta), -sin(beta), sin(alpha),
             l         ,  l        ,  l        , l
        );
        let mat_inv = pinv_3x4(mat);
        RobotModel {wheel_transform_mat: mat, wheel_transform_mat_inv: mat_inv}
    }

    /// Calculate wheel velocities from global frame twist and yaw
    pub fn global_twist_to_wheel_velocities(&self, twist: &Twist, yaw: f64) -> WheelVelocities {
        let rotation_mat = Matrix3::<f64>::new(
            cos(yaw), -sin(yaw), 0.0,
            sin(yaw),  cos(yaw), 0.0,
            0.0     ,  0.0     , 1.0,
        );
        let velocities = nalgebra::Vector3::<f64>::new(
            twist.linear.x, twist.linear.y, twist.angular.z,
        );
        WheelVelocities::from((rotation_mat * self.wheel_transform_mat).transpose() * velocities)
    }

    /// Calculate wheel velocities from local frame twist
    pub fn twist_to_wheel_velocities(&self, twist: &Twist) -> WheelVelocities {
        let velocities = nalgebra::Vector3::<f64>::new(
            twist.linear.x, twist.linear.y, twist.angular.z,
        );
        WheelVelocities::from(self.wheel_transform_mat.transpose() * velocities)
    }

    /// Calculate local frame twist from wheel velocities
    pub fn wheel_velocities_to_twist(&self, wheel_velocities: &WheelVelocities) -> Twist {
        todo!();
    }

    // Calculate local frame accel from wheel torques
    pub fn wheel_torques_to_accel(&self, torques: &WheelTorques) -> Accel {
        let torques_vec = nalgebra::Vector4::from(*torques);
        let accelerations = self.wheel_transform_mat * torques_vec;
        Accel {
            linear: Vector3 { x: accelerations[(0, 0)], y: accelerations[(1, 0)], z: 0.0 },
            angular: Vector3 { x: 0.0, y: 0.0, z: accelerations[(2, 0)] },
        }
    }

    // Calculate global frame accel from wheel torques and yaw
    pub fn wheel_torques_to_global_accel(&self, torques: &WheelTorques, yaw: f64) -> Accel {
        let rotation_mat = Matrix3::<f64>::new(
            cos(yaw),-sin(yaw), 0.0,
            sin(yaw), cos(yaw), 0.0,
            0.0   , 0.0   , 1.0,
        );
        let torques_vec = nalgebra::Vector4::from(*torques);
        let accelerations = rotation_mat * self.wheel_transform_mat * torques_vec;
        Accel {
            linear: Vector3 { x: accelerations[(0, 0)], y: accelerations[(1, 0)], z: 0.0 },
            angular: Vector3 { x: 0.0, y: 0.0, z: accelerations[(2, 0)] },
        }
    }

    // Calculate wheel torques from local frame accel
    pub fn accel_to_wheel_torques(&self, accel: &Accel) -> WheelTorques {
        let accel_vec = nalgebra::Vector3::new(accel.linear.x, accel.linear.y, accel.angular.z);
        WheelTorques::from(self.wheel_transform_mat_inv * accel_vec)
    }

    // Calculate wheel torques from global frame accel and yaw
    pub fn global_accel_to_wheel_torques(&self, accel: &Accel, yaw: f64) -> WheelTorques {
        let rotation_mat = Matrix3::<f64>::new(
            cos(-yaw), sin(yaw) , 0.0,
            sin(-yaw), cos(-yaw), 0.0,
            0.0      , 0.0      , 1.0,
        );
        let accel_vec = nalgebra::Vector3::new(accel.linear.x, accel.linear.y, accel.angular.z);
        WheelTorques::from(self.wheel_transform_mat_inv * rotation_mat * accel_vec)
    }
}

fn pinv_3x4(a: Matrix3x4<f64>) -> Matrix4x3<f64> {
    let a_t = a.clone().transpose();
    let a_at = a * a_t.clone();
    let a_at_inv = a_at.try_inverse().unwrap();
    let a_inv = a_t * a_at_inv;
    a_inv
}