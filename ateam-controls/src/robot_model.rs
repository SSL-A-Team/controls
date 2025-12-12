use nalgebra::{Matrix3, Matrix3x4, Matrix4x3, Vector3};
use libm::{sinf, cosf};
use crate::geometry::{Accel, Twist, Pose};


#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct WheelTorques {
    pub fl: f32,
    pub bl: f32,
    pub br: f32,
    pub fr: f32,
}

impl From<nalgebra::Vector4<f32>> for WheelTorques {
    fn from(v: nalgebra::Vector4<f32>) -> Self {
        WheelTorques {fl: v.x, bl: v.y, br: v.z, fr: v.w}
    }
}

impl From<WheelTorques> for nalgebra::Vector4<f32> {
    fn from(torques: WheelTorques) -> Self {
        Self::new(torques.fl, torques.bl, torques.br, torques.fr)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct WheelVelocities {
    pub fl: f32,
    pub bl: f32,
    pub br: f32,
    pub fr: f32,
}

impl From<nalgebra::Vector4<f32>> for WheelVelocities {
    fn from(v: nalgebra::Vector4<f32>) -> Self {
        WheelVelocities {fl: v.x, bl: v.y, br: v.z, fr: v.w}
    }
}

impl From<WheelVelocities> for nalgebra::Vector4<f32> {
    fn from(velocities: WheelVelocities) -> Self {
        Self::new(velocities.fl, velocities.bl, velocities.br, velocities.fr)
    }
}

pub fn global_frame_to_robot_frame_pose(robot_pose: &Pose, pose: Pose) -> Pose {
    todo!()
}

pub fn robot_frame_to_global_frame_pose(robot_pose: &Pose, pose: Pose) -> Pose {
    todo!()
}

pub fn global_frame_to_robot_frame_twist(robot_pose: &Pose, twist: Twist) -> Twist {
    todo!()
}

pub fn robot_frame_to_global_frame_twist(robot_pose: &Pose, twist: Twist) -> Twist {
    todo!()
}

pub fn global_frame_to_robot_frame_accel(robot_pose: &Pose, accel: Accel) -> Accel {
    todo!()
}

pub fn robot_frame_to_global_frame_accel(robot_pose: &Pose, accel: Accel) -> Accel {
    todo!()
}

struct RobotModel {
    wheel_transform_mat: Matrix3x4<f32>,
    wheel_transform_mat_inv: Matrix4x3<f32>,
}

impl RobotModel {
    pub fn new_from_alpha_beta_l(alpha: f32, beta: f32, l: f32) -> RobotModel {
        let mat = Matrix3x4::new(
            -cosf(alpha), -cosf(beta),  cosf(beta), cosf(alpha),
             sinf(alpha), -sinf(beta), -sinf(beta), sinf(alpha),
             l         ,  l        ,  l        , l
        );
        let mat_inv = pinv_3x4(mat);
        RobotModel {wheel_transform_mat: mat, wheel_transform_mat_inv: mat_inv}
    }

    /// Calculate local frame twist from wheel velocities
    pub fn wheel_velocities_to_local_twist(&self, wheel_velocities: &WheelVelocities) -> Twist {
        todo!();
    }

    /// Calculate local frame twist from wheel velocities
    pub fn wheel_velocities_to_global_twist(&self, wheel_velocities: &WheelVelocities, theta: f32) -> Twist {
        todo!();
    }

    /// Calculate wheel velocities from local frame twist
    pub fn local_twist_to_wheel_velocities(&self, twist: &Twist) -> WheelVelocities {
        let velocities = nalgebra::Vector3::<f32>::new(
            twist.linear.x, twist.linear.y, twist.angular.z,
        );
        WheelVelocities::from(self.wheel_transform_mat.transpose() * velocities)
    }

    // Calculate local frame accel from wheel torques
    pub fn wheel_torques_to_local_accel(&self, torques: &WheelTorques) -> Accel {
        let torques_vec = nalgebra::Vector4::from(*torques);
        let accelerations = self.wheel_transform_mat * torques_vec;
        Accel {
            linear: Vector3::<f32>::new(accelerations[(0, 0)], accelerations[(1, 0)], 0.0 ),
            angular: Vector3::<f32>::new(0.0, 0.0, accelerations[(2, 0)]),
        }
    }

    // Calculate wheel torques from local frame accel
    pub fn local_accel_to_wheel_torques(&self, accel: &Accel) -> WheelTorques {
        let accel_vec = nalgebra::Vector3::new(accel.linear.x, accel.linear.y, accel.angular.z);
        WheelTorques::from(self.wheel_transform_mat_inv * accel_vec)
    }

    /// Calculate wheel velocities from global frame twist and robot theta
    pub fn global_twist_to_wheel_velocities(&self, twist: &Twist, theta: f32) -> WheelVelocities {
        let rotation_mat = Matrix3::<f32>::new(
            cosf(theta), -sinf(theta), 0.0,
            sinf(theta),  cosf(theta), 0.0,
            0.0     ,  0.0     , 1.0,
        );
        let velocities = nalgebra::Vector3::<f32>::new(
            twist.linear.x, twist.linear.y, twist.angular.z,
        );
        WheelVelocities::from((rotation_mat * self.wheel_transform_mat).transpose() * velocities)
    }

    // Calculate global frame accel from wheel torques and theta
    pub fn wheel_torques_to_global_accel(&self, torques: &WheelTorques, theta: f32) -> Accel {
        let rotation_mat = Matrix3::<f32>::new(
            cosf(theta),-sinf(theta), 0.0,
            sinf(theta), cosf(theta), 0.0,
            0.0   , 0.0   , 1.0,
        );
        let torques_vec = nalgebra::Vector4::from(*torques);
        let accelerations = rotation_mat * self.wheel_transform_mat * torques_vec;
        Accel {
            linear: Vector3::<f32>::new(accelerations[(0, 0)], accelerations[(1, 0)], 0.0),
            angular: Vector3::<f32>::new(0.0, 0.0, accelerations[(2, 0)]),
        }
    }

    // Calculate wheel torques from global frame accel and theta
    pub fn global_accel_to_wheel_torques(&self, accel: &Accel, theta: f32) -> WheelTorques {
        let rotation_mat = Matrix3::<f32>::new(
            cosf(-theta), sinf(theta) , 0.0,
            sinf(-theta), cosf(-theta), 0.0,
            0.0      , 0.0      , 1.0,
        );
        let accel_vec = nalgebra::Vector3::new(accel.linear.x, accel.linear.y, accel.angular.z);
        WheelTorques::from(self.wheel_transform_mat_inv * rotation_mat * accel_vec)
    }
}

fn pinv_3x4(a: Matrix3x4<f32>) -> Matrix4x3<f32> {
    let a_t = a.clone().transpose();
    let a_at = a * a_t.clone();
    let a_at_inv = a_at.try_inverse().unwrap();
    let a_inv = a_t * a_at_inv;
    a_inv
}