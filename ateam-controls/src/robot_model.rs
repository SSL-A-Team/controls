use libm::{sinf, cosf};
use crate::trajectory_params::{MAX_ROTATIONAL_VELOCITY, MAX_TRANSLATIONAL_VELOCITY};
use crate::{Matrix3f, Matrix3x4f, Matrix4x3f, Matrix6f, Matrix6x3f, Matrix8f, Matrix8x6f, Vector3f, Vector6f, Vector8f};
use crate::robot_physical_params::*;
use crate::{z_rotation_mat, pinv_3x4};


/// State: Vector6
///     x - x linear position (m)
///     y - y linear position (m)
///     z - z angular orientation (rad)
///     w - x linear velocity (m/s)
///     a - y linear velocity (m/s)
///     b - angular velocity around z axis (rad/s)
/// Pose: Vector3
///     x - x linear position (m)
///     y - y linear position (m)
///     z - z angular orientation (rad)
/// Twist: Vector3
///     x - x linear velocity (m/s)
///     y - y linear velocity (m/s)
///     z - angular velocity around z axis (rad/s)
/// Accel: Vector3
///     x - x linear acceleration (m/s^2)
///     y - y linear acceleration (m/s^2)
///     z - angular aceleration around z axis (rad/s^2)
/// Wheel Velocities: Vector4
///     x - front left angular velocity (rad/s)
///     y - back left angular velocity (rad/s)
///     z - back right angular velocity (rad/s)
///     w - front right angular velocity (rad/s)
/// Wheel Torques: Vector4
///     x - front left torque (N*m)
///     y - back left torque (N*m)
///     z - back right torque (N*m)
///     w - front right torque (N*m)

pub struct RobotModel {
    /// State estimate, global frame body level x, y, theta, dx/dt, dy/dt, dtheta/dt
    pub x: Vector6f,
    /// State covariance
    pub p: Matrix6f,
    /// State update matrix A
    pub a: Matrix6f,
    /// State update matrix B
    pub b: Matrix6x3f,
    /// State to sensor transform matrix
    pub h: Matrix8x6f,
    /// Robot actuator (wheel) space to robot frame cartesian space transform
    pub m: Matrix3x4f,
    /// Inverse of m
    pub m_inv: Matrix4x3f,
    /// Robot inirtial matrix, robot frame x_linear, y_linear, zaxis_moment
    pub i: Matrix3f,
    /// Inverse of i
    pub i_inv: Matrix3f,
    // Radius of robot wheels (m)
    pub r: f32,
    /// Kalman filter process noise covariance matrix
    pub kf_q: Matrix6f,
    /// Kalman filter sensor noise covariance matrix
    pub kf_r: Matrix8f,
    /// Kalman filter update period
    pub kf_dt: f32,
}

impl RobotModel {
    /// alpha: angle between robot y axis and segment between robot origin and robot front left wheel, same as angle between robot -y axis and segment between robot origin and robot front right wheel
    /// beta: angle between robot y axis and segment between robot origin and robot back left wheel, same as angle between robot -y axis and segment between robot origin and robot back right wheel
    /// l: distance from the robot center to each wheel
    /// r: radius of robot wheels
    /// mass: robot body mass
    /// iz: robot body moment of inirtia around z axis
    /// kf_dt: Kalman Filter state update period
    pub fn new(alpha: f32, beta: f32, l: f32, r: f32, mass: f32, iz: f32, kf_max_linear_vel: f32, kf_max_angular_vel: f32, kf_dt: f32) -> RobotModel {
        // Initialize unknown estimated state to zero
        let x = Vector6f::zeros();
        // Initialize state covariance to a large value
        // let p = Matrix6f::identity() * 10000.;
        // let p = Matrix6f::identity() * 100.;
        let p = Matrix6f::new(
            256., 0., 0., 0., 0.,0.,
            0., 256., 0., 0., 0., 0.,
            0., 0., 256., 0., 0., 0.,
            0., 0., 0., kf_max_linear_vel * kf_max_linear_vel, 0., 0.,
            0., 0., 0., 0., kf_max_linear_vel * kf_max_linear_vel, 0.,
            0., 0., 0., 0., 0., kf_max_angular_vel * kf_max_angular_vel,
        );
        let a = Matrix6f::new(
            1., 0., 0., kf_dt, 0., 0.,
            0., 1., 0., 0., kf_dt, 0.,
            0., 0., 1., 0., 0., kf_dt,
            0., 0., 0., 1., 0., 0.,
            0., 0., 0., 0., 1., 0.,
            0., 0., 0., 0., 0., 1.,
        );
        let b = Matrix6x3f::new(
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.,
            kf_dt, 0., 0.,
            0., kf_dt, 0.,
            0., 0., kf_dt,
        );
        let m = Matrix3x4f::new(
            -cosf(alpha), -cosf(beta),  cosf(beta), cosf(alpha),
             sinf(alpha), -sinf(beta), -sinf(beta), sinf(alpha),
             l          ,  l         ,  l         , l
        );
        let m_inv = pinv_3x4(m);
        let i = Matrix3f::new(
            mass, 0., 0.,
            0., mass, 0.,
            0., 0., iz,
        );
        let i_inv = i.try_inverse().unwrap();
        let h = Matrix8x6f::zeros();
        let kf_q = Matrix6f::identity();
        let kf_r = Matrix8f::identity();
        RobotModel { x, p, a, b, h, m, m_inv, i, i_inv, r, kf_q, kf_r, kf_dt}
    }

    pub fn new_from_constants(kf_dt: f32) -> RobotModel {
        RobotModel::new(WHEEL_ANGLE_ALPHA, WHEEL_ANGLE_BETA, WHEEL_DISTANCE, WHEEL_RADIUS, BODY_MASS, BODY_MOMENT_Z, MAX_TRANSLATIONAL_VELOCITY, MAX_ROTATIONAL_VELOCITY, kf_dt)
    }
    
    pub fn kf_predict(&mut self, u: Vector3f) {
        self.x = self.a * self.x + self.b * u;
        self.p = self.a * self.p * self.a.transpose() + self.kf_q;
    }

    pub fn kf_update(&mut self, z: Vector8f, mask_vision: bool, mask_encoder: bool, mask_gyro: bool) {
        self.update_h_transform(self.x[2], mask_vision, mask_encoder, mask_gyro);
        let h_x = self.h * self.x;
        let y = z - h_x;
        let k = self.p * self.h.transpose() * (self.h * self.p * self.h.transpose() + self.kf_r).try_inverse().unwrap();
        self.x = self.x + k * y;
        self.p = (Matrix6f::identity() - k * self.h) * self.p;
    }

    pub fn update_h_transform(&mut self, theta: f32, mask_vision: bool, mask_encoder: bool, mask_gyro: bool) {
        if mask_vision {
            self.h[(0, 0)] = 0.;
            self.h[(1, 1)] = 0.;
            self.h[(2, 2)] = 0.;
        } else {
            self.h[(0, 0)] = 1.;
            self.h[(1, 1)] = 1.;
            self.h[(2, 2)] = 1.;
        }

        if mask_encoder {
            for i in 3..7 {
                for j in 3..6 {
                    self.h[(i, j)] = 0.;
                }
            }
        } else {
            let h_enc = self.transform_twist2wheel(theta);
            for i in 0..4 {
                for j in 0..3 {
                    self.h[(3 + i, 3 + j)] = h_enc[(i, j)];
                }
            }
        }

        if mask_gyro {
            self.h[(7, 5)] = 0.;
        } else {
            self.h[(7, 5)] = 1.;
        }
    }

    /// Calculate transform matrix from wheel angular velocities to global frame twist
    pub fn transform_wheel2twist(&self, theta: f32) -> Matrix3x4f {
        self.r * z_rotation_mat(theta) * self.m_inv.transpose()
    }

    /// Calculate transform matrix from global frame twist to wheel angular velocities
    pub fn transform_twist2wheel(&self, theta: f32) -> Matrix4x3f {
        self.m.transpose() * z_rotation_mat(-theta) / self.r
    }

    /// Calculate transform matrix from wheel torques to global frame accel
    pub fn transform_wheel2accel(&self, theta: f32) -> Matrix3x4f {
        z_rotation_mat(theta)* self.i_inv * self.m / self.r
    }

    /// Calculate transform matrix from global frame accel to wheel torques
    pub fn transform_accel2wheel(&self, theta: f32) -> Matrix4x3f {
        self.r * self.m_inv * self.i * z_rotation_mat(-theta)
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test1() {
        // assert_eq!(result, 4);
    }
}

