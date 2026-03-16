use libm::{sinf, cosf};
use nalgebra::matrix;
use core::f32::consts::PI;
use crate::{ControlsError, Matrix3f, Matrix3x4f, Matrix4x3f, Matrix6f, Matrix6x3f, Matrix8f, Matrix8x6f, Vector2f, Vector3f, Vector4f, Vector6f, Vector8f};
use crate::{z_rotation_mat, wrap_angle};
use crate::physical_params;
use crate::kalman_params;
use crate::trajectory_params;


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

#[repr(C)]
#[derive(Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct KalmanFilterParams {
    #[cfg_attr(feature = "serde", serde(rename = "KF_PROCESS_STD_POS_LINEAR"))]
    pub process_noise_std_pos_linear: f32,
    #[cfg_attr(feature = "serde", serde(rename = "KF_PROCESS_STD_POS_ANGULAR"))]
    pub process_noise_std_pos_angular: f32,
    #[cfg_attr(feature = "serde", serde(rename = "KF_PROCESS_STD_VEL_LINEAR"))]
    pub process_noise_std_vel_linear: f32,
    #[cfg_attr(feature = "serde", serde(rename = "KF_PROCESS_STD_VEL_ANGULAR"))]
    pub process_noise_std_vel_angular: f32,
    #[cfg_attr(feature = "serde", serde(rename = "KF_VISION_STD_LINEAR"))]
    pub measurement_noise_std_vision_pos_linear: f32,
    #[cfg_attr(feature = "serde", serde(rename = "KF_VISION_STD_ANGULAR"))]
    pub measurement_noise_std_vision_pos_angular: f32,
    #[cfg_attr(feature = "serde", serde(rename = "KF_ENCODER_STD_ANGULAR"))]
    pub measurement_noise_std_encoder_vel_angular: f32,
    #[cfg_attr(feature = "serde", serde(rename = "KF_GYRO_STD_ANGULAR"))]
    pub measurement_noise_std_gyro_vel_angular: f32,
    #[cfg_attr(feature = "serde", serde(rename = "KF_MAX_POS_LINEAR"))]
    pub max_pos_linear: f32,  // For initialization
    #[cfg_attr(feature = "serde", serde(rename = "KF_MAX_POS_ANGULAR"))]
    pub max_pos_angular: f32,  // For initialization
    #[cfg_attr(feature = "serde", serde(rename = "KF_MAX_VEL_LINEAR"))]
    pub max_vel_linear: f32,  // For initialization
    #[cfg_attr(feature = "serde", serde(rename = "KF_MAX_VEL_ANGULAR"))]
    pub max_vel_angular: f32,  // For initialization
}

impl Default for KalmanFilterParams {
    fn default() -> Self {
        KalmanFilterParams {
            process_noise_std_pos_linear: kalman_params::PROCESS_NOISE_STDDEV_POS_LINEAR,
            process_noise_std_pos_angular: kalman_params::PROCESS_NOISE_STDDEV_POS_ANGULAR,
            process_noise_std_vel_linear: kalman_params::PROCESS_NOISE_STDDEV_VEL_LINEAR,
            process_noise_std_vel_angular: kalman_params::PROCESS_NOISE_STDDEV_VEL_ANGULAR,
            measurement_noise_std_vision_pos_linear: kalman_params::MEASUREMENT_NOISE_STDDEV_VISION_POS_LINEAR,
            measurement_noise_std_vision_pos_angular: kalman_params::MEASUREMENT_NOISE_STDDEV_VISION_POS_ANGULAR,
            measurement_noise_std_encoder_vel_angular: kalman_params::MEASUREMENT_NOISE_STDDEV_ENCODER_VEL_ANGULAR,
            measurement_noise_std_gyro_vel_angular: kalman_params::MEASUREMENT_NOISE_STDDEV_GYRO_VEL_ANGULAR,
            max_pos_linear: 64.,  // 64 meters should be large enough for initial uncertainty
            max_pos_angular: 2. * PI,
            max_vel_linear: trajectory_params::MAX_VEL_LINEAR,
            max_vel_angular: trajectory_params::MAX_VEL_ANGULAR,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RobotPhysicalParams {
    #[cfg_attr(feature = "serde", serde(rename = "PHYS_WHEEL_ANGLE_ALPHA"))]
    pub alpha: f32,
    #[cfg_attr(feature = "serde", serde(rename = "PHYS_WHEEL_ANGLE_BETA"))]
    pub beta: f32,
    #[cfg_attr(feature = "serde", serde(rename = "PHYS_WHEEL_DISTANCE"))]
    pub l: f32,
    #[cfg_attr(feature = "serde", serde(rename = "PHYS_WHEEL_RADIUS"))]
    pub r: f32,
    #[cfg_attr(feature = "serde", serde(rename = "PHYS_BODY_MASS"))]
    pub mass: f32,
    #[cfg_attr(feature = "serde", serde(rename = "PHYS_BODY_MOMENT_Z"))]
    pub iz: f32,
    #[cfg_attr(feature = "serde", serde(rename = "PHYS_MOTOR_TORQUE_CONSTANT"))]
    pub motor_torque_constant: f32,
    #[cfg_attr(feature = "serde", serde(rename = "PHYS_MOTOR_EFFICIENCY_FACTOR"))]
    pub motor_efficiency_factor: f32,
    #[cfg_attr(feature = "serde", serde(rename = "PHYS_COLOUMB_FRICTION_COEFFICIENT_LINEAR"))]
    pub coulomb_friction_coefficient_linear: f32,
    #[cfg_attr(feature = "serde", serde(rename = "PHYS_COLOUMB_FRICTION_COEFFICIENT_ANGULAR"))]
    pub coulomb_friction_coefficient_angular: f32,
    #[cfg_attr(feature = "serde", serde(rename = "PHYS_VISCOUS_FRICTION_COEFFICIENT_LINEAR"))]
    pub viscous_friction_coefficient_linear: f32,
    #[cfg_attr(feature = "serde", serde(rename = "PHYS_VISCOUS_FRICTION_COEFFICIENT_ANGULAR"))]
    pub viscous_friction_coefficient_angular: f32,
}

impl Default for RobotPhysicalParams {
    fn default() -> Self {
        RobotPhysicalParams {
            alpha: physical_params::WHEEL_ANGLE_ALPHA,
            beta: physical_params::WHEEL_ANGLE_BETA,
            l: physical_params::WHEEL_DISTANCE,
            r: physical_params::WHEEL_RADIUS,
            mass: physical_params::BODY_MASS,
            iz: physical_params::BODY_MOMENT_Z,
            motor_torque_constant: physical_params::MOTOR_TORQUE_CONSTANT,
            motor_efficiency_factor: physical_params::MOTOR_EFFICIENCY_FACTOR,
            coulomb_friction_coefficient_linear: physical_params::COULOMB_FRICTION_COEFFICIENT_LINEAR,
            coulomb_friction_coefficient_angular: physical_params::COULOMB_FRICTION_COEFFICIENT_ANGULAR,
            viscous_friction_coefficient_linear: physical_params::VISCOUS_FRICTION_COEFFICIENT_LINEAR,
            viscous_friction_coefficient_angular: physical_params::VISCOUS_FRICTION_COEFFICIENT_ANGULAR,
        }
    }
}

#[derive(Default, Copy, Clone)]
pub struct RobotModel {
    /// Kalman filter parameters struct for easy updating and access to individual params
    pub kf_params: KalmanFilterParams,
    /// Robot physical parameters struct for easy updating and access to individual params
    pub physical_params: RobotPhysicalParams,
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
    /// Robot inertial matrix, robot frame x_linear, y_linear, zaxis_moment
    pub i: Matrix3f,
    /// Inverse of i
    pub i_inv: Matrix3f,
    // Radius of robot wheels (m)
    pub r: f32,
    /// Kalman filter process noise covariance matrix
    pub kf_q: Matrix6f,
    /// Kalman filter sensor noise covariance matrix
    pub kf_r: Matrix8f,
}

impl RobotModel {
    /// alpha: angle between robot y axis and segment between robot origin and robot front left wheel, same as angle between robot -y axis and segment between robot origin and robot front right wheel
    /// beta: angle between robot y axis and segment between robot origin and robot back left wheel, same as angle between robot -y axis and segment between robot origin and robot back right wheel
    /// l: distance from the robot center to each wheel
    /// r: radius of robot wheels
    /// mass: robot body mass
    /// iz: robot body moment of inertia around z axis
    /// kf_dt: Kalman Filter state update period
    pub fn new(kf_dt: f32, kf_params: KalmanFilterParams, physical_params: RobotPhysicalParams) -> Result<RobotModel, ControlsError> {
        let mut model = RobotModel::default();
        // Initialize unknown estimated state to zero
        model.x = Vector6f::zeros();
        // Initialize state update matrices
        model.a = Matrix6f::new(
            1., 0., 0., kf_dt, 0., 0.,
            0., 1., 0., 0., kf_dt, 0.,
            0., 0., 1., 0., 0., kf_dt,
            0., 0., 0., 1., 0., 0.,
            0., 0., 0., 0., 1., 0.,
            0., 0., 0., 0., 0., 1.,
        );
        model.b = Matrix6x3f::new(
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.,
            kf_dt, 0., 0.,
            0., kf_dt, 0.,
            0., 0., kf_dt,
        );
        model.update_kf_params(kf_params);
        model.update_physical_params(physical_params)?;
        Ok(model)
    }

    pub fn new_from_default_params(kf_dt: f32) -> Result<RobotModel, ControlsError> {
        RobotModel::new(kf_dt, KalmanFilterParams::default(), RobotPhysicalParams::default())
    }

    pub fn update_kf_params(&mut self, kf_params: KalmanFilterParams) {
        self.kf_params = kf_params;
        // Reset state covariance to max reasonable values
        self.p = Matrix6f::new(
            kf_params.max_pos_linear*kf_params.max_pos_linear, 0., 0., 0., 0.,0.,
            0., kf_params.max_pos_linear*kf_params.max_pos_linear, 0., 0., 0., 0.,
            0., 0., kf_params.max_pos_angular*kf_params.max_pos_angular, 0., 0., 0.,
            0., 0., 0., kf_params.max_vel_linear * kf_params.max_vel_linear, 0., 0.,
            0., 0., 0., 0., kf_params.max_vel_linear * kf_params.max_vel_linear, 0.,
            0., 0., 0., 0., 0., kf_params.max_vel_angular * kf_params.max_vel_angular,
        );
        // Initialize Kalman filter noise covariance matrices based on provided std parameters
        self.kf_q = Matrix6f::new(
            kf_params.process_noise_std_pos_linear*kf_params.process_noise_std_pos_linear, 0., 0., 0., 0., 0.,
            0., kf_params.process_noise_std_pos_linear*kf_params.process_noise_std_pos_linear, 0., 0., 0., 0.,
            0., 0., kf_params.process_noise_std_pos_angular*kf_params.process_noise_std_pos_angular, 0., 0., 0.,
            0., 0., 0., kf_params.process_noise_std_vel_linear*kf_params.process_noise_std_vel_linear, 0., 0.,
            0., 0., 0., 0., kf_params.process_noise_std_vel_linear*kf_params.process_noise_std_vel_linear, 0.,
            0., 0., 0., 0., 0., kf_params.process_noise_std_vel_angular*kf_params.process_noise_std_vel_angular,
        );
        self.kf_r = matrix![
            kf_params.measurement_noise_std_vision_pos_linear*kf_params.measurement_noise_std_vision_pos_linear, 0., 0., 0., 0., 0., 0., 0.;
            0., kf_params.measurement_noise_std_vision_pos_linear*kf_params.measurement_noise_std_vision_pos_linear, 0., 0., 0., 0., 0., 0.;
            0., 0., kf_params.measurement_noise_std_vision_pos_angular*kf_params.measurement_noise_std_vision_pos_angular, 0., 0., 0., 0., 0.;
            0., 0., 0., kf_params.measurement_noise_std_encoder_vel_angular*kf_params.measurement_noise_std_encoder_vel_angular, 0., 0., 0., 0.;
            0., 0., 0., 0., kf_params.measurement_noise_std_encoder_vel_angular*kf_params.measurement_noise_std_encoder_vel_angular, 0., 0., 0.;
            0., 0., 0., 0., 0., kf_params.measurement_noise_std_encoder_vel_angular*kf_params.measurement_noise_std_encoder_vel_angular, 0., 0.;
            0., 0., 0., 0., 0., 0., kf_params.measurement_noise_std_encoder_vel_angular*kf_params.measurement_noise_std_encoder_vel_angular, 0.;
            0., 0., 0., 0., 0., 0., 0., kf_params.measurement_noise_std_gyro_vel_angular*kf_params.measurement_noise_std_gyro_vel_angular;
        ];
    }

    pub fn update_physical_params(&mut self, physical_params: RobotPhysicalParams) -> Result<(), ControlsError> {
        self.physical_params = physical_params;
        // Initialize robot physical matrices and params
        self.r = physical_params.r;
        self.m = Matrix3x4f::new(
            -cosf(physical_params.alpha), -cosf(physical_params.beta),  cosf(physical_params.beta), cosf(physical_params.alpha),
             sinf(physical_params.alpha), -sinf(physical_params.beta), -sinf(physical_params.beta), sinf(physical_params.alpha),
             physical_params.l          ,  physical_params.l         ,  physical_params.l         , physical_params.l
        );
        self.m_inv = self.m.pseudo_inverse(1e-6).map_err(|_| ControlsError::SingularMatrix)?;
        self.i = Matrix3f::new(
            physical_params.mass, 0., 0.,
            0., physical_params.mass, 0.,
            0., 0., physical_params.iz,
        );
        self.i_inv = self.i.try_inverse().ok_or(ControlsError::SingularMatrix)?;
        Ok(())
    }

    pub fn kf_predict(&mut self, u: Vector3f) {
        self.x = self.a * self.x + self.b * u;
        self.x[2] = wrap_angle(self.x[2]);
        self.p = self.a * self.p * self.a.transpose() + self.kf_q;
    }

    pub fn kf_update(&mut self, z: Vector8f, mask_vision: bool, mask_encoder: bool, mask_gyro: bool) -> Result<(), ControlsError> {
        self.update_h_transform(self.x[2], mask_vision, mask_encoder, mask_gyro);
        let h_x = self.h * self.x;
        let mut y = z - h_x;
        // Wrap the angular residual for vision heading to [-pi, pi]
        y[2] = wrap_angle(y[2]);
        let k = self.p * self.h.transpose() * (self.h * self.p * self.h.transpose() + self.kf_r).try_inverse().ok_or(ControlsError::SingularMatrix)?;
        self.x = self.x + k * y;
        self.x[2] = wrap_angle(self.x[2]);
        self.p = (Matrix6f::identity() - k * self.h) * self.p;
        Ok(())
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
        z_rotation_mat(theta) * self.i_inv * self.m / self.r
    }

    /// Calculate transform matrix from global frame accel to wheel torques
    pub fn transform_accel2wheel(&self, theta: f32) -> Matrix4x3f {
        self.r * self.m_inv * self.i * z_rotation_mat(-theta)
    }

    /// Calculate wheel currents in amps from wheel torques in Nm
    pub fn torques_to_currents(&self, torques: Vector4f) -> Vector4f {
        torques / self.physical_params.motor_torque_constant / self.physical_params.motor_efficiency_factor
    }

    pub fn compute_friction_force(&self, body_twist_cmd: Vector3f) -> Vector3f {
        let vel_linear: Vector2f = body_twist_cmd.fixed_rows::<2>(0).into();
        let vel_angular: f32 = body_twist_cmd[2];

        let vel_linear_magnitude = vel_linear.magnitude();
        let vel_linear_dir = if vel_linear_magnitude > 0. {
            vel_linear / vel_linear_magnitude
        } else {
            Vector2f::zeros()
        };

        let vel_angular_dir = if vel_angular.abs() > 0. { 
            vel_angular.signum()
        } else {
            0.
        };

        let linear_friction = self.physical_params.viscous_friction_coefficient_linear * (- vel_linear) + self.physical_params.coulomb_friction_coefficient_linear * (- vel_linear_dir);
        let angular_friction = self.physical_params.viscous_friction_coefficient_angular * (- vel_angular) + self.physical_params.coulomb_friction_coefficient_angular * (- vel_angular_dir);

        let friction_force = linear_friction.push(angular_friction);

        friction_force
    }
}
