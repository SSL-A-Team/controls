use libm::{sinf, cosf, sqrtf};
use nalgebra::matrix;
use crate::{ControlsError, Matrix3f, Matrix3x4f, Matrix4x3f, Matrix6f, Matrix6x3f, Matrix8f, Matrix8x6f, Vector3f, Vector4f, Vector6f, Vector8f};
use crate::{z_rotation_mat, wrap_angle};


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
pub struct KalmanFilterParams {
    pub process_noise_std_pos_linear: f32,
    pub process_noise_std_pos_angular: f32,
    pub process_noise_std_vel_linear: f32,
    pub process_noise_std_vel_angular: f32,
    pub measurement_noise_std_vision_pos_linear: f32,
    pub measurement_noise_std_vision_pos_angular: f32,
    pub measurement_noise_std_encoder_vel_angular: f32,
    pub measurement_noise_std_gyro_vel_angular: f32,
    pub max_pos_linear: f32,  // For initialization
    pub max_pos_angular: f32,  // For initialization
    pub max_vel_linear: f32,  // For initialization
    pub max_vel_angular: f32,  // For initialization
}

impl Default for KalmanFilterParams {
    fn default() -> Self {
        use crate::defaults::*;
        Self {
            process_noise_std_pos_linear: DEFAULT_KF_PROCESS_STD_POS_LINEAR,
            process_noise_std_pos_angular: DEFAULT_KF_PROCESS_STD_POS_ANGULAR,
            process_noise_std_vel_linear: DEFAULT_KF_PROCESS_STD_VEL_LINEAR,
            process_noise_std_vel_angular: DEFAULT_KF_PROCESS_STD_VEL_ANGULAR,
            measurement_noise_std_vision_pos_linear: DEFAULT_KF_MEASUREMENT_STD_VISION_POS_LINEAR,
            measurement_noise_std_vision_pos_angular: DEFAULT_KF_MEASUREMENT_STD_VISION_POS_ANGULAR,
            measurement_noise_std_encoder_vel_angular: DEFAULT_KF_MEASUREMENT_STD_ENCODER_VEL_ANGULAR,
            measurement_noise_std_gyro_vel_angular: DEFAULT_KF_MEASUREMENT_STD_GYRO_VEL_ANGULAR,
            max_pos_linear: DEFAULT_KF_MAX_POS_LINEAR,
            max_pos_angular: DEFAULT_KF_MAX_POS_ANGULAR,
            max_vel_linear: DEFAULT_KF_MAX_VEL_LINEAR,
            max_vel_angular: DEFAULT_KF_MAX_VEL_ANGULAR,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RobotPhysicalParams {
    pub alpha: f32,
    pub beta: f32,
    pub l: f32,
    pub r: f32,
    pub mass: f32,
    pub iz: f32,
    pub motor_torque_constant: f32,
    pub motor_efficiency_factor: f32,
    /// Local-frame x-axis (forward/backward) Coulomb friction coefficient.
    pub coulomb_friction_coefficient_linear_x: f32,
    /// Local-frame y-axis (strafe) Coulomb friction coefficient.
    pub coulomb_friction_coefficient_linear_y: f32,
    pub coulomb_friction_coefficient_angular: f32,
    /// Local-frame x-axis viscous friction coefficient.
    pub viscous_friction_coefficient_linear_x: f32,
    /// Local-frame y-axis viscous friction coefficient.
    pub viscous_friction_coefficient_linear_y: f32,
    pub viscous_friction_coefficient_angular: f32,
}

impl Default for RobotPhysicalParams {
    fn default() -> Self {
        use crate::defaults::*;
        Self {
            alpha: DEFAULT_PHYS_ALPHA,
            beta: DEFAULT_PHYS_BETA,
            l: DEFAULT_PHYS_L,
            r: DEFAULT_PHYS_R,
            mass: DEFAULT_PHYS_MASS,
            iz: DEFAULT_PHYS_IZ,
            motor_torque_constant: DEFAULT_PHYS_MOTOR_TORQUE_CONSTANT,
            motor_efficiency_factor: DEFAULT_PHYS_MOTOR_EFFICIENCY_FACTOR,
            coulomb_friction_coefficient_linear_x: DEFAULT_PHYS_COULOMB_FRICTION_LINEAR_X,
            coulomb_friction_coefficient_linear_y: DEFAULT_PHYS_COULOMB_FRICTION_LINEAR_Y,
            coulomb_friction_coefficient_angular: DEFAULT_PHYS_COULOMB_FRICTION_ANGULAR,
            viscous_friction_coefficient_linear_x: DEFAULT_PHYS_VISCOUS_FRICTION_LINEAR_X,
            viscous_friction_coefficient_linear_y: DEFAULT_PHYS_VISCOUS_FRICTION_LINEAR_Y,
            viscous_friction_coefficient_angular: DEFAULT_PHYS_VISCOUS_FRICTION_ANGULAR,
        }
    }
}

#[derive(Copy, Clone)]
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

impl Default for RobotModel {
    fn default() -> Self {
        use crate::defaults::DEFAULT_CONTROL_DT;
        RobotModel::new(
            DEFAULT_CONTROL_DT,
            KalmanFilterParams::default(),
            RobotPhysicalParams::default(),
        )
        .expect("Default RobotModel parameters must be valid")
    }
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
        let mut model = RobotModel {
            kf_params,
            physical_params,
            x: Vector6f::zeros(),
            p: Matrix6f::zeros(),
            a: Matrix6f::zeros(),
            b: Matrix6x3f::zeros(),
            h: Matrix8x6f::zeros(),
            m: Matrix3x4f::zeros(),
            m_inv: Matrix4x3f::zeros(),
            i: Matrix3f::zeros(),
            i_inv: Matrix3f::zeros(),
            r: 0.0,
            kf_q: Matrix6f::zeros(),
            kf_r: Matrix8f::zeros(),
        };
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

    pub fn reset(&mut self) {
        self.x = Vector6f::zeros();
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

    /// Snap the KF position state [x, y, θ] to the provided global pose.
    ///
    /// Velocity state [vx, vy, ω] is left untouched. The position block of the
    /// covariance matrix P is reset: all cross-terms between position and velocity
    /// are zeroed (decoupling the snapped pose from prior accumulated uncertainty),
    /// and the position diagonal is set to the vision measurement variance so the
    /// next predict step grows P from a known baseline.
    ///
    /// When snapping after a large position jump (e.g. robot repositioned by hand),
    /// always follow with `kf_set_vel` so the velocity state is consistent.
    pub fn kf_set_pose(&mut self, pose: Vector3f) {
        self.x[0] = pose.x;
        self.x[1] = pose.y;
        self.x[2] = wrap_angle(pose.z);

        // Zero all 6 rows of the position columns (cols 0-2), then
        // zero all 6 columns of the position rows (rows 0-2).
        // Together these clear the full pos-pos and pos-vel cross blocks.
        self.p.fixed_view_mut::<6, 3>(0, 0).fill(0.0);
        self.p.fixed_view_mut::<3, 6>(0, 0).fill(0.0);

        // Set position diagonal to vision measurement variance.
        let var_lin = self.kf_params.measurement_noise_std_vision_pos_linear
            * self.kf_params.measurement_noise_std_vision_pos_linear;
        let var_ang = self.kf_params.measurement_noise_std_vision_pos_angular
            * self.kf_params.measurement_noise_std_vision_pos_angular;
        self.p[(0, 0)] = var_lin;
        self.p[(1, 1)] = var_lin;
        self.p[(2, 2)] = var_ang;
    }

    /// Snap the KF velocity state [vx, vy, ω] to the provided global-frame twist.
    ///
    /// Position state [x, y, θ] is left untouched. The velocity block of the
    /// covariance matrix P is reset: all cross-terms between velocity and position
    /// are zeroed, and the velocity diagonal is set to the max-velocity variance
    /// so the next predict step grows P from a known baseline. The large initial
    /// variance converges quickly once encoder and gyro measurements resume.
    ///
    /// Call this alongside `kf_set_pose` whenever a large position jump is accepted,
    /// so the velocity state does not reflect motion toward the old position.
    pub fn kf_set_vel(&mut self, vel: Vector3f) {
        self.x[3] = vel.x;
        self.x[4] = vel.y;
        self.x[5] = vel.z;

        // Zero all 6 rows of the velocity columns (cols 3-5), then
        // zero all 6 columns of the velocity rows (rows 3-5).
        // Together these clear the full vel-vel and vel-pos cross blocks.
        self.p.fixed_view_mut::<6, 3>(0, 3).fill(0.0);
        self.p.fixed_view_mut::<3, 6>(3, 0).fill(0.0);

        // Set velocity diagonal to max-velocity variance.
        let var_lin = self.kf_params.max_vel_linear * self.kf_params.max_vel_linear;
        let var_ang = self.kf_params.max_vel_angular * self.kf_params.max_vel_angular;
        self.p[(3, 3)] = var_lin;
        self.p[(4, 4)] = var_lin;
        self.p[(5, 5)] = var_ang;
    }

    pub fn kf_update(&mut self, z: Vector8f, mask_vision: bool, mask_encoder: bool, mask_gyro: bool) -> Result<(), ControlsError> {
        self.update_h_transform(self.x[2], mask_vision, mask_encoder, mask_gyro);
        // Velocity-schedule the vision measurement noise: vision lag makes a frame
        // increasingly stale (and thus less trustworthy) as the robot moves faster,
        // while at rest vision is accurate to a few mm. Inflate the vision R diagonal
        // from a small zero-velocity floor up to the configured value based on the
        // current estimated speed. Cheap: a few FLOPs, only on vision-bearing updates.
        let mut r = self.kf_r;
        if mask_vision {
            let (var_lin, var_ang) = self.scheduled_vision_variance();
            r[(0, 0)] = var_lin;
            r[(1, 1)] = var_lin;
            r[(2, 2)] = var_ang;
        }
        let h_x = self.h * self.x;
        let mut y = z - h_x;
        // Wrap the angular residual for vision heading to [-pi, pi]
        y[2] = wrap_angle(y[2]);
        let k = self.p * self.h.transpose() * (self.h * self.p * self.h.transpose() + r).try_inverse().ok_or(ControlsError::SingularMatrix)?;
        self.x = self.x + k * y;
        self.x[2] = wrap_angle(self.x[2]);
        self.p = (Matrix6f::identity() - k * self.h) * self.p;
        Ok(())
    }

    /// Compute the velocity-scheduled vision measurement variances (linear, angular).
    ///
    /// The std is linearly interpolated from a zero-velocity floor up to the
    /// configured `measurement_noise_std_vision_pos_*` value (reached at the
    /// reference speed) using the current estimated speed, then squared. Returns
    /// `(var_linear, var_angular)`.
    fn scheduled_vision_variance(&self) -> (f32, f32) {
        use crate::defaults::{
            DEFAULT_KF_VISION_SCHED_STD_ANGULAR_ZERO, DEFAULT_KF_VISION_SCHED_STD_LINEAR_ZERO,
            DEFAULT_KF_VISION_SCHED_VEL_REF_ANGULAR, DEFAULT_KF_VISION_SCHED_VEL_REF_LINEAR,
        };

        let speed_lin = sqrtf(self.x[3] * self.x[3] + self.x[4] * self.x[4]);
        let speed_ang = if self.x[5] < 0.0 { -self.x[5] } else { self.x[5] };

        let std_lin = DEFAULT_KF_VISION_SCHED_STD_LINEAR_ZERO
            + (self.kf_params.measurement_noise_std_vision_pos_linear
                - DEFAULT_KF_VISION_SCHED_STD_LINEAR_ZERO)
                * (speed_lin / DEFAULT_KF_VISION_SCHED_VEL_REF_LINEAR);
        let std_ang = DEFAULT_KF_VISION_SCHED_STD_ANGULAR_ZERO
            + (self.kf_params.measurement_noise_std_vision_pos_angular
                - DEFAULT_KF_VISION_SCHED_STD_ANGULAR_ZERO)
                * (speed_ang / DEFAULT_KF_VISION_SCHED_VEL_REF_ANGULAR);

        (std_lin * std_lin, std_ang * std_ang)
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

    /// Compute the friction-only force/torque that opposes a body twist.
    ///
    /// **Frame assumption:** the input ``body_twist_cmd`` must be in the
    /// robot's **local frame** ``(local_vx, local_vy, v_theta)``. The output
    /// is also in the local frame ``[Fx_local, Fy_local, Tz]``. Callers
    /// operating in the global frame must rotate the twist by ``R_z(-theta)``
    /// before calling and rotate the returned force by ``R_z(theta)`` after.
    ///
    /// Per-axis model (independent x and y constants because strafing and
    /// forward/backward friction can differ noticeably):
    ///
    /// ```text
    /// Fx = -c_visc_x * vx - c_coul_x * sign(vx)
    /// Fy = -c_visc_y * vy - c_coul_y * sign(vy)
    /// Tz = -c_visc_ang * v_theta - c_coul_ang * sign(v_theta)
    /// ```
    pub fn compute_friction_force(&self, body_twist_cmd: Vector3f) -> Vector3f {
        let vx = body_twist_cmd[0];
        let vy = body_twist_cmd[1];
        let v_theta = body_twist_cmd[2];

        let sign = |v: f32| if v > 0.0 { 1.0 } else if v < 0.0 { -1.0 } else { 0.0 };

        let fx = -self.physical_params.viscous_friction_coefficient_linear_x * vx
            - self.physical_params.coulomb_friction_coefficient_linear_x * sign(vx);
        let fy = -self.physical_params.viscous_friction_coefficient_linear_y * vy
            - self.physical_params.coulomb_friction_coefficient_linear_y * sign(vy);
        let tz = -self.physical_params.viscous_friction_coefficient_angular * v_theta
            - self.physical_params.coulomb_friction_coefficient_angular * sign(v_theta);

        Vector3f::new(fx, fy, tz)
    }

    pub fn get_state(&self) -> Vector6f {
        self.x
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vector3f;
    use core::f32::consts::PI;

    fn test_kf_params() -> KalmanFilterParams {
        KalmanFilterParams {
            process_noise_std_pos_linear: 0.01,
            process_noise_std_pos_angular: 0.05,
            process_noise_std_vel_linear: 0.02,
            process_noise_std_vel_angular: 0.1,
            measurement_noise_std_vision_pos_linear: 1.0,
            measurement_noise_std_vision_pos_angular: 3.14,
            measurement_noise_std_encoder_vel_angular: 50.0,
            measurement_noise_std_gyro_vel_angular: 0.015,
            max_pos_linear: 64.0,
            max_pos_angular: 3.14,
            max_vel_linear: 3.0,
            max_vel_angular: 3.0 * PI,
        }
    }

    fn test_phys_params() -> RobotPhysicalParams {
        RobotPhysicalParams {
            alpha: PI / 6.0,
            beta: PI / 4.0,
            l: 0.0814,
            r: 0.030,
            mass: 2.7,
            iz: 0.008,
            motor_torque_constant: 0.0335,
            motor_efficiency_factor: 13.0,
            coulomb_friction_coefficient_linear_x: 2.058,
            coulomb_friction_coefficient_linear_y: 2.058,
            coulomb_friction_coefficient_angular: 0.05,
            viscous_friction_coefficient_linear_x: 5.0,
            viscous_friction_coefficient_linear_y: 5.0,
            viscous_friction_coefficient_angular: 0.0063,
        }
    }

    fn test_model(kf_dt: f32) -> RobotModel {
        RobotModel::new(kf_dt, test_kf_params(), test_phys_params()).unwrap()
    }

    #[test]
    fn new_succeeds() {
        let model = test_model(0.01);
        assert_eq!(model.x, Vector6f::zeros());
    }

    #[test]
    fn state_initially_zero() {
        let model = test_model(0.01);
        for i in 0..6 {
            assert!((model.x[i]).abs() < 1e-9);
        }
    }

    #[test]
    fn set_and_get_state() {
        let mut model = test_model(0.01);
        let state = Vector6f::new(1.0, 2.0, 0.5, 0.1, -0.2, 0.3);
        model.x = state;
        let out = model.get_state();
        for i in 0..6 {
            assert!((out[i] - state[i]).abs() < 1e-9);
        }
    }

    #[test]
    fn wheel2twist_and_back() {
        let model = test_model(0.01);
        let theta = 0.0;
        let w2t = model.transform_wheel2twist(theta);
        let t2w = model.transform_twist2wheel(theta);
        let result = w2t * t2w;
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((result[(i, j)] - expected).abs() < 1e-4,
                    "w2t * t2w [{},{}] = {}, expected {}", i, j, result[(i, j)], expected);
            }
        }
    }

    #[test]
    fn wheel2accel_and_back() {
        let model = test_model(0.01);
        let theta = 0.0;
        let w2a = model.transform_wheel2accel(theta);
        let a2w = model.transform_accel2wheel(theta);
        let result = w2a * a2w;
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((result[(i, j)] - expected).abs() < 1e-4,
                    "w2a * a2w [{},{}] = {}, expected {}", i, j, result[(i, j)], expected);
            }
        }
    }

    #[test]
    fn transform_consistency_at_angle() {
        let model = test_model(0.01);
        let theta = PI / 4.0;
        let w2t = model.transform_wheel2twist(theta);
        let t2w = model.transform_twist2wheel(theta);
        let result = w2t * t2w;
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((result[(i, j)] - expected).abs() < 1e-3,
                    "w2t * t2w at pi/4 [{},{}] = {}, expected {}", i, j, result[(i, j)], expected);
            }
        }
    }

    #[test]
    fn torques_to_currents() {
        let model = test_model(0.01);
        let torques = Vector4f::new(1.0, 1.0, 1.0, 1.0);
        let currents = model.torques_to_currents(torques);
        assert!(currents.x > 0.0);
        assert!((currents.x - currents.y).abs() < 1e-6);
        assert!((currents.x - currents.z).abs() < 1e-6);
        assert!((currents.x - currents.w).abs() < 1e-6);

        let zero_torques = Vector4f::new(0.0, 0.0, 0.0, 0.0);
        let zero_currents = model.torques_to_currents(zero_torques);
        for i in 0..4 {
            assert!(zero_currents[i].abs() < 1e-9);
        }
    }

    #[test]
    fn torques_to_currents_linearity() {
        let model = test_model(0.01);
        let t1 = Vector4f::new(1.0, 0.0, 0.0, 0.0);
        let t2 = Vector4f::new(2.0, 0.0, 0.0, 0.0);
        let c1 = model.torques_to_currents(t1);
        let c2 = model.torques_to_currents(t2);
        assert!((c2.x - 2.0 * c1.x).abs() < 1e-6);
    }

    #[test]
    fn kf_predict_updates_state() {
        let mut model = test_model(0.01);
        model.x = Vector6f::new(1.0, 0.0, 0.0, 0.5, 0.0, 0.0);
        let accel = Vector3f::new(0.0, 0.0, 0.0);
        model.kf_predict(accel);
        // x_new ≈ x + dx*dt = 1.0 + 0.5*0.01 = 1.005
        assert!((model.x[0] - 1.005).abs() < 1e-4);
        assert!((model.x[3] - 0.5).abs() < 1e-4);
    }

    #[test]
    fn kf_predict_with_accel() {
        let mut model = test_model(0.01);
        model.x = Vector6f::zeros();
        let accel = Vector3f::new(1.0, 0.0, 0.0);
        model.kf_predict(accel);
        // vel_x should be accel * dt = 1.0 * 0.01 = 0.01
        assert!((model.x[3] - 0.01).abs() < 1e-4);
    }

    #[test]
    fn kf_update_moves_toward_measurement() {
        let mut model = test_model(0.01);
        model.x = Vector6f::zeros();
        model.kf_predict(Vector3f::zeros());
        let mut meas = Vector8f::zeros();
        meas[0] = 1.0; // vision x
        model.kf_update(meas, false, true, true).unwrap();
        assert!(model.x[0] > 0.0);
    }

    #[test]
    fn scheduled_vision_variance_endpoints() {
        use crate::defaults::{
            DEFAULT_KF_VISION_SCHED_STD_ANGULAR_ZERO, DEFAULT_KF_VISION_SCHED_STD_LINEAR_ZERO,
            DEFAULT_KF_VISION_SCHED_VEL_REF_ANGULAR, DEFAULT_KF_VISION_SCHED_VEL_REF_LINEAR,
        };
        let mut model = test_model(0.01);

        // At rest the vision std collapses to the zero-velocity floor.
        model.x = Vector6f::zeros();
        let (var_lin, var_ang) = model.scheduled_vision_variance();
        assert!((var_lin.sqrt() - DEFAULT_KF_VISION_SCHED_STD_LINEAR_ZERO).abs() < 1e-6);
        assert!((var_ang.sqrt() - DEFAULT_KF_VISION_SCHED_STD_ANGULAR_ZERO).abs() < 1e-6);

        // At the reference speeds the std equals the configured measurement std.
        model.x[3] = DEFAULT_KF_VISION_SCHED_VEL_REF_LINEAR; // vx == 1 m/s
        model.x[5] = DEFAULT_KF_VISION_SCHED_VEL_REF_ANGULAR; // ω == 2π rad/s
        let (var_lin, var_ang) = model.scheduled_vision_variance();
        assert!(
            (var_lin.sqrt() - model.kf_params.measurement_noise_std_vision_pos_linear).abs() < 1e-6
        );
        assert!(
            (var_ang.sqrt() - model.kf_params.measurement_noise_std_vision_pos_angular).abs() < 1e-6
        );

        // Speed uses the linear-velocity magnitude, so a diagonal twist interpolates
        // partway: at ‖v‖ = ref the std again equals the configured value.
        let c = DEFAULT_KF_VISION_SCHED_VEL_REF_LINEAR / core::f32::consts::SQRT_2;
        model.x[3] = c;
        model.x[4] = c; // ‖(vx, vy)‖ == ref
        model.x[5] = 0.0;
        let (var_lin, _) = model.scheduled_vision_variance();
        assert!(
            (var_lin.sqrt() - model.kf_params.measurement_noise_std_vision_pos_linear).abs() < 1e-6
        );
    }

    #[test]
    fn update_kf_params_no_crash() {
        let mut model = test_model(0.01);
        let kf = KalmanFilterParams {
            process_noise_std_pos_linear: 0.02,
            process_noise_std_pos_angular: 0.02,
            process_noise_std_vel_linear: 0.2,
            process_noise_std_vel_angular: 0.2,
            measurement_noise_std_vision_pos_linear: 0.02,
            measurement_noise_std_vision_pos_angular: 0.02,
            measurement_noise_std_encoder_vel_angular: 0.2,
            measurement_noise_std_gyro_vel_angular: 0.2,
            max_pos_linear: 64.0,
            max_pos_angular: 6.2832,
            max_vel_linear: 4.0,
            max_vel_angular: 3.0 * PI,
        };
        model.update_kf_params(kf);
        let _ = model.get_state();
    }

    #[test]
    fn update_physical_params_no_crash() {
        let mut model = test_model(0.01);
        let phys = RobotPhysicalParams {
            alpha: 0.7854,
            beta: 0.7854,
            l: 0.08,
            r: 0.03,
            mass: 3.0,
            iz: 0.015,
            motor_torque_constant: 0.025,
            motor_efficiency_factor: 0.9,
            coulomb_friction_coefficient_linear_x: 0.0,
            coulomb_friction_coefficient_linear_y: 0.0,
            coulomb_friction_coefficient_angular: 0.0,
            viscous_friction_coefficient_linear_x: 0.0,
            viscous_friction_coefficient_linear_y: 0.0,
            viscous_friction_coefficient_angular: 0.0,
        };
        model.update_physical_params(phys).unwrap();
        let _ = model.transform_wheel2twist(0.0);
    }

    #[test]
    fn multiple_predict_update_cycles() {
        let mut model = test_model(0.01);
        let accel = Vector3f::zeros();
        let mut meas = Vector8f::zeros();
        meas[0] = 1.0;
        meas[1] = 0.5;

        for _ in 0..100 {
            model.kf_predict(accel);
            model.kf_update(meas, false, true, true).unwrap();
        }

        let state = model.get_state();
        assert!((state[0] - 1.0).abs() < 0.1);
        assert!((state[1] - 0.5).abs() < 0.1);
    }
}
