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
    #[cfg_attr(feature = "serde", serde(rename = "PHYS_BODY_COM_HEIGHT"))]
    pub h_cm: f32,
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
            h_cm: physical_params::BODY_COM_HEIGHT,
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
    /// Null space vector of m (unit vector satisfying m * m_null = 0), used for torque vectoring
    pub m_null: Vector4f,
    /// Normal-force constraint matrix: rows are [1,1,1,1], [x_i], [y_i] (wheel positions in body frame)
    pub w_normal: Matrix3x4f,
    /// Pseudo-inverse of w_normal, used to solve for per-wheel normal forces
    pub w_normal_inv: Matrix4x3f,
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

        // Compute the pseudo-inverse of M for use in velocity and torque transformations.  
        // M is not square (3x4) but has full row rank (rank 3), so the pseudo-inverse gives a 
        // left-inverse that maps from robot frame twist to wheel velocities.
        self.m_inv = self.m.pseudo_inverse(1e-6).map_err(|_| ControlsError::SingularMatrix)?;

        // Compute the null space of M (1D for a 3x4 matrix with rank 3).
        // The orthogonal projector onto null(M) is (I - M⁺M).  Applying it to each
        // standard basis vector until we get a non-degenerate result gives the null vector.
        // This avoids relying on the thin SVD (which only returns 3 right singular vectors
        // and would be missing the fourth that spans the null space).
        self.m_null = {
            let mut null_vec = Vector4f::zeros();
            for i in 0..4 {
                let mut e = Vector4f::zeros();
                e[i] = 1.0;
                let projected = e - self.m_inv * (self.m * e);
                let norm = projected.norm();
                if norm > 1e-6 {
                    null_vec = projected / norm;
                    break;
                }
            }
            null_vec
        };

        // Build the normal-force constraint matrix from wheel contact positions in the body frame.
        // Each wheel's contact point is obtained by rotating its drive direction (M column) by -90°:
        //   drive dir (vx,vy) -> position (vy, -vx) * l
        // This gives:
        //   FL: ( sin(α)*l,  cos(α)*l)   BL: (-sin(β)*l,  cos(β)*l)
        //   BR: (-sin(β)*l, -cos(β)*l)   FR: ( sin(α)*l, -cos(α)*l)
        let (sa, ca) = (sinf(physical_params.alpha), cosf(physical_params.alpha));
        let (sb, cb) = (sinf(physical_params.beta),  cosf(physical_params.beta));
        let l = physical_params.l;
        self.w_normal = Matrix3x4f::new(
            1.0,     1.0,     1.0,    1.0,
            sa * l, -sb * l, -sb * l, sa * l,
            ca * l,  cb * l, -cb * l, -ca * l,
        );
        self.w_normal_inv = self.w_normal.pseudo_inverse(1e-6).map_err(|_| ControlsError::SingularMatrix)?;

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

    /// Compute per-wheel normal forces (N) accounting for load transfer due to acceleration.
    ///
    /// When the robot accelerates, the CoM at height `h_cm` creates a pitching/rolling moment that
    /// redistributes normal force across the four wheels.  Moment equilibrium gives three constraints:
    ///
    ///   Σ N_i         = m·g
    ///   Σ (x_i · N_i) = −h_cm·m·ax   (load shifts rearward under forward accel)
    ///   Σ (y_i · N_i) = −h_cm·m·ay   (load shifts right under leftward accel)
    ///
    /// The system is underdetermined (4 unknowns, 3 equations); the minimum-norm (pseudo-inverse)
    /// solution models equal wheel-contact stiffness.  Normal forces are clamped to ≥ 0 (wheels
    /// cannot pull the floor).
    ///
    /// `body_accel` is the acceleration in the robot body frame [ax, ay, alpha_z].
    pub fn compute_wheel_normal_forces(&self, body_accel: Vector3f) -> Vector4f {
        const G: f32 = 9.81;
        let m   = self.physical_params.mass;
        let h   = self.physical_params.h_cm;
        let ax  = body_accel[0];
        let ay  = body_accel[1];
        let b   = Vector3f::new(m * G, -h * m * ax, -h * m * ay);
        let n   = self.w_normal_inv * b;
        // Clamp: a wheel that lifts off contributes zero traction
        Vector4f::new(n[0].max(0.0), n[1].max(0.0), n[2].max(0.0), n[3].max(0.0))
    }

    /// Compute wheel torques using load-transfer-aware torque vectoring.
    ///
    /// Uses the 1D null space of `M` to redistribute torques across wheels while preserving the
    /// net body wrench.  Unlike the plain pseudo-inverse, the optimisation weights each wheel by
    /// its current normal force so that traction utilisation (`|τ_i| / N_i`) is equalised — wheels
    /// carrying more load are allowed proportionally more torque, and wheels that have partially
    /// unloaded due to acceleration are protected from overload.
    pub fn compute_wheel_torques_vectored(&self, theta: f32, accel_cmd: Vector3f) -> Vector4f {
        let tau_nom = self.transform_accel2wheel(theta) * accel_cmd;
        if self.m_null.norm() < 1e-6 {
            return tau_nom;
        }
        let body_accel = z_rotation_mat(-theta) * accel_cmd;
        let normal_forces = self.compute_wheel_normal_forces(body_accel);
        let k = Self::find_optimal_null_space_gain_weighted(tau_nom, self.m_null, normal_forces);
        tau_nom + self.m_null.scale(k)
    }

    /// Find the scalar `k` that minimises `max_i |tau[i] + k·n[i]| / w[i]` (weighted L∞).
    ///
    /// `w[i]` is the normal force on wheel i; larger normal force = more traction capacity = higher
    /// allowable torque.  The minimum of the convex piecewise-linear objective occurs where two
    /// wheels tie for the peak utilisation ratio, so we check all O(n²) pairwise candidate k
    /// values (same-sign and opposite-sign equalisation) and return the best.
    fn find_optimal_null_space_gain_weighted(tau: Vector4f, n: Vector4f, w: Vector4f) -> f32 {
        let load_ratio = |k: f32| -> f32 {
            let mut max_val = 0.0_f32;
            for i in 0..4 {
                if w[i] > 1e-6 {
                    let v = (tau[i] + k * n[i]).abs() / w[i];
                    if v > max_val { max_val = v; }
                }
            }
            max_val
        };

        let mut best_k   = 0.0_f32;
        let mut best_val = load_ratio(0.0);

        for i in 0..4 {
            for j in (i + 1)..4 {
                if w[i] < 1e-6 || w[j] < 1e-6 { continue; }
                let (wi, wj) = (w[i], w[j]);

                // Case 1: (tau_i + k·n_i)/wi = (tau_j + k·n_j)/wj  (same sign)
                let dn = n[i] * wj - n[j] * wi;
                if dn.abs() > 1e-6 {
                    let k = (tau[j] * wi - tau[i] * wj) / dn;
                    let v = load_ratio(k);
                    if v < best_val { best_val = v; best_k = k; }
                }

                // Case 2: (tau_i + k·n_i)/wi = −(tau_j + k·n_j)/wj  (opposite sign)
                let sn = n[i] * wj + n[j] * wi;
                if sn.abs() > 1e-6 {
                    let k = -(tau[i] * wj + tau[j] * wi) / sn;
                    let v = load_ratio(k);
                    if v < best_val { best_val = v; best_k = k; }
                }
            }
        }

        best_k
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Vector3f, Vector4f};

    fn default_model() -> RobotModel {
        RobotModel::new_from_default_params(0.01).unwrap()
    }

    fn approx_eq(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    // --- compute_wheel_normal_forces ---

    /// At rest: all normal forces are positive, their sum is m*g, and the robot's
    /// left-right symmetry (alpha/beta are equal for left/right pairs) means FL==FR and BL==BR.
    /// Note: FL ≠ BL in general because alpha ≠ beta shifts the CoM projection.
    #[test]
    fn normal_forces_static_physical_validity() {
        let model = default_model();
        let total = model.physical_params.mass * 9.81;
        let n = model.compute_wheel_normal_forces(Vector3f::zeros());
        // All normal forces must be non-negative
        for i in 0..4 {
            assert!(n[i] >= 0.0, "N[{i}] = {} should be non-negative", n[i]);
        }
        // Must sum to m*g
        assert!(approx_eq(n.sum(), total, 1e-3), "N sum = {}, expected {total}", n.sum());
        // Left-right symmetry: FL(0)==FR(3), BL(1)==BR(2)
        assert!(approx_eq(n[0], n[3], 1e-3), "FL ({}) != FR ({}) at rest", n[0], n[3]);
        assert!(approx_eq(n[1], n[2], 1e-3), "BL ({}) != BR ({}) at rest", n[1], n[2]);
    }

    /// Normal forces must always sum to m*g regardless of acceleration.
    #[test]
    fn normal_forces_sum_invariant() {
        let model = default_model();
        let total = model.physical_params.mass * 9.81;
        for &(ax, ay) in &[(0.0_f32, 0.0_f32), (3.0, 0.0), (-3.0, 0.0), (0.0, 2.0), (2.0, -2.0)] {
            let n = model.compute_wheel_normal_forces(Vector3f::new(ax, ay, 0.0));
            assert!(
                approx_eq(n.sum(), total, 1e-3),
                "Sum of N = {} ≠ m*g = {total} at ax={ax} ay={ay}", n.sum()
            );
        }
    }

    /// Under forward acceleration the rear wheels (BL=1, BR=2) carry more load than the
    /// front wheels (FL=0, FR=3).
    #[test]
    fn normal_forces_load_transfer_forward_accel() {
        let model = default_model();
        let n = model.compute_wheel_normal_forces(Vector3f::new(5.0, 0.0, 0.0));
        let front_avg = (n[0] + n[3]) / 2.0;
        let rear_avg  = (n[1] + n[2]) / 2.0;
        assert!(
            rear_avg > front_avg,
            "Rear avg N ({rear_avg:.4}) should exceed front avg N ({front_avg:.4}) under +x accel"
        );
    }

    /// Under leftward acceleration the right wheels (BR=2, FR=3) carry more load than the
    /// left wheels (FL=0, BL=1).
    #[test]
    fn normal_forces_load_transfer_lateral_accel() {
        let model = default_model();
        // +y is left in body frame, so accelerating left loads the right side
        let n = model.compute_wheel_normal_forces(Vector3f::new(0.0, 5.0, 0.0));
        let left_avg  = (n[0] + n[1]) / 2.0;
        let right_avg = (n[2] + n[3]) / 2.0;
        assert!(
            right_avg > left_avg,
            "Right avg N ({right_avg:.4}) should exceed left avg N ({left_avg:.4}) under +y accel"
        );
    }

    /// The computed normal forces satisfy the three moment-equilibrium equations exactly
    /// (up to pseudo-inverse rounding).
    #[test]
    fn normal_forces_satisfy_equilibrium() {
        let model = default_model();
        let m = model.physical_params.mass;
        let h = model.physical_params.h_cm;
        let accel = Vector3f::new(3.0, -1.5, 0.0);
        let n = model.compute_wheel_normal_forces(accel);
        let rhs = Vector3f::new(m * 9.81, -h * m * accel[0], -h * m * accel[1]);
        let residual = model.w_normal * n - rhs;
        for i in 0..3 {
            assert!(
                residual[i].abs() < 1e-4,
                "Equilibrium residual[{i}] = {} (should be ~0)", residual[i]
            );
        }
    }

    // --- compute_wheel_torques_vectored ---

    /// The vectored torques must produce the same net body wrench as the nominal
    /// pseudo-inverse solution: M * tau_vectored == M * tau_nominal.
    #[test]
    fn vectored_torques_preserve_body_wrench() {
        let model = default_model();
        for &(theta, ax, ay, az) in &[
            (0.0_f32, 1.5_f32, -0.8_f32,  0.5_f32),
            (0.785,   2.0,      1.0,       0.0),
            (1.57,   -1.0,      0.0,       0.3),
            (0.0,    5.0,       0.0,       0.0),  // hard forward accel (large load transfer)
        ] {
            let accel_cmd = Vector3f::new(ax, ay, az);
            let tau_nom      = model.transform_accel2wheel(theta) * accel_cmd;
            let tau_vectored = model.compute_wheel_torques_vectored(theta, accel_cmd);
            let wrench_diff  = (model.m * tau_nom - model.m * tau_vectored).norm();
            assert!(
                wrench_diff < 1e-4,
                "Wrench changed by {wrench_diff} at theta={theta} accel=({ax},{ay},{az})"
            );
        }
    }

    /// The vectored torques must never produce a higher peak traction utilisation than the
    /// nominal pseudo-inverse, across a variety of headings and acceleration commands.
    #[test]
    fn vectored_torques_traction_utilisation_not_worse() {
        let model = default_model();
        for &(theta, ax, ay, az) in &[
            (0.0_f32,  2.0_f32,  0.5_f32, 0.2_f32),
            (0.785,   -1.0,      2.0,     0.0),
            (1.57,     0.0,     -3.0,     0.1),
            (0.3,      3.0,     -1.5,    -0.3),
        ] {
            let accel_cmd  = Vector3f::new(ax, ay, az);
            let body_accel = z_rotation_mat(-theta) * accel_cmd;
            let n          = model.compute_wheel_normal_forces(body_accel);

            let peak_utilisation = |tau: Vector4f| -> f32 {
                (0..4)
                    .filter(|&i| n[i] > 1e-6)
                    .map(|i| tau[i].abs() / n[i])
                    .fold(0.0_f32, f32::max)
            };

            let tau_nom      = model.transform_accel2wheel(theta) * accel_cmd;
            let tau_vectored = model.compute_wheel_torques_vectored(theta, accel_cmd);
            let u_nom        = peak_utilisation(tau_nom);
            let u_vectored   = peak_utilisation(tau_vectored);
            assert!(
                u_vectored <= u_nom + 1e-4,
                "Vectored utilisation ({u_vectored:.5}) > nominal ({u_nom:.5}) at theta={theta} accel=({ax},{ay},{az})"
            );
        }
    }

    /// Under significant load transfer the vectored solution must have lower or equal peak
    /// traction utilisation (|τ_i| / N_i) compared to the nominal pseudo-inverse.
    #[test]
    fn vectored_torques_reduce_traction_utilisation() {
        let model = default_model();
        // Hard forward acceleration: FL/FR are nearly unloaded, BL/BR are heavily loaded.
        let theta     = 0.0_f32;
        let accel_cmd = Vector3f::new(5.0, 0.0, 0.0);
        let body_accel = z_rotation_mat(-theta) * accel_cmd;
        let n = model.compute_wheel_normal_forces(body_accel);

        let peak_utilisation = |tau: Vector4f| -> f32 {
            (0..4)
                .filter(|&i| n[i] > 1e-6)
                .map(|i| tau[i].abs() / n[i])
                .fold(0.0_f32, f32::max)
        };

        let tau_nom      = model.transform_accel2wheel(theta) * accel_cmd;
        let tau_vectored = model.compute_wheel_torques_vectored(theta, accel_cmd);

        let u_nom      = peak_utilisation(tau_nom);
        let u_vectored = peak_utilisation(tau_vectored);
        assert!(
            u_vectored <= u_nom + 1e-4,
            "Vectored peak utilisation ({u_vectored:.5}) > nominal ({u_nom:.5})"
        );
    }
}
