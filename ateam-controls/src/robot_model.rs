use libm::{sinf, cosf};
use crate::{ControlsError, Matrix3f, Matrix3x4f, Matrix4x3f, Vector3f, Vector4f};
use crate::z_rotation_mat;


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
    /// Robot physical parameters struct for easy updating and access to individual params
    pub physical_params: RobotPhysicalParams,
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
}

impl Default for RobotModel {
    fn default() -> Self {
        RobotModel::new(RobotPhysicalParams::default())
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
    pub fn new(physical_params: RobotPhysicalParams) -> Result<RobotModel, ControlsError> {
        let mut model = RobotModel {
            physical_params,
            m: Matrix3x4f::zeros(),
            m_inv: Matrix4x3f::zeros(),
            i: Matrix3f::zeros(),
            i_inv: Matrix3f::zeros(),
            r: 0.0,
        };
        model.update_physical_params(physical_params)?;
        Ok(model)
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

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

    fn test_model() -> RobotModel {
        RobotModel::new(test_phys_params()).unwrap()
    }

    #[test]
    fn wheel2twist_and_back() {
        let model = test_model();
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
        let model = test_model();
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
        let model = test_model();
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
        let model = test_model();
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
        let model = test_model();
        let t1 = Vector4f::new(1.0, 0.0, 0.0, 0.0);
        let t2 = Vector4f::new(2.0, 0.0, 0.0, 0.0);
        let c1 = model.torques_to_currents(t1);
        let c2 = model.torques_to_currents(t2);
        assert!((c2.x - 2.0 * c1.x).abs() < 1e-6);
    }

    #[test]
    fn update_physical_params_no_crash() {
        let mut model = test_model();
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
}
