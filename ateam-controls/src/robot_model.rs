use libm::{sinf, cosf};
use nalgebra::{SMatrix, SVector};
use crate::{z_rotation_mat, ControlsError};


/// Frames:
///     local  - robot frame, +x forward, +y left, +z up
///     global - field frame, rotated from local by the robot heading theta
/// Pos: SVector3
///     x - x linear position (m)
///     y - y linear position (m)
///     z - angular orientation around z axis (rad)
/// Vel: SVector3
///     x - x linear velocity (m/s)
///     y - y linear velocity (m/s)
///     z - angular velocity around z axis (rad/s)
/// Accel: SVector3
///     x - x linear acceleration (m/s^2)
///     y - y linear acceleration (m/s^2)
///     z - angular acceleration around z axis (rad/s^2)
/// Force: SVector3
///     x - x linear force (N)
///     y - y linear force (N)
///     z - torque around z axis (N*m)
/// Wheel: SVector4, ordered FL, BL, BR, FR
///     angular velocity (rad/s) or torque (N*m)

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RobotPhysicalParams {
    pub alpha_rad: f32,  // Angle between robot +y axis and the segment from the center of the robot to the front left wheel in radians
    pub beta_rad: f32,  // Angle between robot +y axis and the segment from the center of the robot to the back left wheel in radians
    pub l_m: f32,  // Radial distance from center of robot to each wheel in meters
    pub r_m: f32,  // Radius of each wheel in meters
    pub mass_kg: f32,  // Robot body mass in kilograms
    pub iz_kgm2: f32,  // Robot body moment of inertia around z axis in kg*m^2
    pub motor_torque_constant_nmpa: f32,  // Motor torque constant in N*m/A
    pub motor_efficiency_factor: f32,  // Unitless scale between commanded and realized wheel torque
    pub coulomb_friction_linear_x_n: f32,  // Local x (forward/backward) Coulomb friction in N
    pub coulomb_friction_linear_y_n: f32,  // Local y (strafe) Coulomb friction in N
    pub coulomb_friction_angular_nm: f32,  // Angular Coulomb friction in N*m
    pub viscous_friction_linear_x_nspm: f32,  // Local x viscous friction in N/(m/s)
    pub viscous_friction_linear_y_nspm: f32,  // Local y viscous friction in N/(m/s)
    pub viscous_friction_angular_nmsprad: f32,  // Angular viscous friction in N*m/(rad/s)
}

impl Default for RobotPhysicalParams {
    fn default() -> Self {
        use crate::defaults::*;
        Self {
            alpha_rad: DEFAULT_PHYS_ALPHA,
            beta_rad: DEFAULT_PHYS_BETA,
            l_m: DEFAULT_PHYS_L,
            r_m: DEFAULT_PHYS_R,
            mass_kg: DEFAULT_PHYS_MASS,
            iz_kgm2: DEFAULT_PHYS_IZ,
            motor_torque_constant_nmpa: DEFAULT_PHYS_MOTOR_TORQUE_CONSTANT,
            motor_efficiency_factor: DEFAULT_PHYS_MOTOR_EFFICIENCY_FACTOR,
            coulomb_friction_linear_x_n: DEFAULT_PHYS_COULOMB_FRICTION_LINEAR_X,
            coulomb_friction_linear_y_n: DEFAULT_PHYS_COULOMB_FRICTION_LINEAR_Y,
            coulomb_friction_angular_nm: DEFAULT_PHYS_COULOMB_FRICTION_ANGULAR,
            viscous_friction_linear_x_nspm: DEFAULT_PHYS_VISCOUS_FRICTION_LINEAR_X,
            viscous_friction_linear_y_nspm: DEFAULT_PHYS_VISCOUS_FRICTION_LINEAR_Y,
            viscous_friction_angular_nmsprad: DEFAULT_PHYS_VISCOUS_FRICTION_ANGULAR,
        }
    }
}

#[derive(Copy, Clone)]
pub struct RobotModel {
    params: RobotPhysicalParams,
    wheel2vel: SMatrix<f32, 3, 4>,  // Wheel angular vel (rad/s) -> local vel
    vel2wheel: SMatrix<f32, 4, 3>,  // Local vel -> wheel angular vel (rad/s)
    wheel2accel: SMatrix<f32, 3, 4>,  // Wheel torque (N*m) -> local accel
    accel2wheel: SMatrix<f32, 4, 3>,  // Local accel -> wheel torque (N*m)
    inertia_inv: SMatrix<f32, 3, 3>,  // Force -> accel
}

impl RobotModel {
    pub fn new(
        params: RobotPhysicalParams,
    ) -> Result<Self, ControlsError> {
        let mut model = Self {
            params,
            wheel2vel: SMatrix::zeros(),
            vel2wheel: SMatrix::zeros(),
            wheel2accel: SMatrix::zeros(),
            accel2wheel: SMatrix::zeros(),
            inertia_inv: SMatrix::zeros(),
        };
        model.update_params(params)?;
        Ok(model)
    }

    pub fn update_params(
        &mut self,
        params: RobotPhysicalParams,
    ) -> Result<(), ControlsError> {
        // Each column is D_i stacked ontop of the distance from wheel to robot
        // center. D_i is the 2D unit vector in the robot's CF that points in each
        // wheel's positive direction (starting from the wheel center). Order
        // D1-D4 is FL, BL, BR, FR. Reference for this notation:
        // https://seas.ucla.edu/coopcontrol/papers/02cn05.pdf (Near-Optimal
        // Dynamic Trajectory Generation and Control of an Omnidirectional
        // Vehicle.)
        let wheel_mat = SMatrix::<f32, 3, 4>::new(
            -cosf(params.alpha_rad), -cosf(params.beta_rad),  cosf(params.beta_rad), cosf(params.alpha_rad),
             sinf(params.alpha_rad), -sinf(params.beta_rad), -sinf(params.beta_rad), sinf(params.alpha_rad),
             params.l_m            ,  params.l_m           ,  params.l_m           , params.l_m
        );
        let linear_wheel2vel = wheel_mat
            .transpose()
            .pseudo_inverse(1e-6)
            .map_err(|_| ControlsError::SingularMatrix)?;
        let inertia = SMatrix::<f32, 3, 3>::from_diagonal(
            &SVector::<f32, 3>::new(params.mass_kg, params.mass_kg, params.iz_kgm2),
        );
        let inertia_inv = inertia.try_inverse().ok_or(ControlsError::SingularMatrix)?;

        self.params = params;
        self.wheel2vel = linear_wheel2vel * params.r_m;
        self.vel2wheel = wheel_mat.transpose() / params.r_m;
        self.wheel2accel = inertia_inv * wheel_mat / params.r_m;
        self.accel2wheel = linear_wheel2vel.transpose() * inertia * params.r_m;
        self.inertia_inv = inertia_inv;
        Ok(())
    }

    pub fn get_params(&self) -> RobotPhysicalParams {
        self.params
    }

    /// Convert wheel angular velocities to local (robot frame) velocity
    /// [vx_mps, vy_mps, vw_radps]
    pub fn wheel_to_local_vel(
        &self,
        encoder: &SVector<f32, 4>,  // fl_radps, bl_radps, br_radps, fr_radps
    ) -> SVector<f32, 3> {
        self.wheel2vel * encoder
    }

    /// Convert wheel angular velocities to global (field frame) velocity
    /// [vx_mps, vy_mps, vw_radps]
    pub fn wheel_to_global_vel(
        &self,
        encoder: &SVector<f32, 4>,  // fl_radps, bl_radps, br_radps, fr_radps
        theta: f32,
    ) -> SVector<f32, 3> {
        self.transform_wheel2vel_global(theta) * encoder
    }

    /// Convert wheel torques to local (robot frame) acceleration
    /// [ax_mps2, ay_mps2, aw_radps2]
    pub fn wheel_to_local_accel(
        &self,
        torque: &SVector<f32, 4>,  // fl_nm, bl_nm, br_nm, fr_nm
    ) -> SVector<f32, 3> {
        self.wheel2accel * torque
    }

    /// Convert wheel torques to global (field frame) acceleration
    /// [ax_mps2, ay_mps2, aw_radps2]
    pub fn wheel_to_global_accel(
        &self,
        torque: &SVector<f32, 4>,  // fl_nm, bl_nm, br_nm, fr_nm
        theta: f32,
    ) -> SVector<f32, 3> {
        self.transform_wheel2accel_global(theta) * torque
    }

    /// Transform matrix from wheel angular velocities to local vel
    pub fn transform_wheel2vel_local(&self) -> SMatrix<f32, 3, 4> {
        self.wheel2vel
    }

    /// Transform matrix from wheel angular velocities to global vel
    pub fn transform_wheel2vel_global(&self, theta: f32) -> SMatrix<f32, 3, 4> {
        z_rotation_mat(theta) * self.wheel2vel
    }

    /// Transform matrix from local vel to wheel angular velocities
    pub fn transform_vel2wheel_local(&self) -> SMatrix<f32, 4, 3> {
        self.vel2wheel
    }

    /// Transform matrix from global vel to wheel angular velocities
    pub fn transform_vel2wheel_global(&self, theta: f32) -> SMatrix<f32, 4, 3> {
        self.vel2wheel * z_rotation_mat(-theta)
    }

    /// Transform matrix from wheel torques to local accel
    pub fn transform_wheel2accel_local(&self) -> SMatrix<f32, 3, 4> {
        self.wheel2accel
    }

    /// Transform matrix from wheel torques to global accel
    pub fn transform_wheel2accel_global(&self, theta: f32) -> SMatrix<f32, 3, 4> {
        z_rotation_mat(theta) * self.wheel2accel
    }

    /// Transform matrix from local accel to wheel torques
    pub fn transform_accel2wheel_local(&self) -> SMatrix<f32, 4, 3> {
        self.accel2wheel
    }

    /// Transform matrix from global accel to wheel torques
    pub fn transform_accel2wheel_global(&self, theta: f32) -> SMatrix<f32, 4, 3> {
        self.accel2wheel * z_rotation_mat(-theta)
    }

    /// Convert a force to an accel. Mass is equal in x and y, so this holds in
    /// both local and global frames.
    pub fn force_to_accel(&self, force: &SVector<f32, 3>) -> SVector<f32, 3> {
        self.inertia_inv * force
    }

    /// Convert wheel torques (N*m) to wheel currents (A)
    pub fn torques_to_currents(&self, torques: SVector<f32, 4>) -> SVector<f32, 4> {
        torques / self.params.motor_torque_constant_nmpa / self.params.motor_efficiency_factor
    }

    /// Compute the local friction-only force that opposes a local vel.
    ///
    /// **Frame assumption:** the input ``local_vel`` must be in the robot's
    /// **local frame** ``[vx_mps, vy_mps, vw_radps]``. The output is also in
    /// the local frame ``[Fx_n, Fy_n, Tz_nm]``. Callers operating in the
    /// global frame must rotate the vel by ``R_z(-theta)`` before calling and
    /// rotate the returned force by ``R_z(theta)`` after.
    ///
    /// Per-axis model (independent x and y constants because strafing and
    /// forward/backward friction can differ noticeably):
    ///
    /// ```text
    /// Fx = -c_visc_x * vx - c_coul_x * sign(vx)
    /// Fy = -c_visc_y * vy - c_coul_y * sign(vy)
    /// Tz = -c_visc_ang * vw - c_coul_ang * sign(vw)
    /// ```
    pub fn compute_local_friction_force(&self, local_vel: SVector<f32, 3>) -> SVector<f32, 3> {
        let sign = |v: f32| if v > 0.0 { 1.0 } else if v < 0.0 { -1.0 } else { 0.0 };
        let p = &self.params;
        SVector::<f32, 3>::new(
            -p.viscous_friction_linear_x_nspm * local_vel.x - p.coulomb_friction_linear_x_n * sign(local_vel.x),
            -p.viscous_friction_linear_y_nspm * local_vel.y - p.coulomb_friction_linear_y_n * sign(local_vel.y),
            -p.viscous_friction_angular_nmsprad * local_vel.z - p.coulomb_friction_angular_nm * sign(local_vel.z),
        )
    }
}

impl Default for RobotModel {
    fn default() -> Self {
        Self::new(RobotPhysicalParams::default())
            .expect("Default RobotModel parameters must be valid")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    fn test_params() -> RobotPhysicalParams {
        RobotPhysicalParams {
            alpha_rad: PI / 6.0,
            beta_rad: PI / 4.0,
            l_m: 0.0814,
            r_m: 0.030,
            mass_kg: 2.7,
            iz_kgm2: 0.008,
            motor_torque_constant_nmpa: 0.0335,
            motor_efficiency_factor: 13.0,
            coulomb_friction_linear_x_n: 2.058,
            coulomb_friction_linear_y_n: 2.058,
            coulomb_friction_angular_nm: 0.05,
            viscous_friction_linear_x_nspm: 5.0,
            viscous_friction_linear_y_nspm: 5.0,
            viscous_friction_angular_nmsprad: 0.0063,
        }
    }

    fn test_model() -> RobotModel {
        RobotModel::new(test_params()).unwrap()
    }

    fn assert_identity<const N: usize>(m: &SMatrix<f32, N, N>, tol: f32, label: &str) {
        for i in 0..N {
            for j in 0..N {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((m[(i, j)] - expected).abs() < tol,
                    "{} [{},{}] = {}, expected {}", label, i, j, m[(i, j)], expected);
            }
        }
    }

    #[test]
    fn local_vel_transforms_invert() {
        let model = test_model();
        let result = model.transform_wheel2vel_local() * model.transform_vel2wheel_local();
        assert_identity(&result, 1e-4, "wheel2vel_local * vel2wheel_local");
    }

    #[test]
    fn global_vel_transforms_invert() {
        let model = test_model();
        let theta = PI / 4.0;
        let result = model.transform_wheel2vel_global(theta) * model.transform_vel2wheel_global(theta);
        assert_identity(&result, 1e-3, "wheel2vel_global * vel2wheel_global at pi/4");
    }

    #[test]
    fn local_accel_transforms_invert() {
        let model = test_model();
        let result = model.transform_wheel2accel_local() * model.transform_accel2wheel_local();
        assert_identity(&result, 1e-4, "wheel2accel_local * accel2wheel_local");
    }

    #[test]
    fn global_accel_transforms_invert() {
        let model = test_model();
        let theta = -PI / 3.0;
        let result = model.transform_wheel2accel_global(theta) * model.transform_accel2wheel_global(theta);
        assert_identity(&result, 1e-3, "wheel2accel_global * accel2wheel_global at -pi/3");
    }

    #[test]
    fn wheel_to_local_vel_round_trip() {
        let model = test_model();
        let local_vel = SVector::<f32, 3>::new(0.5, -0.3, 1.2);
        let wheel = model.transform_vel2wheel_local() * local_vel;
        let out = model.wheel_to_local_vel(&wheel);
        for i in 0..3 {
            assert!((out[i] - local_vel[i]).abs() < 1e-4,
                "local vel [{}] = {}, expected {}", i, out[i], local_vel[i]);
        }
    }

    #[test]
    fn wheel_to_global_vel_rotates_local() {
        let model = test_model();
        let theta = PI / 2.0;
        // Pure local forward motion should appear as global +y at theta = pi/2
        let wheel = model.transform_vel2wheel_local() * SVector::<f32, 3>::new(1.0, 0.0, 0.0);
        let global = model.wheel_to_global_vel(&wheel, theta);
        assert!(global.x.abs() < 1e-4);
        assert!((global.y - 1.0).abs() < 1e-4);
        assert!(global.z.abs() < 1e-4);
        // Local form is unaffected by heading
        let local = model.wheel_to_local_vel(&wheel);
        assert!((local.x - 1.0).abs() < 1e-4);
    }

    #[test]
    fn wheel_to_global_accel_rotates_local() {
        let model = test_model();
        let theta = PI / 2.0;
        let torque = model.transform_accel2wheel_local() * SVector::<f32, 3>::new(2.0, 0.0, 0.5);
        let local = model.wheel_to_local_accel(&torque);
        assert!((local.x - 2.0).abs() < 1e-4);
        assert!(local.y.abs() < 1e-4);
        assert!((local.z - 0.5).abs() < 1e-3);
        let global = model.wheel_to_global_accel(&torque, theta);
        assert!(global.x.abs() < 1e-4);
        assert!((global.y - 2.0).abs() < 1e-4);
        assert!((global.z - 0.5).abs() < 1e-3);
    }

    #[test]
    fn force_to_accel_divides_by_inertia() {
        let model = test_model();
        let accel = model.force_to_accel(&SVector::<f32, 3>::new(2.7, 5.4, 0.008));
        assert!((accel.x - 1.0).abs() < 1e-5);
        assert!((accel.y - 2.0).abs() < 1e-5);
        assert!((accel.z - 1.0).abs() < 1e-4);
    }

    #[test]
    fn torques_to_currents() {
        let model = test_model();
        let currents = model.torques_to_currents(SVector::<f32, 4>::new(1.0, 1.0, 1.0, 1.0));
        assert!(currents.x > 0.0);
        assert!((currents.x - currents.y).abs() < 1e-6);
        assert!((currents.x - currents.z).abs() < 1e-6);
        assert!((currents.x - currents.w).abs() < 1e-6);

        let zero_currents = model.torques_to_currents(SVector::<f32, 4>::zeros());
        for i in 0..4 {
            assert!(zero_currents[i].abs() < 1e-9);
        }
    }

    #[test]
    fn torques_to_currents_linearity() {
        let model = test_model();
        let c1 = model.torques_to_currents(SVector::<f32, 4>::new(1.0, 0.0, 0.0, 0.0));
        let c2 = model.torques_to_currents(SVector::<f32, 4>::new(2.0, 0.0, 0.0, 0.0));
        assert!((c2.x - 2.0 * c1.x).abs() < 1e-6);
    }

    #[test]
    fn update_params_no_crash() {
        let mut model = test_model();
        let params = RobotPhysicalParams {
            alpha_rad: 0.7854,
            beta_rad: 0.7854,
            l_m: 0.08,
            r_m: 0.03,
            mass_kg: 3.0,
            iz_kgm2: 0.015,
            motor_torque_constant_nmpa: 0.025,
            motor_efficiency_factor: 0.9,
            coulomb_friction_linear_x_n: 0.0,
            coulomb_friction_linear_y_n: 0.0,
            coulomb_friction_angular_nm: 0.0,
            viscous_friction_linear_x_nspm: 0.0,
            viscous_friction_linear_y_nspm: 0.0,
            viscous_friction_angular_nmsprad: 0.0,
        };
        model.update_params(params).unwrap();
        assert_eq!(model.get_params().mass_kg, 3.0);
    }

    #[test]
    fn update_params_rejects_zero_mass() {
        let mut model = test_model();
        let mut params = test_params();
        params.mass_kg = 0.0;
        assert_eq!(model.update_params(params).err(), Some(ControlsError::SingularMatrix));
        // Model is left unchanged on error
        assert_eq!(model.get_params().mass_kg, test_params().mass_kg);
    }
}
