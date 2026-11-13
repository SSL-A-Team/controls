use libm::{sinf, cosf};
use nalgebra::{SMatrix, SVector};
use crate::{z_rotation_mat, ControlsError};

/// Commanded linear speed (m/s) at which the linear Coulomb compensation reaches full
/// strength; it ramps linearly from zero below this. TIGERs switch it fully on above
/// 0.01 m/s instead, which on our robot (static friction near the available force)
/// makes the robot hunt around a held position with ~2 A current steps. 0.02 m/s still
/// swung the full force around with the feedback velocity near targets (yaw kicks).
pub const FRICTION_XY_RAMP_MPS: f32 = 0.05;
/// Commanded angular speed (rad/s) at which the angular Coulomb compensation reaches
/// full strength (TIGERs: step at 1e-3 rad/s). The commanded rate includes the
/// angular-rate feedback, which dithers ~+/-0.3 rad/s around zero when the robot
/// holds or creeps; at 0.2 the compensation flipped sign with it (+/-15 rad/s^2
/// torque steps that fed the wobble). At 0.6 it varies continuously instead
/// (angular command chatter -55% on streamed position targets, robot 2).
pub const FRICTION_W_RAMP_RADPS: f32 = 0.6;
/// Largest accepted friction parameter (N, N*s/m, N*m, N*m*s/rad); rejects typos.
pub const MAX_FRICTION_PARAM: f32 = 10.0;
/// Smallest accepted output efficiency (bounds the feed-forward gain at 1/MIN_EFFICIENCY).
pub const MIN_EFFICIENCY: f32 = 0.2;


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
    // Friction / output model after TIGERs Mannheim (Firmware src/robot/model_fric.h,
    // ModelFricConfig). Linear terms are for motion along local +-y (sideways) and are
    // scaled by `friction_direction_factor` otherwise.
    pub coulomb_xy_n: f32,  // Linear Coulomb friction in N
    pub viscous_xy_nspm: f32,  // Linear viscous friction in N/(m/s), applied per local axis
    pub coulomb_w_nm: f32,  // Angular Coulomb friction in N*m
    pub viscous_w_nmsprad: f32,  // Angular viscous friction in N*m/(rad/s)
    pub efficiency_xy: f32,  // Realized / commanded linear force (0, 1]
    pub efficiency_w: f32,  // Realized / commanded torque about z (0, 1]
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
            coulomb_xy_n: DEFAULT_PHYS_COULOMB_XY,
            viscous_xy_nspm: DEFAULT_PHYS_VISCOUS_XY,
            coulomb_w_nm: DEFAULT_PHYS_COULOMB_W,
            viscous_w_nmsprad: DEFAULT_PHYS_VISCOUS_W,
            efficiency_xy: DEFAULT_PHYS_EFFICIENCY_XY,
            efficiency_w: DEFAULT_PHYS_EFFICIENCY_W,
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
    wheel_drive_dirs: SMatrix<f32, 2, 4>,  // Unit drive direction D_i of each wheel (local frame)
    roller_speed_ref: f32,  // Sum of roller speeds for unit motion along local +y (friction reference)
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
            wheel_drive_dirs: SMatrix::zeros(),
            roller_speed_ref: 1.0,
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

        let wheel_drive_dirs: SMatrix<f32, 2, 4> = wheel_mat.fixed_rows::<2>(0).into();
        let roller_speed_ref = Self::roller_speed(&wheel_drive_dirs, SVector::<f32, 2>::new(0.0, 1.0));
        if roller_speed_ref < 1e-6 {
            return Err(ControlsError::InvalidInput);
        }
        let friction = [params.coulomb_xy_n, params.viscous_xy_nspm, params.coulomb_w_nm, params.viscous_w_nmsprad];
        if friction.iter().any(|f| !(f.is_finite() && *f >= 0.0 && *f <= MAX_FRICTION_PARAM))
            || !(params.efficiency_xy >= MIN_EFFICIENCY && params.efficiency_xy <= 1.0)
            || !(params.efficiency_w >= MIN_EFFICIENCY && params.efficiency_w <= 1.0)
            || !(params.motor_torque_constant_nmpa > 0.0 && params.motor_torque_constant_nmpa.is_finite())
        {
            return Err(ControlsError::InvalidInput);
        }

        self.params = params;
        self.wheel2vel = linear_wheel2vel * params.r_m;
        self.vel2wheel = wheel_mat.transpose() / params.r_m;
        self.wheel2accel = inertia_inv * wheel_mat / params.r_m;
        self.accel2wheel = linear_wheel2vel.transpose() * inertia * params.r_m;
        self.inertia_inv = inertia_inv;
        self.wheel_drive_dirs = wheel_drive_dirs;
        self.roller_speed_ref = roller_speed_ref;
        Ok(())
    }

    /// Sum of omni-wheel roller speeds for unit robot motion along `dir`. Each
    /// wheel's rollers spin along its axle, perpendicular to the drive
    /// direction D_i, so a roller's speed is |dir x D_i|.
    fn roller_speed(wheel_drive_dirs: &SMatrix<f32, 2, 4>, dir: SVector<f32, 2>) -> f32 {
        wheel_drive_dirs
            .column_iter()
            .map(|d| (dir.x * d.y - dir.y * d.x).abs())
            .sum()
    }

    /// Linear friction direction factor k(dir): total roller speed for unit
    /// motion along the local unit vector `dir`, normalized so sideways motion
    /// (local +-y) gives 1. Linear friction is assumed to come from the
    /// omni-wheel rollers, so it scales with this factor. Equivalent to TIGERs'
    /// `getFrictionFactor` (whose local X axis is our sideways y axis).
    pub fn friction_direction_factor(&self, dir: SVector<f32, 2>) -> f32 {
        Self::roller_speed(&self.wheel_drive_dirs, dir) / self.roller_speed_ref
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
        torques / self.params.motor_torque_constant_nmpa
    }

    /// Friction compensation force for a commanded local velocity, after
    /// TIGERs' `ModelFricGetXY` / `ModelFricGetW` (Firmware src/robot/model_fric.c).
    ///
    /// Input and output are in the robot's local frame. The returned force
    /// points *along* the motion (it is the force needed to overcome friction):
    ///
    /// ```text
    /// dir   = v_xy / |v_xy|
    /// k     = friction_direction_factor(dir)
    /// s_xy  = min(|v_xy| / FRICTION_XY_RAMP_MPS, 1)
    /// F_x   = dir_x * k * (s_xy * coulomb_xy + viscous_xy * |v_x|)
    /// F_y   = dir_y * k * (s_xy * coulomb_xy + viscous_xy * |v_y|)
    /// T_z   = sign(w) * (min(|w| / FRICTION_W_RAMP_RADPS, 1) * coulomb_w + viscous_w * |w|)
    /// ```
    ///
    /// The only deviation from TIGERs is the Coulomb ramp (they step it on above
    /// 0.01 m/s / 1e-3 rad/s).
    pub fn friction_comp_force(&self, local_vel: SVector<f32, 3>) -> SVector<f32, 3> {
        let p = &self.params;
        let speed = SVector::<f32, 2>::new(local_vel.x, local_vel.y).norm();
        let (fx, fy) = if speed > 0.0 {
            let dir = SVector::<f32, 2>::new(local_vel.x / speed, local_vel.y / speed);
            let k = self.friction_direction_factor(dir);
            let coulomb = (speed / FRICTION_XY_RAMP_MPS).min(1.0) * p.coulomb_xy_n;
            (
                dir.x * k * (coulomb + p.viscous_xy_nspm * local_vel.x.abs()),
                dir.y * k * (coulomb + p.viscous_xy_nspm * local_vel.y.abs()),
            )
        } else {
            (0.0, 0.0)
        };
        let w = local_vel.z;
        let tz = if w != 0.0 {
            let coulomb = (w.abs() / FRICTION_W_RAMP_RADPS).min(1.0) * p.coulomb_w_nm;
            w.signum() * (coulomb + p.viscous_w_nmsprad * w.abs())
        } else {
            0.0
        };
        SVector::<f32, 3>::new(fx, fy, tz)
    }

    /// Feed-forward output acceleration, after TIGERs' `CtrlPantheraUpdate`
    /// (Firmware src/robot/ctrl_panthera.c): add the friction compensation
    /// acceleration for the commanded local velocity to the commanded local
    /// acceleration, then divide by the output efficiency. The result is fed to
    /// `transform_accel2wheel_local` to get wheel torques.
    pub fn feedforward_local_accel(
        &self,
        local_vel: SVector<f32, 3>,
        local_accel: SVector<f32, 3>,
    ) -> SVector<f32, 3> {
        let p = &self.params;
        let acc = local_accel + self.inertia_inv * self.friction_comp_force(local_vel);
        SVector::<f32, 3>::new(
            acc.x / p.efficiency_xy,
            acc.y / p.efficiency_xy,
            acc.z / p.efficiency_w,
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
            coulomb_xy_n: 2.65,
            viscous_xy_nspm: 0.5,
            coulomb_w_nm: 0.097,
            viscous_w_nmsprad: 0.002,
            efficiency_xy: 0.72,
            efficiency_w: 0.6,
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
    fn friction_direction_factor_matches_geometry() {
        // alpha = 30 deg, beta = 45 deg: roller speed along x is sum |D_y| =
        // 2 sin a + 2 sin b, along y it is sum |D_x| = 2 cos a + 2 cos b. The
        // reference direction is sideways (local y), as TIGERs' X axis.
        let model = RobotModel::new(RobotPhysicalParams::default()).unwrap();
        let (a, b) = (PI / 6.0, PI / 4.0);
        let k_x = model.friction_direction_factor(SVector::<f32, 2>::new(1.0, 0.0));
        let k_y = model.friction_direction_factor(SVector::<f32, 2>::new(0.0, 1.0));
        let k_neg_x = model.friction_direction_factor(SVector::<f32, 2>::new(-1.0, 0.0));
        assert!((k_y - 1.0).abs() < 1e-6);
        let expected_x = (a.sin() + b.sin()) / (a.cos() + b.cos());
        assert!((k_x - expected_x).abs() < 1e-5);
        assert!((k_neg_x - k_x).abs() < 1e-6);
    }

    #[test]
    fn friction_comp_force_matches_tigers_model() {
        let model = test_model();
        let p = model.get_params();

        // Pure sideways motion: k = 1, force along motion.
        let f = model.friction_comp_force(SVector::<f32, 3>::new(0.0, -0.5, 0.0));
        assert_eq!(f.x, 0.0);
        assert!((f.y + (p.coulomb_xy_n + p.viscous_xy_nspm * 0.5)).abs() < 1e-5);

        // Pure forward motion: scaled by k(+x).
        let f = model.friction_comp_force(SVector::<f32, 3>::new(0.4, 0.0, 0.0));
        let k_x = model.friction_direction_factor(SVector::<f32, 2>::new(1.0, 0.0));
        assert!((f.x - k_x * (p.coulomb_xy_n + p.viscous_xy_nspm * 0.4)).abs() < 1e-5);

        // Diagonal motion: per-axis viscous term, Coulomb along the direction.
        let v = SVector::<f32, 3>::new(0.3, -0.4, 0.0);
        let f = model.friction_comp_force(v);
        let dir = SVector::<f32, 2>::new(0.6, -0.8);
        let k = model.friction_direction_factor(dir);
        assert!((f.x - dir.x * k * (p.coulomb_xy_n + p.viscous_xy_nspm * 0.3)).abs() < 1e-5);
        assert!((f.y - dir.y * k * (p.coulomb_xy_n + p.viscous_xy_nspm * 0.4)).abs() < 1e-5);

        // Angular.
        let f = model.friction_comp_force(SVector::<f32, 3>::new(0.0, 0.0, -2.0));
        assert!((f.z + (p.coulomb_w_nm + p.viscous_w_nmsprad * 2.0)).abs() < 1e-6);

        // Zero velocity: off.
        let f = model.friction_comp_force(SVector::<f32, 3>::zeros());
        assert_eq!(f, SVector::<f32, 3>::zeros());

        // Below the ramp speeds: Coulomb scales linearly with speed.
        let v = 0.25 * FRICTION_XY_RAMP_MPS;
        let f = model.friction_comp_force(SVector::<f32, 3>::new(0.0, v, 0.0));
        assert!((f.y - (0.25 * p.coulomb_xy_n + p.viscous_xy_nspm * v)).abs() < 1e-5);
        let w = 0.5 * FRICTION_W_RAMP_RADPS;
        let f = model.friction_comp_force(SVector::<f32, 3>::new(0.0, 0.0, -w));
        assert!((f.z + (0.5 * p.coulomb_w_nm + p.viscous_w_nmsprad * w)).abs() < 1e-6);
    }

    #[test]
    fn feedforward_local_accel_adds_friction_and_divides_by_efficiency() {
        let model = test_model();
        let p = model.get_params();
        let v = SVector::<f32, 3>::new(0.0, 1.0, 3.0);
        let a = SVector::<f32, 3>::new(1.0, 0.5, -2.0);
        let out = model.feedforward_local_accel(v, a);
        let f = model.friction_comp_force(v);
        assert!((out.x - a.x / p.efficiency_xy).abs() < 1e-5);
        assert!((out.y - (a.y + f.y / p.mass_kg) / p.efficiency_xy).abs() < 1e-5);
        assert!((out.z - (a.z + f.z / p.iz_kgm2) / p.efficiency_w).abs() < 1e-4);
    }

    #[test]
    fn update_params_rejects_invalid_friction_model() {
        let mut model = test_model();
        let mut params = test_params();
        params.efficiency_xy = 0.0;
        assert_eq!(model.update_params(params).err(), Some(ControlsError::InvalidInput));
        let mut params = test_params();
        params.efficiency_w = 1.5;
        assert_eq!(model.update_params(params).err(), Some(ControlsError::InvalidInput));
        let mut params = test_params();
        params.coulomb_xy_n = f32::NAN;
        assert_eq!(model.update_params(params).err(), Some(ControlsError::InvalidInput));
        let mut params = test_params();
        params.viscous_w_nmsprad = -0.1;
        assert_eq!(model.update_params(params).err(), Some(ControlsError::InvalidInput));
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
            coulomb_xy_n: 0.0,
            viscous_xy_nspm: 0.0,
            coulomb_w_nm: 0.0,
            viscous_w_nmsprad: 0.0,
            efficiency_xy: 1.0,
            efficiency_w: 1.0,
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
