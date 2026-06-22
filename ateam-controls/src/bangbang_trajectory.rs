use core::f32::consts::PI;
use libm::{cosf, sinf, sqrtf};
use crate::{ControlsError, Vector3f, Vector6f, wrap_angle};
use crate::trajectory::Trajectory;


/// Parameters controlling trajectory velocity/acceleration limits and error tolerances.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct TrajectoryParams {
    pub max_vel_linear: f32,
    pub max_vel_angular: f32,
    pub max_accel_linear: f32,
    pub max_accel_angular: f32,
}

impl Default for TrajectoryParams {
    fn default() -> Self {
        use crate::defaults::*;
        Self {
            max_vel_linear: DEFAULT_MAX_VEL_LINEAR,
            max_vel_angular: DEFAULT_MAX_VEL_ANGULAR,
            max_accel_linear: DEFAULT_MAX_ACCEL_LINEAR,
            max_accel_angular: DEFAULT_MAX_ACCEL_ANGULAR,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
/// sdd1: t1 -> t2, sdd2: t2 -> t3, sdd3: t3 -> t4
pub struct BangBangTraj1D {
    pub sdd1: f32,
    pub sdd2: f32,
    pub sdd3: f32,
    pub t1: f32,
    pub t2: f32,
    pub t3: f32,
    pub t4: f32,
}

impl BangBangTraj1D {
    pub fn time_shift(&mut self, dt: f32) {
        self.t1 += dt;
        self.t2 += dt;
        self.t3 += dt;
        self.t4 += dt;
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct BangBangTraj3D {
    /// X dimension bang-bang trajectory component
    pub x: BangBangTraj1D,
    /// Y dimension bang-bang trajectory component
    pub y: BangBangTraj1D,
    /// Z dimension (theta) bang-bang trajectory component
    pub z: BangBangTraj1D,
    /// Tracked robot state `[x, y, θ, ẋ, ẏ, θ̇]` at the current t=0.
    pub state: Vector6f,
}

impl BangBangTraj3D {
    /// Compute optimal bang-bang trajectory to reach a target pose from an initial state.
    pub fn from_target_pose(init_state: Vector6f, target_pose: Vector3f, params: TrajectoryParams) -> Result<Self, ControlsError> {
        let mut alpha = PI / 4.0;
        let mut increment = PI / 8.0;
        let precision = 0.1;
        let mut x_traj;
        let mut y_traj;
        loop {
            let cos_alpha = cosf(alpha);
            let sin_alpha = sinf(alpha);
            x_traj = solve_1d_pose(init_state[0], init_state[3], target_pose[0], cos_alpha * params.max_vel_linear, cos_alpha * params.max_accel_linear)?;
            y_traj = solve_1d_pose(init_state[1], init_state[4], target_pose[1], sin_alpha * params.max_vel_linear, sin_alpha * params.max_accel_linear)?;
            if x_traj.t4 > y_traj.t4 {
                alpha -= increment;
            } else {
                alpha += increment;
            }
            if increment <= precision {
                break
            }
            increment *= 0.5;
        }
        // Compute theta target via shortest angular path so the trajectory
        // wraps through +/- pi instead of going the long way around.
        let theta_target = init_state[2] + wrap_angle(target_pose[2] - init_state[2]);
        Ok(BangBangTraj3D {
            x: x_traj,
            y: y_traj,
            z: solve_1d_pose(init_state[2], init_state[5], theta_target, params.max_vel_angular, params.max_accel_angular)?,
            state: init_state,
        })
    }

    /// Compute optimal bang-bang trajectory to reach a target twist from an
    /// initial full state. Only the velocity components of `init_state` shape the
    /// profile; the full state seeds the trajectory's internal current state.
    pub fn from_target_twist(init_state: Vector6f, target_twist: Vector3f, params: TrajectoryParams) -> Result<Self, ControlsError> {
        let init_twist = Vector3f::new(init_state[3], init_state[4], init_state[5]);
        let diff = target_twist - init_twist;
        let diff_xy_mag = sqrtf(diff.x * diff.x + diff.y * diff.y);

        // Solve for optimal ratio of x and y accelerations using magnitude to avoid division by zero
        let (xdd, x_time, ydd, y_time) = if diff_xy_mag < 1e-9 {
            (0.0, 0.0, 0.0, 0.0)
        } else {
            let xdd = diff.x * params.max_accel_linear / diff_xy_mag;
            let ydd = diff.y * params.max_accel_linear / diff_xy_mag;
            let t = diff_xy_mag / params.max_accel_linear;
            (xdd, t, ydd, t)
        };
        // Set max z (theta) acceleration
        let (zdd, z_time) = if diff.z.abs() < 1e-9 {
            (0.0, 0.0)
        } else {
            let zdd = params.max_accel_angular * diff.z.signum();
            (zdd, diff.z / zdd)
        };
        Ok(BangBangTraj3D {
            x: BangBangTraj1D { sdd1: xdd, sdd2: 0.0, sdd3: 0.0, t1: 0.0, t2: x_time, t3: x_time, t4: x_time },
            y: BangBangTraj1D { sdd1: ydd, sdd2: 0.0, sdd3: 0.0, t1: 0.0, t2: y_time, t3: y_time, t4: y_time },
            z: BangBangTraj1D { sdd1: zdd, sdd2: 0.0, sdd3: 0.0, t1: 0.0, t2: z_time, t3: z_time, t4: z_time },
            state: init_state,
        })
    }

    pub fn time_shift(&mut self, dt: f32) {
        self.x.time_shift(dt);
        self.y.time_shift(dt);
        self.z.time_shift(dt);
    }

    pub fn end_time(&self) -> f32 {
        self.x.t4.max(self.y.t4).max(self.z.t4)
    }

    /// Get the full state (position + velocity) at time `t`, evaluated from the
    /// trajectory's internal current state at internal time 0.
    pub fn state_at(&self, t: f32) -> Result<Vector6f, ControlsError> {
        let (x_f, xd_f) = eval_1d_state_at(self.x, self.state[0], self.state[3], 0.0, t)?;
        let (y_f, yd_f) = eval_1d_state_at(self.y, self.state[1], self.state[4], 0.0, t)?;
        let (z_f, zd_f) = eval_1d_state_at(self.z, self.state[2], self.state[5], 0.0, t)?;
        Ok(Vector6f::new(x_f, y_f, wrap_angle(z_f), xd_f, yd_f, zd_f))
    }

    /// Get the acceleration command at time `t`.
    pub fn accel_at(&self, t: f32) -> Result<Vector3f, ControlsError> {
        Ok(Vector3f::new(
            eval_1d_accel_at(self.x, t)?,
            eval_1d_accel_at(self.y, t)?,
            eval_1d_accel_at(self.z, t)?,
        ))
    }
}

impl Trajectory for BangBangTraj3D {
    fn tick(&mut self, dt: f32) {
        if let Ok(next) = self.state_at(dt) {
            self.state = next;
        }
        self.time_shift(-dt);
    }

    fn sample(&self) -> (Vector6f, Vector3f) {
        // accel should always be defined for t=0, but default to zero if something goes wrong
        let accel = self.accel_at(0.0).unwrap_or_default();
        (self.state, accel)
    }
}

// --- 1D helper functions ---

/// Evaluate the bang-bang acceleration at time `t` for a 1D trajectory segment.
pub(crate) fn eval_1d_accel_at(traj: BangBangTraj1D, t: f32) -> Result<f32, ControlsError> {
    if t >= traj.t4 {
        return Ok(0.0);
    }
    if t >= traj.t3 {
        return Ok(traj.sdd3);
    }
    if t >= traj.t2 {
        return Ok(traj.sdd2);
    }
    if t >= traj.t1 {
        return Ok(traj.sdd1);
    }
    Err(ControlsError::InvalidTime)
}

/// Takes the initial velocity sd0, the desired positive change in position ds, and the bang-bang acceleration sdd.
/// Returns the positive acceleration time T1, the negative acceleration time T2, and the peak velocity reached Vpeak.
fn triangular_profile(sd0: f32, ds: f32, sdd: f32) -> Result<(f32, f32, f32), ControlsError> {
    if sd0 < 0.0 || ds < 0.0 || sdd < 0.0 {
        return Err(ControlsError::InvalidInput);
    }

    let t2 = sqrtf((sdd * ds + 0.5 * sd0 * sd0) / (sdd * sdd));
    let t1 = t2 - sd0 / sdd;
    let vpeak = sdd * t2;
    Ok((t1, t2, vpeak))
}

/// Takes the initial velocity sd0, the desired positive change in position ds, the bang-bang acceleration sdd, and the max velocity constraint sd_max.
/// Returns the acceleration or deceleration time T1, the coasting time T2, the negative acceleration time T3, and the acceleration used for T1 (sdd1).
fn trapezoidal_profile(sd0: f32, ds: f32, sdd: f32, sd_max: f32) -> Result<(f32, f32, f32, f32), ControlsError> {
    if sd0 < 0.0 || ds <= 0.0 || sdd <= 0.0 || sd_max <= 0.0 {  // allow sd0 to be zero
        return Err(ControlsError::InvalidInput);
    }

    // The first period of time can either be acceleration to max velocity or deceleration to max velocity
    let t1;
    let d1;
    let sdd1;
    if sd0 > sd_max {
        t1 = (sd0 - sd_max) / sdd;  // deceleration time
        d1 = sd_max * t1 + 0.5 * (sd0 - sd_max) * t1;  // distance traveled during deceleration
        sdd1 = -sdd;
    } else {
        t1 = (sd_max - sd0) / sdd;  // acceleration time
        d1 = sd0 * t1 + 0.5 * (sd_max - sd0) * t1;  // distance traveled during acceleration
        sdd1 = sdd;
    }
    let t3 = sd_max / sdd;  // breaking time
    let d3 = 0.5 * sd_max * t3;  // distance traveled during the deceleration
    let d2 = ds - d1 - d3;  // distance traveled during the coast
    let t2 = d2 / sd_max;  // coasting time
    if t1 < 0.0 || t2 < 0.0 || t3 < 0.0 {
        return Err(ControlsError::NoSolution);
    }
    Ok((t1, t2, t3, sdd1))
}

/// Returns the time it took to brake and the resulting position.
fn compute_brake(s0: f32, sd0: f32, sdd: f32) -> (f32, f32) {
    let time_to_rest = sd0.abs() / sdd.abs();
    let sf = s0 + 0.5 * sd0 * time_to_rest;
    (time_to_rest, sf)
}

/// Solve one-dimensional bang-bang trajectory to reach target position `s_trg`.
pub(crate) fn solve_1d_pose(s0: f32, sd0: f32, s_trg: f32, sd_max: f32, sdd_max: f32) -> Result<BangBangTraj1D, ControlsError> {
    if sdd_max <= 0.0 || sd_max <= 0.0 {
        return Err(ControlsError::InvalidInput);
    }

    let mut traj = BangBangTraj1D::default();
    let mut s = s0;
    let mut sd = sd0;

    // First check if s will need to turn around (change sign in velocity)
    if sd != 0.0 {
        // a. check if initial velocity is in the wrong direction
        // b. check if full brake will overshoot s_trg
        let (brake_time, s_after_full_brake) = compute_brake(s, sd, sdd_max);
        // Check if full brake position is outside the segment between s0 and s_trg
        if s_after_full_brake != s_trg && (s_after_full_brake - s).is_sign_positive() == (s_after_full_brake - s_trg).is_sign_positive() {
            // The trajectory requires a full brake to rest immediately to turn around
            traj.sdd1 = - sd.signum() * sdd_max;  // acceleration should be in opposite direction of initial velocity
            traj.t2 += brake_time;
            traj.t3 += brake_time;
            traj.t4 += brake_time;
            s = s_after_full_brake;
            sd = 0.0;
        }
    }

    // Solve triangular profile and check if unconstrained velocity is greater than sd_max
    let ds = s_trg - s;
    let direction = ds.signum();
    let (t1, t2, vpeak) = triangular_profile(sd.abs(), ds.abs(), sdd_max)?;
    if vpeak < sd_max {

        // accelerate towards target
        traj.sdd1 = direction * sdd_max;  // NOTE: this could have already been set if brake to zero was added in the beginning of the trajectory, but the acceleration should be in the same direction here if that is the case
        traj.t2 += t1;
        traj.t3 += t1;
        traj.t4 += t1;

        // Skip the coasting time period
        // traj.sdd2 = 0.0;
        // traj.t3 += 0.0;
        // traj.t4 += 0.0;

        // brake towards target
        traj.sdd3 = - direction * sdd_max;
        traj.t4 += t2;

    } else {
        // Solve trapezoidal profile
        let (t1, t2, t3, sdd1) = trapezoidal_profile(sd.abs(), ds.abs(), sdd_max, sd_max)?;

        // accelerate or decelerate to max velocity
        traj.sdd1 = direction * sdd1;  // NOTE: this could have already been set if brake to zero was added in the beginning of the trajectory, but the acceleration should be in the same direction here if that is the case
        traj.t2 += t1;
        traj.t3 += t1;
        traj.t4 += t1;

        // coast
        traj.sdd2 = 0.0;
        traj.t3 += t2;
        traj.t4 += t2;

        // brake towards target
        traj.sdd3 = - direction * sdd_max;
        traj.t4 += t3;
    }

    Ok(traj)
}

/// Evaluate one-dimensional trajectory state (position, velocity) at time `t`.
pub(crate) fn eval_1d_state_at(traj: BangBangTraj1D, s: f32, sd: f32, current_time: f32, t: f32) -> Result<(f32, f32), ControlsError> {
    let mut s = s;
    let mut sd = sd;
    let mut current_time = current_time;

    if t < current_time {
        return Err(ControlsError::InvalidTime);
    }
    if traj.t1 > current_time {
        return Err(ControlsError::InvalidTime);
    }

    for (part_end_time, part_acceleration) in [(traj.t2, traj.sdd1), (traj.t3, traj.sdd2), (traj.t4, traj.sdd3)] {
        if current_time < part_end_time {
            let time_in_part;
            if t < part_end_time {
                time_in_part = t - current_time;
            } else {
                time_in_part = part_end_time - current_time;
            }
            s += sd * time_in_part + 0.5 * part_acceleration * time_in_part * time_in_part;  // v * t + 1/2 * a * t^2
            sd += part_acceleration * time_in_part;  // a * t
            if t < part_end_time {
                return Ok((s, sd))  // reached the desired time
            } else {
                current_time = part_end_time;
            }
        }
    }
    // Coast at final velocity for any remaining time past the trajectory end
    s += sd * (t - current_time);
    Ok((s, sd))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vector6f;
    use core::f32::consts::PI;
    use libm::fabsf;

    fn test_params() -> TrajectoryParams {
        TrajectoryParams {
            max_vel_linear: 3.0,
            max_vel_angular: 3.0 * PI,
            max_accel_linear: 2.0,
            max_accel_angular: 2.0 * PI,
        }
    }

    #[test]
    fn pose_x() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(1.0, 0.0, 0.0);
        let traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        let end = traj.end_time();
        assert!(end > 0.0);
        let final_state = traj.state_at(end).unwrap();
        assert!((final_state[0] - 1.0).abs() < 1e-3);
        assert!((final_state[1]).abs() < 1e-3);
        assert!((final_state[2]).abs() < 1e-3);
    }

    #[test]
    fn pose_y() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(0.0, 1.0, 0.0);
        let traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        assert!((final_state[0]).abs() < 1e-3);
        assert!((final_state[1] - 1.0).abs() < 1e-3);
        assert!((final_state[2]).abs() < 1e-3);
    }

    #[test]
    fn pose_z() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(0.0, 0.0, 1.0);
        let traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        assert!((final_state[0]).abs() < 1e-3);
        assert!((final_state[1]).abs() < 1e-3);
        assert!((final_state[2] - 1.0).abs() < 1e-3);
    }

    #[test]
    fn pose_xyz() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(1.0, 1.0, 1.0);
        let traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        assert!((final_state[0] - 1.0).abs() < 1e-3);
        assert!((final_state[1] - 1.0).abs() < 1e-3);
        assert!((final_state[2] - 1.0).abs() < 1e-3);
    }

    #[test]
    fn pose_negative() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(-2.0, -1.5, -0.5);
        let traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        assert!((final_state[0] - (-2.0)).abs() < 1e-3);
        assert!((final_state[1] - (-1.5)).abs() < 1e-3);
        assert!((final_state[2] - (-0.5)).abs() < 1e-3);
    }

    #[test]
    fn pose_final_velocity_zero() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(1.0, 0.5, 0.0);
        let traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        assert!((final_state[3]).abs() < 1e-3);
        assert!((final_state[4]).abs() < 1e-3);
        assert!((final_state[5]).abs() < 1e-3);
    }

    #[test]
    fn accel_symmetry() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(1.0, 0.0, 1.0);
        let traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        assert!((traj.x.sdd1 - (-traj.x.sdd3)).abs() < 1e-6);
        assert!((traj.z.sdd1 - (-traj.z.sdd3)).abs() < 1e-6);
    }

    #[test]
    fn accel_at_time_after_end() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(1.0, 0.0, 0.0);
        let traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        let end = traj.end_time();
        let accel = traj.accel_at(end + 1.0).unwrap();
        assert!(accel.x.abs() < 1e-6);
        assert!(accel.y.abs() < 1e-6);
        assert!(accel.z.abs() < 1e-6);
    }

    #[test]
    fn accel_at_time_start() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(1.0, 0.0, 0.0);
        let traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        let accel = traj.accel_at(0.0).unwrap();
        assert!(accel.x.abs() > 0.0);
    }

    #[test]
    fn time_shift_3d() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(1.0, 0.0, 0.0);
        let mut traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        let shift = 2.5;
        let initial_end = traj.end_time();
        traj.time_shift(shift);
        let shifted_end = traj.end_time();
        assert!((shifted_end - initial_end - shift).abs() < 1e-6);
    }

    #[test]
    fn time_shift_1d() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(1.0, 0.5, 0.0);
        let mut traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        let orig_t1 = traj.x.t1;
        let shift = 3.0;
        traj.x.time_shift(shift);
        assert!((traj.x.t1 - orig_t1 - shift).abs() < 1e-6);
    }

    #[test]
    fn already_at_target() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(0.0, 0.0, 0.0);
        let traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        assert!(traj.end_time().abs() < 1e-6);
    }

    #[test]
    fn twist_target() {
        let init_state = Vector6f::zeros();
        let target_twist = Vector3f::new(0.3, 0.0, 0.0);
        let traj = BangBangTraj3D::from_target_twist(init_state, target_twist, test_params()).unwrap();
        let end = traj.end_time();
        assert!(end > 0.0);
        assert!(traj.x.sdd1 > 0.0);
    }

    #[test]
    fn twist_already_at_target() {
        let init_state = Vector6f::new(0.0, 0.0, 0.0, 0.2, 0.0, 0.0);
        let twist = Vector3f::new(0.2, 0.0, 0.0);
        let traj = BangBangTraj3D::from_target_twist(init_state, twist, test_params()).unwrap();
        assert!(traj.end_time().abs() < 1e-6);
    }

    #[test]
    fn state_at_midpoint() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(2.0, 0.0, 0.0);
        let traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        let end = traj.end_time();
        let mid = end / 2.0;
        let mid_state = traj.state_at(mid).unwrap();
        assert!(mid_state[0] > 0.0);
        assert!(mid_state[0] < 2.0);
    }

    #[test]
    fn theta_wraps_around_pi() {
        let start_theta = 2.8_f32;
        let target_theta = -2.8_f32;
        let init = Vector6f::new(0.0, 0.0, start_theta, 0.0, 0.0, 0.0);
        let target = Vector3f::new(0.0, 0.0, target_theta);
        let traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        let end = traj.end_time();
        assert!(end > 0.0);

        let n = 50;
        for i in 0..=n {
            let t = end * (i as f32) / (n as f32);
            let st = traj.state_at(t).unwrap();
            let theta = st[2];
            assert!(fabsf(theta) > PI / 2.0,
                "theta crossed through 0 at sample {} (t={}, theta={})", i, t, theta);
        }
    }

    #[test]
    fn theta_stays_within_pi_bounds() {
        let cases: &[(f32, f32)] = &[
            (0.0, 2.5), (0.0, -2.5),
            (2.8, -2.8), (-2.8, 2.8),
            (1.0, -1.0), (3.0, 0.5), (-3.0, -0.5),
        ];

        let params = test_params();
        let n = 100;

        for &(start, target_theta) in cases {
            let init = Vector6f::new(0.0, 0.0, start, 0.0, 0.0, 0.0);
            let target = Vector3f::new(0.0, 0.0, target_theta);
            let traj = BangBangTraj3D::from_target_pose(init, target, params).unwrap();
            let end = traj.end_time();

            for i in 0..=n {
                let t = end * (i as f32) / (n as f32);
                let st = traj.state_at(t).unwrap();
                let theta = st[2];
                assert!(theta <= PI + 1e-3,
                    "theta > π: start={} target={} t={} theta={}", start, target_theta, t, theta);
                assert!(theta >= -PI - 1e-3,
                    "theta < -π: start={} target={} t={} theta={}", start, target_theta, t, theta);
            }
        }
    }
}
