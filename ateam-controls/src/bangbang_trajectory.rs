use core::f32::consts::PI;
use libm::{cosf, sinf, sqrtf};
use crate::{ControlsError, Vector3f, Vector6f, wrap_angle};
use crate::trajectory::{solve_1d_pose, eval_1d_state_at, eval_1d_accel_at};


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
    pub x: BangBangTraj1D,
    pub y: BangBangTraj1D,
    pub z: BangBangTraj1D,
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
        })
    }

    /// Compute optimal bang-bang trajectory to reach a target twist from an initial twist.
    pub fn from_target_twist(init_twist: Vector3f, target_twist: Vector3f, params: TrajectoryParams) -> Result<Self, ControlsError> {
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

    /// Get the full state (position + velocity) at time `t` given `current_state` and `current_time`.
    pub fn state_at(&self, current_state: Vector6f, current_time: f32, t: f32) -> Result<Vector6f, ControlsError> {
        let (x_f, xd_f) = eval_1d_state_at(self.x, current_state[0], current_state[3], current_time, t)?;
        let (y_f, yd_f) = eval_1d_state_at(self.y, current_state[1], current_state[4], current_time, t)?;
        let (z_f, zd_f) = eval_1d_state_at(self.z, current_state[2], current_state[5], current_time, t)?;
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

// --- 1D helper functions ---
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
        let final_state = traj.state_at(init, 0.0, end).unwrap();
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
        let final_state = traj.state_at(init, 0.0, end).unwrap();
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
        let final_state = traj.state_at(init, 0.0, end).unwrap();
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
        let final_state = traj.state_at(init, 0.0, end).unwrap();
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
        let final_state = traj.state_at(init, 0.0, end).unwrap();
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
        let final_state = traj.state_at(init, 0.0, end).unwrap();
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
        let init_twist = Vector3f::new(0.0, 0.0, 0.0);
        let target_twist = Vector3f::new(0.3, 0.0, 0.0);
        let traj = BangBangTraj3D::from_target_twist(init_twist, target_twist, test_params()).unwrap();
        let end = traj.end_time();
        assert!(end > 0.0);
        assert!(traj.x.sdd1 > 0.0);
    }

    #[test]
    fn twist_already_at_target() {
        let twist = Vector3f::new(0.0, 0.0, 0.0);
        let traj = BangBangTraj3D::from_target_twist(twist, twist, test_params()).unwrap();
        assert!(traj.end_time().abs() < 1e-6);
    }

    #[test]
    fn state_at_midpoint() {
        let init = Vector6f::zeros();
        let target = Vector3f::new(2.0, 0.0, 0.0);
        let traj = BangBangTraj3D::from_target_pose(init, target, test_params()).unwrap();
        let end = traj.end_time();
        let mid = end / 2.0;
        let mid_state = traj.state_at(init, 0.0, mid).unwrap();
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
            let st = traj.state_at(init, 0.0, t).unwrap();
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
                let st = traj.state_at(init, 0.0, t).unwrap();
                let theta = st[2];
                assert!(theta <= PI + 1e-3,
                    "theta > π: start={} target={} t={} theta={}", start, target_theta, t, theta);
                assert!(theta >= -PI - 1e-3,
                    "theta < -π: start={} target={} t={} theta={}", start, target_theta, t, theta);
            }
        }
    }
}
