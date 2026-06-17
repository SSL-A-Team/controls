use libm::{cosf, sinf};
use crate::{wrap_angle, ControlsError, Vector2f, Vector3f, Vector6f};
use crate::bangbang_trajectory::{BangBangTraj1D, solve_1d_pose, eval_1d_state_at, eval_1d_accel_at};

/// Default SSL ball radius in meters.
pub const DEFAULT_BALL_RADIUS: f32 = 0.0215;
/// Default robot front-face radius in meters (center-to-front distance).
pub const DEFAULT_ROBOT_RADIUS: f32 = 0.090;

/// Parameters for a pivot trajectory.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PivotParams {
    /// Maximum heading angular velocity (rad/s).
    pub max_vel_angular: f32,
    /// Maximum heading angular acceleration (rad/s²).
    pub max_accel_angular: f32,
    /// Radius of the pivot orbit (m).
    pub orbit_radius: f32,
}

impl Default for PivotParams {
    fn default() -> Self {
        use core::f32::consts::PI;
        Self {
            max_vel_angular: 1.0 * PI,
            max_accel_angular: 2.0 * PI,
            orbit_radius: DEFAULT_BALL_RADIUS + DEFAULT_ROBOT_RADIUS,
        }
    }
}

/// Time-optimal trajectory that pivots the robot around a stationary ball.
///
/// The robot's heading θ(t) follows a trapezoidal velocity (bang-bang acceleration)
/// profile from the current heading to `target_heading`.  The robot's X/Y position
/// is derived from θ(t) so the front face remains in contact with the ball at all
/// times:
///
/// ```text
///   x(t) = ball_x - r · cos(θ(t))
///   y(t) = ball_y - r · sin(θ(t))
/// ```
///
/// where `r = ball_radius + robot_radius`.
///
/// Velocity:
/// ```text
///   ẋ(t) =  r · sin(θ) · θ̇
///   ẏ(t) = -r · cos(θ) · θ̇
/// ```
///
/// Acceleration:
/// ```text
///   ẍ(t) = r · ( cos(θ) · θ̇² + sin(θ) · θ̈)
///   ÿ(t) = r · ( sin(θ) · θ̇² - cos(θ) · θ̈)
/// ```
#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct PivotTrajectory {
    /// Heading (θ) bang-bang 1D trajectory.
    pub theta: BangBangTraj1D,
    /// Orbit center x in the global frame (m).
    pub center_x: f32,
    /// Orbit center y in the global frame (m).
    pub center_y: f32,
    /// Orbit radius: `ball_radius + robot_radius` (m).
    pub orbit_radius: f32,
}

impl PivotTrajectory {
    /// Construct a pivot trajectory from the current robot state.
    ///
    /// The robot pivots from its current heading (`init_state[2]`) to
    /// `target_heading`, keeping the front face touching the ball at `ball_pos`.
    /// The shortest angular path is used (heading wraps through ±π).
    ///
    /// # Arguments
    /// * `init_state`     – `[x, y, θ, ẋ, ẏ, θ̇]` current robot state.
    /// * `ball_pos`       – ball center position; only x and y components are used.
    /// * `target_heading` – desired final heading (rad).
    /// * `params`         – velocity/acceleration limits and geometry constants.
    pub fn new(
        init_state: Vector6f,
        center_pos: Vector2f,
        target_heading: f32,
        params: PivotParams,
    ) -> Result<Self, ControlsError> {
        if params.max_vel_angular <= 0.0 || params.max_accel_angular <= 0.0 {
            return Err(ControlsError::InvalidInput);
        }
        if params.orbit_radius <= 0.0 {
            return Err(ControlsError::InvalidInput);
        }
        let current_heading = init_state[2];
        let current_heading_dot = init_state[5];
        // Shortest angular path: adjust target so the solver sees a direct displacement
        let theta_target = current_heading + wrap_angle(target_heading - current_heading);

        let theta = solve_1d_pose(
            current_heading,
            current_heading_dot,
            theta_target,
            params.max_vel_angular,
            params.max_accel_angular,
        )?;

        Ok(PivotTrajectory {
            theta,
            center_x: center_pos.x,
            center_y: center_pos.y,
            orbit_radius: params.orbit_radius,
        })
    }

    /// Shift all trajectory segment times forward by `dt` seconds.
    pub fn time_shift(&mut self, dt: f32) {
        self.theta.time_shift(dt);
    }

    /// Time at which the heading trajectory reaches its target (seconds).
    pub fn end_time(&self) -> f32 {
        self.theta.t4
    }

    /// Evaluate the full robot state `[x, y, θ, ẋ, ẏ, θ̇]` at time `t`.
    ///
    /// X/Y position and velocity are derived from θ(t) to maintain ball contact.
    /// θ is wrapped to [-π, π] in the returned state.
    pub fn state_at(
        &self,
        current_state: Vector6f,
        current_time: f32,
        t: f32,
    ) -> Result<Vector6f, ControlsError> {
        let (theta_t, theta_dot_t) = eval_1d_state_at(
            self.theta,
            current_state[2],
            current_state[5],
            current_time,
            t,
        )?;

        let ct = cosf(theta_t);
        let st = sinf(theta_t);
        let r = self.orbit_radius;

        let x = self.center_x - r * ct;
        let y = self.center_y - r * st;
        // ẋ = r · sin(θ) · θ̇
        let xd = r * st * theta_dot_t;
        // ẏ = -r · cos(θ) · θ̇
        let yd = -r * ct * theta_dot_t;

        Ok(Vector6f::new(x, y, wrap_angle(theta_t), xd, yd, theta_dot_t))
    }

    /// Evaluate the robot acceleration command `[ẍ, ÿ, θ̈]` at time `t`.
    ///
    /// Both the centripetal (θ̇² term) and tangential (θ̈ term) contributions to
    /// the X/Y acceleration are included.
    pub fn accel_at(
        &self,
        current_state: Vector6f,
        current_time: f32,
        t: f32,
    ) -> Result<Vector3f, ControlsError> {
        let theta_ddot = eval_1d_accel_at(self.theta, t)?;
        let (theta_t, theta_dot_t) = eval_1d_state_at(
            self.theta,
            current_state[2],
            current_state[5],
            current_time,
            t,
        )?;
        let ct = cosf(theta_t);
        let st = sinf(theta_t);
        let r = self.orbit_radius;
        let w = theta_dot_t;
        let wd = theta_ddot;

        // ẍ = r · (cos(θ) · θ̇² + sin(θ) · θ̈)
        let xdd = r * (ct * w * w + st * wd);
        // ÿ = r · (sin(θ) · θ̇² - cos(θ) · θ̈)
        let ydd = r * (st * w * w - ct * wd);

        Ok(Vector3f::new(xdd, ydd, theta_ddot))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vector6f;
    use core::f32::consts::PI;
    use libm::{fabsf, sqrtf};

    fn test_params() -> PivotParams {
        PivotParams::default()
    }

    fn init_state_at_heading(theta: f32) -> Vector6f {
        let r = test_params().orbit_radius;
        Vector6f::new(
            -r * cosf(theta), // x: robot positioned behind ball at given heading
            -r * sinf(theta), // y
            theta,
            0.0, 0.0, 0.0,
        )
    }

    #[test]
    fn heading_reaches_target() {
        let init = init_state_at_heading(0.0);
        let ball = Vector3f::zeros();
        let target = PI / 2.0;
        let traj = PivotTrajectory::new(init, ball, target, test_params()).unwrap();
        let end = traj.end_time();
        assert!(end > 0.0);
        let final_state = traj.state_at(init, 0.0, end).unwrap();
        assert!(fabsf(wrap_angle(final_state[2] - target)) < 1e-3);
    }

    #[test]
    fn heading_reaches_target_negative() {
        let init = init_state_at_heading(0.0);
        let ball = Vector3f::zeros();
        let target = -PI / 2.0;
        let traj = PivotTrajectory::new(init, ball, target, test_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(init, 0.0, end).unwrap();
        assert!(fabsf(wrap_angle(final_state[2] - target)) < 1e-3);
    }

    #[test]
    fn xy_position_on_orbit_at_all_times() {
        let init = init_state_at_heading(0.0);
        let ball = Vector3f::new(1.0, 0.5, 0.0);
        let target = PI / 2.0;
        let traj = PivotTrajectory::new(init, ball, target, test_params()).unwrap();
        let r = traj.orbit_radius;
        let end = traj.end_time();
        // Sample several times along trajectory
        let n = 20;
        for i in 0..=n {
            let t = end * (i as f32) / (n as f32);
            let state = traj.state_at(init, 0.0, t).unwrap();
            let dx = state[0] - traj.center_x;
            let dy = state[1] - traj.center_y;
            let dist = sqrtf(dx * dx + dy * dy);
            assert!(fabsf(dist - r) < 1e-4, "t={}: dist={}, r={}", t, dist, r);
        }
    }

    #[test]
    fn xy_position_consistent_with_heading() {
        let init = init_state_at_heading(0.5);
        let ball = Vector3f::zeros();
        let target = PI;
        let traj = PivotTrajectory::new(init, ball, target, test_params()).unwrap();
        let end = traj.end_time();
        let n = 10;
        for i in 0..=n {
            let t = end * (i as f32) / (n as f32);
            let state = traj.state_at(init, 0.0, t).unwrap();
            // Robot should be at center - r*[cos(theta), sin(theta)]
            let theta = state[2];
            let expected_x = traj.center_x - traj.orbit_radius * cosf(theta);
            let expected_y = traj.center_y - traj.orbit_radius * sinf(theta);
            assert!(fabsf(state[0] - expected_x) < 1e-4);
            assert!(fabsf(state[1] - expected_y) < 1e-4);
        }
    }

    #[test]
    fn final_velocity_zero() {
        let init = init_state_at_heading(0.0);
        let ball = Vector3f::zeros();
        let traj = PivotTrajectory::new(init, ball, PI / 3.0, test_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(init, 0.0, end).unwrap();
        assert!(fabsf(final_state[3]) < 1e-3, "ẋ should be zero at end");
        assert!(fabsf(final_state[4]) < 1e-3, "ẏ should be zero at end");
        assert!(fabsf(final_state[5]) < 1e-3, "θ̇ should be zero at end");
    }

    #[test]
    fn zero_displacement_has_zero_end_time() {
        let init = init_state_at_heading(1.0);
        let ball = Vector3f::zeros();
        let traj = PivotTrajectory::new(init, ball, 1.0, test_params()).unwrap();
        assert!(traj.end_time() < 1e-6);
    }

    #[test]
    fn uses_shortest_angular_path() {
        // Target PI - epsilon should pivot CCW (small positive angle), not CW the long way
        let init = init_state_at_heading(0.0);
        let ball = Vector3f::zeros();
        let target = PI - 0.1;
        let traj = PivotTrajectory::new(init, ball, target, test_params()).unwrap();
        // The heading trajectory sdd1 should be positive (CCW)
        assert!(traj.theta.sdd1 >= 0.0);
    }

    #[test]
    fn invalid_params_rejected() {
        let init = Vector6f::zeros();
        let ball = Vector3f::zeros();
        let mut params = test_params();
        params.max_vel_angular = 0.0;
        assert!(PivotTrajectory::new(init, ball, 1.0, params).is_err());
        params.max_vel_angular = 1.0;
        params.max_accel_angular = -1.0;
        assert!(PivotTrajectory::new(init, ball, 1.0, params).is_err());
    }
}
