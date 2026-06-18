use libm::{atan2f, cosf, sinf};
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
    /// Maximum angular velocity for both trajectories (rad/s).
    pub max_vel_angular: f32,
    /// Maximum angular acceleration for both trajectories (rad/s²).
    pub max_accel_angular: f32,
    /// Radius of the pivot orbit (m).
    pub orbit_radius: f32,
    /// Angular distance (radians, ≥ 0) that the orbit travels before the heading
    /// trajectory starts. Internally converted to a time delay by solving a
    /// sub-trajectory for the first `heading_lag` radians of orbit travel.
    /// `heading_lag = 0` starts both trajectories simultaneously.
    pub heading_lag: f32,
}

impl Default for PivotParams {
    fn default() -> Self {
        use core::f32::consts::PI;
        Self {
            max_vel_angular: 1.0 * PI,
            max_accel_angular: 2.0 * PI,
            orbit_radius: DEFAULT_BALL_RADIUS + DEFAULT_ROBOT_RADIUS,
            heading_lag: 0.05,
        }
    }
}

/// Time-optimal pivot trajectory: two independent 1D bang-bang profiles.
///
/// # Orbit trajectory (`orbit`)
/// Tracks the orbital position angle φ from `orbit_start` to `orbit_target`.
/// - `orbit_start` is derived from the robot's X/Y position relative to `center_pos`.
/// - `orbit_start_dot` is derived from the robot's XY velocity.
/// - X/Y position: `x = center_x − r·cos(φ)`, `y = center_y − r·sin(φ)`.
///
/// # Heading trajectory (`heading`)
/// Tracks the robot's facing angle θ from `orbit_start` to `orbit_target`.
/// - Always starts at `orbit_start` with zero velocity (heading points at center).
/// - **Delayed** by `t_heading_start` seconds — computed by solving how long the
///   orbit takes to travel `heading_lag` radians. Before that time, θ = `orbit_start`.
///
/// Both profiles may finish at different times. `end_time()` returns
/// `max(orbit.t4, heading.t4)`.
#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct PivotTrajectory {
    /// Orbital position angle trajectory (`orbit_start` → `orbit_target`).
    pub orbit: BangBangTraj1D,
    /// Heading angle trajectory (`orbit_start` → `orbit_target`, time-shifted by
    /// `t_heading_start`).
    pub heading: BangBangTraj1D,
    /// Orbit center x (m).
    pub center_x: f32,
    /// Orbit center y (m).
    pub center_y: f32,
    /// Orbit radius (m).
    pub orbit_radius: f32,
    /// Initial orbit angle (derived from robot XY at creation time).
    pub orbit_start: f32,
    /// Initial orbit angular velocity (derived from robot XY velocity).
    pub orbit_start_dot: f32,
    /// Orbit target angle (= desired target heading).
    pub orbit_target: f32,
    /// Wall-clock time at which the heading trajectory starts (seconds).
    /// Before this time, θ is held at `orbit_start`.
    pub t_heading_start: f32,
}

impl PivotTrajectory {
    /// Construct a pivot trajectory.
    ///
    /// # Arguments
    /// * `init_state`     – `[x, y, θ, ẋ, ẏ, θ̇]` current robot state.
    /// * `center_pos`     – orbit center (ball position) in the global frame.
    /// * `target_heading` – desired final heading (rad).
    /// * `params`         – velocity/acceleration limits, orbit radius, heading lag.
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

        // Orbit start from XY position: direction from robot to center.
        let dx = center_pos.x - init_state[0];
        let dy = center_pos.y - init_state[1];
        let orbit_start = atan2f(dy, dx);

        // Orbit start angular velocity from XY body velocity:
        //   ω = (ẋ·sin(orbit) − ẏ·cos(orbit)) / r
        let ct = cosf(orbit_start);
        let st = sinf(orbit_start);
        let orbit_start_dot =
            (init_state[3] * st - init_state[4] * ct) / params.orbit_radius;

        // Unwrap orbit target to the nearest equivalent angle (shortest path).
        let displacement = wrap_angle(target_heading - orbit_start);
        let direction = if displacement >= 0.0 { 1.0_f32 } else { -1.0_f32 };
        let orbit_target = orbit_start + displacement;

        // Orbit trajectory: full journey from orbit_start to orbit_target.
        let orbit = solve_1d_pose(
            orbit_start,
            orbit_start_dot,
            orbit_target,
            params.max_vel_angular,
            params.max_accel_angular,
        )?;

        // Heading start delay: solve a sub-trajectory for the first `heading_lag`
        // radians of orbit travel to find when the heading should start (seconds).
        // Cap lag at the full displacement so it never overshoots orbit_target.
        let lag_magnitude = params.heading_lag.abs().min(displacement.abs());
        let t_heading_start = if lag_magnitude == 0.0 {
            0.0
        } else {
            let lag_end = orbit_start + lag_magnitude * direction;
            let lag_traj = solve_1d_pose(
                orbit_start,
                orbit_start_dot,
                lag_end,
                params.max_vel_angular,
                params.max_accel_angular,
            )?;
            lag_traj.t4
        };

        // Heading trajectory: from orbit_start to orbit_target, starting from rest,
        // time-shifted so it begins after t_heading_start seconds.
        let mut heading = solve_1d_pose(
            orbit_start,
            0.0,
            orbit_target,
            params.max_vel_angular,
            params.max_accel_angular,
        )?;
        heading.time_shift(t_heading_start);

        Ok(PivotTrajectory {
            orbit,
            heading,
            center_x: center_pos.x,
            center_y: center_pos.y,
            orbit_radius: params.orbit_radius,
            orbit_start,
            orbit_start_dot,
            orbit_target,
            t_heading_start,
        })
    }

    /// Shift all trajectory segment times forward by `dt` seconds.
    pub fn time_shift(&mut self, dt: f32) {
        self.orbit.time_shift(dt);
        self.heading.time_shift(dt);
        self.t_heading_start += dt;
    }

    /// Time at which both the orbit and heading trajectories have finished.
    pub fn end_time(&self) -> f32 {
        self.orbit.t4.max(self.heading.t4)
    }

    /// Evaluate the full robot state `[x, y, θ, ẋ, ẏ, θ̇]` at time `t`.
    ///
    /// - X/Y and their velocities come from the orbit trajectory.
    /// - θ/θ̇ come from the heading trajectory (held at `orbit_start` before
    ///   `t_heading_start`, then following its own bang-bang profile).
    ///
    /// Uses stored initial conditions; `current_state` is unused.
    /// `current_time` must satisfy `0 ≤ current_time ≤ t`.
    pub fn state_at(
        &self,
        _current_state: Vector6f,
        current_time: f32,
        t: f32,
    ) -> Result<Vector6f, ControlsError> {
        let (orbit_t, orbit_dot_t) = eval_1d_state_at(
            self.orbit,
            self.orbit_start,
            self.orbit_start_dot,
            current_time,
            t,
        )?;

        let (heading_t, heading_dot_t) = if t < self.t_heading_start {
            (self.orbit_start, 0.0)
        } else {
            eval_1d_state_at(
                self.heading,
                self.orbit_start,
                0.0,
                self.t_heading_start,
                t,
            )?
        };

        let r = self.orbit_radius;
        let ct = cosf(orbit_t);
        let st = sinf(orbit_t);
        let x  = self.center_x - r * ct;
        let y  = self.center_y - r * st;
        let xd =  r * st * orbit_dot_t;
        let yd = -r * ct * orbit_dot_t;

        Ok(Vector6f::new(x, y, wrap_angle(heading_t), xd, yd, heading_dot_t))
    }

    /// Evaluate the robot acceleration `[ẍ, ÿ, θ̈]` at time `t`.
    ///
    /// - X/Y acceleration comes from the orbit trajectory.
    /// - θ̈ comes from the heading trajectory (zero before `t_heading_start`).
    ///
    /// `current_state` is unused. `current_time` is typically 0.0.
    pub fn accel_at(
        &self,
        _current_state: Vector6f,
        current_time: f32,
        t: f32,
    ) -> Result<Vector3f, ControlsError> {
        let orbit_ddot = eval_1d_accel_at(self.orbit, t)?;
        let (orbit_t, orbit_dot_t) = eval_1d_state_at(
            self.orbit,
            self.orbit_start,
            self.orbit_start_dot,
            current_time,
            t,
        )?;

        let heading_ddot = if t < self.t_heading_start {
            0.0
        } else {
            eval_1d_accel_at(self.heading, t)?
        };

        let r = self.orbit_radius;
        let ct = cosf(orbit_t);
        let st = sinf(orbit_t);
        let w = orbit_dot_t;
        let xdd = r * (ct * w * w + st * orbit_ddot);
        let ydd = r * (st * w * w - ct * orbit_ddot);

        Ok(Vector3f::new(xdd, ydd, heading_ddot))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vector6f;
    use core::f32::consts::PI;
    use libm::{fabsf, sqrtf};

    fn test_params() -> PivotParams {
        PivotParams { heading_lag: 0.0, ..PivotParams::default() }
    }

    fn test_params_with_lag(lag: f32) -> PivotParams {
        PivotParams { heading_lag: lag, ..PivotParams::default() }
    }

    /// Robot on the orbit at `orbit_angle` relative to `center`, at rest.
    /// With center at (0,0): `x = -r·cos(orbit)`, `y = -r·sin(orbit)`.
    fn init_state_on_orbit(orbit_angle: f32) -> Vector6f {
        let r = PivotParams::default().orbit_radius;
        Vector6f::new(
            -r * cosf(orbit_angle),
            -r * sinf(orbit_angle),
            orbit_angle, // heading = orbit_start (facing center)
            0.0, 0.0, 0.0,
        )
    }

    #[test]
    fn heading_reaches_target() {
        let init = init_state_on_orbit(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let target = PI / 2.0;
        let traj = PivotTrajectory::new(init, ball, target, test_params()).unwrap();
        let end = traj.end_time();
        assert!(end > 0.0);
        let final_state = traj.state_at(init, 0.0, end).unwrap();
        assert!(fabsf(wrap_angle(final_state[2] - target)) < 1e-3);
    }

    #[test]
    fn heading_reaches_target_negative() {
        let init = init_state_on_orbit(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let target = -PI / 2.0;
        let traj = PivotTrajectory::new(init, ball, target, test_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(init, 0.0, end).unwrap();
        assert!(fabsf(wrap_angle(final_state[2] - target)) < 1e-3);
    }

    #[test]
    fn xy_position_on_orbit_at_all_times() {
        let r = PivotParams::default().orbit_radius;
        let ball = Vector2f::new(1.0, 0.5);
        let orbit_angle = 0.0_f32;
        let init = Vector6f::new(
            ball.x - r * cosf(orbit_angle),
            ball.y - r * sinf(orbit_angle),
            orbit_angle, 0.0, 0.0, 0.0,
        );
        let traj = PivotTrajectory::new(init, ball, orbit_angle + PI / 2.0, test_params()).unwrap();
        let end = traj.end_time();
        for i in 0..=20 {
            let t = end * (i as f32) / 20.0;
            let state = traj.state_at(init, 0.0, t).unwrap();
            let dx = state[0] - traj.center_x;
            let dy = state[1] - traj.center_y;
            let dist = sqrtf(dx * dx + dy * dy);
            assert!(fabsf(dist - r) < 1e-4, "t={}: dist={}, r={}", t, dist, r);
        }
    }

    #[test]
    fn no_lag_heading_equals_orbit_angle() {
        // With lag=0, heading tracks the orbit angle at all times.
        let init = init_state_on_orbit(0.5);
        let ball = Vector2f::new(0.0, 0.0);
        let r = PivotParams::default().orbit_radius;
        let traj = PivotTrajectory::new(init, ball, PI, test_params()).unwrap();
        let end = traj.end_time();
        for i in 0..=10 {
            let t = end * (i as f32) / 10.0;
            let state = traj.state_at(init, 0.0, t).unwrap();
            // x = center - r*cos(orbit), and heading == orbit with lag=0
            let expected_x = traj.center_x - r * cosf(state[2]);
            let expected_y = traj.center_y - r * sinf(state[2]);
            assert!(fabsf(state[0] - expected_x) < 1e-4);
            assert!(fabsf(state[1] - expected_y) < 1e-4);
        }
    }

    #[test]
    fn final_velocity_zero() {
        let init = init_state_on_orbit(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let traj = PivotTrajectory::new(init, ball, PI / 3.0, test_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(init, 0.0, end).unwrap();
        assert!(fabsf(final_state[3]) < 1e-3, "ẋ: {}", final_state[3]);
        assert!(fabsf(final_state[4]) < 1e-3, "ẏ: {}", final_state[4]);
        assert!(fabsf(final_state[5]) < 1e-3, "θ̇: {}", final_state[5]);
    }

    #[test]
    fn zero_displacement_has_zero_end_time() {
        let init = init_state_on_orbit(1.0);
        let ball = Vector2f::new(0.0, 0.0);
        let traj = PivotTrajectory::new(init, ball, 1.0, test_params()).unwrap();
        assert!(traj.end_time() < 1e-6);
    }

    #[test]
    fn uses_shortest_angular_path() {
        let init = init_state_on_orbit(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let traj = PivotTrajectory::new(init, ball, PI - 0.1, test_params()).unwrap();
        assert!(traj.orbit.sdd1 >= 0.0);
        assert!(traj.heading.sdd1 >= 0.0);
    }

    #[test]
    fn invalid_params_rejected() {
        let init = init_state_on_orbit(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let mut params = test_params();
        params.max_vel_angular = 0.0;
        assert!(PivotTrajectory::new(init, ball, 1.0, params).is_err());
        params.max_vel_angular = 1.0;
        params.max_accel_angular = -1.0;
        assert!(PivotTrajectory::new(init, ball, 1.0, params).is_err());
    }

    #[test]
    fn heading_starts_at_orbit_start() {
        // With any lag, the heading is held at orbit_start before t_heading_start.
        let lag = PI / 4.0;
        let init = init_state_on_orbit(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let target = PI / 2.0;
        let traj = PivotTrajectory::new(init, ball, target, test_params_with_lag(lag)).unwrap();

        // At t=0 heading = orbit_start
        let state_0 = traj.state_at(init, 0.0, 0.0).unwrap();
        assert!(fabsf(state_0[2] - traj.orbit_start) < 1e-6,
            "θ at t=0 should be orbit_start, got {}", state_0[2]);

        // Just before t_heading_start, heading is still orbit_start
        if traj.t_heading_start > 1e-6 {
            let t_before = traj.t_heading_start * 0.5;
            let state_before = traj.state_at(init, 0.0, t_before).unwrap();
            assert!(fabsf(state_before[2] - traj.orbit_start) < 1e-6,
                "θ before lag period should still be orbit_start, got {}", state_before[2]);
        }
    }

    #[test]
    fn heading_reaches_target_with_lag() {
        let lag = PI / 4.0;
        let init = init_state_on_orbit(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let target = PI / 2.0;
        let traj = PivotTrajectory::new(init, ball, target, test_params_with_lag(lag)).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(init, 0.0, end).unwrap();
        assert!(fabsf(wrap_angle(final_state[2] - target)) < 1e-3,
            "heading at end: {}", final_state[2]);
    }

    #[test]
    fn heading_lag_heading_finishes_after_orbit() {
        // With lag > 0, heading starts late and should finish after orbit.
        let lag = PI / 4.0;
        let init = init_state_on_orbit(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let target = PI / 2.0;
        let traj = PivotTrajectory::new(init, ball, target, test_params_with_lag(lag)).unwrap();
        // Heading is time-shifted so it finishes later than orbit.
        assert!(traj.heading.t4 >= traj.orbit.t4 - 1e-6,
            "heading.t4={} should be >= orbit.t4={}", traj.heading.t4, traj.orbit.t4);
    }

    #[test]
    fn xy_on_orbit_with_lag() {
        let r = PivotParams::default().orbit_radius;
        let lag = PI / 6.0;
        let init = init_state_on_orbit(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let traj = PivotTrajectory::new(init, ball, PI / 2.0, test_params_with_lag(lag)).unwrap();
        let end = traj.end_time();
        for i in 0..=30 {
            let t = end * (i as f32) / 30.0;
            let state = traj.state_at(init, 0.0, t).unwrap();
            let dx = state[0] - traj.center_x;
            let dy = state[1] - traj.center_y;
            let dist = sqrtf(dx * dx + dy * dy);
            assert!(fabsf(dist - r) < 1e-4, "t={}: dist={}, r={}", t, dist, r);
        }
    }

    #[test]
    fn final_velocity_zero_with_lag() {
        let lag = PI / 4.0;
        let init = init_state_on_orbit(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let traj = PivotTrajectory::new(init, ball, PI / 2.0, test_params_with_lag(lag)).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(init, 0.0, end).unwrap();
        assert!(fabsf(final_state[3]) < 1e-3, "ẋ: {}", final_state[3]);
        assert!(fabsf(final_state[4]) < 1e-3, "ẏ: {}", final_state[4]);
        assert!(fabsf(final_state[5]) < 1e-3, "θ̇: {}", final_state[5]);
    }

    #[test]
    fn no_lag_orbit_and_heading_same_duration() {
        let init = init_state_on_orbit(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let traj = PivotTrajectory::new(init, ball, PI / 2.0, test_params()).unwrap();
        // With lag=0 and zero initial velocity, both should have the same duration.
        assert!(fabsf(traj.orbit.t4 - traj.heading.t4) < 1e-6);
    }
}
