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
    /// Maximum orbital angular velocity (rad/s).
    pub max_vel_angular: f32,
    /// Maximum orbital angular acceleration (rad/s²).
    pub max_accel_angular: f32,
    /// Radius of the pivot orbit (m).
    pub orbit_radius: f32,
    /// How far the heading lags behind the orbital position (radians, ≥ 0).
    ///
    /// With `heading_lag = λ > 0`:
    /// - The robot's X/Y position leads the heading by λ radians throughout the
    ///   maneuver, so the robot pushes the ball forward rather than facing it
    ///   straight on.
    /// - The heading starts at `orbit_start` (not `orbit_start − λ`) and catches
    ///   up to `orbit_target` after the position has already arrived.
    pub heading_lag: f32,
}

impl Default for PivotParams {
    fn default() -> Self {
        use core::f32::consts::PI;
        Self {
            max_vel_angular: 1.0 * PI,
            max_accel_angular: 2.0 * PI,
            // orbit_radius: DEFAULT_BALL_RADIUS + DEFAULT_ROBOT_RADIUS,
            orbit_radius: 0.2,
            heading_lag: 1.0,
        }
    }
}

/// Time-optimal trajectory that pivots the robot around a stationary ball,
/// with an optional heading lag so the robot always pushes the ball forward.
///
/// A single 1D bang-bang trajectory tracks the **orbital position angle** φ(t),
/// running from `orbit_start` to `(orbit_target + heading_lag · direction)`.
///
/// **X/Y position** is derived from φ(t) clamped to `[orbit_start, orbit_target]`:
/// ```text
///   pos(t) = clamp(φ(t), orbit_start, orbit_target)
///   x(t)   = center_x − r · cos(pos(t))
///   y(t)   = center_y − r · sin(pos(t))
/// ```
///
/// **Heading** lags behind φ by `heading_lag`, clipped at both ends:
/// ```text
///   θ(t)   = clamp(φ(t) − lag_signed, orbit_start, orbit_target)
/// ```
///
/// This creates two velocity discontinuities in θ(t):
/// - **Start clip**: θ holds at `orbit_start` while φ advances the first `lag`
///   radians (position leads heading); then jumps to tracking φ − lag.
/// - **End clip**: once φ passes `orbit_target`, X/Y freeze; φ continues to
///   `orbit_target + lag` so θ catches up to `orbit_target` (heading catches up).
///
/// `orbit_start` is derived from the robot's X/Y position (via `atan2f`) so that
/// replans from `trajectory_state` resume the orbit correctly even while the
/// heading is in its start-clip phase.
#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct PivotTrajectory {
    /// 1D bang-bang orbit trajectory: orbit_start → (orbit_target + lag_signed).
    pub theta: BangBangTraj1D,
    /// Orbit center x (m).
    pub center_x: f32,
    /// Orbit center y (m).
    pub center_y: f32,
    /// Orbit radius (m).
    pub orbit_radius: f32,
    /// Heading lag magnitude (radians, ≥ 0).
    pub heading_lag: f32,
    /// Initial orbit angle, derived from the robot's X/Y position at creation time.
    pub orbit_start: f32,
    /// Initial orbit angular velocity.
    pub orbit_start_dot: f32,
    /// Orbit angle at which X/Y position freezes (= the desired target heading).
    pub orbit_target: f32,
    /// Direction of orbit: +1.0 CCW, -1.0 CW.
    pub direction: f32,
}

impl PivotTrajectory {
    /// Construct a pivot trajectory from the current robot state.
    ///
    /// `orbit_start` is derived from the robot's X/Y position relative to
    /// `center_pos` so that replanning from `trajectory_state` tracks the orbit
    /// continuously even while the heading is in the start-clip phase.
    ///
    /// # Arguments
    /// * `init_state`     – `[x, y, θ, ẋ, ẏ, θ̇]` current robot state.
    /// * `center_pos`     – orbit center (ball position) in the global frame.
    /// * `target_heading` – desired final heading (rad).
    /// * `params`         – velocity/acceleration limits, orbit radius, and heading lag.
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

        // Derive the current orbit angle from (x, y) so replans from
        // trajectory_state use the orbit position, not the lagged heading.
        //   center - robot = r·[cos(orbit), sin(orbit)]
        let dx = center_pos.x - init_state[0];
        let dy = center_pos.y - init_state[1];
        let orbit_start = atan2f(dy, dx);

        // Derive orbit angular velocity from body velocity (numerically stable):
        //   ẋ =  r·sin(orbit)·ω  →  ẋ·sin(orbit)  = r·sin²·ω
        //   ẏ = -r·cos(orbit)·ω  → -ẏ·cos(orbit)  = r·cos²·ω
        //   ⇒  ω = (ẋ·sin(orbit) − ẏ·cos(orbit)) / r
        let ct = cosf(orbit_start);
        let st = sinf(orbit_start);
        let orbit_start_dot =
            (init_state[3] * st - init_state[4] * ct) / params.orbit_radius;

        // Shortest angular path from orbit_start to target_heading.
        let displacement = wrap_angle(target_heading - orbit_start);
        let direction = if displacement > 0.0 { 1.0_f32 }
                        else if displacement < 0.0 { -1.0_f32 }
                        else { 1.0_f32 };
        let lag_signed = params.heading_lag * direction;
        let orbit_target = orbit_start + displacement;

        // The 1D orbit trajectory overshoots orbit_target by lag_signed so the
        // heading can catch up after the position has frozen.
        let orbit_plan_end = orbit_target + lag_signed;

        let theta = solve_1d_pose(
            orbit_start,
            orbit_start_dot,
            orbit_plan_end,
            params.max_vel_angular,
            params.max_accel_angular,
        )?;

        Ok(PivotTrajectory {
            theta,
            center_x: center_pos.x,
            center_y: center_pos.y,
            orbit_radius: params.orbit_radius,
            heading_lag: params.heading_lag,
            orbit_start,
            orbit_start_dot,
            orbit_target,
            direction,
        })
    }

    /// Shift all trajectory segment times forward by `dt` seconds.
    pub fn time_shift(&mut self, dt: f32) {
        self.theta.time_shift(dt);
    }

    /// Time at which both position and heading have reached `orbit_target`.
    pub fn end_time(&self) -> f32 {
        self.theta.t4
    }

    /// Evaluate the full robot state `[x, y, θ, ẋ, ẏ, θ̇]` at time `t`.
    ///
    /// Uses stored `orbit_start` / `orbit_start_dot` as initial conditions;
    /// `current_state` is unused. `current_time` is the trajectory time at which
    /// evaluation starts (typically 0.0 for freshly-planned trajectories).
    pub fn state_at(
        &self,
        _current_state: Vector6f,
        current_time: f32,
        t: f32,
    ) -> Result<Vector6f, ControlsError> {
        let (orbit_angle_t, orbit_dot_t) = eval_1d_state_at(
            self.theta,
            self.orbit_start,
            self.orbit_start_dot,
            current_time,
            t,
        )?;

        let lag_signed = self.heading_lag * self.direction;

        // ── Orbit position (X/Y): clamped once orbit_target is reached ──────
        let orbit_frozen = if self.direction > 0.0 {
            orbit_angle_t >= self.orbit_target
        } else {
            orbit_angle_t <= self.orbit_target
        };
        let orbit_pos = if orbit_frozen { self.orbit_target } else { orbit_angle_t };
        let orbit_pos_dot = if orbit_frozen { 0.0 } else { orbit_dot_t };

        // ── Heading: φ − lag_signed, clipped at orbit_start (start) ─────────
        // The heading is held at orbit_start while the orbit is still in the
        // initial lag region (φ < orbit_start + lag for CCW).
        let in_start_clip = if self.direction > 0.0 {
            orbit_angle_t < self.orbit_start + lag_signed
        } else {
            orbit_angle_t > self.orbit_start + lag_signed
        };
        let raw_heading = orbit_angle_t - lag_signed;
        let heading = if in_start_clip {
            self.orbit_start
        } else if self.direction > 0.0 {
            raw_heading.min(self.orbit_target)
        } else {
            raw_heading.max(self.orbit_target)
        };
        // Velocity is zero while clipped, orbit_dot otherwise (d(φ−lag)/dt = dφ/dt).
        let heading_dot = if in_start_clip { 0.0 } else { orbit_dot_t };

        let r = self.orbit_radius;
        let ct = cosf(orbit_pos);
        let st = sinf(orbit_pos);
        let x = self.center_x - r * ct;
        let y = self.center_y - r * st;
        let xd =  r * st * orbit_pos_dot;
        let yd = -r * ct * orbit_pos_dot;

        Ok(Vector6f::new(x, y, wrap_angle(heading), xd, yd, heading_dot))
    }

    /// Evaluate the robot acceleration `[ẍ, ÿ, θ̈]` at time `t`.
    ///
    /// At the heading velocity discontinuity (where θ jumps from the start-clip
    /// phase to tracking φ − lag), returns the acceleration *immediately after*
    /// the jump (the 1D trajectory acceleration) rather than undefined/infinite.
    ///
    /// `current_state` is unused. `current_time` is typically 0.0.
    pub fn accel_at(
        &self,
        _current_state: Vector6f,
        current_time: f32,
        t: f32,
    ) -> Result<Vector3f, ControlsError> {
        let theta_1d_ddot = eval_1d_accel_at(self.theta, t)?;
        let (orbit_angle_t, orbit_dot_t) = eval_1d_state_at(
            self.theta,
            self.orbit_start,
            self.orbit_start_dot,
            current_time,
            t,
        )?;

        let lag_signed = self.heading_lag * self.direction;

        // ── Orbit position acceleration ──────────────────────────────────────
        let orbit_frozen = if self.direction > 0.0 {
            orbit_angle_t >= self.orbit_target
        } else {
            orbit_angle_t <= self.orbit_target
        };
        let orbit_pos = if orbit_frozen { self.orbit_target } else { orbit_angle_t };
        let orbit_pos_dot  = if orbit_frozen { 0.0 } else { orbit_dot_t };
        let orbit_pos_ddot = if orbit_frozen { 0.0 } else { theta_1d_ddot };

        // ── Heading acceleration ─────────────────────────────────────────────
        // Before the velocity discontinuity: heading is frozen → ddot = 0.
        // At/after the discontinuity: ddot = theta_1d_ddot (value right after).
        let in_start_clip = if self.direction > 0.0 {
            orbit_angle_t < self.orbit_start + lag_signed
        } else {
            orbit_angle_t > self.orbit_start + lag_signed
        };
        let heading_ddot = if in_start_clip { 0.0 } else { theta_1d_ddot };

        let r = self.orbit_radius;
        let ct = cosf(orbit_pos);
        let st = sinf(orbit_pos);
        // ẍ = r·(cos(orbit)·ω² + sin(orbit)·α)
        let xdd = r * (ct * orbit_pos_dot * orbit_pos_dot + st * orbit_pos_ddot);
        // ÿ = r·(sin(orbit)·ω² − cos(orbit)·α)
        let ydd = r * (st * orbit_pos_dot * orbit_pos_dot - ct * orbit_pos_ddot);

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
        PivotParams::default()
    }

    fn test_params_with_lag(lag: f32) -> PivotParams {
        PivotParams { heading_lag: lag, ..PivotParams::default() }
    }

    /// Place robot on the orbit at the given heading with zero velocity.
    /// With center at (0,0): x = -r·cos(θ), y = -r·sin(θ), so orbit_start = θ.
    fn init_state_at_heading(theta: f32) -> Vector6f {
        let r = test_params().orbit_radius;
        Vector6f::new(
            -r * cosf(theta),
            -r * sinf(theta),
            theta,
            0.0, 0.0, 0.0,
        )
    }

    #[test]
    fn heading_reaches_target() {
        let init = init_state_at_heading(0.0);
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
        let init = init_state_at_heading(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let target = -PI / 2.0;
        let traj = PivotTrajectory::new(init, ball, target, test_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(init, 0.0, end).unwrap();
        assert!(fabsf(wrap_angle(final_state[2] - target)) < 1e-3);
    }

    #[test]
    fn xy_position_on_orbit_at_all_times() {
        let init = init_state_at_heading(0.0);
        // Use a non-origin center to test the general case.
        let ball = Vector2f::new(1.0, 0.5);
        // Re-derive init state relative to this center
        let r = test_params().orbit_radius;
        let theta = 0.0_f32;
        let init2 = Vector6f::new(
            ball.x - r * cosf(theta),
            ball.y - r * sinf(theta),
            theta, 0.0, 0.0, 0.0,
        );
        let target = PI / 2.0;
        let traj = PivotTrajectory::new(init2, ball, target, test_params()).unwrap();
        let end = traj.end_time();
        let n = 20;
        for i in 0..=n {
            let t = end * (i as f32) / (n as f32);
            let state = traj.state_at(init2, 0.0, t).unwrap();
            let dx = state[0] - traj.center_x;
            let dy = state[1] - traj.center_y;
            let dist = sqrtf(dx * dx + dy * dy);
            assert!(fabsf(dist - r) < 1e-4, "t={}: dist={}, r={}", t, dist, r);
        }
    }

    #[test]
    fn xy_position_consistent_with_orbit_angle() {
        // Without lag, x/y track the orbit angle AND the heading (they're equal).
        let init = init_state_at_heading(0.5);
        let ball = Vector2f::new(0.0, 0.0);
        let target = PI;
        let traj = PivotTrajectory::new(init, ball, target, test_params()).unwrap();
        let end = traj.end_time();
        let n = 10;
        for i in 0..=n {
            let t = end * (i as f32) / (n as f32);
            let state = traj.state_at(init, 0.0, t).unwrap();
            // With lag=0, heading == orbit_pos, so x = center - r*cos(heading).
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
        let ball = Vector2f::new(0.0, 0.0);
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
        let ball = Vector2f::new(0.0, 0.0);
        let traj = PivotTrajectory::new(init, ball, 1.0, test_params()).unwrap();
        assert!(traj.end_time() < 1e-6);
    }

    #[test]
    fn uses_shortest_angular_path() {
        let init = init_state_at_heading(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let target = PI - 0.1;
        let traj = PivotTrajectory::new(init, ball, target, test_params()).unwrap();
        assert!(traj.theta.sdd1 >= 0.0);
    }

    #[test]
    fn invalid_params_rejected() {
        let init = Vector6f::zeros();
        let ball = Vector2f::new(0.0, 0.0);
        let mut params = test_params();
        params.max_vel_angular = 0.0;
        assert!(PivotTrajectory::new(init, ball, 1.0, params).is_err());
        params.max_vel_angular = 1.0;
        params.max_accel_angular = -1.0;
        assert!(PivotTrajectory::new(init, ball, 1.0, params).is_err());
    }

    #[test]
    fn heading_lag_position_leads_heading() {
        let lag = PI / 4.0;
        let init = init_state_at_heading(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let target = PI / 2.0;
        let traj = PivotTrajectory::new(init, ball, target, test_params_with_lag(lag)).unwrap();
        let end = traj.end_time();

        // At start: heading = orbit_start (clipped), x/y may have advanced.
        let start_state = traj.state_at(init, 0.0, 0.0).unwrap();
        assert!(fabsf(start_state[2] - 0.0) < 1e-4, "heading at t=0 should be orbit_start");

        // At end: heading should reach orbit_target.
        let final_state = traj.state_at(init, 0.0, end).unwrap();
        assert!(fabsf(wrap_angle(final_state[2] - target)) < 1e-3,
            "heading should reach target at end, got {}", final_state[2]);

        // At end: X/Y should be at orbit_target position.
        let r = traj.orbit_radius;
        let expected_x = traj.center_x - r * cosf(traj.orbit_target);
        let expected_y = traj.center_y - r * sinf(traj.orbit_target);
        assert!(fabsf(final_state[0] - expected_x) < 1e-3);
        assert!(fabsf(final_state[1] - expected_y) < 1e-3);

        // The orbit end time (when pos freezes) is earlier than the traj end time.
        // Verify: at some midpoint, orbit position can be at target while heading is still behind.
        // End time should be longer than the zero-lag equivalent.
        let traj_no_lag = PivotTrajectory::new(init, ball, target, test_params()).unwrap();
        assert!(end > traj_no_lag.end_time() - 1e-4,
            "lagged trajectory should not finish faster than unlagged");
    }

    #[test]
    fn heading_lag_xy_on_orbit_at_all_times() {
        let lag = PI / 6.0;
        let init = init_state_at_heading(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let target = PI / 2.0;
        let traj = PivotTrajectory::new(init, ball, target, test_params_with_lag(lag)).unwrap();
        let r = traj.orbit_radius;
        let end = traj.end_time();
        let n = 30;
        for i in 0..=n {
            let t = end * (i as f32) / (n as f32);
            let state = traj.state_at(init, 0.0, t).unwrap();
            let dx = state[0] - traj.center_x;
            let dy = state[1] - traj.center_y;
            let dist = sqrtf(dx * dx + dy * dy);
            assert!(fabsf(dist - r) < 1e-4, "t={}: xy not on orbit (dist={}, r={})", t, dist, r);
        }
    }

    #[test]
    fn heading_lag_final_velocity_zero() {
        let lag = PI / 4.0;
        let init = init_state_at_heading(0.0);
        let ball = Vector2f::new(0.0, 0.0);
        let traj = PivotTrajectory::new(init, ball, PI / 2.0, test_params_with_lag(lag)).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(init, 0.0, end).unwrap();
        assert!(fabsf(final_state[3]) < 1e-3, "ẋ at end: {}", final_state[3]);
        assert!(fabsf(final_state[4]) < 1e-3, "ẏ at end: {}", final_state[4]);
        assert!(fabsf(final_state[5]) < 1e-3, "θ̇ at end: {}", final_state[5]);
    }
}

