use core::f32::consts::PI;
use libm::{atan2f, cosf, sinf};
use crate::defaults::{DEFAULT_PIVOT_ORBIT_MAX_VEL_ANGULAR, DEFAULT_PIVOT_ORBIT_MAX_ACCEL_ANGULAR, DEFAULT_PIVOT_ORBIT_RADIUS, DEFAULT_PIVOT_ORBIT_INSET_ANGLE};
use crate::{wrap_angle, ControlsError, Vector3f, Vector6f};
use crate::bangbang_trajectory::{BangBangTraj1D, solve_1d_pose, eval_1d_state_at, eval_1d_accel_at};
use crate::trajectory::Trajectory;


/// Parameters for a pivot trajectory.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PivotParams {
    /// Maximum angular velocity of the orbit (rad/s).
    pub max_vel_angular: f32,
    /// Maximum angular acceleration of the orbit (rad/s²).
    pub max_accel_angular: f32,
    /// Radius of the pivot orbit (m).
    pub orbit_radius: f32,
    /// Angle (radians) between the radius line pointing from the robot to the
    /// orbit center and the robot's heading, held constant for the entire pivot.
    /// `inset_angle = 0` points the robot straight at the orbit center; a positive
    /// value rotates the heading toward the direction of travel, and a negative
    /// value rotates it away. `inset_angle = +π/2` points the robot tangent to the
    /// orbit, facing the direction of travel (moving forward); `inset_angle = −π/2`
    /// points it tangent to the orbit facing backward (moving in reverse).
    pub inset_angle: f32,
}

impl Default for PivotParams {
    fn default() -> Self {
        Self {
            max_vel_angular: DEFAULT_PIVOT_ORBIT_MAX_VEL_ANGULAR,
            max_accel_angular: DEFAULT_PIVOT_ORBIT_MAX_ACCEL_ANGULAR,
            orbit_radius: DEFAULT_PIVOT_ORBIT_RADIUS,
            inset_angle: DEFAULT_PIVOT_ORBIT_INSET_ANGLE,
        }
    }
}

/// A single 1D bang-bang profile on the orbit angle φ (the global angle from
/// the orbit center to the robot's position).
///
/// The robot rides the orbit circle while holding a fixed `inset_angle` between
/// the radius line (pointing from the robot to the orbit center) and its heading,
/// measured positive toward the direction of travel. `inset_angle = 0` points the
/// robot at the center; `inset_angle = ±π/2` points it tangent to the orbit. The
/// heading is a constant offset from the orbit angle:
///
/// ```text
/// robot_pos = center + r·(cos φ, sin φ)
/// heading   = φ + π − d·inset_angle
/// ```
///
/// where `d = ±1` is the orbit direction (+1 = CCW, −1 = CW), chosen as the
/// shortest angular path. Both the orbit start and target angles follow from the
/// robot's current/target heading and the inset angle:
///
/// ```text
/// φ_start  = θ_start        − (π − d·inset_angle)
/// φ_target = target_heading − (π − d·inset_angle)
/// ```
///
/// Because heading is φ plus a constant, a single bang-bang on φ drives both the
/// orbital position and the robot heading.
#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct PivotTrajectory {
    /// Orbit angle trajectory (`orbit_start` → `orbit_target`).
    pub orbit: BangBangTraj1D,
    /// Orbit center x (m). Derived from the seed state at construction; fixed.
    pub center_x: f32,
    /// Orbit center y (m). Derived from the seed state at construction; fixed.
    pub center_y: f32,
    /// Orbit radius (m).
    pub orbit_radius: f32,
    /// Current orbit angle φ (rad): direction from center to robot. Updated each `tick`.
    pub orbit_start: f32,
    /// Current orbit angular velocity φ̇ (rad/s). Updated each `tick`.
    pub orbit_start_dot: f32,
    /// Final orbit angle (rad).
    pub orbit_target: f32,
    /// Constant offset between heading and orbit angle: `θ = φ + heading_offset`.
    /// Equals `π − d·inset_angle`, where `d` is the orbit direction. Fixed at
    /// construction.
    pub heading_offset: f32,
    /// Current robot state `[x, y, θ, ẋ, ẏ, θ̇]`. Updated each `tick`.
    pub state: Vector6f,
}

impl PivotTrajectory {
    /// Construct a pivot trajectory seeded from the robot's current state.
    ///
    /// The orbit circle is fully determined by the seed state, `orbit_radius`,
    /// and `inset_angle`: the robot's heading and the inset angle fix the orbit
    /// tangent at the start, which places the orbit center one radius away along
    /// the inward normal.
    ///
    /// # Arguments
    /// * `init_state`     – `[x, y, θ, ẋ, ẏ, θ̇]` seed robot state.
    /// * `target_heading` – desired final heading (rad).
    /// * `params`         – velocity/acceleration limits, orbit radius, inset angle.
    pub fn new(
        init_state: Vector6f,
        target_heading: f32,
        params: PivotParams,
    ) -> Result<Self, ControlsError> {
        if params.max_vel_angular <= 0.0 || params.max_accel_angular <= 0.0 {
            return Err(ControlsError::InvalidInput);
        }
        if params.orbit_radius <= 0.0 {
            return Err(ControlsError::InvalidInput);
        }

        let r = params.orbit_radius;

        // Orbit angular displacement equals the heading displacement (heading is
        // a constant offset from the orbit angle). Pick the shortest path; its
        // sign is the orbit direction `d` (+1 = CCW, −1 = CW).
        let displacement = wrap_angle(target_heading - init_state[2]);
        let d = if displacement >= 0.0 { 1.0_f32 } else { -1.0_f32 };

        // Heading is held a fixed `inset_angle` off the radius line that points
        // from the robot toward the orbit center (`φ + π`), measured positive
        // toward the direction of travel (`φ + d·π/2`). A positive `inset_angle`
        // always rotates the heading toward the direction of travel regardless of
        // orbit direction:
        //   θ = φ + π − d·inset_angle
        //     ⇒ heading_offset = π − d·inset_angle.
        let heading_offset = PI - d * params.inset_angle;

        // Orbit start angle (center → robot direction) from the seed heading.
        let orbit_start = init_state[2] - heading_offset;
        let cs = cosf(orbit_start);
        let sn = sinf(orbit_start);

        // robot = center + r·(cos φ, sin φ)  ⇒  center = robot − r·(cos φ, sin φ).
        let center_x = init_state[0] - r * cs;
        let center_y = init_state[1] - r * sn;

        // Orbit angular velocity from the seed XY velocity:
        //   ẋ = −r·sin φ·φ̇,  ẏ = r·cos φ·φ̇  ⇒  φ̇ = (ẏ·cos φ − ẋ·sin φ) / r.
        let orbit_start_dot = (init_state[4] * cs - init_state[3] * sn) / r;

        // Target orbit angle is the start plus the shared displacement.
        let orbit_target = orbit_start + displacement;

        let orbit = solve_1d_pose(
            orbit_start,
            orbit_start_dot,
            orbit_target,
            params.max_vel_angular,
            params.max_accel_angular,
        )?;

        Ok(PivotTrajectory {
            orbit,
            center_x,
            center_y,
            orbit_radius: r,
            orbit_start,
            orbit_start_dot,
            orbit_target,
            heading_offset,
            state: init_state,
        })
    }

    /// Shift the trajectory segment times forward by `dt` seconds.
    pub fn time_shift(&mut self, dt: f32) {
        self.orbit.time_shift(dt);
    }

    /// Time at which the orbit trajectory has finished.
    pub fn end_time(&self) -> f32 {
        self.orbit.t4
    }

    /// Evaluate the full robot state `[x, y, θ, ẋ, ẏ, θ̇]` at time `t`, using the
    /// trajectory's internal orbit state at internal time 0.
    pub fn state_at(&self, t: f32) -> Result<Vector6f, ControlsError> {
        let (phi, phi_dot) = eval_1d_state_at(
            self.orbit,
            self.orbit_start,
            self.orbit_start_dot,
            0.0,
            t,
        )?;

        let r = self.orbit_radius;
        let c = cosf(phi);
        let s = sinf(phi);
        let x = self.center_x + r * c;
        let y = self.center_y + r * s;
        let xd = -r * s * phi_dot;
        let yd = r * c * phi_dot;
        let theta = wrap_angle(phi + self.heading_offset);

        Ok(Vector6f::new(x, y, theta, xd, yd, phi_dot))
    }

    /// Evaluate the robot acceleration `[ẍ, ÿ, θ̈]` at time `t`, using the
    /// trajectory's internal orbit state at internal time 0.
    pub fn accel_at(&self, t: f32) -> Result<Vector3f, ControlsError> {
        let phi_ddot = eval_1d_accel_at(self.orbit, t)?;
        let (phi, phi_dot) = eval_1d_state_at(
            self.orbit,
            self.orbit_start,
            self.orbit_start_dot,
            0.0,
            t,
        )?;

        let r = self.orbit_radius;
        let c = cosf(phi);
        let s = sinf(phi);
        let w = phi_dot;
        let xdd = -r * (c * w * w + s * phi_ddot);
        let ydd = r * (-s * w * w + c * phi_ddot);

        Ok(Vector3f::new(xdd, ydd, phi_ddot))
    }
}

impl Trajectory for PivotTrajectory {
    fn tick(&mut self, dt: f32) {
        // Integrate the orbit state forward by dt using current initial conditions.
        if let Ok(next) = self.state_at(dt) {
            self.state = next;
            // Re-derive the orbit initial conditions from the new position/velocity
            // so the next tick's state_at evaluates from the right starting point.
            let dx = next[0] - self.center_x;
            let dy = next[1] - self.center_y;
            self.orbit_start = atan2f(dy, dx);
            let c = cosf(self.orbit_start);
            let s = sinf(self.orbit_start);
            self.orbit_start_dot =
                (next[4] * c - next[3] * s) / self.orbit_radius;
        }
        // Shift segment times so t=0 always means "now".
        self.time_shift(-dt);
    }

    fn sample(&self) -> (Vector6f, Vector3f) {
        // accel should always be defined for t=0, but default to zero if something goes wrong
        let accel = self.accel_at(0.0).unwrap_or_default();
        (self.state, accel)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vector6f;
    use core::f32::consts::PI;
    use libm::{fabsf, sqrtf};

    fn test_params() -> PivotParams {
        PivotParams { inset_angle: 0.0, ..PivotParams::default() }
    }

    fn test_params_with_inset(inset: f32) -> PivotParams {
        PivotParams { inset_angle: inset, ..PivotParams::default() }
    }

    /// Robot at rest with the given heading at `(x, y)`.
    fn init_state_at(x: f32, y: f32, heading: f32) -> Vector6f {
        Vector6f::new(x, y, heading, 0.0, 0.0, 0.0)
    }

    #[test]
    fn heading_reaches_target() {
        let init = init_state_at(0.0, 0.0, 0.0);
        let target = PI / 2.0;
        let traj = PivotTrajectory::new(init, target, test_params()).unwrap();
        let end = traj.end_time();
        assert!(end > 0.0);
        let final_state = traj.state_at(end).unwrap();
        assert!(fabsf(wrap_angle(final_state[2] - target)) < 1e-3,
            "heading at end: {}", final_state[2]);
    }

    #[test]
    fn heading_reaches_target_negative() {
        let init = init_state_at(0.0, 0.0, 0.0);
        let target = -PI / 2.0;
        let traj = PivotTrajectory::new(init, target, test_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        assert!(fabsf(wrap_angle(final_state[2] - target)) < 1e-3);
    }

    #[test]
    fn heading_reaches_target_with_inset() {
        let inset = PI / 6.0;
        let init = init_state_at(0.3, -0.2, 0.4);
        let target = PI / 2.0;
        let traj = PivotTrajectory::new(init, target, test_params_with_inset(inset)).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        assert!(fabsf(wrap_angle(final_state[2] - target)) < 1e-3,
            "heading at end: {}", final_state[2]);
    }

    #[test]
    fn xy_position_on_orbit_at_all_times() {
        let r = PivotParams::default().orbit_radius;
        let init = init_state_at(1.0, 0.5, 0.3);
        let traj = PivotTrajectory::new(init, 0.3 + PI / 2.0, test_params()).unwrap();
        let end = traj.end_time();
        for i in 0..=20 {
            let t = end * (i as f32) / 20.0;
            let state = traj.state_at(t).unwrap();
            let dx = state[0] - traj.center_x;
            let dy = state[1] - traj.center_y;
            let dist = sqrtf(dx * dx + dy * dy);
            assert!(fabsf(dist - r) < 1e-4, "t={}: dist={}, r={}", t, dist, r);
        }
    }

    #[test]
    fn inset_angle_maintained_throughout() {
        check_inset_maintained(-0.6, 1.2, PI / 5.0); // CCW (positive displacement)
    }

    #[test]
    fn inset_angle_maintained_throughout_cw() {
        check_inset_maintained(0.6, -1.2, PI / 5.0); // CW (negative displacement)
    }

    /// A positive `inset_angle` must hold the heading a fixed angle off the radius
    /// line (pointing from the robot to the orbit center), measured toward the
    /// direction of travel, for the whole trajectory regardless of orbit direction.
    fn check_inset_maintained(start_heading: f32, target: f32, inset: f32) {
        let init = init_state_at(0.2, 0.1, start_heading);
        let traj = PivotTrajectory::new(init, target, test_params_with_inset(inset)).unwrap();
        // Orbit direction: +1 for CCW, -1 for CW.
        let d = (traj.orbit_target - traj.orbit_start).signum();
        let end = traj.end_time();
        for i in 0..=20 {
            let t = end * (i as f32) / 20.0;
            let state = traj.state_at(t).unwrap();
            // Orbit angle φ from center to robot.
            let phi = atan2f(state[1] - traj.center_y, state[0] - traj.center_x);
            // Radius line pointing from the robot toward the orbit center.
            let toward_center = phi + PI;
            // Inset is the signed angle from that radius line to the heading,
            // positive toward the direction of travel.
            let measured_inset = -d * wrap_angle(state[2] - toward_center);
            assert!(fabsf(wrap_angle(measured_inset - inset)) < 1e-3,
                "t={}: inset {} != expected {}", t, measured_inset, inset);
        }
    }

    #[test]
    fn final_velocity_zero() {
        let init = init_state_at(0.0, 0.0, 0.0);
        let traj = PivotTrajectory::new(init, PI / 3.0, test_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        assert!(fabsf(final_state[3]) < 1e-3, "ẋ: {}", final_state[3]);
        assert!(fabsf(final_state[4]) < 1e-3, "ẏ: {}", final_state[4]);
        assert!(fabsf(final_state[5]) < 1e-3, "θ̇: {}", final_state[5]);
    }

    #[test]
    fn seeds_orbital_velocity_from_tangential_projection() {
        // Robot at heading 0 with inset 0 points at the center, which sits at
        // (+r, 0); the CCW orbit tangent at the start is (0, −1). A seed velocity
        // of (0, −v) is therefore purely tangential and should seed the orbit at
        // φ̇ = v / r, and `state_at(0)` should report that same XY velocity.
        let r = PivotParams::default().orbit_radius;
        let v = 0.5;
        let init = Vector6f::new(0.0, 0.0, 0.0, 0.0, -v, 0.0);
        let traj = PivotTrajectory::new(init, PI / 2.0, test_params()).unwrap();
        assert!(fabsf(fabsf(traj.orbit_start_dot) - v / r) < 1e-4,
            "orbit_start_dot {} != v/r {}", traj.orbit_start_dot, v / r);
        let s0 = traj.state_at(0.0).unwrap();
        assert!(fabsf(s0[3]) < 1e-3, "ẋ0: {}", s0[3]);
        assert!(fabsf(s0[4] - (-v)) < 1e-3, "ẏ0: {}", s0[4]);
    }

    #[test]
    fn radial_seed_velocity_is_dropped() {
        // With the same geometry, a seed velocity of (v, 0) points straight at the
        // center (purely radial), which cannot be represented on the fixed orbit
        // circle and must produce zero orbital angular velocity.
        let v = 0.5;
        let init = Vector6f::new(0.0, 0.0, 0.0, v, 0.0, 0.0);
        let traj = PivotTrajectory::new(init, PI / 2.0, test_params()).unwrap();
        assert!(fabsf(traj.orbit_start_dot) < 1e-4,
            "radial velocity not dropped: {}", traj.orbit_start_dot);
    }

    #[test]
    fn zero_displacement_has_zero_end_time() {
        let init = init_state_at(0.0, 0.0, 1.0);
        let traj = PivotTrajectory::new(init, 1.0, test_params()).unwrap();
        assert!(traj.end_time() < 1e-6);
    }

    #[test]
    fn uses_shortest_angular_path() {
        // Target slightly under +π ⇒ positive (CCW) displacement.
        let init = init_state_at(0.0, 0.0, 0.0);
        let traj = PivotTrajectory::new(init, PI - 0.1, test_params()).unwrap();
        assert!(traj.orbit.sdd1 >= 0.0);
        // Target just short of −π ⇒ negative (CW) displacement.
        let traj_neg = PivotTrajectory::new(init, -(PI - 0.1), test_params()).unwrap();
        assert!(traj_neg.orbit.sdd1 <= 0.0);
    }

    #[test]
    fn invalid_params_rejected() {
        let init = init_state_at(0.0, 0.0, 0.0);
        let mut params = test_params();
        params.max_vel_angular = 0.0;
        assert!(PivotTrajectory::new(init, 1.0, params).is_err());
        params.max_vel_angular = 1.0;
        params.max_accel_angular = -1.0;
        assert!(PivotTrajectory::new(init, 1.0, params).is_err());
        params.max_accel_angular = 1.0;
        params.orbit_radius = 0.0;
        assert!(PivotTrajectory::new(init, 1.0, params).is_err());
    }

    #[test]
    fn center_derived_from_seed() {
        // With inset = 0 and heading = 0, the robot points at the center, so
        // φ_start = θ − π = −π, and
        // center = pos − r·(cos(−π), sin(−π)) = pos + (r, 0).
        let r = PivotParams::default().orbit_radius;
        let init = init_state_at(1.0, 2.0, 0.0);
        let traj = PivotTrajectory::new(init, 0.5, test_params()).unwrap();
        assert!(fabsf(traj.center_x - (1.0 + r)) < 1e-5, "cx={}", traj.center_x);
        assert!(fabsf(traj.center_y - 2.0) < 1e-5, "cy={}", traj.center_y);
    }
}
