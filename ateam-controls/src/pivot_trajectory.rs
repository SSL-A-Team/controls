use core::f32::consts::PI;
use libm::{atan2f, cosf, fabsf, sinf, sqrtf};
use crate::defaults::{DEFAULT_PIVOT_ORBIT_MAX_VEL_ANGULAR, DEFAULT_PIVOT_ORBIT_MAX_ACCEL_ANGULAR, DEFAULT_PIVOT_ORBIT_RADIUS, DEFAULT_PIVOT_ORBIT_INSET_ANGLE, DEFAULT_PIVOT_INSET_ANGLE_PER_ANGULAR_VEL};
use crate::{wrap_angle, ControlsError, Vector3f, Vector6f};
use crate::bangbang_trajectory::{BangBangTraj1D, solve_1d_pose, eval_1d_state_at, eval_1d_accel_at};
use crate::trajectory::Trajectory;

/// Two candidate circles tie on travel distance when both orbits cover (close to)
/// half a revolution; within this tolerance the CCW orbit is preferred.
const PIVOT_TIE_EPS: f32 = 1e-4;

/// Whether the robot drives forward or backward around the pivot orbit.
///
/// This selects how the robot's velocity vector relates to its heading. With a
/// `Forward` pivot the velocity makes an acute angle with the heading (the robot
/// looks like it is moving forward); with a `Backward` pivot the velocity makes
/// an obtuse angle with the heading (the robot looks like it is reversing).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PivotDirection {
    /// Velocity is acute with the heading (robot drives forward).
    Forward,
    /// Velocity is obtuse with the heading (robot drives backward).
    Backward,
}

impl PivotDirection {
    /// Signed convention: `+1` for forward, `−1` for backward. The orbit travel
    /// direction is `d = s · g`, where `s = ±1` is the circle side and `g` is this
    /// value.
    fn sign(self) -> f32 {
        match self {
            PivotDirection::Forward => 1.0,
            PivotDirection::Backward => -1.0,
        }
    }
}

impl Default for PivotDirection {
    fn default() -> Self {
        PivotDirection::Forward
    }
}

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
    /// Angle (radians) between the orbit tangent vector and the robot's heading,
    /// measured toward the orbit center and held constant for the entire pivot.
    /// It is absolute-valued (always ≥ 0) on construction. `inset_angle = 0` points
    /// the robot tangent to the orbit (aligned with the direction of travel for a
    /// forward pivot); `inset_angle = π/2` points the robot straight at the orbit
    /// center. Combined with [`PivotDirection`], this also fixes whether the robot
    /// appears to drive forward or backward.
    ///
    /// Ignored when [`compute_inset_angle`](Self::compute_inset_angle) is set.
    pub inset_angle: f32,
    /// When `true`, `inset_angle` is ignored and the inset is instead derived from
    /// the orbit's peak angular velocity (`max_vel_angular`) via the linear model
    /// `inset = DEFAULT_PIVOT_INSET_ANGLE_PER_ANGULAR_VEL · max_vel_angular`,
    /// leaning the robot into the centrifugal force the ball produces during the
    /// pivot. The result is clamped to `[0, π/2]`.
    pub compute_inset_angle: bool,
    /// Whether the robot drives forward or backward around the orbit.
    pub direction: PivotDirection,
}

impl Default for PivotParams {
    fn default() -> Self {
        Self {
            max_vel_angular: DEFAULT_PIVOT_ORBIT_MAX_VEL_ANGULAR,
            max_accel_angular: DEFAULT_PIVOT_ORBIT_MAX_ACCEL_ANGULAR,
            orbit_radius: DEFAULT_PIVOT_ORBIT_RADIUS,
            inset_angle: DEFAULT_PIVOT_ORBIT_INSET_ANGLE,
            compute_inset_angle: false,
            direction: PivotDirection::Forward,
        }
    }
}

impl PivotParams {
    /// Resolve the absolute inset angle (rad) to use, honoring `compute_inset_angle`.
    ///
    /// When `compute_inset_angle` is set, the inset is a linear function of the
    /// commanded peak angular velocity (`max_vel_angular`), clamped to `[0, π/2]`;
    /// otherwise it is `|inset_angle|`.
    fn resolved_inset(&self) -> f32 {
        if self.compute_inset_angle {
            let psi = DEFAULT_PIVOT_INSET_ANGLE_PER_ANGULAR_VEL * self.max_vel_angular;
            psi.clamp(0.0, PI / 2.0)
        } else {
            fabsf(self.inset_angle)
        }
    }
}

/// Per-circle geometry derived from the seed robot state for a given side `s`.
///
/// Two circles pass through the robot at the start with the required inset: one
/// with its center to the left of the heading (`s = +1`) and one to the right
/// (`s = −1`). Each is paired with the orbit travel direction `d = s · g` implied
/// by the requested [`PivotDirection`].
#[derive(Clone, Copy, Debug)]
struct CircleSeed {
    center_x: f32,
    center_y: f32,
    /// Constant offset `θ = φ + heading_offset`, equal to `s·(π/2 + ψ)`.
    heading_offset: f32,
    /// Orbit angle φ (center → robot) at the seed state.
    orbit_start: f32,
    /// Orbit angular velocity φ̇ seeded from the projected seed velocity.
    orbit_start_dot: f32,
    /// Travel direction `d = s · g` (+1 CCW, −1 CW).
    d: f32,
}

/// Build the geometry of the orbit circle on side `s` (+1 left, −1 right) for the
/// seed state, orbit radius `r`, absolute inset `psi`, and forward/backward sign
/// `g`.
fn circle_seed(init_state: Vector6f, r: f32, psi: f32, g: f32, s: f32) -> CircleSeed {
    // The robot→center line is (π/2 − ψ) off the heading, on side `s`, so the
    // heading is a constant `s·(π/2 + ψ)` ahead of the orbit angle φ.
    let heading_offset = s * (PI / 2.0 + psi);
    let orbit_start = init_state[2] - heading_offset;
    let cs = cosf(orbit_start);
    let sn = sinf(orbit_start);
    // robot = center + r·(cos φ, sin φ)  ⇒  center = robot − r·(cos φ, sin φ).
    let center_x = init_state[0] - r * cs;
    let center_y = init_state[1] - r * sn;
    // ẋ = −r·sin φ·φ̇,  ẏ = r·cos φ·φ̇  ⇒  φ̇ = (ẏ·cos φ − ẋ·sin φ) / r.
    let orbit_start_dot = (init_state[4] * cs - init_state[3] * sn) / r;
    CircleSeed {
        center_x,
        center_y,
        heading_offset,
        orbit_start,
        orbit_start_dot,
        d: s * g,
    }
}

/// Smallest orbit displacement (radians) in travel direction `d` that brings the
/// orbit angle from `0` to a value whose wrapped offset equals `raw` (a value in
/// [−π, π]). The result has the same sign as `d`.
fn directed_displacement(raw: f32, d: f32) -> f32 {
    if d >= 0.0 {
        if raw >= 0.0 { raw } else { raw + 2.0 * PI }
    } else if raw <= 0.0 {
        raw
    } else {
        raw - 2.0 * PI
    }
}

/// Find the smallest orbit displacement in the travel direction `d` of `seed`
/// after which the robot's heading aims at `(tx, ty)`.
///
/// The robot rides the orbit circle while its heading stays a constant offset
/// ahead of the orbit angle, so over one lap the heading sweeps a full revolution
/// and points directly at the target exactly once. That "facing" condition is
/// `heading × (target − position) = 0` with `heading · (target − position) > 0`
/// (heading parallel to, and pointing toward, the target).
///
/// This is built to be cheap on an FPU-but-no-hardware-trig target (Cortex-M7):
/// it makes **no** transcendental calls inside the loop. The orbit-angle unit
/// vector `(cos φ, sin φ)` and the heading unit vector `(cos θ, sin θ)` are
/// advanced one fixed step at a time by incremental rotation (angle-addition with
/// a precomputed `(cos Δ, sin Δ)`), so the only `sin`/`cos` calls are the handful
/// of setup values. The signed cross product is smooth (no angle wrapping), so the
/// first sign change with a forward-facing endpoint brackets the root. Because the
/// cross product is very nearly linear across the bracket, the root is refined by a
/// single linear interpolation of the two already-computed cross values (no extra
/// evaluation). Returns `None` if no facing crossing is found within one
/// revolution.
///
/// `SCAN_STEPS` is the dominant cost (the loop is pure arithmetic, so execution
/// time is roughly linear in it). Bracketing is robust at any practical
/// resolution — the single toward-facing crossing is always found — so this knob
/// trades only the refine accuracy, which degrades as `step²` (worst facing error
/// ≈ `(2π/SCAN_STEPS)² / 8`). At 50 steps (7.2°) that is ~0.11° (≈2 mm at 1 m),
/// still far below vision/mechanical noise; raise it for more precision or lower
/// it for less compute.
fn solve_face_point(seed: &CircleSeed, r: f32, tx: f32, ty: f32) -> Option<f32> {
    // Signed cross product and dot product of the heading unit vector with the
    // vector from the robot position to the target, given the orbit-angle and
    // heading unit vectors `(cphi, sphi)` and `(ch, sh)`.
    let cross_dot = |cphi: f32, sphi: f32, ch: f32, sh: f32| -> (f32, f32) {
        let px = seed.center_x + r * cphi;
        let py = seed.center_y + r * sphi;
        let wx = tx - px;
        let wy = ty - py;
        (ch * wy - sh * wx, ch * wx + sh * wy)
    };

    const SCAN_STEPS: i32 = 50; // 7.2° resolution over one revolution (~0.11° refine error)
    const ROOT_TOL: f32 = 1e-6;

    let step = seed.d * (2.0 * PI / SCAN_STEPS as f32);
    let cd = cosf(step);
    let sd = sinf(step);

    // Unit vectors at disp = 0, advanced incrementally through the scan.
    let mut cphi = cosf(seed.orbit_start);
    let mut sphi = sinf(seed.orbit_start);
    let theta0 = seed.orbit_start + seed.heading_offset;
    let mut ch = cosf(theta0);
    let mut sh = sinf(theta0);

    let (mut prev_cross, mut prev_dot) = cross_dot(cphi, sphi, ch, sh);
    if fabsf(prev_cross) < ROOT_TOL && prev_dot > 0.0 {
        return Some(0.0);
    }

    let mut prev_disp = 0.0_f32;
    for i in 1..=SCAN_STEPS {
        // Rotate both unit vectors by one step (angle addition).
        let ncphi = cphi * cd - sphi * sd;
        let nsphi = sphi * cd + cphi * sd;
        cphi = ncphi;
        sphi = nsphi;
        let nch = ch * cd - sh * sd;
        let nsh = sh * cd + ch * sd;
        ch = nch;
        sh = nsh;

        let disp = step * i as f32;
        let (cur_cross, cur_dot) = cross_dot(cphi, sphi, ch, sh);
        // A transversal sign change of the cross product with the heading pointing
        // toward (not away from) the target on at least one side is the facing root.
        let crossing = (prev_cross < 0.0) != (cur_cross < 0.0)
            && (prev_dot > 0.0 || cur_dot > 0.0);
        if crossing {
            // The cross product is ~linear over the narrow bracket, so interpolate
            // its zero between the two already-computed samples.
            let denom = prev_cross - cur_cross;
            let t = if fabsf(denom) > 0.0 { prev_cross / denom } else { 0.5 };
            return Some(prev_disp + t * (disp - prev_disp));
        }
        prev_cross = cur_cross;
        prev_dot = cur_dot;
        prev_disp = disp;
    }
    None
}

/// A single 1D bang-bang profile on the orbit angle φ (the global angle from
/// the orbit center to the robot's position).
///
/// The robot rides the orbit circle while holding a fixed `inset_angle` (ψ)
/// between the orbit tangent vector and its heading, measured toward the orbit
/// center. `inset_angle = 0` points the robot tangent to the orbit;
/// `inset_angle = π/2` points it straight at the center. The heading is a constant
/// offset from the orbit angle:
///
/// ```text
/// robot_pos      = center + r·(cos φ, sin φ)
/// heading        = φ + heading_offset
/// heading_offset = s·(π/2 + ψ)
/// ```
///
/// where `s = ±1` selects which of the two candidate circles is used (center on
/// the left/right of the heading). The orbit travel direction is `d = s · g`,
/// where `g = ±1` is the [`PivotDirection`] (forward/backward). Both the start
/// and target orbit angles follow from the robot's current/target heading:
///
/// ```text
/// φ_start  = θ_start        − heading_offset
/// φ_target = target_heading − heading_offset
/// ```
///
/// Because heading is φ plus a constant, a single bang-bang on φ drives both the
/// orbital position and the robot heading. Of the two candidate circles, the one
/// covering the least travel distance is chosen (ties at a half revolution go to
/// the CCW orbit).
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
    /// Equals `s·(π/2 + ψ)`, where `s = ±1` is the circle side and `ψ` the inset
    /// angle. Fixed at construction.
    pub heading_offset: f32,
    /// Current robot state `[x, y, θ, ẋ, ẏ, θ̇]`. Updated each `tick`.
    pub state: Vector6f,
}

impl PivotTrajectory {
    /// Construct a pivot trajectory that drives the robot to a target heading.
    ///
    /// Two candidate orbit circles pass through the seed state with the required
    /// inset (one curving left, one curving right); the circle that reaches
    /// `target_heading` with the least travel distance is chosen. A heading change
    /// of exactly half a revolution is a tie and resolves to the CCW orbit.
    ///
    /// # Arguments
    /// * `init_state`     – `[x, y, θ, ẋ, ẏ, θ̇]` seed robot state.
    /// * `target_heading` – desired final heading (rad).
    /// * `params`         – velocity/acceleration limits, orbit radius, inset
    ///   angle (absolute-valued here), and forward/backward direction.
    pub fn from_target_heading(
        init_state: Vector6f,
        target_heading: f32,
        params: PivotParams,
    ) -> Result<Self, ControlsError> {
        Self::validate_params(params)?;

        let r = params.orbit_radius;
        let psi = params.resolved_inset();
        let g = params.direction.sign();

        // Heading displacement, wrapped to [−π, π].
        let raw = wrap_angle(target_heading - init_state[2]);

        // Evaluate both candidate circles and keep the one with the least travel.
        let mut best: Option<(f32, CircleSeed, f32)> = None;
        for &s in &[1.0_f32, -1.0_f32] {
            let seed = circle_seed(init_state, r, psi, g, s);
            let displacement = directed_displacement(raw, seed.d);
            let orbit_target = seed.orbit_start + displacement;
            Self::consider(&mut best, fabsf(displacement), seed, orbit_target);
        }

        let (_, seed, orbit_target) = best.ok_or(ControlsError::NoSolution)?;
        Self::build(init_state, seed, orbit_target, r, params)
    }

    /// Construct a pivot trajectory that leaves the robot facing a target point.
    ///
    /// Unlike [`from_target_heading`](Self::from_target_heading), the final heading
    /// is solved so that, once the orbit completes, the robot's heading points at
    /// `(target_x, target_y)`. Because the robot translates along the orbit, the
    /// required heading depends on where the robot ends up, so it is found by
    /// searching the orbit for the first angle at which the heading aims at the
    /// point. The least-distance circle is chosen.
    ///
    /// Returns [`ControlsError::InvalidInput`] if the target point lies inside
    /// either candidate orbit circle (the heading-to-point is ill-conditioned
    /// there), and [`ControlsError::NoSolution`] if neither circle ever faces the
    /// point.
    ///
    /// # Arguments
    /// * `init_state` – `[x, y, θ, ẋ, ẏ, θ̇]` seed robot state.
    /// * `target_x`   – x of the point to face after the pivot (m).
    /// * `target_y`   – y of the point to face after the pivot (m).
    /// * `params`     – velocity/acceleration limits, orbit radius, inset angle
    ///   (absolute-valued here), and forward/backward direction.
    pub fn from_target_point(
        init_state: Vector6f,
        target_x: f32,
        target_y: f32,
        params: PivotParams,
    ) -> Result<Self, ControlsError> {
        Self::validate_params(params)?;

        let r = params.orbit_radius;
        let psi = params.resolved_inset();
        let g = params.direction.sign();

        // Build both circles first so the point can be rejected if it falls inside
        // either one before any orbit is committed to.
        let seeds = [
            circle_seed(init_state, r, psi, g, 1.0),
            circle_seed(init_state, r, psi, g, -1.0),
        ];
        for seed in &seeds {
            let dcx = target_x - seed.center_x;
            let dcy = target_y - seed.center_y;
            if sqrtf(dcx * dcx + dcy * dcy) <= r {
                return Err(ControlsError::InvalidInput);
            }
        }

        let mut best: Option<(f32, CircleSeed, f32)> = None;
        for seed in seeds {
            if let Some(displacement) = solve_face_point(&seed, r, target_x, target_y) {
                let orbit_target = seed.orbit_start + displacement;
                Self::consider(&mut best, fabsf(displacement), seed, orbit_target);
            }
        }

        let (_, seed, orbit_target) = best.ok_or(ControlsError::NoSolution)?;
        Self::build(init_state, seed, orbit_target, r, params)
    }

    /// Validate the shared limit/radius parameters.
    fn validate_params(params: PivotParams) -> Result<(), ControlsError> {
        if params.max_vel_angular <= 0.0 || params.max_accel_angular <= 0.0 {
            return Err(ControlsError::InvalidInput);
        }
        if params.orbit_radius <= 0.0 {
            return Err(ControlsError::InvalidInput);
        }
        Ok(())
    }

    /// Keep the candidate with the smaller travel distance, breaking ties (within
    /// [`PIVOT_TIE_EPS`]) in favor of the CCW orbit (`d = +1`).
    fn consider(
        best: &mut Option<(f32, CircleSeed, f32)>,
        abs_disp: f32,
        seed: CircleSeed,
        orbit_target: f32,
    ) {
        let take = match best {
            None => true,
            Some((best_abs, best_seed, _)) => {
                if fabsf(abs_disp - *best_abs) < PIVOT_TIE_EPS {
                    seed.d > best_seed.d
                } else {
                    abs_disp < *best_abs
                }
            }
        };
        if take {
            *best = Some((abs_disp, seed, orbit_target));
        }
    }

    /// Assemble a [`PivotTrajectory`] from a chosen circle and orbit target.
    fn build(
        init_state: Vector6f,
        seed: CircleSeed,
        orbit_target: f32,
        r: f32,
        params: PivotParams,
    ) -> Result<Self, ControlsError> {
        let orbit = solve_1d_pose(
            seed.orbit_start,
            seed.orbit_start_dot,
            orbit_target,
            params.max_vel_angular,
            params.max_accel_angular,
        )?;

        Ok(PivotTrajectory {
            orbit,
            center_x: seed.center_x,
            center_y: seed.center_y,
            orbit_radius: r,
            orbit_start: seed.orbit_start,
            orbit_start_dot: seed.orbit_start_dot,
            orbit_target,
            heading_offset: seed.heading_offset,
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
    use libm::{atan2f, fabsf, sqrtf};

    fn fwd_params() -> PivotParams {
        PivotParams { inset_angle: 0.0, direction: PivotDirection::Forward, ..PivotParams::default() }
    }

    fn fwd_params_with_inset(inset: f32) -> PivotParams {
        PivotParams { inset_angle: inset, direction: PivotDirection::Forward, ..PivotParams::default() }
    }

    /// Robot at rest with the given heading at `(x, y)`.
    fn init_state_at(x: f32, y: f32, heading: f32) -> Vector6f {
        Vector6f::new(x, y, heading, 0.0, 0.0, 0.0)
    }

    #[test]
    fn heading_reaches_target() {
        let init = init_state_at(0.0, 0.0, 0.0);
        let target = PI / 2.0;
        let traj = PivotTrajectory::from_target_heading(init, target, fwd_params()).unwrap();
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
        let traj = PivotTrajectory::from_target_heading(init, target, fwd_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        assert!(fabsf(wrap_angle(final_state[2] - target)) < 1e-3);
    }

    #[test]
    fn heading_reaches_target_with_inset() {
        let inset = PI / 6.0;
        let init = init_state_at(0.3, -0.2, 0.4);
        let target = PI / 2.0;
        let traj = PivotTrajectory::from_target_heading(init, target, fwd_params_with_inset(inset)).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        assert!(fabsf(wrap_angle(final_state[2] - target)) < 1e-3,
            "heading at end: {}", final_state[2]);
    }

    #[test]
    fn xy_position_on_orbit_at_all_times() {
        let r = PivotParams::default().orbit_radius;
        let init = init_state_at(1.0, 0.5, 0.3);
        let traj = PivotTrajectory::from_target_heading(init, 0.3 + PI / 2.0, fwd_params()).unwrap();
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

    /// The heading must stay a constant offset ahead of the orbit angle (and hence
    /// hold the inset) for the whole trajectory, regardless of orbit direction.
    #[test]
    fn heading_offset_constant_throughout() {
        check_offset_constant(-0.6, 1.2, PI / 5.0); // CCW (positive displacement)
        check_offset_constant(0.6, -1.2, PI / 5.0); // CW (negative displacement)
    }

    fn check_offset_constant(start_heading: f32, target: f32, inset: f32) {
        let init = init_state_at(0.2, 0.1, start_heading);
        let traj = PivotTrajectory::from_target_heading(init, target, fwd_params_with_inset(inset)).unwrap();
        let end = traj.end_time();
        for i in 0..=20 {
            let t = end * (i as f32) / 20.0;
            let state = traj.state_at(t).unwrap();
            // Orbit angle φ from center to robot.
            let phi = atan2f(state[1] - traj.center_y, state[0] - traj.center_x);
            let offset = wrap_angle(state[2] - phi);
            assert!(fabsf(wrap_angle(offset - traj.heading_offset)) < 1e-3,
                "t={}: offset {} != heading_offset {}", t, offset, traj.heading_offset);
        }
    }

    /// A forward pivot keeps the velocity within the inset angle of the heading
    /// (acute when the inset is small).
    #[test]
    fn forward_velocity_acute_with_heading() {
        let inset = PI / 5.0;
        let init = init_state_at(0.0, 0.0, 0.3);
        let params = PivotParams { inset_angle: inset, direction: PivotDirection::Forward, ..PivotParams::default() };
        let traj = PivotTrajectory::from_target_heading(init, 0.3 + PI / 2.0, params).unwrap();
        let end = traj.end_time();
        let s = traj.state_at(end / 2.0).unwrap();
        let vel_dir = atan2f(s[4], s[3]);
        let angle = fabsf(wrap_angle(s[2] - vel_dir));
        assert!(fabsf(angle - inset) < 1e-2, "angle {} != inset {}", angle, inset);
        assert!(angle < PI / 2.0, "forward velocity not acute: {}", angle);
    }

    /// A backward pivot makes the velocity obtuse with the heading (`π − ψ`).
    #[test]
    fn backward_velocity_obtuse_with_heading() {
        let inset = PI / 5.0;
        let init = init_state_at(0.0, 0.0, 0.3);
        let params = PivotParams { inset_angle: inset, direction: PivotDirection::Backward, ..PivotParams::default() };
        let traj = PivotTrajectory::from_target_heading(init, 0.3 + PI / 2.0, params).unwrap();
        let end = traj.end_time();
        let s = traj.state_at(end / 2.0).unwrap();
        let vel_dir = atan2f(s[4], s[3]);
        let angle = fabsf(wrap_angle(s[2] - vel_dir));
        assert!(fabsf(angle - (PI - inset)) < 1e-2, "angle {} != π−inset {}", angle, PI - inset);
        assert!(angle > PI / 2.0, "backward velocity not obtuse: {}", angle);
    }

    /// A negative inset parameter is treated identically to its absolute value.
    #[test]
    fn inset_angle_is_abs_valued() {
        let init = init_state_at(0.1, -0.2, 0.5);
        let pos = PivotTrajectory::from_target_heading(init, 1.5, fwd_params_with_inset(0.4)).unwrap();
        let neg = PivotTrajectory::from_target_heading(init, 1.5, fwd_params_with_inset(-0.4)).unwrap();
        assert!(fabsf(pos.heading_offset - neg.heading_offset) < 1e-6);
        assert!(fabsf(pos.center_x - neg.center_x) < 1e-6);
        assert!(fabsf(pos.center_y - neg.center_y) < 1e-6);
        assert!(fabsf(pos.orbit_target - neg.orbit_target) < 1e-6);
    }

    /// With `compute_inset_angle`, the provided `inset_angle` is ignored and the
    /// inset follows the linear map of `max_vel_angular`, reflected in the
    /// constant heading offset `s·(π/2 + ψ)`.
    #[test]
    fn computed_inset_overrides_provided() {
        use crate::defaults::DEFAULT_PIVOT_INSET_ANGLE_PER_ANGULAR_VEL;
        let init = init_state_at(0.0, 0.0, 0.0);
        let max_vel = 1.5_f32;
        let params = PivotParams {
            inset_angle: 999.0, // must be ignored
            compute_inset_angle: true,
            max_vel_angular: max_vel,
            direction: PivotDirection::Forward,
            ..PivotParams::default()
        };
        let traj = PivotTrajectory::from_target_heading(init, 0.5, params).unwrap();
        let expected_psi =
            (DEFAULT_PIVOT_INSET_ANGLE_PER_ANGULAR_VEL * max_vel).clamp(0.0, PI / 2.0);
        // Forward + small CCW target ⇒ left circle (s = +1) ⇒ offset = π/2 + ψ.
        assert!(fabsf(traj.heading_offset - (PI / 2.0 + expected_psi)) < 1e-5,
            "offset {} != π/2 + ψ ({})", traj.heading_offset, PI / 2.0 + expected_psi);
    }

    /// The computed inset is clamped into `[0, π/2]` for large angular velocities.
    #[test]
    fn computed_inset_clamped_to_half_pi() {
        let init = init_state_at(0.0, 0.0, 0.0);
        let params = PivotParams {
            compute_inset_angle: true,
            max_vel_angular: 1000.0,
            direction: PivotDirection::Forward,
            ..PivotParams::default()
        };
        let traj = PivotTrajectory::from_target_heading(init, 0.5, params).unwrap();
        // ψ clamped to π/2 ⇒ offset = π/2 + π/2 = π.
        assert!(fabsf(traj.heading_offset - PI) < 1e-5,
            "offset {} != π", traj.heading_offset);
    }

    #[test]
    fn final_velocity_zero() {
        let init = init_state_at(0.0, 0.0, 0.0);
        let traj = PivotTrajectory::from_target_heading(init, PI / 3.0, fwd_params()).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        assert!(fabsf(final_state[3]) < 1e-3, "ẋ: {}", final_state[3]);
        assert!(fabsf(final_state[4]) < 1e-3, "ẏ: {}", final_state[4]);
        assert!(fabsf(final_state[5]) < 1e-3, "θ̇: {}", final_state[5]);
    }

    #[test]
    fn seeds_orbital_velocity_from_tangential_projection() {
        // Forward, inset 0, heading 0 ⇒ chosen (left) circle has center at (0, +r)
        // and start tangent along +x. A seed velocity of (v, 0) is purely tangential
        // and should seed φ̇ = v / r, with `state_at(0)` reporting the same velocity.
        let r = PivotParams::default().orbit_radius;
        let v = 0.5;
        let init = Vector6f::new(0.0, 0.0, 0.0, v, 0.0, 0.0);
        let traj = PivotTrajectory::from_target_heading(init, PI / 2.0, fwd_params()).unwrap();
        assert!(fabsf(traj.orbit_start_dot - v / r) < 1e-4,
            "orbit_start_dot {} != v/r {}", traj.orbit_start_dot, v / r);
        let s0 = traj.state_at(0.0).unwrap();
        assert!(fabsf(s0[3] - v) < 1e-3, "ẋ0: {}", s0[3]);
        assert!(fabsf(s0[4]) < 1e-3, "ẏ0: {}", s0[4]);
    }

    #[test]
    fn radial_seed_velocity_is_dropped() {
        // Same geometry: a seed velocity of (0, v) points straight at the center
        // (purely radial) and must produce zero orbital angular velocity.
        let v = 0.5;
        let init = Vector6f::new(0.0, 0.0, 0.0, 0.0, v, 0.0);
        let traj = PivotTrajectory::from_target_heading(init, PI / 2.0, fwd_params()).unwrap();
        assert!(fabsf(traj.orbit_start_dot) < 1e-4,
            "radial velocity not dropped: {}", traj.orbit_start_dot);
    }

    #[test]
    fn zero_displacement_has_zero_end_time() {
        let init = init_state_at(0.0, 0.0, 1.0);
        let traj = PivotTrajectory::from_target_heading(init, 1.0, fwd_params()).unwrap();
        assert!(traj.end_time() < 1e-6);
    }

    #[test]
    fn selects_least_distance_circle() {
        // A +135° forward turn: the chosen orbit must travel ≤ π (the short way),
        // not the 225° long way around the other circle.
        let init = init_state_at(0.0, 0.0, 0.0);
        let target = 3.0 * PI / 4.0;
        let traj = PivotTrajectory::from_target_heading(init, target, fwd_params()).unwrap();
        let displacement = traj.orbit_target - traj.orbit_start;
        assert!(fabsf(displacement) <= PI + 1e-4, "displacement {} exceeds π", displacement);
        assert!(displacement > 0.0, "expected CCW short way, got {}", displacement);
    }

    #[test]
    fn exactly_half_revolution_picks_ccw() {
        // A 180° heading change ties on distance; the CCW orbit must win.
        let init = init_state_at(0.0, 0.0, 0.0);
        let traj = PivotTrajectory::from_target_heading(init, PI, fwd_params()).unwrap();
        let displacement = traj.orbit_target - traj.orbit_start;
        assert!(displacement > 0.0, "half-revolution not CCW: {}", displacement);
        assert!(fabsf(displacement - PI) < 1e-3, "displacement {} != π", displacement);
    }

    #[test]
    fn uses_shortest_angular_path() {
        // Target just under +π ⇒ positive (CCW) displacement.
        let init = init_state_at(0.0, 0.0, 0.0);
        let traj = PivotTrajectory::from_target_heading(init, PI - 0.1, fwd_params()).unwrap();
        assert!(traj.orbit.sdd1 >= 0.0);
        // Target just short of −π ⇒ negative (CW) displacement.
        let traj_neg = PivotTrajectory::from_target_heading(init, -(PI - 0.1), fwd_params()).unwrap();
        assert!(traj_neg.orbit.sdd1 <= 0.0);
    }

    #[test]
    fn invalid_params_rejected() {
        let init = init_state_at(0.0, 0.0, 0.0);
        let mut params = fwd_params();
        params.max_vel_angular = 0.0;
        assert!(PivotTrajectory::from_target_heading(init, 1.0, params).is_err());
        params.max_vel_angular = 1.0;
        params.max_accel_angular = -1.0;
        assert!(PivotTrajectory::from_target_heading(init, 1.0, params).is_err());
        params.max_accel_angular = 1.0;
        params.orbit_radius = 0.0;
        assert!(PivotTrajectory::from_target_heading(init, 1.0, params).is_err());
    }

    #[test]
    fn center_derived_from_seed() {
        // Forward, inset 0, heading 0, small +CCW target ⇒ left circle:
        // heading_offset = +π/2 ⇒ φ_start = −π/2 ⇒
        // center = pos − r·(cos(−π/2), sin(−π/2)) = pos + (0, r).
        let r = PivotParams::default().orbit_radius;
        let init = init_state_at(1.0, 2.0, 0.0);
        let traj = PivotTrajectory::from_target_heading(init, 0.5, fwd_params()).unwrap();
        assert!(fabsf(traj.center_x - 1.0) < 1e-5, "cx={}", traj.center_x);
        assert!(fabsf(traj.center_y - (2.0 + r)) < 1e-5, "cy={}", traj.center_y);
    }

    #[test]
    fn from_target_point_faces_point() {
        let init = init_state_at(0.0, 0.0, 0.3);
        let target = (2.0_f32, 1.0_f32);
        let traj = PivotTrajectory::from_target_point(init, target.0, target.1, fwd_params_with_inset(0.3)).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        let bearing = atan2f(target.1 - final_state[1], target.0 - final_state[0]);
        assert!(fabsf(wrap_angle(final_state[2] - bearing)) < 1e-2,
            "final heading {} does not face target (bearing {})", final_state[2], bearing);
    }

    #[test]
    fn from_target_point_accounts_for_translation() {
        // The point sits where the robot starts pointing, but because the robot
        // translates along the orbit the solved final heading must differ from the
        // start heading to keep facing the point.
        let init = init_state_at(0.0, 0.0, 0.0);
        let target = (3.0_f32, 0.0_f32);
        let traj = PivotTrajectory::from_target_point(init, target.0, target.1, fwd_params_with_inset(0.25)).unwrap();
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        let bearing = atan2f(target.1 - final_state[1], target.0 - final_state[0]);
        assert!(fabsf(wrap_angle(final_state[2] - bearing)) < 1e-2);
    }

    #[test]
    fn from_target_point_rejects_point_inside_orbit() {
        // The robot sits on both candidate circles, so facing its own position is
        // on the orbit boundary and must be rejected.
        let init = init_state_at(0.0, 0.0, 0.7);
        let err = PivotTrajectory::from_target_point(init, 0.0, 0.0, fwd_params_with_inset(0.3));
        assert_eq!(err.err(), Some(ControlsError::InvalidInput));
    }

    #[test]
    fn from_target_point_least_distance_circle() {
        let init = init_state_at(0.0, 0.0, 0.0);
        let target = (-2.0_f32, 0.5_f32);
        let traj = PivotTrajectory::from_target_point(init, target.0, target.1, fwd_params_with_inset(0.2)).unwrap();
        let displacement = traj.orbit_target - traj.orbit_start;
        // The least-distance solution should be within a full revolution.
        assert!(fabsf(displacement) <= 2.0 * PI + 1e-4);
        let end = traj.end_time();
        let final_state = traj.state_at(end).unwrap();
        let bearing = atan2f(target.1 - final_state[1], target.0 - final_state[0]);
        assert!(fabsf(wrap_angle(final_state[2] - bearing)) < 1e-2);
    }
}
