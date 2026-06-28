use libm::{atan2f, fabsf, sqrtf};
use crate::bangbang_trajectory::{BangBangTraj1D, solve_1d_pose, solve_1d_twist, eval_1d_state_at, eval_1d_accel_at};
use crate::defaults::DEFAULT_CONTROL_DT;
use crate::{ControlsError, Vector2f, Vector3f, Vector6f, wrap_angle};
use crate::trajectory::Trajectory;

/// How the heading axis of a [`LinearTrajectory`] is driven.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinearHeadingMode {
    /// Heading follows a fixed bang-bang to a constant target heading
    /// (`LinearTrajectory::from_line`).
    Fixed = 0,
    /// Heading continuously faces a fixed global point: a bounded-rate angular
    /// servo acquires the line-of-sight and then tracks it lag-free
    /// (`LinearTrajectory::from_point`).
    FacePoint = 1,
}

impl Default for LinearHeadingMode {
    fn default() -> Self {
        LinearHeadingMode::Fixed
    }
}

/// Parameters controlling a line-following trajectory.
///
/// The trajectory drives the robot perpendicular onto a line and rotates it to a
/// target heading (behaving like the 3D bang-bang during the approach, including
/// actively bringing the colinear velocity to rest), then accelerates along the
/// line to a target colinear velocity once it is within
/// `colinear_start_thresh_linear` of the line. The three axes — colinear (along
/// the line), perpendicular (toward the line), and angular (heading) — each have
/// independent velocity and acceleration limits.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LinearParams {
    /// Maximum velocity along the line (m/s).
    pub max_vel_colinear: f32,
    /// Maximum velocity perpendicular to the line, used during approach (m/s).
    pub max_vel_perp: f32,
    /// Maximum heading angular velocity (rad/s).
    pub max_vel_angular: f32,
    /// Maximum acceleration perpendicular to the line (m/s²).
    pub max_accel_perp: f32,
    /// Maximum acceleration along the line (m/s²).
    pub max_accel_colinear: f32,
    /// Maximum heading angular acceleration (rad/s²).
    pub max_accel_angular: f32,
    /// Perpendicular distance from the line (m) below which the robot begins
    /// accelerating along the line toward the target colinear velocity.
    pub colinear_start_thresh_linear: f32,
}

impl Default for LinearParams {
    fn default() -> Self {
        use crate::defaults::*;
        Self {
            max_vel_colinear: DEFAULT_MAX_VEL_LINEAR,
            max_vel_perp: DEFAULT_MAX_VEL_LINEAR,
            max_vel_angular: DEFAULT_MAX_VEL_ANGULAR,
            max_accel_perp: DEFAULT_MAX_ACCEL_LINEAR,
            max_accel_colinear: DEFAULT_MAX_ACCEL_LINEAR,
            max_accel_angular: DEFAULT_MAX_ACCEL_ANGULAR,
            colinear_start_thresh_linear: DEFAULT_LINEAR_COLINEAR_START_THRESH,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct LinearTrajectory {
    /// 1-D bang-bang trajectory perpendicular to the line (position target: the line).
    pub traj_perp: BangBangTraj1D,
    /// 1-D bang-bang that brakes the colinear velocity to rest during the approach.
    /// Starts at internal time 0.
    pub traj_colinear: BangBangTraj1D,
    /// 1-D bang-bang that accelerates the colinear velocity to the target speed.
    /// Begins at `colinear_accel_start_time`.
    pub traj_colinear_accel: BangBangTraj1D,
    /// 1-D bang-bang trajectory for heading. Only used in
    /// [`LinearHeadingMode::Fixed`] (target heading from `from_line`).
    pub traj_theta: BangBangTraj1D,
    /// Internal time at which the perpendicular distance first drops below the
    /// threshold; the colinear acceleration profile begins here.
    pub colinear_accel_start_time: f32,
    /// How the heading axis is driven.
    pub heading_mode: LinearHeadingMode,
    /// Global point the robot faces in [`LinearHeadingMode::FacePoint`] (m).
    /// Unused in `Fixed` mode.
    pub target_point: Vector2f,
    /// Angular velocity limit (rad/s) used by the line-of-sight servo in
    /// [`LinearHeadingMode::FacePoint`]. Baked into `traj_theta` for `Fixed`.
    pub heading_max_vel: f32,
    /// Angular acceleration limit (rad/s²) used by the line-of-sight servo in
    /// [`LinearHeadingMode::FacePoint`]. Baked into `traj_theta` for `Fixed`.
    pub heading_max_accel: f32,
    /// Internal time at which the heading axis is considered settled. For
    /// `Fixed` this is `traj_theta.t4`; for `FacePoint` it is the time the
    /// angular servo finishes acquiring the line-of-sight (computed once at
    /// construction). Drives `end_time` along with the position profiles.
    pub heading_end_time: f32,
    /// A point the line passes through (m). Fixed at construction.
    pub start_point: Vector2f,
    /// Unit direction of the line (normalized on construction). Fixed.
    pub line_dir: Vector2f,
    /// Tracked robot state `[x, y, θ, ẋ, ẏ, θ̇]` at the current t=0.
    pub state: Vector6f,
}

impl LinearTrajectory {
    /// Construct a trajectory that drives the robot onto the line (a point
    /// `start_point` plus unit direction `line_dir`) and, once within the
    /// perpendicular threshold, accelerates along the line to `line_vel` while
    /// holding `target_theta`.
    ///
    /// # Arguments
    /// * `init_state`   – `[x, y, θ, ẋ, ẏ, θ̇]` seed robot state.
    /// * `target_theta` – desired final heading (rad), reached via shortest path.
    /// * `start_point`  – a point the line passes through (m).
    /// * `line_dir`     – direction of the line (normalized internally).
    /// * `line_vel`     – target velocity along `line_dir` once on the line (m/s).
    /// * `params`       – per-axis velocity/acceleration limits and the
    ///   perpendicular start threshold.
    pub fn from_line(
        init_state: Vector6f,
        target_theta: f32,
        start_point: Vector2f,
        line_dir: Vector2f,
        line_vel: f32,
        params: LinearParams,
    ) -> Result<Self, ControlsError> {
        let mut traj = Self::build_position(init_state, start_point, line_dir, line_vel, params)?;

        // Heading: shortest angular path to the target heading.
        let theta0 = init_state[2];
        let thetad0 = init_state[5];
        let theta_target = theta0 + wrap_angle(target_theta - theta0);
        traj.traj_theta = solve_1d_pose(theta0, thetad0, theta_target, params.max_vel_angular, params.max_accel_angular)?;
        traj.heading_mode = LinearHeadingMode::Fixed;
        traj.heading_end_time = traj.traj_theta.t4;
        Ok(traj)
    }

    /// Construct a trajectory that drives the robot onto the line (a point
    /// `start_point` plus unit direction `line_dir`) and, once within the
    /// perpendicular threshold, accelerates along the line to `line_vel`, while
    /// **continuously facing the global point** `(target_x, target_y)`.
    ///
    /// The position path is identical to [`from_line`](Self::from_line); only the
    /// heading differs: a bounded-rate angular servo (respecting
    /// `max_vel_angular` / `max_accel_angular`) acquires the line-of-sight to the
    /// point from the robot's initial heading, then tracks it lag-free as the
    /// robot translates along the line.
    ///
    /// Returns [`ControlsError::InvalidInput`] for non-positive limits, a
    /// degenerate line direction, or if the robot starts exactly on the target
    /// point (line-of-sight undefined).
    ///
    /// # Arguments
    /// * `init_state`  – `[x, y, θ, ẋ, ẏ, θ̇]` seed robot state.
    /// * `target_x`    – x of the global point to face (m).
    /// * `target_y`    – y of the global point to face (m).
    /// * `start_point` – a point the line passes through (m).
    /// * `line_dir`    – direction of the line (normalized internally).
    /// * `line_vel`    – target velocity along `line_dir` once on the line (m/s).
    /// * `params`      – per-axis velocity/acceleration limits and the
    ///   perpendicular start threshold.
    pub fn from_point(
        init_state: Vector6f,
        target_x: f32,
        target_y: f32,
        start_point: Vector2f,
        line_dir: Vector2f,
        line_vel: f32,
        params: LinearParams,
    ) -> Result<Self, ControlsError> {
        let target_point = Vector2f::new(target_x, target_y);

        // Reject only when the robot starts exactly on the point (LOS undefined).
        let dx0 = target_point.x - init_state[0];
        let dy0 = target_point.y - init_state[1];
        if sqrtf(dx0 * dx0 + dy0 * dy0) < 1e-6 {
            return Err(ControlsError::InvalidInput);
        }

        let mut traj = Self::build_position(init_state, start_point, line_dir, line_vel, params)?;
        traj.heading_mode = LinearHeadingMode::FacePoint;
        traj.target_point = target_point;
        // Find when the angular servo finishes acquiring the line-of-sight.
        traj.heading_end_time = traj.compute_heading_acquire_end(params);
        Ok(traj)
    }

    /// Build the shared position (perpendicular + colinear) portion of the
    /// trajectory; heading fields are left at their defaults for the caller to
    /// fill in. Validates limits and the line direction.
    fn build_position(
        init_state: Vector6f,
        start_point: Vector2f,
        line_dir: Vector2f,
        line_vel: f32,
        params: LinearParams,
    ) -> Result<Self, ControlsError> {
        if params.max_vel_colinear <= 0.0
            || params.max_vel_perp <= 0.0
            || params.max_vel_angular <= 0.0
            || params.max_accel_colinear <= 0.0
            || params.max_accel_perp <= 0.0
            || params.max_accel_angular <= 0.0
        {
            return Err(ControlsError::InvalidInput);
        }

        let dir_norm = sqrtf(line_dir.x * line_dir.x + line_dir.y * line_dir.y);
        if dir_norm < 1e-6 {
            return Err(ControlsError::InvalidInput);
        }
        let line_dir = line_dir / dir_norm;
        let perp_dir = perp_of(line_dir);

        // Project the seed state into the line frame.
        let (s0, vs0, d0, vd0) = project(init_state, start_point, line_dir, perp_dir);

        // Perpendicular: drive onto the line (target perp position 0).
        let traj_perp = solve_1d_pose(d0, vd0, 0.0, params.max_vel_perp, params.max_accel_perp)?;

        // Colinear braking to rest during the approach.
        let traj_colinear = solve_1d_twist(vs0, 0.0, params.max_accel_colinear)?;

        // Time at which the perpendicular distance crosses below the threshold.
        let t_start = perp_threshold_time(traj_perp, d0, vd0, params.colinear_start_thresh_linear);

        // Colinear velocity at the engagement time, seeding the accel profile so it
        // works whether or not braking has finished by then.
        let (_, vs_at_start) = eval_1d_state_at(traj_colinear, s0, vs0, 0.0, t_start)?;
        let mut traj_colinear_accel = solve_1d_twist(vs_at_start, line_vel, params.max_accel_colinear)?;
        traj_colinear_accel.time_shift(t_start);

        Ok(LinearTrajectory {
            traj_perp,
            traj_colinear,
            traj_colinear_accel,
            traj_theta: BangBangTraj1D::default(),
            colinear_accel_start_time: t_start,
            heading_mode: LinearHeadingMode::Fixed,
            target_point: Vector2f::zeros(),
            heading_max_vel: params.max_vel_angular,
            heading_max_accel: params.max_accel_angular,
            heading_end_time: 0.0,
            start_point,
            line_dir,
            state: init_state,
        })
    }

    /// Shift all segment times (and the engagement/heading-end times) forward by
    /// `dt` seconds.
    pub fn time_shift(&mut self, dt: f32) {
        self.traj_perp.time_shift(dt);
        self.traj_colinear.time_shift(dt);
        self.traj_colinear_accel.time_shift(dt);
        self.traj_theta.time_shift(dt);
        self.colinear_accel_start_time += dt;
        self.heading_end_time += dt;
    }

    /// Time at which the trajectory stops commanding any acceleration. Past this
    /// time the robot is on the line moving at the target colinear velocity.
    pub fn end_time(&self) -> f32 {
        self.traj_perp
            .t4
            .max(self.heading_end_time)
            .max(self.traj_colinear_accel.t4)
    }

    /// Evaluate the full robot state `[x, y, θ, ẋ, ẏ, θ̇]` at time `t`.
    pub fn state_at(&self, t: f32) -> Result<Vector6f, ControlsError> {
        let (pos, vel, _acc) = self.kinematics_at(t)?;
        let (theta, thetad, _thetadd) = self.heading_at(t)?;
        Ok(Vector6f::new(pos.x, pos.y, wrap_angle(theta), vel.x, vel.y, thetad))
    }

    /// Evaluate the acceleration command `[ẍ, ÿ, θ̈]` at time `t`.
    pub fn accel_at(&self, t: f32) -> Result<Vector3f, ControlsError> {
        let (_pos, _vel, acc) = self.kinematics_at(t)?;
        let (_theta, _thetad, thetadd) = self.heading_at(t)?;
        Ok(Vector3f::new(acc.x, acc.y, thetadd))
    }

    /// Closed-form translational kinematics in the global frame at time `t`,
    /// returning `(position, velocity, acceleration)` as 2-D vectors. Independent
    /// of the heading mode.
    fn kinematics_at(&self, t: f32) -> Result<(Vector2f, Vector2f, Vector2f), ControlsError> {
        let perp_dir = perp_of(self.line_dir);
        let (s0, vs0, d0, vd0) = project(self.state, self.start_point, self.line_dir, perp_dir);

        let (s, vs) = self.colinear_state_at(s0, vs0, t)?;
        let (d, vd) = eval_1d_state_at(self.traj_perp, d0, vd0, 0.0, t)?;
        let a_s = self.colinear_accel_at(t)?;
        let a_d = eval_1d_accel_at(self.traj_perp, t)?;

        let pos = Vector2f::new(
            self.start_point.x + s * self.line_dir.x + d * perp_dir.x,
            self.start_point.y + s * self.line_dir.y + d * perp_dir.y,
        );
        let vel = Vector2f::new(
            vs * self.line_dir.x + vd * perp_dir.x,
            vs * self.line_dir.y + vd * perp_dir.y,
        );
        let acc = Vector2f::new(
            a_s * self.line_dir.x + a_d * perp_dir.x,
            a_s * self.line_dir.y + a_d * perp_dir.y,
        );
        Ok((pos, vel, acc))
    }

    /// Heading `(θ, θ̇, θ̈)` at time `t`, dispatching on the heading mode.
    fn heading_at(&self, t: f32) -> Result<(f32, f32, f32), ControlsError> {
        match self.heading_mode {
            LinearHeadingMode::Fixed => {
                let (theta, thetad) = eval_1d_state_at(self.traj_theta, self.state[2], self.state[5], 0.0, t)?;
                let thetadd = eval_1d_accel_at(self.traj_theta, t)?;
                Ok((theta, thetad, thetadd))
            }
            LinearHeadingMode::FacePoint => self.face_point_heading_at(
                t,
                AngularLimits { max_vel: self.heading_max_vel, max_accel: self.heading_max_accel },
            ),
        }
    }

    /// Integrate the line-of-sight angular servo from the current heading state
    /// (`self.state[2]/[5]`) forward to time `t`, returning `(θ, θ̇, θ̈)`. The
    /// servo acquires the line-of-sight at a bounded rate then tracks it lag-free
    /// using the closed-form LOS angular velocity/acceleration as feed-forward.
    fn face_point_heading_at(&self, t: f32, lim: AngularLimits) -> Result<(f32, f32, f32), ControlsError> {
        if t <= 0.0 {
            // At t = 0 report the seed heading and the instantaneous command.
            let (_, _, thetadd) = self.servo_command(0.0, self.state[2], self.state[5], lim)?;
            return Ok((self.state[2], self.state[5], thetadd));
        }

        let mut theta = self.state[2];
        let mut thetad = self.state[5];
        let mut thetadd = 0.0;
        let dt = DEFAULT_CONTROL_DT;
        let mut elapsed = 0.0;
        while elapsed < t - 1e-9 {
            let h = dt.min(t - elapsed);
            let (theta_n, thetad_n, thetadd_n) = self.servo_step(elapsed, theta, thetad, h, lim)?;
            theta = theta_n;
            thetad = thetad_n;
            thetadd = thetadd_n;
            elapsed += h;
        }
        Ok((theta, thetad, thetadd))
    }

    /// Compute the commanded angular acceleration `θ̈` at sub-step starting time
    /// `t` given the current `(θ, θ̇)`. Combines the line-of-sight feed-forward
    /// `φ̈` with a time-optimal relative-frame correction, clamped to the angular
    /// limits. Returns `(φ, φ̇, θ̈_cmd)`.
    fn servo_command(&self, t: f32, theta: f32, thetad: f32, lim: AngularLimits) -> Result<(f32, f32, f32), ControlsError> {
        let (pos, vel, acc) = self.kinematics_at(t)?;
        let (phi, phi_dot, phi_ddot) = los_state(self.target_point, pos, vel, acc);

        let e = wrap_angle(theta - phi);
        let edot = thetad - phi_dot;

        // Time-optimal relative correction toward (e, edot) = (0, 0).
        let u = min_time_accel(e, edot, lim.max_accel);
        let theta_ddot = clamp(phi_ddot + u, -lim.max_accel, lim.max_accel);
        Ok((phi, phi_dot, theta_ddot))
    }

    /// Advance the servo one sub-step of length `h` from sub-step start time `t`
    /// with state `(θ, θ̇)`. Returns the next `(θ, θ̇, θ̈)` with `θ̇` clamped to the
    /// angular velocity limit.
    fn servo_step(&self, t: f32, theta: f32, thetad: f32, h: f32, lim: AngularLimits) -> Result<(f32, f32, f32), ControlsError> {
        let (_phi, _phi_dot, theta_ddot_cmd) = self.servo_command(t, theta, thetad, lim)?;

        // Integrate velocity with the velocity-limit clamp, then recover the
        // effective acceleration so accel reporting stays consistent.
        let thetad_unclamped = thetad + theta_ddot_cmd * h;
        let thetad_next = clamp(thetad_unclamped, -lim.max_vel, lim.max_vel);
        let theta_ddot = (thetad_next - thetad) / h;
        let theta_next = wrap_angle(theta + thetad * h + 0.5 * theta_ddot * h * h);
        Ok((theta_next, thetad_next, theta_ddot))
    }

    /// Integrate the servo from the seed state to find when it has acquired the
    /// line-of-sight (both heading error and rate error within tolerance). Capped
    /// at the translational end time plus a margin.
    fn compute_heading_acquire_end(&self, params: LinearParams) -> f32 {
        let lim = AngularLimits {
            max_vel: params.max_vel_angular,
            max_accel: params.max_accel_angular,
        };
        const E_TOL: f32 = 1e-3;       // rad
        const EDOT_TOL: f32 = 1e-2;    // rad/s
        let pos_end = self.traj_perp.t4.max(self.traj_colinear_accel.t4);
        let horizon = pos_end + 5.0;
        let dt = DEFAULT_CONTROL_DT;

        let mut theta = self.state[2];
        let mut thetad = self.state[5];
        let mut t = 0.0;
        while t < horizon {
            // Check acquisition at the current sub-step start.
            if let Ok((pos, vel, acc)) = self.kinematics_at(t) {
                let (phi, phi_dot, _) = los_state(self.target_point, pos, vel, acc);
                if fabsf(wrap_angle(theta - phi)) < E_TOL && fabsf(thetad - phi_dot) < EDOT_TOL {
                    return t;
                }
            }
            match self.servo_step(t, theta, thetad, dt, lim) {
                Ok((theta_n, thetad_n, _)) => {
                    theta = theta_n;
                    thetad = thetad_n;
                }
                Err(_) => break,
            }
            t += dt;
        }
        horizon
    }


    /// Colinear position/velocity at time `t`, composing the brake and accelerate
    /// profiles with the handoff at `colinear_accel_start_time`.
    fn colinear_state_at(&self, s0: f32, vs0: f32, t: f32) -> Result<(f32, f32), ControlsError> {
        let t_start = self.colinear_accel_start_time;
        if t <= t_start {
            // Still braking toward rest (or holding at rest before engagement).
            eval_1d_state_at(self.traj_colinear, s0, vs0, 0.0, t)
        } else if t_start > 0.0 {
            // Braked until t_start, then accelerate from the velocity reached there.
            let (s_mid, vs_mid) = eval_1d_state_at(self.traj_colinear, s0, vs0, 0.0, t_start)?;
            eval_1d_state_at(self.traj_colinear_accel, s_mid, vs_mid, t_start, t)
        } else {
            // Engagement already in the past; integrate the accel profile from the
            // current colinear state.
            eval_1d_state_at(self.traj_colinear_accel, s0, vs0, 0.0, t)
        }
    }

    /// Colinear acceleration at time `t`, selecting the brake or accelerate profile.
    fn colinear_accel_at(&self, t: f32) -> Result<f32, ControlsError> {
        if t < self.colinear_accel_start_time {
            eval_1d_accel_at(self.traj_colinear, t)
        } else {
            eval_1d_accel_at(self.traj_colinear_accel, t)
        }
    }
}

impl Trajectory for LinearTrajectory {
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

// --- helpers ---

/// Angular velocity/acceleration limits used by the line-of-sight heading servo.
#[derive(Clone, Copy)]
struct AngularLimits {
    max_vel: f32,
    max_accel: f32,
}

/// Perpendicular (left-hand, +90°) of a 2D direction.
fn perp_of(dir: Vector2f) -> Vector2f {
    Vector2f::new(-dir.y, dir.x)
}

/// Clamp a scalar to `[lo, hi]`.
fn clamp(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo { lo } else if v > hi { hi } else { v }
}

/// Closed-form line-of-sight angle `φ` toward `target` from position `pos`, plus
/// its time derivatives `(φ̇, φ̈)` given the robot velocity `vel` and acceleration
/// `acc`. With `Δ = target − pos` (so `dΔ/dt = −vel`):
/// `φ = atan2(Δy, Δx)`, `φ̇ = (Δy·vx − Δx·vy)/|Δ|²`, and `φ̈` follows from the
/// quotient rule. Falls back to zeros if `|Δ|` is degenerate.
fn los_state(target: Vector2f, pos: Vector2f, vel: Vector2f, acc: Vector2f) -> (f32, f32, f32) {
    let dx = target.x - pos.x;
    let dy = target.y - pos.y;
    let r2 = dx * dx + dy * dy;
    let phi = atan2f(dy, dx);
    if r2 < 1e-12 {
        return (phi, 0.0, 0.0);
    }
    // N = Δy·vx − Δx·vy ; φ̇ = N / r²
    let n = dy * vel.x - dx * vel.y;
    let phi_dot = n / r2;
    // dN/dt = Δy·ax − Δx·ay   (the velocity cross terms cancel)
    let n_dot = dy * acc.x - dx * acc.y;
    // d(r²)/dt = 2·Δ·dΔ/dt = −2·(Δx·vx + Δy·vy)
    let r2_dot = -2.0 * (dx * vel.x + dy * vel.y);
    // φ̈ = (dN/dt)/r² − N·d(r²)/dt / r⁴
    let phi_ddot = n_dot / r2 - n * r2_dot / (r2 * r2);
    (phi, phi_dot, phi_ddot)
}

/// Time-optimal relative acceleration that drives `(e, ė)` toward `(0, 0)` under
/// the acceleration bound `a`. Uses the standard second-order switching curve
/// `s = e + ė·|ė|/(2a)`; a small deadband near the origin returns zero so the
/// caller's feed-forward yields lag-free tracking without chatter.
fn min_time_accel(e: f32, edot: f32, a: f32) -> f32 {
    const E_DEAD: f32 = 1e-3;     // rad
    const EDOT_DEAD: f32 = 1e-2;  // rad/s
    if fabsf(e) < E_DEAD && fabsf(edot) < EDOT_DEAD {
        return 0.0;
    }
    if a <= 0.0 {
        return 0.0;
    }
    let s = e + edot * fabsf(edot) / (2.0 * a);
    if s > 0.0 {
        -a
    } else if s < 0.0 {
        a
    } else {
        // On the switching curve: decelerate toward rest.
        -edot.signum() * a
    }
}

/// Project a full robot state into the line frame, returning colinear position,
/// colinear velocity, perpendicular position, and perpendicular velocity.
fn project(state: Vector6f, start_point: Vector2f, line_dir: Vector2f, perp_dir: Vector2f) -> (f32, f32, f32, f32) {
    let rel = Vector2f::new(state[0] - start_point.x, state[1] - start_point.y);
    let vel = Vector2f::new(state[3], state[4]);
    let s = rel.dot(&line_dir);
    let d = rel.dot(&perp_dir);
    let vs = vel.dot(&line_dir);
    let vd = vel.dot(&perp_dir);
    (s, vs, d, vd)
}

/// Smallest root `τ` in `[0, dur]` of `½·accel·τ² + vel·τ + c0 = 0`, if any.
fn smallest_root_in(accel: f32, vel: f32, c0: f32, dur: f32) -> Option<f32> {
    const EPS: f32 = 1e-9;
    let mut best: Option<f32> = None;
    let mut consider = |t: f32| {
        if t >= -1e-6 && t <= dur + 1e-6 {
            let t = if t < 0.0 { 0.0 } else { t };
            if best.map_or(true, |b| t < b) {
                best = Some(t);
            }
        }
    };
    if accel.abs() < EPS {
        if vel.abs() > EPS {
            consider(-c0 / vel);
        }
    } else {
        let disc = vel * vel - 2.0 * accel * c0;
        if disc >= 0.0 {
            let sq = sqrtf(disc);
            consider((-vel + sq) / accel);
            consider((-vel - sq) / accel);
        }
    }
    best
}

/// Time at which the perpendicular trajectory first brings `|d|` to the
/// threshold. Returns 0 when the robot already starts within the threshold, and
/// the perpendicular end time as a fallback if no crossing is found.
fn perp_threshold_time(traj: BangBangTraj1D, d0: f32, vd0: f32, thresh: f32) -> f32 {
    if d0.abs() <= thresh {
        return 0.0;
    }
    let segments = [
        (traj.t1, traj.t2, traj.sdd1),
        (traj.t2, traj.t3, traj.sdd2),
        (traj.t3, traj.t4, traj.sdd3),
    ];
    for (t_a, t_b, accel) in segments {
        let dur = t_b - t_a;
        if dur <= 0.0 {
            continue;
        }
        let (d_a, vd_a) = eval_1d_state_at(traj, d0, vd0, 0.0, t_a).unwrap_or((d0, vd0));
        let mut earliest: Option<f32> = None;
        for c in [thresh, -thresh] {
            if let Some(t) = smallest_root_in(accel, vd_a, d_a - c, dur) {
                earliest = Some(match earliest {
                    Some(e) => e.min(t),
                    None => t,
                });
            }
        }
        if let Some(t) = earliest {
            return t_a + t;
        }
    }
    traj.t4
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;
    use libm::{cosf, sinf};

    fn test_params() -> LinearParams {
        LinearParams {
            max_vel_colinear: 3.0,
            max_vel_perp: 3.0,
            max_vel_angular: 3.0 * PI,
            max_accel_perp: 2.0,
            max_accel_colinear: 2.0,
            max_accel_angular: 2.0 * PI,
            colinear_start_thresh_linear: 0.1,
        }
    }

    /// Signed perpendicular distance of `state` from the line.
    fn perp_dist(state: Vector6f, start: Vector2f, dir: Vector2f) -> f32 {
        let p = perp_of(dir);
        let rel = Vector2f::new(state[0] - start.x, state[1] - start.y);
        rel.dot(&p)
    }

    /// Colinear velocity of `state` along the line direction.
    fn colinear_vel(state: Vector6f, dir: Vector2f) -> f32 {
        Vector2f::new(state[3], state[4]).dot(&dir)
    }

    #[test]
    fn default_params_are_sane() {
        let p = LinearParams::default();
        assert!(p.max_vel_colinear > 0.0);
        assert!(p.max_accel_colinear > 0.0);
        assert!(p.colinear_start_thresh_linear > 0.0);
    }

    #[test]
    fn invalid_direction_rejected() {
        let init = Vector6f::zeros();
        let res = LinearTrajectory::from_line(
            init,
            0.0,
            Vector2f::new(0.0, 0.0),
            Vector2f::new(0.0, 0.0),
            1.0,
            test_params(),
        );
        assert!(res.is_err());
    }

    #[test]
    fn reaches_line_heading_and_speed_x_axis() {
        // Line along +x through origin; robot offset 1 m in +y.
        let init = Vector6f::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        let start = Vector2f::new(0.0, 0.0);
        let dir = Vector2f::new(1.0, 0.0);
        let line_vel = 1.5;
        let target_theta = 0.0;
        let traj = LinearTrajectory::from_line(init, target_theta, start, dir, line_vel, test_params()).unwrap();
        let end = traj.end_time();
        assert!(end > 0.0);
        let final_state = traj.state_at(end).unwrap();
        // On the line.
        assert!(perp_dist(final_state, start, dir).abs() < 1e-2);
        // Facing the target heading.
        assert!(wrap_angle(final_state[2] - target_theta).abs() < 1e-2);
        // Moving at the target colinear velocity.
        assert!((colinear_vel(final_state, dir) - line_vel).abs() < 1e-2);
        // Perpendicular velocity has died out.
        let p = perp_of(dir);
        let vperp = Vector2f::new(final_state[3], final_state[4]).dot(&p);
        assert!(vperp.abs() < 1e-2);
    }

    #[test]
    fn approach_brakes_colinear_before_engaging() {
        // Robot starts far from the line (2 m perp) and moving fast along the line.
        // Before engagement it should bring colinear velocity toward rest.
        let dir = Vector2f::new(1.0, 0.0);
        let start = Vector2f::new(0.0, 0.0);
        let init = Vector6f::new(0.0, 2.0, 0.0, 2.0, 0.0, 0.0);
        let traj = LinearTrajectory::from_line(init, 0.0, start, dir, 1.0, test_params()).unwrap();
        let t_start = traj.colinear_accel_start_time;
        assert!(t_start > 0.0);
        // At engagement, the perpendicular distance is ~threshold.
        let at_start = traj.state_at(t_start).unwrap();
        assert!((perp_dist(at_start, start, dir).abs() - 0.1).abs() < 2e-2);
        // The colinear speed at engagement is well below the initial 2 m/s.
        assert!(colinear_vel(at_start, dir).abs() < 1.0);
    }

    #[test]
    fn already_inside_threshold_engages_immediately() {
        // Robot starts within the threshold of the line.
        let dir = Vector2f::new(0.0, 1.0);
        let start = Vector2f::new(0.0, 0.0);
        let init = Vector6f::new(0.05, 0.0, PI / 2.0, 0.0, 0.0, 0.0);
        let traj = LinearTrajectory::from_line(init, PI / 2.0, start, dir, 2.0, test_params()).unwrap();
        assert_eq!(traj.colinear_accel_start_time, 0.0);
        let final_state = traj.state_at(traj.end_time()).unwrap();
        assert!(perp_dist(final_state, start, dir).abs() < 1e-2);
        assert!((colinear_vel(final_state, dir) - 2.0).abs() < 1e-2);
    }

    #[test]
    fn diagonal_line_reaches_target() {
        // Non-axis-aligned line at 30°, robot offset to the side.
        let theta = PI / 6.0;
        let dir = Vector2f::new(cosf(theta), sinf(theta));
        let start = Vector2f::new(0.5, -0.5);
        let init = Vector6f::new(-1.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        let line_vel = 1.0;
        let traj = LinearTrajectory::from_line(init, theta, start, dir, line_vel, test_params()).unwrap();
        let final_state = traj.state_at(traj.end_time()).unwrap();
        assert!(perp_dist(final_state, start, dir).abs() < 1e-2);
        assert!(wrap_angle(final_state[2] - theta).abs() < 1e-2);
        assert!((colinear_vel(final_state, dir) - line_vel).abs() < 1e-2);
    }

    #[test]
    fn stays_on_line_at_speed_after_end() {
        let dir = Vector2f::new(1.0, 0.0);
        let start = Vector2f::new(0.0, 0.0);
        let init = Vector6f::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        let line_vel = 1.5;
        let traj = LinearTrajectory::from_line(init, 0.0, start, dir, line_vel, test_params()).unwrap();
        let end = traj.end_time();
        // Well past the end the robot remains on the line at target speed.
        let late = traj.state_at(end + 2.0).unwrap();
        assert!(perp_dist(late, start, dir).abs() < 1e-2);
        assert!((colinear_vel(late, dir) - line_vel).abs() < 1e-2);
        // And it has advanced along the line.
        let at_end = traj.state_at(end).unwrap();
        assert!(late[0] - at_end[0] > 1.0);
    }

    #[test]
    fn tick_advances_state_consistently() {
        let dir = Vector2f::new(1.0, 0.0);
        let start = Vector2f::new(0.0, 0.0);
        let init = Vector6f::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        let mut traj = LinearTrajectory::from_line(init, 0.0, start, dir, 1.5, test_params()).unwrap();
        let predicted = traj.state_at(0.01).unwrap();
        traj.tick(0.01);
        let (sampled, _) = traj.sample();
        for i in 0..6 {
            assert!((sampled[i] - predicted[i]).abs() < 1e-4);
        }
    }

    // --- from_point (face a global point) ---

    /// Line-of-sight angle from a state's position to the target point.
    fn los_angle(state: Vector6f, point: Vector2f) -> f32 {
        libm::atan2f(point.y - state[1], point.x - state[0])
    }

    #[test]
    fn from_point_rejects_start_on_point() {
        let init = Vector6f::new(1.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        let res = LinearTrajectory::from_point(
            init, 1.0, 1.0,
            Vector2f::new(0.0, 0.0), Vector2f::new(1.0, 0.0), 1.0, test_params(),
        );
        assert_eq!(res.err(), Some(ControlsError::InvalidInput));
    }

    #[test]
    fn from_point_position_matches_from_line() {
        // The translational path must be identical regardless of heading mode.
        let dir = Vector2f::new(1.0, 0.0);
        let start = Vector2f::new(0.0, 0.0);
        let init = Vector6f::new(-0.5, 1.0, 0.3, 0.2, 0.0, 0.0);
        let line_vel = 1.5;
        let by_line = LinearTrajectory::from_line(init, 0.0, start, dir, line_vel, test_params()).unwrap();
        let by_point = LinearTrajectory::from_point(init, 3.0, -2.0, start, dir, line_vel, test_params()).unwrap();
        for k in 0..40 {
            let t = 0.1 * k as f32;
            let a = by_line.state_at(t).unwrap();
            let b = by_point.state_at(t).unwrap();
            // x, y, vx, vy match (indices 0,1,3,4); heading (2,5) is allowed to differ.
            for &i in &[0usize, 1, 3, 4] {
                assert!((a[i] - b[i]).abs() < 1e-4, "idx {i} t {t}: {} vs {}", a[i], b[i]);
            }
        }
    }

    #[test]
    fn from_point_acquires_and_tracks() {
        // Robot offset from a line along +x, point off to the side. Start facing
        // the wrong way so there is a real acquisition phase.
        let dir = Vector2f::new(1.0, 0.0);
        let start = Vector2f::new(0.0, 0.0);
        let point = Vector2f::new(2.0, 2.0);
        let init = Vector6f::new(0.0, 1.0, -2.0, 0.0, 0.0, 0.0);
        let traj = LinearTrajectory::from_point(
            init, point.x, point.y, start, dir, 1.0, test_params(),
        ).unwrap();

        // Acquisition completes within the horizon.
        let acq = traj.heading_end_time;
        assert!(acq > 0.0 && acq.is_finite());

        // After acquisition the heading tracks the line-of-sight lag-free.
        for k in 0..20 {
            let t = acq + 0.1 * k as f32;
            let st = traj.state_at(t).unwrap();
            let los = los_angle(st, point);
            assert!(wrap_angle(st[2] - los).abs() < 2e-2, "t {t}: theta {} los {}", st[2], los);
        }
    }

    #[test]
    fn from_point_respects_angular_velocity_limit() {
        // Tight angular velocity limit; the heading rate must never exceed it.
        let dir = Vector2f::new(1.0, 0.0);
        let start = Vector2f::new(0.0, 0.0);
        let point = Vector2f::new(1.0, 0.5);
        let init = Vector6f::new(0.0, 1.0, 3.0, 0.0, 0.0, 0.0); // facing way off
        let mut params = test_params();
        params.max_vel_angular = 2.0;
        let traj = LinearTrajectory::from_point(
            init, point.x, point.y, start, dir, 1.0, params,
        ).unwrap();
        for k in 0..60 {
            let t = 0.05 * k as f32;
            let st = traj.state_at(t).unwrap();
            assert!(st[5].abs() <= params.max_vel_angular + 1e-2, "t {t}: thetad {}", st[5]);
        }
    }

    #[test]
    fn from_point_already_aligned_stays_aligned() {
        // Start already facing the point with the matching LOS rate (~0 here since
        // the robot starts at rest): heading should remain locked on the point.
        let dir = Vector2f::new(1.0, 0.0);
        let start = Vector2f::new(0.0, 0.0);
        let point = Vector2f::new(0.0, 3.0);
        let init_theta = libm::atan2f(point.y - 1.0, point.x - 0.0);
        let init = Vector6f::new(0.0, 1.0, init_theta, 0.0, 0.0, 0.0);
        let traj = LinearTrajectory::from_point(
            init, point.x, point.y, start, dir, 1.0, test_params(),
        ).unwrap();
        for k in 0..15 {
            let t = 0.1 * k as f32;
            let st = traj.state_at(t).unwrap();
            let los = los_angle(st, point);
            assert!(wrap_angle(st[2] - los).abs() < 3e-2, "t {t}: theta {} los {}", st[2], los);
        }
    }

    #[test]
    fn from_point_tick_matches_state_at() {
        let dir = Vector2f::new(1.0, 0.0);
        let start = Vector2f::new(0.0, 0.0);
        let init = Vector6f::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        let mut traj = LinearTrajectory::from_point(
            init, 2.0, 2.0, start, dir, 1.5, test_params(),
        ).unwrap();
        let predicted = traj.state_at(DEFAULT_CONTROL_DT).unwrap();
        traj.tick(DEFAULT_CONTROL_DT);
        let (sampled, _) = traj.sample();
        for i in 0..6 {
            assert!((sampled[i] - predicted[i]).abs() < 1e-4, "idx {i}: {} vs {}", sampled[i], predicted[i]);
        }
    }

    #[test]
    fn from_point_steady_state_on_line_facing_point() {
        let dir = Vector2f::new(1.0, 0.0);
        let start = Vector2f::new(0.0, 0.0);
        let point = Vector2f::new(5.0, 3.0);
        let init = Vector6f::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        let line_vel = 1.0;
        let traj = LinearTrajectory::from_point(
            init, point.x, point.y, start, dir, line_vel, test_params(),
        ).unwrap();
        let end = traj.end_time();
        let st = traj.state_at(end + 1.0).unwrap();
        // On the line and at target colinear speed.
        assert!(perp_dist(st, start, dir).abs() < 1e-2);
        assert!((colinear_vel(st, dir) - line_vel).abs() < 1e-2);
        // Still facing the point.
        let los = los_angle(st, point);
        assert!(wrap_angle(st[2] - los).abs() < 3e-2);
    }
}
