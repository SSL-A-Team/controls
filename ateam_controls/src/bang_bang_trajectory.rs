use crate::trajectory_params::*;
use crate::GlobalState;
use core::f64::consts::PI;


/// sdd1: t1 -> t2, sdd2: t2 -> t3, sdd3: t3 -> t4
#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct BangBangTraj1D {
    pub sdd1: f64,
    pub sdd2: f64,
    pub sdd3: f64,
    pub t1: f64,
    pub t2: f64,
    pub t3: f64,
    pub t4: f64,
}

impl BangBangTraj1D {
    pub fn time_shift(&mut self, dt: f64) {
        self.t1 += dt;
        self.t2 += dt;
        self.t3 += dt;
        self.t4 += dt;
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct BangBangTraj3D {
    pub x_traj: BangBangTraj1D,
    pub y_traj: BangBangTraj1D,
    pub z_traj: BangBangTraj1D,
}

impl BangBangTraj3D {
    pub fn time_shift(&mut self, dt: f64) {
        self.x_traj.time_shift(dt);
        self.y_traj.time_shift(dt);
        self.z_traj.time_shift(dt);
    }
}

pub fn compute_optimal_bang_bang_traj_3d(init_state: GlobalState, target_state: GlobalState) -> BangBangTraj3D {
    let mut alpha = PI / 4.0;
    let mut increment = PI / 8.0;
    let precision = 0.0000001;
    let mut x_traj;
    let mut y_traj;
    loop {
        let cos_alpha = alpha.cos();
        let sin_alpha = alpha.sin();
        x_traj = compute_bang_bang_traj_1d(init_state.x, init_state.xd, target_state.x, cos_alpha * MAX_TRANSLATIONAL_ACCELERATION, cos_alpha * MAX_TRANSLATIONAL_VELOCITY);
        y_traj = compute_bang_bang_traj_1d(init_state.y, init_state.yd, target_state.y, sin_alpha * MAX_TRANSLATIONAL_ACCELERATION, sin_alpha * MAX_TRANSLATIONAL_VELOCITY);
        // x_traj = compute_bang_bang_traj_1d(init_state.x, init_state.xd, target_state.x, cos_alpha * MAX_TRANSLATIONAL_ACCELERATION, (PI / 4.0).cos() * MAX_TRANSLATIONAL_VELOCITY);
        // y_traj = compute_bang_bang_traj_1d(init_state.y, init_state.yd, target_state.y, sin_alpha * MAX_TRANSLATIONAL_ACCELERATION, (PI / 4.0).sin() * MAX_TRANSLATIONAL_VELOCITY);
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
    let traj = BangBangTraj3D { 
        x_traj: x_traj,
        y_traj: y_traj,
        z_traj: compute_bang_bang_traj_1d(init_state.z, init_state.zd, target_state.z, MAX_ROTATIONAL_ACCELERATION, MAX_ROTATIONAL_VELOCITY),
    };
    return traj;
}

pub fn compute_bang_bang_traj_3d_state_at_t(traj: BangBangTraj3D, current_state: GlobalState, current_time: f64, t: f64) -> GlobalState {
    let (x_f, xd_f) = compute_bang_bang_traj_1d_state_at_t(traj.x_traj, current_state.x, current_state.xd, current_time, t);
    let (y_f, yd_f) = compute_bang_bang_traj_1d_state_at_t(traj.y_traj, current_state.y, current_state.yd, current_time, t);
    let (z_f, zd_f) = compute_bang_bang_traj_1d_state_at_t(traj.z_traj, current_state.z, current_state.zd, current_time, t);
    GlobalState { x: x_f, y: y_f, z: z_f, xd: xd_f, yd: yd_f, zd: zd_f }
}

/// Takes the initial velocity sd0, the desired positive change in position ds, and the bang-bang acceleration sdd
/// Returns the positive acceleration time T1, the negative acceleration time T2, and the peak velocity reached Vpeak
fn compute_positive_triangular_profile(sd0: f64, ds: f64, sdd: f64) -> (f64, f64, f64) {
    if sd0 < 0.0 || ds < 0.0 || sdd < 0.0 {
        panic!("compute_positive_triangular_profile: All values should be positive")
    }

    let t2 = ((sdd * ds + 0.5 * sd0 * sd0) / (sdd * sdd)).sqrt();
    let t1 = t2 - sd0 / sdd;
    let vpeak = sdd * t2;
    (t1, t2, vpeak)
}

/// Takes the initial velocity sd0, the desired positive change in position ds, the bang-bang acceleration sdd, and the max velocity contraint sd_max
/// Returns the acceleration or deceleration time T1, the coasting time T2, the negative acceleration time T3, and the acceleration used for T1 (sdd1)
fn compute_positive_trapezoidal_profile(sd0: f64, ds: f64, sdd: f64, sd_max: f64) -> (f64, f64, f64, f64) {
    if sd0 < 0.0 || ds <= 0.0 || sdd <= 0.0 || sd_max <= 0.0 {  // allow sd0 to be zero
        panic!("compute_positive_triangular_profile: All values should be positive")
    }

    // The first period of time can either be acceleration to max velocity or deceleration to max velocity
    let t1;
    let d1;
    let sdd1;
    if sd0 > sd_max {
        t1 = (sd0 - sd_max) / sdd;  // deceleration time
        d1 = sd_max * t1 + 0.5 * (sd0 - sd_max) * t1;  // distance traveled during deceleration
        sdd1 = -sd_max;
    } else {
        t1 = (sd_max - sd0) / sdd;  // acceleration time
        d1 = sd0 * t1 + 0.5 * (sd_max - sd0) * t1;  // distance traveled during acceleration
        sdd1 = sd_max;
    }
    let t3 = sd_max / sdd;  // breaking time
    let d3 = 0.5 * sd_max * t3;  // distance traveled during the deceleration
    let d2 = ds - d1 - d3;  // distance traveled during the coast
    let t2 = d2 / sd_max;  // coasting time
    if t1 < 0.0 || t2 < 0.0 || t3 < 0.0 {
        panic!("compute_positive_trapezoidal_profile: No solution found")
    }
    return (t1, t2, t3, sdd1)
}

/// Returns the time it took to break and the resulting position
fn compute_break(s0: f64, sd0: f64, sdd: f64) -> (f64, f64) {
    let time_to_rest = sd0.abs() / sdd.abs();
    let sf = s0 + 0.5 * sd0 * time_to_rest;
    (time_to_rest, sf)
}

fn compute_bang_bang_traj_1d(s0: f64, sd0: f64, s_trg: f64, sd_max: f64, sdd_max: f64) -> BangBangTraj1D {
    if sdd_max <= 0.0 || sd_max <= 0.0 {
        panic!("compute_optimal_traj1d: Can't compute trajectory when max velocity or acceleration is 0.0")
    }

    let mut traj = BangBangTraj1D::default();
    let mut s = s0;
    let mut sd = sd0;

    // First check if s will need to turn around (change sign in velocity)
    if sd != 0.0 {
        // a. check if initial velocity is in the wrong direction
        // b. check if full break will overshoot s_trg
        let (break_time, s_after_full_break) = compute_break(s, sd, sdd_max);
        // Check if full break position is outside the segment between s0 and s_trg
        if s_after_full_break != s_trg && (s_after_full_break - s).is_sign_positive() == (s_after_full_break - s_trg).is_sign_positive() {
            // The trajectory requires a full break to rest immediately to turn around
            traj.sdd1 = - sd.signum() * sdd_max;  // acceleration should be in opposite direction of initial velocity
            traj.t2 += break_time;
            traj.t3 += break_time;
            traj.t4 += break_time;
            s = s_after_full_break;
            sd = 0.0;
        }
    }

    // Solve triangular profile and check if unconstrained velocity is greater than sd_max
    let ds = s_trg - s;
    let direction = ds.signum();
    let (t1, t2, vpeak) = compute_positive_triangular_profile(sd.abs(), ds.abs(), sdd_max);
    if vpeak < sd_max {

        // accelerate towards target
        traj.sdd1 = direction * sdd_max;  // NOTE: this could have already been set if break to zero was added in the beginning of the trajectory, but the acceleration should be in the same direction here if that is the case
        traj.t2 += t1;
        traj.t3 += t1;
        traj.t4 += t1;

        // Skip the coasting time period
        // traj.sdd2 = 0.0;
        // traj.t3 += 0.0;
        // traj.t4 += 0.0;

        // break away from target
        traj.sdd3 = - direction * sdd_max;
        traj.t4 += t2;

    } else {
        // Solve trapezoidal profile
        let (t1, t2, t3, sdd1) = compute_positive_trapezoidal_profile(sd.abs(), ds.abs(), sdd_max, sd_max);

        // accelerate or decelerate to max velocity
        traj.sdd1 = direction * sdd1;  // NOTE: this could have already been set if break to zero was added in the beginning of the trajectory, but the acceleration should be in the same direction here if that is the case
        traj.t2 += t1;
        traj.t3 += t1;
        traj.t4 += t1;

        // coast
        traj.sdd2 = 0.0;
        traj.t3 += t2;
        traj.t4 += t2;

        // break away from target
        traj.sdd3 = - direction * sdd_max;
        traj.t4 += t3;
    }
    
    traj
}

fn compute_bang_bang_traj_1d_state_at_t(traj: BangBangTraj1D, s: f64, sd: f64, current_time: f64, t: f64) -> (f64, f64) {
    let mut s = s;
    let mut sd = sd;
    let mut current_time = current_time;

    if t < current_time {
        panic!("Unable to compute state of trajectory in the past");
    }
    if traj.t1 > current_time {
        panic!("Unable to compute state of trajectory that hasn't started");
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
                return (s, sd)  // reached the desired time
            } else {
                current_time = part_end_time;
            }
        }
    }
    return (s, sd)  // t was equal to or after the trajectory end time
}
