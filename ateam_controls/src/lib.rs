pub use std::f64::consts::PI;
pub const ALLOWABLE_ERROR_POS: f64 = 0.01;  // m
pub const ALLOWABLE_ERROR_VEL: f64 = 0.01;  // m/s
pub const MAX_TRANSLATIONAL_ACCELERATION: f64 = 1.0;  // m/s^2
pub const MAX_TRANSLATIONAL_VELOCITY: f64 = 1.0;  // m/s
pub const MAX_ROTATIONAL_ACCELERATION: f64 = 2.0 * PI;  // rad/s^2
pub const MAX_ROTATIONAL_VELOCITY: f64 = 2.0 * PI;  // rad/s

#[derive(Clone, Copy, Default, Debug)]
pub struct GlobalState {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub xd: f64,
    pub yd: f64,
    pub zd: f64,
}

#[derive(Clone, Copy, Default, Debug)]
pub struct GlobalControl {
    pub xdd: f64,
    pub ydd: f64,
    pub zdd: f64,
}

/// xdd_1: t_1 -> t_2, xdd_2: t_2 -> t_3, xdd_3: t_3 -> t_4
#[derive(Clone, Copy, Default, Debug)]
pub struct Trajectory1D {
    pub xdd_1: f64,
    pub xdd_2: f64,
    pub xdd_3: f64,
    pub t_1: f64,
    pub t_2: f64,
    pub t_3: f64,
    pub t_4: f64,
}

impl Trajectory1D {
    pub fn time_shift(&mut self, dt: f64) {
        self.t_1 += dt;
        self.t_2 += dt;
        self.t_3 += dt;
        self.t_4 += dt;
    }
}

#[derive(Clone, Copy, Default, Debug)]
pub struct Trajectory3D {
    pub x_traj: Trajectory1D,
    pub y_traj: Trajectory1D,
    pub z_traj: Trajectory1D,
}

impl Trajectory3D {
    pub fn time_shift(&mut self, dt: f64) {
        self.x_traj.time_shift(dt);
        self.y_traj.time_shift(dt);
        self.z_traj.time_shift(dt);
    }
}

/// Takes the initial velocity xd0, the desired positive change in position dx, and the bang-bang acceleration xdd
/// Returns the positive acceleration time T1, the negative acceleration time T2, and the peak velocity reached Vpeak
pub fn compute_positive_triangular_profile(xd0: f64, dx: f64, xdd: f64) -> (f64, f64, f64) {
    if xd0 < 0.0 || dx < 0.0 || xdd < 0.0 {
        panic!("compute_positive_triangular_profile: All values should be positive")
    }

    let t2 = ((xdd * dx + 0.5 * xd0 * xd0) / (xdd * xdd)).sqrt();
    let t1 = t2 - xd0 / xdd;
    let vpeak = xdd * t2;
    (t1, t2, vpeak)
}

/// Takes the initial velocity xd0, the desired positive change in position dx, the bang-bang acceleration xdd, and the max velocity contraint xdmax
/// Returns the acceleration or deceleration time T1, the coasting time T2, the negative acceleration time T3, and the acceleration used for T1 (xdd1)
pub fn compute_positive_trapezoidal_profile(xd0: f64, dx: f64, xdd: f64, xdmax: f64) -> (f64, f64, f64, f64) {
    if xd0 < 0.0 || dx <= 0.0 || xdd <= 0.0 || xdmax <= 0.0 {  // allow xd0 to be zero
        panic!("compute_positive_triangular_profile: All values should be positive")
    }

    // The first period of time can either be acceleration to max velocity or deceleration to max velocity
    let t1;
    let d1;
    let xdd1;
    if xd0 > xdmax {
        t1 = (xd0 - xdmax) / xdd;  // deceleration time
        d1 = xdmax * t1 + 0.5 * (xd0 - xdmax) * t1;  // distance traveled during deceleration
        xdd1 = -xdmax;
    } else {
        t1 = (xdmax - xd0) / xdd;  // acceleration time
        d1 = xd0 * t1 + 0.5 * (xdmax - xd0) * t1;  // distance traveled during acceleration
        xdd1 = xdmax;
    }
    let t3 = xdmax / xdd;  // breaking time
    let d3 = 0.5 * xdmax * t3;  // distance traveled during the deceleration
    let d2 = dx - d1 - d3;  // distance traveled during the coast
    let t2 = d2 / xdmax;  // coasting time
    if t1 < 0.0 || t2 < 0.0 || t3 < 0.0 {
        panic!("compute_positive_trapezoidal_profile: No solution found")
    }
    return (t1, t2, t3, xdd1)
}

/// Returns the time it took to break and the resulting position
pub fn compute_break(x0: f64, xd0: f64, xdd: f64) -> (f64, f64) {
    let time_to_rest = xd0.abs() / xdd.abs();
    let xf = x0 + 0.5 * xd0 * time_to_rest;
    (time_to_rest, xf)
}

pub fn compute_optimal_traj_1d(x0: f64, xd0: f64, xtrg: f64, xdmax: f64, xddmax: f64) -> Trajectory1D {
    if xddmax <= 0.0 || xdmax <= 0.0 {
        panic!("compute_optimal_traj_1d: Can't compute trajectory when max velocity or acceleration is 0.0")
    }

    let mut traj = Trajectory1D::default();
    let mut x = x0;
    let mut xd = xd0;

    // First check if x will need to turn around (change sign in velocity)
    if xd != 0.0 {
        // a. check if initial velocity is in the wrong direction
        // b. check if full break will overshoot xtrg
        let (break_time, x_after_full_break) = compute_break(x, xd, xddmax);
        // Check if full break position is outside the segment between x0 and xtrg
        if x_after_full_break != xtrg && (x_after_full_break - x).is_sign_positive() == (x_after_full_break - xtrg).is_sign_positive() {
            // The trajectory requires a full break to rest immediately to turn around
            traj.xdd_1 = - xd.signum() * xddmax;  // acceleration should be in opposite direction of initial velocity
            traj.t_2 += break_time;
            traj.t_3 += break_time;
            traj.t_4 += break_time;
            x = x_after_full_break;
            xd = 0.0;
        }
    }

    // Solve triangular profile and check if unconstrained velocity is greater than xdmax
    let dx = xtrg - x;
    let direction = dx.signum();
    let (t1, t2, vpeak) = compute_positive_triangular_profile(xd.abs(), dx.abs(), xddmax);
    if vpeak < xdmax {

        // accelerate towards target
        traj.xdd_1 = direction * xddmax;  // NOTE: this could have already been set if break to zero was added in the beginning of the trajectory, but the acceleration should be in the same direction here if that is the case
        traj.t_2 += t1;
        traj.t_3 += t1;
        traj.t_4 += t1;

        // Skip the coasting time period
        // traj.xdd_2 = 0.0;
        // traj.t_3 += 0.0;
        // traj.t_4 += 0.0;

        // break away from target
        traj.xdd_3 = - direction * xddmax;
        traj.t_4 += t2;

    } else {
        // Solve trapezoidal profile
        let (t1, t2, t3, xdd1) = compute_positive_trapezoidal_profile(xd.abs(), dx.abs(), xddmax, xdmax);

        // accelerate or decelerate to max velocity
        traj.xdd_1 = direction * xdd1;  // NOTE: this could have already been set if break to zero was added in the beginning of the trajectory, but the acceleration should be in the same direction here if that is the case
        traj.t_2 += t1;
        traj.t_3 += t1;
        traj.t_4 += t1;

        // coast
        traj.xdd_2 = 0.0;
        traj.t_3 += t2;
        traj.t_4 += t2;

        // break away from target
        traj.xdd_3 = - direction * xddmax;
        traj.t_4 += t3;
    }
    
    traj
}

pub fn compute_optimal_traj_3d(init_state: GlobalState, target_state: GlobalState) -> Trajectory3D {
    let mut alpha = PI / 4.0;
    let mut increment = PI / 8.0;
    let precision = 0.0000001;
    let mut x_traj;
    let mut y_traj;
    loop {
        let cos_alpha = alpha.cos();
        let sin_alpha = alpha.sin();
        // x_traj = compute_optimal_traj_1d(current_state.x, current_state.xd, 0.0, cos_alpha * MAX_TRANSLATIONAL_ACCELERATION, (PI / 4.0).cos() * MAX_TRANSLATIONAL_VELOCITY, current_time);
        // y_traj = compute_optimal_traj_1d(current_state.y, current_state.yd, 0.0, sin_alpha * MAX_TRANSLATIONAL_ACCELERATION, (PI / 4.0).sin() * MAX_TRANSLATIONAL_VELOCITY, current_time);
        x_traj = compute_optimal_traj_1d(init_state.x, init_state.xd, target_state.x, cos_alpha * MAX_TRANSLATIONAL_ACCELERATION, cos_alpha * MAX_TRANSLATIONAL_VELOCITY);
        y_traj = compute_optimal_traj_1d(init_state.y, init_state.yd, target_state.y, sin_alpha * MAX_TRANSLATIONAL_ACCELERATION, sin_alpha * MAX_TRANSLATIONAL_VELOCITY);
        if x_traj.t_4 > y_traj.t_4 {
            alpha -= increment;
        } else {
            alpha += increment;
        }
        if increment <= precision {
            break
        }
        increment *= 0.5;
    }
    let traj = Trajectory3D { 
        x_traj: x_traj,
        y_traj: y_traj,
        z_traj: compute_optimal_traj_1d(init_state.z, init_state.zd, target_state.z, MAX_ROTATIONAL_ACCELERATION, MAX_ROTATIONAL_VELOCITY),
    };
    return traj;
}

pub fn compute_traj1d_state_at_t(traj: Trajectory1D, x: f64, xd: f64, current_time: f64, t: f64) -> (f64, f64) {
    let mut x = x;
    let mut xd = xd;
    let mut current_time = current_time;

    if t < current_time {
        panic!("Unable to compute state of trajectory in the past");
    }
    if traj.t_1 > current_time {
        panic!("Unable to compute state of trajectory that hasn't started");
    }

    for (part_end_time, part_acceleration) in [(traj.t_2, traj.xdd_1), (traj.t_3, traj.xdd_2), (traj.t_4, traj.xdd_3)] {
        if current_time < part_end_time {
            if t < part_end_time {
                let time_in_part = t - current_time;
                x += xd * time_in_part + 0.5 * part_acceleration * time_in_part * time_in_part;  // v * t + 1/2 * a * t^2
                xd += part_acceleration * time_in_part;  // a * t
                return (x, xd)  // reached the desired time
            } else {
                let time_in_part = part_end_time - current_time;
                x += xd * time_in_part + 0.5 * part_acceleration * time_in_part * time_in_part;  // v * t + 1/2 * a * t^2
                xd += part_acceleration * time_in_part;  // a * t
                current_time = part_end_time;
            }
        }
    }
    return (x, xd)  // t was equal to or after the trajectory end time
}

pub fn compute_traj3d_state_at_t(traj: Trajectory3D, current_state: GlobalState, current_time: f64, t: f64) -> GlobalState {
    let (x_f, xd_f) = compute_traj1d_state_at_t(traj.x_traj, current_state.x, current_state.xd, current_time, t);
    let (y_f, yd_f) = compute_traj1d_state_at_t(traj.y_traj, current_state.y, current_state.yd, current_time, t);
    let (z_f, zd_f) = compute_traj1d_state_at_t(traj.z_traj, current_state.z, current_state.zd, current_time, t);
    GlobalState { x: x_f, y: y_f, z: z_f, xd: xd_f, yd: yd_f, zd: zd_f }
}