use libm::sqrtf;
use crate::{ControlsError, bangbang_trajectory::BangBangTraj1D};

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
