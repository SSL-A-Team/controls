pub use core::f64::consts::PI;


pub const ALLOWABLE_ERROR_POS: f64 = 0.01;  // m
pub const ALLOWABLE_ERROR_VEL: f64 = 0.01;  // m/s
pub const MAX_TRANSLATIONAL_ACCELERATION: f64 = 1.0;  // m/s^2
pub const MAX_TRANSLATIONAL_VELOCITY: f64 = 1.0;  // m/s
pub const MAX_ROTATIONAL_ACCELERATION: f64 = 2.0 * PI;  // rad/s^2
pub const MAX_ROTATIONAL_VELOCITY: f64 = 2.0 * PI;  // rad/s