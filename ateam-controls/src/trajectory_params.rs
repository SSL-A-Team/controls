pub use core::f32::consts::PI;


pub const ALLOWABLE_ERROR_POS: f32 = 0.001;  // m
pub const ALLOWABLE_ERROR_VEL: f32 = 0.001;  // m/s
pub const MAX_TRANSLATIONAL_ACCELERATION: f32 = 3.0;  // m/s^2
pub const MAX_TRANSLATIONAL_VELOCITY: f32 = 3.0;  // m/s
pub const MAX_ROTATIONAL_ACCELERATION: f32 = 2.0 * PI;  // rad/s^2
pub const MAX_ROTATIONAL_VELOCITY: f32 = 2.0 * PI;  // rad/s