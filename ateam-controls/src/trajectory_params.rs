pub use core::f32::consts::PI;


pub const ALLOWABLE_ERROR_POS_LINEAR: f32 = 0.010;  // m
pub const ALLOWABLE_ERROR_POS_ANGULAR: f32 = 0.010;  // rad
pub const ALLOWABLE_ERROR_VEL_LINEAR: f32 = 0.010;  // m/s
pub const ALLOWABLE_ERROR_VEL_ANGULAR: f32 = 0.010;  // rad/s
pub const MAX_VEL_LINEAR: f32 = 3.0;  // m/s
pub const MAX_VEL_ANGULAR: f32 = 10.0;  // rad/s
pub const MAX_ACCEL_LINEAR: f32 = 8.0;  // m/s^2
pub const MAX_ACCEL_ANGULAR: f32 = 36.0;  // rad/s^2