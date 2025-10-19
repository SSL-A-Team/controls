use core::f64::consts::PI;


pub const BODY_MASS: f64 = 2.0;  // kg
pub const BODY_MOMENT_Z: f64 = 0.0009;  // kg * m^2  // scalar moment of inertia around the z axis  // I = 0.5 * M * R^2  // 0.0009 = 0.5 * 2 * 0.03^2
pub const WHEEL_ANGLE_ALPHA: f64 = PI / 6.0;  // rad
pub const WHEEL_ANGLE_BETA: f64 = PI / 4.0;  // rad
pub const WHEEL_DISTANCE: f64 = 0.0814;  // m  // Distance from center of robot to center of wheel
pub const WHEEL_RADIUS: f64 = 0.030;  // m