use core::f32::consts::PI;


pub const BODY_MASS: f32 = 2.0;  // kg
pub const BODY_MOMENT_Z: f32 = 0.0009;  // kg * m^2  // scalar moment of inertia around the z axis  // I = 0.5 * M * R^2  // 0.0009 = 0.5 * 2 * 0.03^2
pub const WHEEL_ANGLE_ALPHA: f32 = PI / 6.0;  // rad
pub const WHEEL_ANGLE_BETA: f32 = PI / 4.0;  // rad
pub const WHEEL_DISTANCE: f32 = 0.0814;  // m  // Distance from center of robot to center of wheel
pub const WHEEL_RADIUS: f32 = 0.030;  // m