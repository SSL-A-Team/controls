use core::f32::consts::PI;


pub const BODY_MASS: f32 = 2.7;  // kg
pub const BODY_MOMENT_Z: f32 = 0.009;  // kg * m^2  // scalar moment of inertia around the z axis  // I = 0.5 * M * R^2  // 0.009 = 0.5 * 2.7 * 0.0814^2
pub const WHEEL_ANGLE_ALPHA: f32 = PI / 6.0;  // rad
pub const WHEEL_ANGLE_BETA: f32 = PI / 4.0;  // rad
pub const WHEEL_DISTANCE: f32 = 0.0814;  // m  // Distance from center of robot to center of wheel
pub const WHEEL_RADIUS: f32 = 0.030;  // m
pub const MOTOR_TORQUE_CONSTANT: f32 = 0.0335;  // Nm/A
// TODO: this should be less than 1.0, need to test robot model
pub const MOTOR_EFFICIENCY_FACTOR: f32 = 5.1;  // unitless, empirically determined scaling factor to match observed robot performance
pub const COULOMB_FRICTION_COEFFICIENT_LINEAR: f32 = 0.0;  // N / (m/s)
pub const COULOMB_FRICTION_COEFFICIENT_ANGULAR: f32 = 0.0;  // Nm / (rad/s)
pub const VISCOUS_FRICTION_COEFFICIENT_LINEAR: f32 = 0.0;  // N / (m/s)
pub const VISCOUS_FRICTION_COEFFICIENT_ANGULAR: f32 = 0.0;  // Nm / (rad/s)