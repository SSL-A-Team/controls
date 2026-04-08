// Kalman filter params
pub const PROCESS_NOISE_STDDEV_POS_LINEAR: f32 = 0.01;  // m
pub const PROCESS_NOISE_STDDEV_POS_ANGULAR: f32 = 0.05;  // rad
pub const PROCESS_NOISE_STDDEV_VEL_LINEAR: f32 = 0.02;  // m/s
pub const PROCESS_NOISE_STDDEV_VEL_ANGULAR: f32 = 0.1;  // rad/s
pub const MEASUREMENT_NOISE_STDDEV_VISION_POS_LINEAR: f32 = 1.0;  // m  (HIGH LAG)
pub const MEASUREMENT_NOISE_STDDEV_VISION_POS_ANGULAR: f32 = 3.14;  // rad  (HIGH LAG)
pub const MEASUREMENT_NOISE_STDDEV_ENCODER_VEL_ANGULAR: f32 = 0.5;  // rad/s
pub const MEASUREMENT_NOISE_STDDEV_GYRO_VEL_ANGULAR: f32 = 0.015;  // rad/s
pub const MAX_POS_LINEAR: f32 = 64.0;  // m
pub const MAX_POS_ANGULAR: f32 = 3.14;  // rad
pub const MAX_VEL_LINEAR: f32 = 3.0;  // m/s
pub const MAX_VEL_ANGULAR: f32 = 20.0;  // rad/s
