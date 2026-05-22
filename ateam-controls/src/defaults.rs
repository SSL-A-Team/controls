use core::f32::consts::PI;

// Control loop period
pub const DEFAULT_CONTROL_DT: f32 = 0.001;              // s (1ms, 1kHz control loop)

// Trajectory limits
pub const DEFAULT_MAX_VEL_LINEAR: f32 = 3.0;            // m/s
pub const DEFAULT_MAX_VEL_ANGULAR: f32 = 3.0 * PI;      // rad/s
pub const DEFAULT_MAX_ACCEL_LINEAR: f32 = 2.0;           // m/s^2
pub const DEFAULT_MAX_ACCEL_ANGULAR: f32 = 2.0 * PI;     // rad/s^2

// Kalman filter noise standard deviations
pub const DEFAULT_KF_PROCESS_STD_POS_LINEAR: f32 = 0.01;
pub const DEFAULT_KF_PROCESS_STD_POS_ANGULAR: f32 = 0.05;
pub const DEFAULT_KF_PROCESS_STD_VEL_LINEAR: f32 = 0.02;
pub const DEFAULT_KF_PROCESS_STD_VEL_ANGULAR: f32 = 0.1;
pub const DEFAULT_KF_MEASUREMENT_STD_VISION_POS_LINEAR: f32 = 1.0;
pub const DEFAULT_KF_MEASUREMENT_STD_VISION_POS_ANGULAR: f32 = 3.14;
pub const DEFAULT_KF_MEASUREMENT_STD_ENCODER_VEL_ANGULAR: f32 = 50.0;
pub const DEFAULT_KF_MEASUREMENT_STD_GYRO_VEL_ANGULAR: f32 = 0.015;

// Kalman filter max state values (for covariance initialization)
pub const DEFAULT_KF_MAX_POS_LINEAR: f32 = 64.0;         // m (half-field)
pub const DEFAULT_KF_MAX_POS_ANGULAR: f32 = 3.14;        // rad
pub const DEFAULT_KF_MAX_VEL_LINEAR: f32 = 3.0;          // m/s
pub const DEFAULT_KF_MAX_VEL_ANGULAR: f32 = 3.0 * PI;    // rad/s

// Robot physical parameters
pub const DEFAULT_PHYS_ALPHA: f32 = PI / 6.0;            // 30 deg, front wheel angle
pub const DEFAULT_PHYS_BETA: f32 = PI / 4.0;             // 45 deg, back wheel angle
pub const DEFAULT_PHYS_L: f32 = 0.0814;                  // m, wheel distance to center
pub const DEFAULT_PHYS_R: f32 = 0.030;                   // m, wheel radius
pub const DEFAULT_PHYS_MASS: f32 = 2.7;                  // kg
pub const DEFAULT_PHYS_IZ: f32 = 0.008;                  // kg*m^2, moment of inertia
pub const DEFAULT_PHYS_MOTOR_TORQUE_CONSTANT: f32 = 0.0335;       // N*m/A
pub const DEFAULT_PHYS_MOTOR_EFFICIENCY_FACTOR: f32 = 13.0;
pub const DEFAULT_PHYS_COULOMB_FRICTION_LINEAR: f32 = 2.058;
pub const DEFAULT_PHYS_COULOMB_FRICTION_ANGULAR: f32 = 0.05;
pub const DEFAULT_PHYS_VISCOUS_FRICTION_LINEAR: f32 = 5.0;
pub const DEFAULT_PHYS_VISCOUS_FRICTION_ANGULAR: f32 = 0.0063;
