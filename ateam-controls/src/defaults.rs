use core::f32::consts::PI;
use nalgebra::SMatrix;

// Control loop period
pub const DEFAULT_CONTROL_DT_US: u32 = 1000;  // 1ms

// Trajectory limits
pub const DEFAULT_MAX_VEL_LINEAR: f32 = 3.0;            // m/s
pub const DEFAULT_MAX_VEL_ANGULAR: f32 = 5.0 * PI;      // rad/s
pub const DEFAULT_MAX_ACCEL_LINEAR: f32 = 2.0;           // m/s^2
pub const DEFAULT_MAX_ACCEL_ANGULAR: f32 = 10.0 * PI;     // rad/s^2

// Pivot Params
pub const DEFAULT_PIVOT_ORBIT_MAX_VEL_ANGULAR: f32 = 10.0;                 // rad/s
pub const DEFAULT_PIVOT_ORBIT_MAX_ACCEL_ANGULAR: f32 = 7.0;                 // rad/s^2
pub const DEFAULT_PIVOT_ORBIT_RADIUS: f32 = 0.12;                 // m
pub const DEFAULT_PIVOT_ORBIT_INSET_ANGLE: f32 = 1.1;                 // rad
/// Slope of the linear map from the orbit's peak angular velocity (rad/s) to the
/// inset angle (rad) when `PivotParams::compute_inset_angle` is set. Larger peak
/// angular velocity ⇒ larger ball centrifugal force ⇒ more inset to lean into it.
pub const DEFAULT_PIVOT_INSET_ANGLE_PER_ANGULAR_VEL: f32 = 0.35;       // rad / (rad/s)

// Linear (line-following) trajectory limits
/// Perpendicular distance from the line (m) below which the robot begins
/// accelerating along the line toward the target colinear velocity.
pub const DEFAULT_LINEAR_COLINEAR_START_THRESH: f32 = 0.01;             // m

// Robot physical parameters
pub const DEFAULT_PHYS_ALPHA: f32 = PI / 6.0;            // 30 deg, front wheel angle
pub const DEFAULT_PHYS_BETA: f32 = PI / 4.0;             // 45 deg, back wheel angle
pub const DEFAULT_PHYS_L: f32 = 0.0814;                  // m, wheel distance to center
pub const DEFAULT_PHYS_R: f32 = 0.030;                   // m, wheel radius
pub const DEFAULT_PHYS_MASS: f32 = 2.7;                  // kg
pub const DEFAULT_PHYS_IZ: f32 = 0.008;                  // kg*m^2, moment of inertia
pub const DEFAULT_PHYS_MOTOR_TORQUE_CONSTANT: f32 = 0.0335;       // N*m/A
// Friction / output efficiency model after TIGERs Mannheim (model_fric.h). Linear
// terms are for sideways (local y) motion and scaled by
// RobotModel::friction_direction_factor otherwise. Robot 2 on carpet, after the
// wheel-speed and commutation fixes (motor-side force, i.e. friction / efficiency):
// kinetic (plateau_profile.py) ~5.3 N sideways-normalized (fwd 3.6-4.4, side
// 5.6-6.4, diag 2.6-3.8 N), speed independent 0.1-0.2 m/s (viscous = 0, as TIGERs);
// rotation 0.12 N*m kinetic, 0.21-0.26 N*m breakaway. Efficiency from open-loop
// acceleration steps (accel_steps.py): ~0.8-1.0 linear, ~0.8 rotation (TIGERs: 0.72).
// Nov 12, closed-loop 0.2 m moves at 3.5 A (least squares m*a = eff*F - fric):
// 0.78 forward, 0.73 sideways -> 0.75 (tracking error -40% vs 0.85).
// Coulomb terms below are efficiency * motor-side friction.
pub const DEFAULT_PHYS_COULOMB_XY: f32 = 4.5;     // N
pub const DEFAULT_PHYS_VISCOUS_XY: f32 = 0.0;     // N/(m/s)
pub const DEFAULT_PHYS_COULOMB_W: f32 = 0.1;      // N*m
pub const DEFAULT_PHYS_VISCOUS_W: f32 = 0.0;      // N*m/(rad/s)
pub const DEFAULT_PHYS_EFFICIENCY_XY: f32 = 0.75;
pub const DEFAULT_PHYS_EFFICIENCY_W: f32 = 0.8;




// Defines how far back in time the EKF should run within the buffered EKF
pub const DEFAULT_EKF_BUFF_LEN: usize = 192;
pub const EKF_STATE_LEN: usize = 5;
pub const EKF_INPUT_LEN: usize = 3;
pub const EKF_MEAS_LEN: usize = 3;
const DEFAULT_EKF_VISION_VAR_M: f32 = 0.05*0.05;
const DEFAULT_EKF_VISION_VAR_RAD: f32 = 0.1*0.1;
// Defines the measurement covariance matrix for the EKF
pub const DEFAULT_EKF_R: SMatrix<f32, EKF_MEAS_LEN, EKF_MEAS_LEN> = SMatrix::<f32, EKF_MEAS_LEN, EKF_MEAS_LEN>::new(
    DEFAULT_EKF_VISION_VAR_M, 0.,                       0.,
    0.,                       DEFAULT_EKF_VISION_VAR_M, 0.,
    0.,                       0.,                       DEFAULT_EKF_VISION_VAR_RAD,
);
const EKF_PROC_VAR_M: f32 = 0.001*0.001;  // m
const EKF_PROC_VAR_RAD: f32 = 0.001*0.001;  // rad
const EKF_PROC_VAR_MPS: f32 = 0.005*0.005;  // m/s
// Defines the process covariance matrix for the EKF
pub const DEFAULT_EKF_Q: SMatrix<f32, EKF_STATE_LEN, EKF_STATE_LEN> = SMatrix::<f32, EKF_STATE_LEN, EKF_STATE_LEN>::new(
    EKF_PROC_VAR_M,   0.,             0.,               0.,               0.,
    0.,               EKF_PROC_VAR_M, 0.,               0.,               0.,
    0.,               0.,             EKF_PROC_VAR_RAD, 0.,               0.,
    0.,               0.,             0.,               EKF_PROC_VAR_MPS, 0.,
    0.,               0.,             0.,               0.,               EKF_PROC_VAR_MPS,
);
pub const DEFAULT_EKF_CORR_COEF: f32 = 1.0;


pub const DEFAULT_VISION_BUFF_LEN: usize = 256;
// DEFAULT_VISION_ACCEPT_VARIANCE_M2 and DEFAULT_VISION_SEED_SAMPLES control the
// startup stability check: the robot must receive SEED_SAMPLES consecutive
// vision measurements whose positional standard deviation is below the
// threshold before the KF is seeded, implicitly requiring the robot to be
// near-stationary at boot.
pub const DEFAULT_VISION_ACCEPT_VARIANCE_M2: f32 = 0.03 * 0.03;  // 3 cm standard deviation
pub const DEFAULT_VISION_SEED_SAMPLES: usize = 20;
// Experimentally measured minimum latency from the time of camera capture to
// the time the robot receives the vision measurement. Used to determine the
// clock skew between the robot and the vision host. This is then used to place
// the vision measurement at the correct instant in the EKF buffer and throw out
// old measurements.
// Includes the camera pipeline before SSL-Vision stamps t_capture: t_capture -> PC
// is only 8-12 ms (+ up to 17 ms for the 60 Hz bridge, + radio), yet the image is
// older. Measured by aligning vision heading, at its EKF placement time, with gyro-
// and encoder-integrated heading during fast rotation (Nov 13, robot 2): 15 ms put
// vision 42 ms late, 87 ms 32 ms early, 97 ms 42 ms early -> 56 ms.
pub const DEFAULT_VISION_MIN_LATENCY_US: u64 = 56_000;  // 56 ms
// Applied ages then run ~60-120 ms (30 fps vision, 60 Hz commands)
pub const DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US: u64 = 130_000;  // 130 ms
// Timeout to consider vision as "inactive". This may stop the robot if the
// active control mode requires absolute vision positioning.
pub const DEFAULT_VISION_INACTIVE_THRESHOLD_US: u64 = 500_000;  // 500 ms
// TODO: Tighten slack with testing
pub const DEFAULT_VISION_STATE_ESTIMATE_ERROR_SLACK_M: f32 = 0.5;  // 0.5 m
// DEFAULT_VISION_ACCEPT_RADIUS_BASE_M is derived from the worst-case error between a valid
// vision measurement and the state estimate's dead-reckoned position:
// base_radius = v_max * t_latency + slack
pub const DEFAULT_VISION_ACCEPT_RADIUS_BASE_M: f32 = DEFAULT_MAX_VEL_LINEAR * (DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US as f32) * 1e-6 + DEFAULT_VISION_STATE_ESTIMATE_ERROR_SLACK_M;
// DEFAULT_VISION_ACCEPT_RADIUS_RATE_MPS is the expansion rate for accepting a
// vision measurement as valid after initialization or after vision loss
pub const DEFAULT_VISION_ACCEPT_RADIUS_RATE_MPS: f32 = 2.0;  // 2 m/s
// Robot angular velocity threshold measured from encoders to consider the robot at standstill, gyro measurement is used otherwise
pub const DEFAULT_ENCODER_STANDSTILL_RADPS: f32 = 0.01;  // 0.01 rad/s
// Robot linear speed threshold measured from encoders to consider the robot at standstill,
// which applies a zero-velocity update (ZUPT) to the EKF
pub const DEFAULT_ENCODER_STANDSTILL_MPS: f32 = 0.005;  // 5 mm/s
// Variance of the EKF zero-velocity pseudo-measurement on [vx, vy]
pub const DEFAULT_EKF_ZUPT_VAR_MPS2: f32 = 0.002*0.002;  // (m/s)^2