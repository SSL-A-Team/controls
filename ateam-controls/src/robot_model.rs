use libm::{sinf, cosf};
use crate::{Matrix3f, Matrix3x4f, Matrix4x3f, Vector3f, Vector4f};
use crate::robot_physical_params::*;

/// Pose: Vector3
///     x - x linear position (m)
///     y - y linear position (m)
///     z - z angular orientation (rad)
/// Twist: Vector3
///     x - x linear velocity (m/s)
///     y - y linear velocity (m/s)
///     z - angular velocity around z axis (rad/s)
/// Accel: Vector3
///     x - x linear acceleration (m/s^2)
///     y - y linear acceleration (m/s^2)
///     z - angular aceleration around z axis (rad/s^2)
/// Wheel Velocities: Vector4
///     x - front left angular velocity (rad/s)
///     y - back left angular velocity (rad/s)
///     z - back right angular velocity (rad/s)
///     w - front right angular velocity (rad/s)
/// Wheel Torques: Vector4
///     x - front left torque (N*m)
///     y - back left torque (N*m)
///     z - back right torque (N*m)
///     w - front right torque (N*m)

/// Rotation matrix around z axis by theta radians
fn z_rotation_mat(theta: f32) -> Matrix3f {
    let c = cosf(theta);
    let s = sinf(theta);
    Matrix3f::new(
        c  , -s  , 0.0,
        s  ,  c  , 0.0,
        0.0,  0.0, 1.0
    )
}

/// Pseudo matrix inverse
fn pinv_3x4(a: Matrix3x4f) -> Matrix4x3f {
    let a_t = a.clone().transpose();
    let a_at = a * a_t.clone();
    let a_at_inv = a_at.try_inverse().unwrap();
    let a_inv = a_t * a_at_inv;
    a_inv
}

pub fn transform_frame_global2robot_pose(robot_pose: Vector3f, pose: Vector3f) -> Vector3f {
    z_rotation_mat(-robot_pose.z) * (pose - robot_pose)
}

pub fn transform_frame_robot2global_pose(robot_pose: Vector3f, pose: Vector3f) -> Vector3f {
    z_rotation_mat(robot_pose.z) * pose + robot_pose
}

pub fn transform_frame_global2robot_twist(robot_pose: Vector3f, twist: Vector3f) -> Vector3f {
    z_rotation_mat(-robot_pose.z) * twist
}

pub fn transform_frame_robot2global_twist(robot_pose: Vector3f, twist: Vector3f) -> Vector3f {
    z_rotation_mat(robot_pose.z) * twist
}

pub fn transform_frame_global2robot_accel(robot_pose: Vector3f, accel: Vector3f) -> Vector3f {
    z_rotation_mat(-robot_pose.z) * accel
}

pub fn transform_frame_robot2global_accel(robot_pose: Vector3f, accel: Vector3f) -> Vector3f {
    z_rotation_mat(robot_pose.z) * accel
}

pub struct RobotModel {
    pub wheel_transform_mat: Matrix3x4f,
    pub wheel_transform_mat_inv: Matrix4x3f,
    pub wheel_radius: f32,
    pub body_inirtia: Matrix3f,
    pub body_inirtia_inv: Matrix3f,
}

impl RobotModel {
    /// alpha: angle between robot y axis and segment between robot origin and robot front left wheel, same as angle between robot -y axis and segment between robot origin and robot front right wheel
    /// beta: angle between robot y axis and segment between robot origin and robot back left wheel, same as angle between robot -y axis and segment between robot origin and robot back right wheel
    /// l: distance from the robot center to each wheel
    /// r: radius of each wheel
    /// m: robot body mass
    /// i: robot body moment of inirtia around z axis
    pub fn new(alpha: f32, beta: f32, l: f32, r: f32, m: f32, i: f32) -> RobotModel {
        let wheel_transform_mat = Matrix3x4f::new(
            -cosf(alpha), -cosf(beta),  cosf(beta), cosf(alpha),
             sinf(alpha), -sinf(beta), -sinf(beta), sinf(alpha),
             l          ,  l         ,  l         , l
        );
        let body_inirtia = Matrix3f::new(
            m , 0., 0.,
            0., m , 0.,
            0., 0., i ,
        );
        RobotModel {
            wheel_transform_mat, 
            wheel_transform_mat_inv: pinv_3x4(wheel_transform_mat),
            wheel_radius: r,
            body_inirtia,
            body_inirtia_inv: body_inirtia.try_inverse().unwrap(),
        }
    }

    pub fn new_from_constants() -> RobotModel {
        RobotModel::new(WHEEL_ANGLE_ALPHA, WHEEL_ANGLE_BETA, WHEEL_DISTANCE, WHEEL_RADIUS, BODY_MASS, BODY_MOMENT_Z)
    }

    /// Calculate robot frame twist from wheel velocities
    pub fn wheel_velocities_to_twist(&self, wheel_velocities: Vector4f) -> Vector3f {
        self.wheel_transform_mat_inv.transpose() * wheel_velocities
    }

    /// Calculate wheel velocities from robot frame twist
    pub fn twist_to_wheel_velocities(&self, twist: Vector3f) -> Vector4f {
        self.wheel_transform_mat.transpose() * twist
    }

    /// Calculate robot frame accel from wheel torques
    pub fn wheel_torques_to_accel(&self, torques: Vector4f) -> Vector3f {
        self.body_inirtia_inv * self.wheel_transform_mat * torques / self.wheel_radius
    }

    // Calculate wheel torques from robot frame accel
    pub fn accel_to_wheel_torques(&self, accel: Vector3f) -> Vector4f {
        self.wheel_radius * self.wheel_transform_mat_inv * self.body_inirtia * accel
    }
}
