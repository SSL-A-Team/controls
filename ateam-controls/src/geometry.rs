use nalgebra;
use crate::algebra::{Vector3C, Vector4C}


#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Pose {
    pub position: Vector3,
    pub orientation: Vector4,
}

impl Pose {
    pub fn theta(&self) -> f32 {
        let siny_cosp = 2.0 * (self.orientation.w * self.orientation.z + self.orientation.x * self.orientation.y);
        let cosy_cosp = 1.0 - 2.0 * (self.orientation.y * self.orientation.y + self.orientation.z * self.orientation.z);
        libm::atan2(siny_cosp, cosy_cosp)
    }

    /// Create a Pose from x, y, and theta values. x and y are cartesian
    /// coordinates for a point on the xy plane (z=0) and theta is the rotation
    /// about the z axis in radians.
    pub fn from_xy_theta(x: f32, y: f32, theta: f32) -> Self{
        let half = theta * 0.5;
        let sin_half = libm::sin(half);
        let cos_half = libm::cos(half);
        Self {
            position: Vector3 {x: x, y: y, z: 0.0},
            orientation: Vector4 {x: 0.0, y: 0.0, z: sin_half, w: cos_half}
        }
    }

    /// Returns the x, y, and theta values of a pose. x and y are cartesian
    /// coordinates for a point on the xy plane (z=0) and theta is the rotation
    /// about the z axis in radians.
    pub fn to_xy_theta(&self) -> Vector3 {
        Vector3 { 
            x: self.position.x,
            y: self.position.y,
            z: self.theta(),
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Accel {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Wrench {
    pub force: Vector3,
    pub torque: Vector3,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct RigidBodyState {
    pub pose: Pose,
    pub twist: Twist,
}