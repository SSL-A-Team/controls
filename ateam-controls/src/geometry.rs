use nalgebra::{Vector3, Vector4};
use crate::algebra::{Vector3C, Vector4C};


#[derive(Clone, Copy, Debug)]
pub struct Pose {
    pub position: Vector3<f32>,
    pub orientation: Vector4<f32>,
}

impl Pose {
    pub fn theta(&self) -> f32 {
        let siny_cosp = 2.0 * (self.orientation.w * self.orientation.z + self.orientation.x * self.orientation.y);
        let cosy_cosp = 1.0 - 2.0 * (self.orientation.y * self.orientation.y + self.orientation.z * self.orientation.z);
        libm::atan2f(siny_cosp, cosy_cosp)
    }

    /// Create a Pose from x, y, and theta values. x and y are cartesian
    /// coordinates for a point on the xy plane (z=0) and theta is the rotation
    /// about the z axis in radians.
    pub fn from_xy_theta(x: f32, y: f32, theta: f32) -> Self{
        let half = theta * 0.5;
        let sin_half = libm::sinf(half);
        let cos_half = libm::cosf(half);
        Self {
            position: Vector3::<f32>::new(x, y, 0.0),
            orientation: Vector4::<f32>::new(0.0, 0.0, sin_half, cos_half),
        }
    }

    /// Returns the x, y, and theta values of a pose. x and y are cartesian
    /// coordinates for a point on the xy plane (z=0) and theta is the rotation
    /// about the z axis in radians.
    pub fn to_xy_theta(&self) -> Vector3<f32> {
        Vector3::<f32>::new(self.position.x, self.position.y, self.theta())
    }
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            position: Vector3::default(),
            orientation: Vector4::<f32>::new(0.0, 0.0, 0.0, 1.0),
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct PoseC {
    pub position: Vector3C,
    pub orientation: Vector4C,
}

impl From<Pose> for PoseC {
    fn from(pose: Pose) -> Self {
        Self {
            position: pose.position.into(),
            orientation: pose.orientation.into(),
        }
    }
}

impl From<PoseC> for Pose {
    fn from(pose: PoseC) -> Self {
        Self {
            position: pose.position.into(),
            orientation: pose.orientation.into(),
        }
    }
}

#[derive(Clone, Copy, Default, Debug)]
pub struct Twist {
    pub linear: Vector3<f32>,
    pub angular: Vector3<f32>,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct TwistC {
    pub linear: Vector3C,
    pub angular: Vector3C,
}

impl From<Twist> for TwistC {
    fn from(twist: Twist) -> Self {
        Self {
            linear: twist.linear.into(),
            angular: twist.angular.into(),
        }
    }
}

impl From<TwistC> for Twist {
    fn from(twist: TwistC) -> Self {
        Self {
            linear: twist.linear.into(),
            angular: twist.angular.into(),
        }
    }
}

#[derive(Clone, Copy, Default, Debug)]
pub struct Accel {
    pub linear: Vector3<f32>,
    pub angular: Vector3<f32>,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct AccelC {
    pub linear: Vector3C,
    pub angular: Vector3C,
}

impl From<Accel> for AccelC {
    fn from(accel: Accel) -> Self {
        Self {
            linear: accel.linear.into(),
            angular: accel.angular.into(),
        }
    }
}

impl From<AccelC> for Accel {
    fn from(accel: AccelC) -> Self {
        Self {
            linear: accel.linear.into(),
            angular: accel.angular.into(),
        }
    }
}

#[derive(Clone, Copy, Default, Debug)]
pub struct Wrench {
    pub force: Vector3<f32>,
    pub torque: Vector3<f32>,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct WrenchC {
    pub force: Vector3C,
    pub torque: Vector3C,
}

impl From<Wrench> for WrenchC {
    fn from(wrench: Wrench) -> Self {
        Self {
            force: wrench.force.into(),
            torque: wrench.torque.into(),
        }
    }
}

impl From<WrenchC> for Wrench {
    fn from(wrench: WrenchC) -> Self {
        Self {
            force: wrench.force.into(),
            torque: wrench.torque.into(),
        }
    }
}

#[derive(Clone, Copy, Default, Debug)]
pub struct RigidBodyState {
    pub pose: Pose,
    pub twist: Twist,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct RigidBodyStateC {
    pub pose: PoseC,
    pub twist: TwistC,
}

impl From<RigidBodyState> for RigidBodyStateC {
    fn from(state: RigidBodyState) -> Self {
        Self {
            pose: state.pose.into(),
            twist: state.twist.into(),
        }
    }
}

impl From<RigidBodyStateC> for RigidBodyState {
    fn from(state: RigidBodyStateC) -> Self {
        Self {
            pose: state.pose.into(),
            twist: state.twist.into(),
        }
    }
}
