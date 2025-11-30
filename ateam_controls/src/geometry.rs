use nalgebra;

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl From<nalgebra::Vector3<f64>> for Vector3 {
    fn from(v: nalgebra::Vector3<f64>) -> Self {
        Vector3 {x: v.x, y: v.y, z: v.z}
    }
}

impl From<Vector3> for nalgebra::Vector3<f64> {
    fn from(v: Vector3) -> Self {
        Self::new(v.x, v.y, v.z)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Vector4 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl From<nalgebra::Vector4<f64>> for Vector4 {
    fn from(v: nalgebra::Vector4<f64>) -> Self {
        Vector4 {x: v.x, y: v.y, z: v.z, w: v.w}
    }
}

impl From<Vector4> for nalgebra::Vector4<f64> {
    fn from(v: Vector4) -> Self {
        Self::new(v.x, v.y, v.z, v.w)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Pose {
    pub position: Vector3,
    pub orientation: Vector4,
}

impl Pose {
    /// Create a Pose from x, y, and yaw values. x and y are cartesian
    /// coordinates for a point on the xy plane (z=0) and yaw is the rotation
    /// about the z axis in radians.
    pub fn from_xy_yaw(x: f64, y: f64, yaw: f64) -> Self{
        let half = yaw * 0.5;
        let sin_half = libm::sin(half);
        let cos_half = libm::cos(half);
        Self {
            position: Vector3 {x: x, y: y, z: 0.0},
            orientation: Vector4 {x: 0.0, y: 0.0, z: sin_half, w: cos_half}
        }
    }

    /// Returns the x, y, and yaw values of a pose. x and y are cartesian
    /// coordinates for a point on the xy plane (z=0) and yaw is the rotation
    /// about the z axis in radians.
    pub fn to_xy_yaw(&self) -> Vector3 {
        let siny_cosp = 2.0 * (self.orientation.w * self.orientation.z + self.orientation.x * self.orientation.y);
        let cosy_cosp = 1.0 - 2.0 * (self.orientation.y * self.orientation.y + self.orientation.z * self.orientation.z);
        let yaw = libm::atan2(siny_cosp, cosy_cosp);
        Vector3 { 
            x: self.position.x,
            y: self.position.y,
            z: yaw,
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