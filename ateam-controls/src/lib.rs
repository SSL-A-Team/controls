#![cfg_attr(not(feature = "std"), no_std)]

use nalgebra;

pub mod ctypes;
pub mod bangbang_trajectory;
pub mod defaults;
pub mod pivot_trajectory;
pub mod linear_trajectory;
pub mod robot_model;
pub mod trajectory;

pub type Vector1f = nalgebra::Vector1<f32>;
pub type Vector2f = nalgebra::Vector2<f32>;
pub type Vector3f = nalgebra::Vector3<f32>;
pub type Vector4f = nalgebra::Vector4<f32>;
pub type Vector5f = nalgebra::Vector5<f32>;
pub type Vector6f = nalgebra::Vector6<f32>;
pub type Vector7f = nalgebra::SVector<f32, 7>;
pub type Vector8f = nalgebra::SVector<f32, 8>;
pub type Matrix1f = nalgebra::Matrix1<f32>;
pub type Matrix2f = nalgebra::Matrix2<f32>;
pub type Matrix3f = nalgebra::Matrix3<f32>;
pub type Matrix4f = nalgebra::Matrix4<f32>;
pub type Matrix5f = nalgebra::Matrix5<f32>;
pub type Matrix6f = nalgebra::Matrix6<f32>;
pub type Matrix7f = nalgebra::SMatrix<f32, 7, 7>;
pub type Matrix8f = nalgebra::SMatrix<f32, 8, 8>;
pub type Matrix3x4f = nalgebra::Matrix3x4<f32>;
pub type Matrix4x3f = nalgebra::Matrix4x3<f32>;
pub type Matrix6x3f = nalgebra::Matrix6x3<f32>;
pub type Matrix8x6f = nalgebra::SMatrix<f32, 8, 6>;
pub type Rotation3f = nalgebra::Rotation3<f32>;

use libm::{sinf, cosf, atan2f};


/// Rotation matrix around z axis by theta radians
pub fn z_rotation_mat(theta: f32) -> Matrix3f {
    let c = cosf(theta);
    let s = sinf(theta);
    Matrix3f::new(
        c  , -s  , 0.0,
        s  ,  c  , 0.0,
        0.0,  0.0, 1.0
    )
}

/// Wrap an angle to the range [-pi, pi]
pub fn wrap_angle(angle: f32) -> f32 {
    atan2f(sinf(angle), cosf(angle))
}

/// Error type for ateam-controls operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum ControlsError {
    /// Input values had invalid signs or magnitudes
    InvalidInput = -1,
    /// A required matrix inversion failed (singular matrix)
    SingularMatrix = -2,
    /// Trajectory solver found no valid solution
    NoSolution = -3,
    /// Requested evaluation at an invalid time
    InvalidTime = -4,
    /// Target exceeds configured limits
    ExceedsLimits = -5,
}

impl ControlsError {
    pub fn as_str(&self) -> &'static str {
        match self {
            ControlsError::InvalidInput => "ControlsError::InvalidInput",
            ControlsError::SingularMatrix => "ControlsError::SingularMatrix",
            ControlsError::NoSolution => "ControlsError::NoSolution",
            ControlsError::InvalidTime => "ControlsError::InvalidTime",
            ControlsError::ExceedsLimits => "ControlsError::ExceedsLimits",
        }
    }
}

impl core::fmt::Display for ControlsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ControlsError::InvalidInput => write!(f, "Invalid input parameters"),
            ControlsError::SingularMatrix => write!(f, "Singular matrix inversion failed"),
            ControlsError::NoSolution => write!(f, "No solution found"),
            ControlsError::InvalidTime => write!(f, "Invalid time for trajectory evaluation"),
            ControlsError::ExceedsLimits => write!(f, "Value exceeds configured limits"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for ControlsError {}

#[cfg(feature = "defmt")]
impl defmt::Format for ControlsError {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            ControlsError::InvalidInput => defmt::write!(fmt, "InvalidInput"),
            ControlsError::SingularMatrix => defmt::write!(fmt, "SingularMatrix"),
            ControlsError::NoSolution => defmt::write!(fmt, "NoSolution"),
            ControlsError::InvalidTime => defmt::write!(fmt, "InvalidTime"),
            ControlsError::ExceedsLimits => defmt::write!(fmt, "ExceedsLimits"),
        }
    }
}

