#![cfg_attr(not(feature = "std"), no_std)]

use nalgebra;

pub mod ctypes;
pub mod trajectory_params;
pub mod physical_params;
pub mod kalman_params;
pub mod bangbang_trajectory;
pub mod robot_model;

pub type Vector3f = nalgebra::Vector3<f32>;
pub type Vector4f = nalgebra::Vector4<f32>;
pub type Vector6f = nalgebra::Vector6<f32>;
pub type Vector8f = nalgebra::SVector<f32, 8>;
pub type Matrix3f = nalgebra::Matrix3<f32>;
pub type Matrix6f = nalgebra::Matrix6<f32>;
pub type Matrix8f = nalgebra::SMatrix<f32, 8, 8>;
pub type Matrix3x4f = nalgebra::Matrix3x4<f32>;
pub type Matrix4x3f = nalgebra::Matrix4x3<f32>;
pub type Matrix6x3f = nalgebra::Matrix6x3<f32>;
pub type Matrix8x6f = nalgebra::SMatrix<f32, 8, 6>;
pub type Rotation3f = nalgebra::Rotation3<f32>;

use libm::{sinf, cosf, atan2f};


pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

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

/// Wrap an angle to the range [-pi, pi]
pub fn wrap_angle(angle: f32) -> f32 {
    atan2f(sinf(angle), cosf(angle))
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
