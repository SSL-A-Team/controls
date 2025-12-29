#![cfg_attr(not(feature = "std"), no_std)]

use nalgebra;

pub mod ctypes;
pub mod trajectory_params;
pub mod robot_physical_params;
pub mod bangbang_trajectory;
pub mod robot_model;
pub mod state_estimation;

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

use libm::{sinf, cosf};


#[derive(Clone, Copy, Default, Debug)]
pub struct RigidBodyState {
    pub pose: Vector3f,
    pub twist: Vector3f,
}

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

/// Pseudo matrix inverse
fn pinv_3x4(a: Matrix3x4f) -> Matrix4x3f {
    let a_t = a.clone().transpose();
    let a_at = a * a_t.clone();
    let a_at_inv = a_at.try_inverse().unwrap();
    let a_inv = a_t * a_at_inv;
    a_inv
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
