#![cfg_attr(not(feature = "std"), no_std)]

use nalgebra;

pub mod ctypes;
pub mod trajectory_params;
pub mod robot_physical_params;
pub mod bangbang_trajectory;
pub mod robot_model;

pub type Vector3f = nalgebra::Vector3<f32>;
pub type Vector4f = nalgebra::Vector4<f32>;
pub type Matrix3x4f = nalgebra::Matrix3x4<f32>;
pub type Matrix4x3f = nalgebra::Matrix4x3<f32>;
pub type Matrix3f = nalgebra::Matrix3<f32>;
pub type Rotation3f = nalgebra::Rotation3<f32>;


#[derive(Clone, Copy, Default, Debug)]
pub struct RigidBodyState {
    pub pose: Vector3f,
    pub twist: Vector3f,
}

pub fn add(left: u64, right: u64) -> u64 {
    left + right
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
