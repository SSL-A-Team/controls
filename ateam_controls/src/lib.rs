#![cfg_attr(not(feature = "std"), no_std)]

pub mod trajectory_params;
pub mod robot_physical_params;
pub mod bang_bang_trajectory;
pub mod robot_model;
use nalgebra::{Vector3, Vector4};


#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct GlobalState {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub xd: f64,
    pub yd: f64,
    pub zd: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct GlobalPosition {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct GlobalVelocity {
    pub xd: f64,
    pub yd: f64,
    pub zd: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct GlobalControl2Order {
    pub xdd: f64,
    pub ydd: f64,
    pub zdd: f64,
}

impl GlobalControl2Order {
    pub fn from_vec(vec: &Vector3<f64>) -> Self {
        Self {
            xdd: vec[0],
            ydd: vec[1],
            zdd: vec[2],
        }
    }

    pub fn to_vec(&self) -> Vector3<f64> {
        Vector3::<f64>::new(
            self.xdd,
            self.ydd,
            self.zdd,
        )
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct GlobalControl1Order {
    pub xd: f64,
    pub yd: f64,
    pub zd: f64,
}

impl GlobalControl1Order {
    pub fn from_vec(vec: &Vector3<f64>) -> Self {
        Self {
            xd: vec[0],
            yd: vec[1],
            zd: vec[2],
        }
    }

    pub fn to_vec(&self) -> Vector3<f64> {
        Vector3::<f64>::new(
            self.xd,
            self.yd,
            self.zd,
        )
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct WheelTorques {
    pub torque_fl: f64,
    pub torque_bl: f64,
    pub torque_br: f64,
    pub torque_fr: f64,
}

impl WheelTorques {
    pub fn from_vec(vec: &Vector4<f64>) -> Self {
        Self {
            torque_fl: vec[0],
            torque_bl: vec[1],
            torque_br: vec[2],
            torque_fr: vec[3],
        }
    }

    pub fn to_vec(&self) -> Vector4<f64> {
        Vector4::<f64>::new(
            self.torque_fl,
            self.torque_bl,
            self.torque_br,
            self.torque_fr,
        )
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct WheelVelocities {
    pub velocity_fl: f64,
    pub velocity_bl: f64,
    pub velocity_br: f64,
    pub velocity_fr: f64,
}

impl WheelVelocities {
    pub fn from_vec(vec: &Vector4<f64>) -> Self {
        Self {
            velocity_fl: vec[0],
            velocity_bl: vec[1],
            velocity_br: vec[2],
            velocity_fr: vec[3],
        }
    }

    pub fn to_vec(&self) -> Vector4<f64> {
        Vector4::<f64>::new(
            self.velocity_fl,
            self.velocity_bl,
            self.velocity_br,
            self.velocity_fr,
        )
    }
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
