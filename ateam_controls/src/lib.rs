pub mod trajectory_params;
pub mod robot_physical_params;
pub mod bang_bang_trajectory;
pub mod wheel_conversion;


#[derive(Clone, Copy, Default, Debug)]
pub struct GlobalState {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub xd: f64,
    pub yd: f64,
    pub zd: f64,
}

#[derive(Clone, Copy, Default, Debug)]
pub struct GlobalControl2Order {
    pub xdd: f64,
    pub ydd: f64,
    pub zdd: f64,
}

#[derive(Clone, Copy, Default, Debug)]
pub struct GlobalControl1Order {
    pub xd: f64,
    pub yd: f64,
    pub zd: f64,
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
