use nalgebra::{Vector2, Vector3, Vector4, SVector, Matrix3, Matrix3x4, Matrix4x3};

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Vector2C {
    pub x: f32,
    pub y: f32,
}

impl From<Vector2<f32>> for Vector2C {
    fn from(v: Vector2<f32>) -> Self {
        Vector2C { x: v.x, y: v.y }
    }
}

impl From<Vector2C> for Vector2<f32> {
    fn from(v: Vector2C) -> Self {
        Self::new(v.x, v.y)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Vector3C {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl From<Vector3<f32>> for Vector3C {
    fn from(v: Vector3<f32>) -> Self {
        Vector3C {x: v.x, y: v.y, z: v.z}
    }
}

impl From<Vector3C> for Vector3<f32> {
    fn from(v: Vector3C) -> Self {
        Self::new(v.x, v.y, v.z)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Vector4C {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl From<Vector4<f32>> for Vector4C {
    fn from(v: Vector4<f32>) -> Self {
        Vector4C {x: v.x, y: v.y, z: v.z, w: v.w}
    }
}

impl From<Vector4C> for Vector4<f32> {
    fn from(v: Vector4C) -> Self {
        Self::new(v.x, v.y, v.z, v.w)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Vector6C {
    pub data: [f32; 6],
}

impl From<SVector<f32, 6>> for Vector6C {
    fn from(v: SVector<f32, 6>) -> Self {
        let mut data = [0.0f32; 6];
        for i in 0..6 {
            data[i] = v[i];
        }
        Vector6C { data }
    }
}

impl From<Vector6C> for SVector<f32, 6> {
    fn from(v: Vector6C) -> Self {
        SVector::from_column_slice(&v.data)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Vector8C {
    pub data: [f32; 8],
}

impl From<SVector<f32, 8>> for Vector8C {
    fn from(v: SVector<f32, 8>) -> Self {
        let mut data = [0.0f32; 8];
        for i in 0..8 {
            data[i] = v[i];
        }
        Vector8C { data }
    }
}

impl From<Vector8C> for SVector<f32, 8> {
    fn from(v: Vector8C) -> Self {
        SVector::from_column_slice(&v.data)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Matrix3C {
    pub data: [f32; 9],
}

impl Matrix3C {
    pub fn new(data: [f32; 9]) -> Self {
        Matrix3C { data }
    }

    pub fn get(&self, row: usize, col: usize) -> f32 {
        self.data[col * 3 + row]
    }

    pub fn set(&mut self, row: usize, col: usize, value: f32) {
        self.data[col * 3 + row] = value;
    }
}

impl From<Matrix3<f32>> for Matrix3C {
    fn from(mat: Matrix3<f32>) -> Self {
        let mut data = [0.0f32; 9];
        for i in 0..3 {
            for j in 0..3 {
                data[j * 3 + i] = mat[(i, j)];
            }
        }
        Matrix3C { data }
    }
}

impl From<Matrix3C> for Matrix3<f32> {
    fn from(mat: Matrix3C) -> Self {
        Matrix3::from_column_slice(&mat.data)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Matrix3x4C {
    pub data: [f32; 12],
}

impl Matrix3x4C {
    pub fn new(data: [f32; 12]) -> Self {
        Matrix3x4C { data }
    }

    pub fn get(&self, row: usize, col: usize) -> f32 {
        self.data[col * 3 + row]
    }

    pub fn set(&mut self, row: usize, col: usize, value: f32) {
        self.data[col * 3 + row] = value;
    }
}

impl From<Matrix3x4<f32>> for Matrix3x4C {
    fn from(mat: Matrix3x4<f32>) -> Self {
        let mut data = [0.0f32; 12];
        for i in 0..3 {
            for j in 0..4 {
                data[j * 3 + i] = mat[(i, j)];
            }
        }
        Matrix3x4C { data }
    }
}

impl From<Matrix3x4C> for Matrix3x4<f32> {
    fn from(mat: Matrix3x4C) -> Self {
        Matrix3x4::from_column_slice(&mat.data)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Matrix4x3C {
    pub data: [f32; 12],
}

impl Matrix4x3C {
    pub fn new(data: [f32; 12]) -> Self {
        Matrix4x3C { data }
    }

    pub fn get(&self, row: usize, col: usize) -> f32 {
        self.data[col * 4 + row]
    }

    pub fn set(&mut self, row: usize, col: usize, value: f32) {
        self.data[col * 4 + row] = value;
    }
}

impl From<Matrix4x3<f32>> for Matrix4x3C {
    fn from(mat: Matrix4x3<f32>) -> Self {
        let mut data = [0.0f32; 12];
        for i in 0..4 {
            for j in 0..3 {
                data[j * 4 + i] = mat[(i, j)];
            }
        }
        Matrix4x3C { data }
    }
}

impl From<Matrix4x3C> for Matrix4x3<f32> {
    fn from(mat: Matrix4x3C) -> Self {
        Matrix4x3::from_column_slice(&mat.data)
    }
}