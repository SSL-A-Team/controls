// Algebra types with C-compatible layouts and conversions to/from nalgebra types
use nalgebra;

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Vector3C {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl From<nalgebra::Vector3<f32>> for Vector3C {
    fn from(v: nalgebra::Vector3<f32>) -> Self {
        Vector3C {x: v.x, y: v.y, z: v.z}
    }
}

impl From<Vector3C> for nalgebra::Vector3<f32> {
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

impl From<nalgebra::Vector4<f32>> for Vector4C {
    fn from(v: nalgebra::Vector4<f32>) -> Self {
        Vector4C {x: v.x, y: v.y, z: v.z, w: v.w}
    }
}

impl From<Vector4C> for nalgebra::Vector4<f32> {
    fn from(v: Vector4C) -> Self {
        Self::new(v.x, v.y, v.z, v.w)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Matrix3C {
    pub data: [f32; 9],
}

impl Matrix3C {
    /// Create a new Matrix3C from a flat array in column-major order
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

impl From<nalgebra::Matrix3<f32>> for Matrix3C {
    fn from(mat: nalgebra::Matrix3<f32>) -> Self {
        let mut data = [0.0f32; 9];
        for i in 0..3 {
            for j in 0..3 {
                data[j * 3 + i] = mat[(i, j)];
            }
        }
        Matrix3C { data }
    }
}

impl From<Matrix3C> for nalgebra::Matrix3<f32> {
    fn from(mat: Matrix3C) -> Self {
        nalgebra::Matrix3::from_column_slice(&mat.data)
    }
}


#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Matrix3x4C {
    pub data: [f32; 12],
}

impl Matrix3x4C {
    /// Create a new Matrix3x4C from a flat array in column-major order
    pub fn new(data: [f32; 12]) -> Self {
        Matrix3x4C { data }
    }

    pub fn get(&self, row: usize, col: usize) -> f32 {
        // 3 rows, 4 cols
        self.data[col * 3 + row]
    }

    pub fn set(&mut self, row: usize, col: usize, value: f32) {
        self.data[col * 3 + row] = value;
    }
}

impl From<nalgebra::Matrix3x4<f32>> for Matrix3x4C {
    fn from(mat: nalgebra::Matrix3x4<f32>) -> Self {
        let mut data = [0.0f32; 12];
        for i in 0..3 {
            for j in 0..4 {
                data[j * 3 + i] = mat[(i, j)];
            }
        }
        Matrix3x4C { data }
    }
}

impl From<Matrix3x4C> for nalgebra::Matrix3x4<f32> {
    fn from(mat: Matrix3x4C) -> Self {
        nalgebra::Matrix3x4::from_column_slice(&mat.data)
    }
}


#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Matrix4x3C {
    pub data: [f32; 12],
}

impl Matrix4x3C {
    /// Create a new Matrix4x3C from a flat array in column-major order
    pub fn new(data: [f32; 12]) -> Self {
        Matrix4x3C { data }
    }

    pub fn get(&self, row: usize, col: usize) -> f32 {
        // 4 rows, 3 cols
        self.data[col * 4 + row]
    }

    pub fn set(&mut self, row: usize, col: usize, value: f32) {
        self.data[col * 4 + row] = value;
    }
}

impl From<nalgebra::Matrix4x3<f32>> for Matrix4x3C {
    fn from(mat: nalgebra::Matrix4x3<f32>) -> Self {
        let mut data = [0.0f32; 12];
        for i in 0..4 {
            for j in 0..3 {
                data[j * 4 + i] = mat[(i, j)];
            }
        }
        Matrix4x3C { data }
    }
}

impl From<Matrix4x3C> for nalgebra::Matrix4x3<f32> {
    fn from(mat: Matrix4x3C) -> Self {
        nalgebra::Matrix4x3::from_column_slice(&mat.data)
    }
}