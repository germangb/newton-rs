#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(C)]
#[rustfmt::skip]
pub struct Vector { pub x: f32, pub y: f32, pub z: f32, pub w: f32 }

#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(C)]
#[rustfmt::skip]
pub struct Matrix { pub c0: Vector, pub c1: Vector, pub c2: Vector, pub c3: Vector }

#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(C)]
#[rustfmt::skip]
pub struct Quaternion { pub x: f32, pub y: f32, pub z: f32, pub w: f32 }

impl Matrix {
    pub const fn identity() -> Self {
        Self {
            c0: Vector::new(1.0, 0.0, 0.0, 0.0),
            c1: Vector::new(0.0, 1.0, 0.0, 0.0),
            c2: Vector::new(0.0, 0.0, 1.0, 0.0),
            c3: Vector::new(0.0, 0.0, 0.0, 1.0),
        }
    }

    #[inline]
    pub fn as_ptr(&self) -> *const f32 {
        self.c0.as_ptr() as _
    }

    #[inline]
    pub fn as_mut_ptr(&mut self) -> *mut f32 {
        self.c0.as_mut_ptr() as _
    }
}

impl From<[f32; 3]> for Vector {
    #[inline]
    fn from([x, y, z]: [f32; 3]) -> Self {
        Self::new3(x, y, z)
    }
}

impl From<[f32; 4]> for Vector {
    #[inline]
    fn from([x, y, z, w]: [f32; 4]) -> Self {
        Self::new(x, y, z, w)
    }
}

impl From<(f32, f32, f32)> for Vector {
    #[inline]
    fn from((x, y, z): (f32, f32, f32)) -> Self {
        Self::new3(x, y, z)
    }
}

impl From<(f32, f32, f32, f32)> for Vector {
    #[inline]
    fn from((x, y, z, w): (f32, f32, f32, f32)) -> Self {
        Self::new(x, y, z, w)
    }
}

impl Default for Vector {
    #[inline]
    fn default() -> Self {
        Self::zero()
    }
}

impl Vector {
    #[rustfmt::skip]
    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0, w: 0.0 }
    }

    pub const fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    /// w = 0
    pub const fn new3(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z, w: 0.0 }
    }

    #[inline]
    pub fn as_ptr(&self) -> *const f32 {
        &self.x as *const _
    }

    #[inline]
    pub fn as_mut_ptr(&mut self) -> *mut f32 {
        &mut self.x as *mut _
    }
}
