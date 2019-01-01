use std::mem;
use std::ops::{Index, IndexMut};

use num_traits::NumCast;
use num_traits::Zero;

/// Collision parameters
#[derive(Debug)]
pub enum Params {
    /// Box (dx, dy, dz)
    Box(f32, f32, f32),
    /// Sphere (radius)
    Sphere(f32),
    /// Cone (radius, height)
    Cone(f32, f32),
    /// Cylinder (radius0, radius1, height)
    Cylinder(f32, f32, f32),
    /// Capsule (radius0, radius1, height)
    Capsule(f32, f32, f32),
    /// Heightfield with a `f32` elevation field
    HeightFieldF32(HeightFieldParams<f32>),
    /// Heightfield with a `u16` elevation field
    HeightFieldU16(HeightFieldParams<u16>),
    /// Null collision shape
    Null,
}

/// Ways HeightField grid cells can be constructed
#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum GridConstruction {
    NormalDiagonals = 0,
    InvertedDiagonals = 1,
    AlternateOddRowsDiagonals = 2,
    AlternateEvenRowsDiagonals = 3,
    AlternateOddColumsDiagonals = 4,
    AlternateEvenColumsDiagonals = 5,
    StarDiagonals = 6,
    StarInvertexDiagonals = 7,
}

/// 2D Array
#[derive(Debug, Clone)]
pub struct Field<T>(Vec<T>, /*rows*/ usize, /*columns*/ usize);

/// Definition of a HeightField collision
#[derive(Debug, Clone)]
pub struct HeightFieldParams<T> {
    /// Width of the heightfield
    rows: usize,
    /// Height of the heightfield
    columns: usize,

    /// grid construction
    grid: GridConstruction,

    /// Height elevation (width * height elements)
    elevation: Field<T>,
    /// Shape ids of each face
    attribs: Field<i8>,

    /// (X, Y, Z) scale
    scale: (f32, f32, f32),
}

/*
//#[doc(hidden)]
pub trait HeightFieldType: Sized + Copy + Clone {
    const MAX_VALUE: Self;
    const ZERO: Self;

    fn as_f32(&self) -> f32;
}
*/

/*
impl HeightFieldType for u16 {
    const MAX_VALUE: u16 = 0xFFFF;
    const ZERO: u16 = 0x0;

    #[inline]
    fn as_f32(&self) -> f32 {
        *self as f32
    }
}
*/

/*
impl HeightFieldType for f32 {
    const MAX_VALUE: f32 = 1.0;
    const ZERO: f32 = 0.0;

    #[inline]
    fn as_f32(&self) -> f32 {
        *self
    }
}
*/

impl<T> Field<T> {
    pub fn row(&self, row: usize) -> &[T] {
        assert!(row < self.1);

        let offset = self.columns() * row;
        &self.0[offset..offset + self.columns()]
    }

    pub fn row_mut(&mut self, row: usize) -> &mut [T] {
        assert!(row < self.1);

        let offset = self.columns() * row;
        let c = self.columns();
        &mut self.0[offset..offset + c]
    }

    pub fn as_ptr(&self) -> *const T {
        self.0.as_ptr()
    }

    #[inline]
    pub fn get(&self, row: usize, column: usize) -> Option<&T> {
        self.0.get(row * self.columns() + column)
    }

    #[inline]
    pub fn rows(&self) -> usize {
        self.1
    }

    #[inline]
    pub fn columns(&self) -> usize {
        self.2
    }
}

impl<T> Index<(usize, usize)> for Field<T> {
    type Output = T;

    fn index(&self, (r, c): (usize, usize)) -> &Self::Output {
        &self.row(r)[c]
    }
}

impl<T> IndexMut<(usize, usize)> for Field<T> {
    fn index_mut(&mut self, (r, c): (usize, usize)) -> &mut Self::Output {
        &mut self.row_mut(r)[c]
    }
}

impl<T> Index<usize> for Field<T> {
    type Output = [T];

    fn index(&self, index: usize) -> &Self::Output {
        self.row(index)
    }
}

impl<T> IndexMut<usize> for Field<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        self.row_mut(index)
    }
}

impl<T> HeightFieldParams<T> {
    #[inline]
    pub fn set_grid(&mut self, grid: GridConstruction) {
        self.grid = grid
    }

    #[inline]
    pub fn grid(&self) -> GridConstruction {
        self.grid
    }

    #[inline]
    pub fn set_scale_x(&mut self, scale: f32) {
        self.scale.0 = scale;
    }

    #[inline]
    pub fn set_scale_y(&mut self, scale: f32) {
        self.scale.1 = scale;
    }

    #[inline]
    pub fn set_scale_z(&mut self, scale: f32) {
        self.scale.2 = scale;
    }

    #[inline]
    pub fn set_scale_xz(&mut self, x: f32, z: f32) {
        self.scale.0 = x;
        self.scale.2 = z;
    }

    #[inline]
    pub fn rows(&self) -> usize {
        self.rows
    }

    #[inline]
    pub fn columns(&self) -> usize {
        self.columns
    }

    /// The scale along (x, y, z) dimensions
    #[inline]
    pub fn scale(&self) -> (f32, f32, f32) {
        self.scale
    }

    #[inline]
    pub fn elevation(&self) -> &Field<T> {
        &self.elevation
    }

    #[inline]
    pub fn elevation_mut(&mut self) -> &mut Field<T> {
        &mut self.elevation
    }

    #[inline]
    pub fn ids(&self) -> &Field<i8> {
        &self.attribs
    }

    #[inline]
    pub fn ids_mut(&mut self) -> &mut Field<i8> {
        &mut self.attribs
    }
}

#[doc(hidden)]
pub trait Num: Copy + num_traits::Num + NumCast {
    fn max_norm() -> Self;
}

impl Num for f32 {
    #[inline]
    fn max_norm() -> Self {
        1.0
    }
}

impl Num for u16 {
    #[inline]
    fn max_norm() -> Self {
        0xFFFF
    }
}

impl<T: Num> HeightFieldParams<T> {
    pub fn new(rows: usize, columns: usize) -> Self {
        HeightFieldParams {
            grid: GridConstruction::NormalDiagonals,
            rows,
            columns,
            elevation: Field(vec![T::zero(); rows * columns], rows, columns),
            attribs: Field(vec![0; rows * columns], rows, columns),
            scale: (1.0, 1.0, 1.0),
        }
    }

    /// Sets the maximum height, assuming that the maximum range of the elevation map is [0.0, 1.0]
    pub fn set_max_height(&mut self, max_height: f32) {
        self.scale.1 = max_height / num_traits::cast::<_, f32>(T::max_norm()).unwrap();
    }

    /// Returns the max height, which is equivalent to `scale_y * T::MAX_VALUE`
    pub fn max_height(&self) -> f32 {
        self.scale.1 * num_traits::cast::<_, f32>(T::max_norm()).unwrap()
    }
}
