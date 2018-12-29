use std::mem;
use std::ops::{Index, IndexMut};

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

#[derive(Debug, Clone)]
pub struct Field<T>(Vec<T>, /*rows*/ usize, /*columns*/ usize);

#[derive(Debug, Clone)]
pub struct HeightFieldParams<T> {
    /// Width of the heightfield
    rows: usize,
    /// Height of the heightfield
    columns: usize,

    /// Height elevation (width * height elements)
    elevation: Field<T>,
    /// Shape ids of each face
    attribs: Field<i8>,

    /// (X, Y, Z) scale
    scale: (f32, f32, f32),
}

pub trait HeightFieldType: Sized + Copy + Clone {
    const MAX_VALUE: Self;
    const ZERO: Self;

    fn as_f32(&self) -> f32;
}

impl HeightFieldType for u16 {
    const MAX_VALUE: u16 = 0xFFFF;
    const ZERO: u16 = 0x0;

    #[inline]
    fn as_f32(&self) -> f32 {
        *self as f32
    }
}

impl HeightFieldType for f32 {
    const MAX_VALUE: f32 = 1.0;
    const ZERO: f32 = 0.0;

    #[inline]
    fn as_f32(&self) -> f32 {
        *self
    }
}

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

    /// The scale along (x, y, z) dimensions
    #[inline]
    pub fn scale(&self) -> (f32, f32, f32) {
        self.scale
    }

    pub fn elevation(&self) -> &Field<T> {
        &self.elevation
    }

    pub fn elevation_mut(&mut self) -> &mut Field<T> {
        &mut self.elevation
    }

    pub fn ids(&self) -> &Field<i8> {
        &self.attribs
    }

    pub fn ids_mut(&mut self) -> &mut Field<i8> {
        &mut self.attribs
    }
}

impl<T: HeightFieldType> HeightFieldParams<T> {
    pub fn new(rows: usize, columns: usize) -> Self {
        HeightFieldParams {
            rows,
            columns,
            elevation: Field(vec![T::ZERO; rows * columns], rows, columns),
            attribs: Field(vec![0; rows * columns], rows, columns),
            scale: (1.0, 1.0, 1.0),
        }
    }

    /// Sets the maximum height, assuming that the maximum range of the elevation map is [0.0, 1.0]
    pub fn set_max_height(&mut self, max_height: f32) {
        self.scale.1 = max_height / T::MAX_VALUE.as_f32();
    }

    /// Returns the max height, which is equivalent to `scale_y * T::MAX_VALUE`
    pub fn max_height(&self) -> f32 {
        self.scale.1 * T::MAX_VALUE.as_f32()
    }
}
