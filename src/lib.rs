use std::fmt::Debug;

pub mod ffi;
pub mod collision;
pub mod world;
pub mod body;

/// Algebraic types
pub trait NewtonData: Debug {
    /// 3D vector type.
    ///
    /// It should have a size of 3 * std::mem::size_of::<f32>()
    type Vector3: Copy;
    /// 4D vector type
    ///
    /// It should have a size of 4 * std::mem::size_of::<f32>()
    type Vector4: Copy;
    /// 4x4 Matrix type. The first 4 `f32`s correspond to the first column, the next 4 to the
    /// second column and so on...
    ///
    /// It should have a size of 16 * std::mem::size_of::<f32>(),
    type Matrix4: Copy;
    /// Quaternion type
    type Quaternion: Copy;
}

#[derive(Debug)]
pub enum Array {}
impl NewtonData for Array {
    type Vector3 = [f32; 3];
    type Vector4 = [f32; 4];
    type Matrix4 = [[f32; 4]; 4];
    type Quaternion = [f32; 4];
}
