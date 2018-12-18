use std::fmt::Debug;

pub mod ffi;
pub mod collision;
pub mod world;
pub mod body;

/// Algebraic types
pub trait NewtonData: Debug {
    /// 3D vector type
    type Vector3: Copy;
    /// 4D vector type
    type Vector4: Copy;
    /// 4x4 Matrix type
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
