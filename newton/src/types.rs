use super::Types;

#[derive(Debug, Clone)]
#[cfg(feature = "cgmath_types")]
pub enum Cgmath {}
#[cfg(feature = "cgmath_types")]
unsafe impl Types for Cgmath {
    type Vector = cgmath::Vector3<f32>;
    type Matrix = cgmath::Matrix4<f32>;
    type Quaternion = cgmath::Quaternion<f32>;
}

#[derive(Debug, Clone)]
pub enum Array {}
unsafe impl Types for Array {
    type Vector = [f32; 3];
    type Matrix = [[f32; 4]; 4];
    type Quaternion = [f32; 4];
}
