use std::rc::Rc;

#[cfg(feature = "cgmath_math")]
use cgmath::{
    prelude::{Array, SquareMatrix, Zero},
    Matrix4, Vector3, Vector4,
};

pub trait Vector: Sized {
    fn zero() -> Self;
    fn identity() -> Self;

    fn as_ptr(&self) -> *const f32;
    fn as_mut_ptr(&mut self) -> *mut f32;
}

macro_rules! array_impl {
    ($arr:ty, $zero:expr, $ident:expr) => {
        impl Vector for $arr {
            fn as_ptr(&self) -> *const f32 {
                self as *const $arr as *const f32
            }
            fn as_mut_ptr(&mut self) -> *mut f32 {
                self as *mut $arr as *mut f32
            }
            fn zero() -> $arr {
                $zero
            }
            fn identity() -> $arr {
                $ident
            }
        }
    };
}

array_impl!([f32; 3], [0.0; 3], [0.0; 3]);
array_impl!([f32; 4], [0.0; 4], [0.0; 4]);
array_impl!(
    [f32; 16],
    [0.0; 16],
    [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
);

#[cfg(feature = "cgmath_math")]
array_impl!(
    Vector3<f32>,
    <Vector3<f32> as Zero>::zero(),
    <Vector3<f32> as Zero>::zero()
);
#[cfg(feature = "cgmath_math")]
array_impl!(
    Vector4<f32>,
    <Vector4<f32> as Zero>::zero(),
    <Vector4<f32> as Zero>::zero()
);
#[cfg(feature = "cgmath_math")]
array_impl!(
    Matrix4<f32>,
    <Matrix4<f32> as Zero>::zero(),
    <Matrix4<f32> as SquareMatrix>::identity()
);

pub trait NewtonMath {
    type Vector3: Vector + Copy + Send;
    type Vector4: Vector + Copy + Send;
    type Matrix4: Vector + Copy + Send;
}

pub trait NewtonCallbacks<V> {
    fn force_torque() {}
}

#[derive(Debug, Clone)]
pub enum NewtonArray {}

impl NewtonMath for NewtonArray {
    type Vector3 = [f32; 3];
    type Vector4 = [f32; 4];
    type Matrix4 = [f32; 16];
}

#[cfg(feature = "cgmath_math")]
#[derive(Debug, Clone)]
pub enum NewtonCgmath {}

#[cfg(feature = "cgmath_math")]
impl NewtonMath for NewtonCgmath {
    type Vector3 = Vector3<f32>;
    type Vector4 = Vector4<f32>;
    type Matrix4 = Matrix4<f32>;
}
