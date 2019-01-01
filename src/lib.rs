pub use ffi;

mod callbacks;

pub mod body;
pub mod collision;
//pub mod contact;
pub mod joint;
/// Types used in dynamic borrow checking
pub mod lock;
pub mod macros;
pub mod material;
#[cfg(feature = "sandbox")]
pub mod sandbox;
pub mod world;

use failure::Error;

use std::sync::mpsc;
use std::sync::mpsc::{Receiver, Sender};

/// Custom Result type
pub type Result<T> = std::result::Result<T, Error>;

///Vector 3D type
#[cfg(not(feature = "cgmath_types"))]
pub type Vector = [f32; 3];
#[cfg(feature = "cgmath_types")]
pub type Vector = cgmath::Vector3<f32>;

/// Matrix 4x4 type
#[cfg(not(feature = "cgmath_types"))]
pub type Matrix = [[f32; 4]; 4];
#[cfg(feature = "cgmath_types")]
pub type Matrix = cgmath::Matrix4<f32>;

/// Quaternion type
#[cfg(not(feature = "cgmath_types"))]
pub type Quaternion = [f32; 4];
#[cfg(feature = "cgmath_types")]
pub type Quaternion = cgmath::Quaternion<f32>;

type Tx<T> = Sender<T>;
type Rx<T> = Receiver<T>;

fn channel<T>() -> (Tx<T>, Rx<T>) {
    mpsc::channel()
}
