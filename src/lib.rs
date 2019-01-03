pub use ffi;

mod callbacks;

pub mod body;
pub mod collision;
pub mod lock;
pub mod material;
#[cfg(feature = "sandbox")]
pub mod sandbox;
pub mod world;

use failure::Error;

use std::sync::mpsc;
use std::sync::mpsc::{Receiver, Sender};

pub use self::body::Body;
pub use self::collision::Collision;
pub use self::world::World;

/// Custom Result type
pub type Result<T> = std::result::Result<T, Error>;
/// Shared strong reference
pub type Shared<T> = std::sync::Arc<T>;
/// Weak reference
pub type Weak<T> = std::sync::Weak<T>;

/// Vector 3D type
#[cfg(not(feature = "cgmath_types"))]
pub type Vector = [f32; 3];
/// Vector 3D type
#[cfg(feature = "cgmath_types")]
pub type Vector = cgmath::Vector3<f32>;

/// Matrix 4x4 type
#[cfg(not(feature = "cgmath_types"))]
pub type Matrix = [[f32; 4]; 4];
/// Matrix 4x4 type
#[cfg(feature = "cgmath_types")]
pub type Matrix = cgmath::Matrix4<f32>;

#[cfg(not(feature = "cgmath_types"))]
const IDENTITY: Matrix = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];
#[cfg(feature = "cgmath_types")]
#[rustfmt::skip]
const IDENTITY: Matrix = cgmath::Matrix4 {
    x: cgmath::Vector4 { x: 1.0, y: 0.0, z: 0.0, w: 0.0 },
    y: cgmath::Vector4 { x: 0.0, y: 1.0, z: 0.0, w: 0.0 },
    z: cgmath::Vector4 { x: 0.0, y: 0.0, z: 1.0, w: 0.0 },
    w: cgmath::Vector4 { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
};

/// Quaternion type
#[cfg(not(feature = "cgmath_types"))]
pub type Quaternion = [f32; 4];
/// Quaternion type
#[cfg(feature = "cgmath_types")]
pub type Quaternion = cgmath::Quaternion<f32>;

type Tx<T> = Sender<T>;
type Rx<T> = Receiver<T>;

fn channel<T>() -> (Tx<T>, Rx<T>) {
    mpsc::channel()
}
