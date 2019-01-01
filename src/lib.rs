pub use ffi;

pub mod body;
pub mod collision;
pub mod joint;
pub mod macros;
pub mod material;
#[cfg(feature = "sandbox")]
pub mod sandbox;
pub mod world;

mod callbacks;
mod lock;
use self::lock::*;

use failure::Error;

use std::sync::mpsc;
use std::sync::mpsc::{Receiver, Sender};

/// Custom Result type
pub type Result<T> = std::result::Result<T, Error>;

type Tx<T> = Sender<T>;
type Rx<T> = Receiver<T>;

fn channel<T>() -> (Tx<T>, Rx<T>) {
    mpsc::channel()
}

pub unsafe trait Types: Clone {
    /// Vector 3D type. It must have a memory layout of 3 consecutive `f32`.
    type Vector: Copy + Sync + Send;
    /// Matrix 4x4 type
    ///
    /// The memory layout must be 16 consecutive `f32`. The first 4 define the first column, the
    /// next 4 the second column, and so on.
    type Matrix: Copy + Sync + Send;
    /// Quaternion type
    type Quaternion: Copy + Sync + Send;
}

#[cfg(feature = "cgmath_types")]
types! {
    Cgmath {
        type Vector = cgmath::Vector3<f32>;
        type Matrix = cgmath::Matrix4<f32>;
        type Quaternion = cgmath::Quaternion<f32>;
    }
}

types! {
    Array {
        type Vector = [f32; 3];
        type Matrix = [[f32; 4]; 4];
        type Quaternion = [f32; 4];
    }
}
