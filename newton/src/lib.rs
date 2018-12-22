pub extern crate newton_sys as ffi;

pub mod body;
pub mod callback;
pub mod collision;
pub mod joint;
pub mod mesh;
mod pointer;
pub mod world;

pub use crate::body::{Mass, NewtonBody, SleepState};
pub use crate::callback::{DoNothing, Gravity};
pub use crate::world::NewtonWorld;

/// Trait to adapt the types returned by the Newton APIs to each application.
///
/// This trait is marked `unsafe` because the Rust compiler may change the memory layout of the types.
pub unsafe trait NewtonConfig: std::fmt::Debug {
    /// Default gravity
    const GRAVITY: Self::Vector3;
    /// 3D vector type.
    type Vector3: Copy;
    /// 4D vector type
    type Vector4: Copy;
    /// 4x4 Matrix type
    type Matrix4: Copy;
    /// Quaternion type
    type Quaternion: Copy;
}
