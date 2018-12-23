pub extern crate newton_sys as ffi;

mod pointer;
mod userdata;

pub mod body;
pub mod callback;
pub mod collision;
pub mod joint;
pub mod mesh;
pub mod world;

// TODO kill
pub use crate::callback::{DoNothing, Gravity};

pub use crate::body::{Body, Mass, SleepState};
pub use crate::world::World;

pub use crate::collision::Capsule;
pub use crate::collision::Cone;
pub use crate::collision::Cuboid;
pub use crate::collision::Cylinder;
pub use crate::collision::Sphere;

/// Trait to adapt the types returned by the Newton APIs to each application.
///
/// This trait is marked `unsafe` because the Rust compiler may change the memory layout of the types.
pub unsafe trait NewtonConfig: std::fmt::Debug {
    /// 3D vector type.
    type Vector: Copy;
    /// 4x4 Matrix type
    type Matrix: Copy;
    /// Quaternion type
    type Quaternion: Copy;
}
