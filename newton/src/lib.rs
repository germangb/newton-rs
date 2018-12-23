mod pointer;
mod userdata;

#[cfg(feature = "sandbox")]
pub mod sandbox;

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

pub use crate::collision::BoxCollision;
pub use crate::collision::CapsuleCollision;
pub use crate::collision::ConeCollision;
pub use crate::collision::CylinderCollision;
pub use crate::collision::SphereCollision;

/// Trait to adapt the types returned by the Newton APIs to each application.
///
/// This trait is marked `unsafe` because the Rust compiler may change the memory layout of the types.
pub unsafe trait NewtonApp: std::fmt::Debug {
    /// 3D vector type.
    type Vector: Copy;
    /// 4x4 Matrix type
    type Matrix: Copy;
    /// Quaternion type
    type Quaternion: Copy;
}
