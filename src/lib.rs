//! A rust wrapper around the [newton-dynamics][newton] C API
//!
//! - **Newton version: 3.13a**
//!
//! [newton]: https://github.com/MADEAPPS/newton-dynamics
//!
//! ## Thread safety
//!
//! Newton has built-in support to perform the simulation step on multiple threads, where bodies, collisions, and joints, are updated through application-implemented callbacks.
//!
//! `Collision`, `Body`, and `Joint` types are **NOT** thread safe. You'll have to convert them to `Handle`s first.
//!
//! [handle]: #
pub use ffi;

pub use body::{Body, DynamicBody, KinematicBody};
pub use collision::{
    ChamferCylinder, Collision, Compound, Cone, Cuboid, Cylinder, DeformableSolid,
    FracturedCompound, MassSpringDamperSystem, Null, Scene, Sphere, Tree, UserMesh,
};
pub use joint::{Ball, Constraint, Corkscrew, Hinge, Slider, Universal, UpVector, UserJoint};
pub use utils::*;

pub use crate::newton::Newton;

include!("macros.rs");

/// Dynamic & Kinematic body wrappers.
pub mod body;
/// NewtonCollision wrappers.
pub mod collision;
/// A type for referencing bodies, collisions, and joints.
pub mod handle;
/// Wrappers around Newton joints.
pub mod joint;
/// Types and function for user mesh definition.
pub mod mesh;
/// Coverage for newton's utility functions.
mod utils;
/// Reexport of the most used traits.
pub mod prelude {
    pub use crate::body::NewtonBody;
    pub use crate::collision::NewtonCollision;
    pub use crate::handle::{AsHandle, FromHandle, IntoHandle};
    pub use crate::joint::NewtonJoint;
    pub use crate::mesh::Mesh;
    pub use crate::newton::storage::NewtonStorage;
}
/// NewtonWorld wrapper.
pub mod newton;
/// Framework to inspect Newton simulations.
///
/// This module is included only if the *testbed* feature is enabled.
#[cfg(feature = "testbed")]
pub mod testbed;

/// 3D vector
pub type Vec3 = [f32; 3];
/// 4D vector
pub type Vec4 = [f32; 4];
/// Quaternion
pub type Quat = [f32; 4];
/// 4x4 matrix, arranged in columns
pub type Mat4 = [Vec4; 4];
