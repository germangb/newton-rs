//! A rust wrapper around the [newton-dynamics][newton] C API
//!
//! - **Newton version: 3.13a**
//!
//! [newton]: https://github.com/MADEAPPS/newton-dynamics
//!
//! ## Thread safety
//!
//! Newton supports running the simulation step on multiple threads, where bodies, collisions, and joints, are updated through application-implemented callbacks.
//!
//! `Collision`, `Body`, and `Joint` types are **NOT** `Sync` nor `Send`. You'll have to convert them to `Handle`s first.
//!
//! [handle]: #
pub use ffi;

include!("macros.rs");

/// Dynamic & Kinematic body wrappers.
pub mod body;
/// NewtonCollision wrappers.
pub mod collision;
/// A type for referencing bodies, collisions, and joints.
pub mod handle;
/// Wrappers around Newton joints.
pub mod joint;
/// Newton math functions.
pub mod math;
/// Types and function for user mesh definition.
pub mod mesh;
/// NewtonWorld wrapper.
pub mod newton;
/// Reexport of the most used traits.
pub mod prelude;
/// Framework to inspect Newton simulations.
#[cfg(feature = "testbed")]
pub mod testbed;
