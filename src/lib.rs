//! A safe wrapper around the [newton-dynamics][newton] C API
//!
//! - **Newton version: 3.13a**
//!
//! [newton]: https://github.com/MADEAPPS/newton-dynamics
//!
//! ## Example
//!
//! The following example creates a plane, and a unit sphere bounding on it.
//!
//! ```
//! use newton::prelude::*;
//! use newton::{Newton, Sphere, Cuboid, DynamicBody};
//!
//! // init newton world
//! let mut world = Newton::create();
//!
//! // Create collisions
//! let sphere = Sphere::create(&world, 1.0, None);
//! let plane = Cuboid::create(&world, 16.0, 0.1, 16.0, None);
//!
//! // create plane
//! DynamicBody::create(&world, &plane, pos(0.0, 0.0, 0.0), None).into_handle(&world);
//!
//! /// Create bounding body
//! let body = DynamicBody::create(&world, &sphere, pos(0.0, 8.0, 0.0), None);
//! body.set_mass(1.0, &sphere);
//! body.set_force_and_torque_callback(|b, _, _| {
//!     let (mass, _) = b.mass();
//!     b.set_force([0.0, -9.8 * mass, 0.0])
//! });
//!
//! // drop collisions because we no longer need them and they keep the world borrowed.
//! drop(sphere);
//! drop(plane);
//!
//! println!("Body starts at position = {:?}", body.position());
//!
//! // save body for later...
//! let h = body.into_handle(&world);
//!
//! // simulate 4 seconds...
//! for it in 0..(60*4) {
//!     world.update(std::time::Duration::new(0, 1_000_000_000 / 60));
//!
//!     let body = world.body(h).unwrap();
//!     println!("position = {:?}", body.position());
//! }
//!
//! # fn pos(x: f32, y: f32, z: f32) -> [[f32; 4]; 4] {
//! #    [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [x, y, z, 1.0]]
//! # }
//! ```
//!
//! ### Result
//!
//! ![](https://i.imgur.com/Pbxbzfl.png)
pub use ffi;

pub use crate::newton::Newton;
pub use body::{Body, DynamicBody, KinematicBody};
pub use collision::{Collision, Compound, Cone, Cuboid, Cylinder, Null, Scene, Sphere, Tree};
pub use utils::*;

/// Dynamic & Kinematic body wrappers.
pub mod body;
/// NewtonCollision wrappers.
pub mod collision;
/// Coverage for newton's utility functions.
mod utils;
/// Reexport of the most used traits.
pub mod prelude {
    pub use super::body::{Dynamic, NewtonBody};
    pub use super::collision::NewtonCollision;
    pub use super::{AsHandle, IntoHandle};
}
/// NewtonWorld wrapper.
pub mod newton;
/// Framework to inspect Newton simulations.
///
/// This module is included only if the *testbed* feature is enabled.
#[cfg(feature = "testbed")]
pub mod testbed;

/// Opaque type used to access not-owned Collisions & Bodies.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub struct Handle(HandleInner);

impl Handle {
    fn from_usize(idx: usize) -> Self {
        Self(HandleInner::Index(idx))
    }

    fn from_ptr(ptr: *const ()) -> Self {
        Self(HandleInner::Pointer(ptr))
    }

    fn inner(&self) -> HandleInner {
        self.0
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum HandleInner {
    Pointer(*const ()),
    Index(usize),
}

unsafe impl Send for Handle {}
unsafe impl Sync for Handle {}

pub trait IntoHandle {
    /// Moves the object into the given Newton and returns a handle to
    /// borrow it or retake ownership of it later.
    ///
    /// ## Panics
    /// On both Bodies and Collisions, this method panics if the object
    /// is not owned, but you can still call `as_handle` on these.
    fn into_handle(self, newton: &Newton) -> Handle;
}

pub trait AsHandle: IntoHandle {
    /// Returns the same vale that would be returned by `into_handle`, but
    /// without moving the object.
    fn as_handle(&self) -> Handle;
}
