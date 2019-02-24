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
//! use newton::{Newton, Sphere, Cuboid, DynamicBody, Mat4};
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
//!     let body = world.storage().body(h).unwrap();
//!     println!("position = {:?}", body.position());
//! }
//!
//! # fn pos(x: f32, y: f32, z: f32) -> Mat4 {
//! #    [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [x, y, z, 1.0]]
//! # }
//! ```
//!
//! ### Result
//!
//! ![](https://i.imgur.com/Pbxbzfl.png)
use std::ptr;

pub use ffi;

pub use body::{Body, DynamicBody, KinematicBody};
pub use collision::{
    ChamferCylinder, Collision, Compound, Cone, Cuboid, Cylinder, DeformableSolid,
    FracturedCompound, MassSpringDamperSystem, Null, Scene, Sphere, Tree,
};
pub use joint::{Ball, Constraint, Corkscrew, Hinge, Slider, Universal, UpVector, UserJoint};
pub use utils::*;

pub use crate::newton::Newton;

/// Dynamic & Kinematic body wrappers.
pub mod body;
/// NewtonCollision wrappers.
pub mod collision;
/// Newton Constraints.
pub mod joint;
/// Coverage for newton's utility functions.
mod utils;
/// Reexport of the most used traits.
pub mod prelude {
    pub use super::body::NewtonBody;
    pub use super::collision::NewtonCollision;
    pub use super::joint::NewtonJoint;
    pub use super::newton::storage::NewtonStorage;
    pub use super::{AsHandle, FromHandle, IntoHandle};
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

/// ```
/// use newton::prelude::*;
/// use newton::{Handle, Newton, Sphere, Collision};
///
/// let newton = Newton::create();
/// let sphere = Sphere::create(&newton, 1.0, None).into_handle(&newton);
///
/// let collision = newton.storage().collision(sphere);
///
/// assert!(collision.is_some());
/// assert_eq!(sphere, collision.unwrap().as_handle(&newton));
/// assert_eq!(None, newton.storage().body(Handle::null()));
/// ```
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct Handle(HandleInner);

impl Handle {
    /// Handle that references nothing.
    pub fn null() -> Self {
        Self::from_ptr(ptr::null())
    }

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

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd)]
enum HandleInner {
    Pointer(*const ()),
    Index(usize),
}

unsafe impl Send for Handle {}
unsafe impl Sync for Handle {}

pub trait FromHandle<'a>: Sized {
    /// Borrows object from newton storage.
    fn from_handle(newton: &'a Newton, handle: Handle) -> Option<Self>;

    /// Retakes ownership of an object stored in Newton.
    /// The returned object is destroyed after drop.
    fn from_handle_owned(newton: &'a mut Newton, handle: Handle) -> Option<Self>;
}

pub trait IntoHandle {
    /// Moves the object into the given Newton and returns a handle to
    /// borrow it or retake ownership of it later.
    ///
    /// ## Panics
    /// On both Bodies and Collisions, this method panics if the object
    /// is not owned, but you can still call `as_handle` on those.
    fn into_handle(self, newton: &Newton) -> Handle;
}

pub trait AsHandle {
    /// Returns the same vale that would be returned by `into_handle`, but
    /// without moving the object.
    fn as_handle(&self, newton: &Newton) -> Handle;
}
