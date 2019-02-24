//! A safe wrapper around the [newton-dynamics][newton] C API
//!
//! - **Newton version: 3.13a**
//!
//! [newton]: https://github.com/MADEAPPS/newton-dynamics
include!("macros.rs");

pub use body::{Body, DynamicBody, KinematicBody};
pub use collision::{
    ChamferCylinder, Collision, Compound, Cone, Cuboid, Cylinder, DeformableSolid,
    FracturedCompound, MassSpringDamperSystem, Null, Scene, Sphere, Tree,
};
pub use ffi;
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

/// ```ignore
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
    pub fn null() -> Self {
        Self::from_ptr(std::ptr::null())
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
