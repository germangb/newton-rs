pub use ffi;

// reexports
pub use body::{Body, DynamicBody, KinematicBody};
pub use collision::{
    BoxCollision, CapsuleCollision, Collision, CompoundCollision, ConeCollision, CylinderCollision,
    NullCollision, SphereCollision, TreeCollision,
};
pub use world::Newton;

pub mod body;
pub mod collision;
pub mod prelude {
    pub use super::body::{IntoBody, NewtonBody};
    pub use super::collision::{IntoCollision, NewtonCollision};
    pub use super::IntoHandle;
}
#[cfg(feature = "testbed")]
pub mod testbed;
pub mod world;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub struct Handle(pub(crate) *const ());

// Generic over T because it is implemented twice, with different trait bounds.
// The compiler doesn't like that because the trait bounds could overlap, event though
// it doesn't happen in practice.
pub trait IntoHandle<T> {
    fn into_handle(self, newton: &Newton) -> Handle;

    /// Returns the same vale that would be returned by `into_handle`, but
    /// without moving the object.
    fn as_handle(&self) -> Handle;
}

unsafe impl Send for Handle {}
unsafe impl Sync for Handle {}
