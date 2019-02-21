use std::os::raw::c_void;

pub use ffi;

// reexports
pub use body::{Body, DynamicBody, KinematicBody, SleepState, Type as BodyType};
pub use collision::{
    Capsule, Collision, Compound, Cone, Cuboid, Cylinder, Null, Scene, Sphere, Tree,
    Type as CollisionType,
};
pub use world::{AsyncUpdate, Newton};

/// Dynamic & kinematic body wrappers.
pub mod body;
/// NewtonCollision wrappers
pub mod collision;
/// Reexports commonly used traits & types.
pub mod prelude {
    pub use super::body::{IntoBody, NewtonBody};
    pub use super::collision::{IntoCollision, NewtonCollision};
    pub use super::IntoHandle;
}
/// Framework to inspect Newton simulations.
#[cfg(all(feature = "testbed", not(feature = "ci")))]
pub mod testbed;

// mock for CI
#[cfg(all(feature = "testbed", feature = "ci"))]
pub mod testbed {
    pub trait Demo {
        fn reset(newton: &mut super::world::Newton) -> Self;
    }
    pub fn run<T: Demo + Send + Sync>() {
        panic!("Newton Testbed can't run on a CI environment");
    }
}

mod world;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum Handle {
    Pointer(*const ()),
    Index(usize),
}

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
