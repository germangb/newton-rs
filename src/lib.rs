pub use ffi;

pub use body::{BodyOld, HandleOld as BodyHandle};
pub use collision::CollisionOld;
pub use math::{Matrix, Quaternion, Vector};
pub use world::Newton;

pub mod body;
pub mod collision;
pub mod math;
#[cfg(feature = "testbed")]
pub mod testbed;
pub mod world;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub struct Handle(pub(crate) *const ());