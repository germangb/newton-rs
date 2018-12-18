extern crate cgmath;

pub mod body;
pub mod collision;
pub mod ffi;
pub mod traits;
pub mod world;

pub mod prelude {
    pub use crate::traits::*;
}

pub use body::NewtonBody;
pub use collision::{NewtonCollision, ShapeId};
pub use world::NewtonWorld;
