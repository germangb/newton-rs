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

#[cfg(feature = "arc")]
pub type RefCount<T> = std::sync::Arc<T>;

#[cfg(not(feature = "arc"))]
pub type RefCount<T> = std::rc::Rc<T>;

macro_rules! newton_ref {
    (
        $(#[$meta:meta])+
        pub struct $struct:ident(*mut ffi::$ptr:ident) => ffi::$drop:ident;
    ) => {
        $(#[$meta])+
        pub struct $struct {
            pub(crate) raw: *mut ffi::$ptr,
            owned: bool,
        }
        impl $struct {
            pub unsafe fn from_raw_parts(raw: *mut ffi::$ptr, owned: bool) -> Self {
                $struct { raw, owned }
            }
        }
        impl Drop for $struct {
            fn drop(&mut self) {
                if self.owned {
                    unsafe { ffi::$drop(self.raw) }
                }
            }
        }
    }
}

newton_ref! {
    #[doc(hidden)]
    #[derive(Debug, Clone)]
    pub struct WorldRef(*mut ffi::NewtonWorld) => ffi::NewtonDestroy;
}

newton_ref! {
    #[doc(hidden)]
    #[derive(Debug, Clone)]
    pub struct CollisionRef(*mut ffi::NewtonCollision) => ffi::NewtonDestroyCollision;
}

newton_ref! {
    #[doc(hidden)]
    #[derive(Debug, Clone)]
    pub struct BodyRef(*mut ffi::NewtonBody) => ffi::NewtonDestroyBody;
}
