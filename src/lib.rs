extern crate cgmath;

pub mod body;
pub mod collision;
pub mod ffi;
pub mod traits;
pub mod world;

pub mod prelude {
    pub use crate::traits::*;
}

#[cfg(feature = "arc")]
pub type RefCount<T> = std::sync::Arc<T>;

#[cfg(not(feature = "arc"))]
pub type RefCount<T> = std::rc::Rc<T>;

macro_rules! newton_ref {
    (
        $(#[$meta:meta])+
        pub struct $struct:ident(*mut ffi::$ptr:ident ) => ffi::$drop:ident;
    ) => {
        $(#[$meta])+
        pub struct $struct(pub(crate) *mut ffi::$ptr);
        impl Drop for $struct {
            fn drop(&mut self) {
                unsafe { ffi::$drop(self.0) }
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
