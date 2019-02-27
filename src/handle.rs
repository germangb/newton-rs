//! ```
//! use newton::prelude::*;
//! use newton::handle::Handle;
//! use newton::{Newton, Sphere, Collision};
//!
//! let newton = Newton::create();
//!
//! let sphere = Sphere::create(&newton, 1.0, None);
//!
//! sphere.set_user_id(42);
//!
//! let sphere = sphere.into_handle(&newton);
//!
//! let col = newton.storage().collision(sphere);
//!
//! assert_eq!(Some(42), col.map(|b| b.user_id()));
//! assert_eq!(None, newton.storage().collision(Handle::null()));
//! ```
use crate::newton::Newton;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct Handle(HandleInner);

impl Handle {
    pub fn null() -> Self {
        Self::from_ptr(std::ptr::null())
    }

    pub(crate) fn from_usize(idx: usize) -> Self {
        Self(HandleInner::Index(idx))
    }

    pub(crate) fn from_ptr(ptr: *const ()) -> Self {
        Self(HandleInner::Pointer(ptr))
    }

    pub(crate) fn inner(&self) -> HandleInner {
        self.0
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub(crate) enum HandleInner {
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
    fn into_handle(self, newton: &Newton) -> Handle;
}

pub trait AsHandle {
    /// Returns the same vale that would be returned by `into_handle`, but
    /// without moving the object.
    fn as_handle(&self, newton: &Newton) -> Handle;
}
