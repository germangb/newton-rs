use std::marker::PhantomData;
use std::os::raw::c_void;

use crate::collision::Collision;
use crate::ffi;
use crate::handle::{Handle, HandleInner};

/// An iterator that yields collision handles from a compound or a scene.
///
/// The type is normally constructed by calling the `handles` method
/// on either a Compound or Scene collision.
pub struct Handles<'a> {
    pub(super) collision: *const ffi::NewtonCollision,
    pub(super) next: *const c_void,
    pub(super) get_next: Box<dyn Fn(*const ffi::NewtonCollision, *const c_void) -> *const c_void>,
    pub(super) _phantom: PhantomData<&'a ()>,
}

/// An iterator that yields the collisions from a compound or scene collision.
///
/// The type is normally constructed by calling the `collisions` method
/// on either a Compound or Scene collision.
pub struct Collisions<'a> {
    pub(crate) handles: Handles<'a>,
    pub(crate) get_col:
        Box<dyn Fn(*const ffi::NewtonCollision, *const c_void) -> *const ffi::NewtonCollision>,
}

impl<'a> Iterator for Handles<'a> {
    type Item = Handle;

    fn next(&mut self) -> Option<Self::Item> {
        let current = self.next;
        if current.is_null() {
            None
        } else {
            self.next = (self.get_next)(self.collision, current);
            Some(Handle::from_ptr(current as _))
        }
    }
}

impl<'a> Iterator for Collisions<'a> {
    type Item = Collision<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let collision = self.handles.collision;
        self.handles.next().and_then(|h| unsafe {
                               match h.inner() {
                                   HandleInner::Index(_) => panic!("Unexpected index handle."),
                                   HandleInner::Pointer(ptr) => {
                                       let col = (self.get_col)(collision, ptr as _);
                                       Some(Collision::from_raw(col, false))
                                   }
                               }
                           })
    }
}
