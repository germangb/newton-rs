use std::cell::RefCell;
use std::collections::{BTreeSet, HashSet};

use crate::body::{Body, NewtonBody};
use crate::collision::{Collision, NewtonCollision};
use crate::{Handle, HandleInner};

/// Data structure for Newton Bodies & Collisions.
pub trait NewtonStorage {
    /// Stores the given Newton Body.
    fn move_body(&self, body: Body) -> Handle;

    /// Stores the given collision.
    fn move_collision(&self, col: Collision) -> Handle;

    /// Borrows a Newton Body.
    fn body(&self, handle: Handle) -> Option<Body>;

    /// Borrows a Newton Collision.
    fn collision(&self, handle: Handle) -> Option<Collision>;

    /// Retakes ownership of a Newton Body.
    fn take_body(&mut self, handle: Handle) -> Option<Body>;

    /// Retakes ownership of a Newton Collision.
    fn take_collision(&mut self, handle: Handle) -> Option<Collision>;
}

macro_rules! set {
    ($( $(#[$($meta:meta)+])* struct $name:ident < $data_struct:ident > )*) => {
        $(
            #[derive(Debug, Default)]
            $(#[$($meta)+])*
            pub struct $name {
                bodies: RefCell<$data_struct<Handle>>,
                collisions: RefCell<$data_struct<Handle>>,
            }

            impl NewtonStorage for $name {
                fn move_body(&self, body: Body) -> Handle {
                    let handle = Handle::from_ptr(body.as_raw() as _);
                    self.bodies.borrow_mut().insert(handle.clone());
                    handle
                }

                fn move_collision(&self, col: Collision) -> Handle {
                    let handle = Handle::from_ptr(col.as_raw() as _);
                    self.collisions.borrow_mut().insert(handle.clone());
                    handle
                }

                fn body(&self, handle: Handle) -> Option<Body> {
                    let body = self.bodies.borrow().get(&handle).cloned();
                    unsafe {
                        body.map(|h| match h.inner() {
                            HandleInner::Pointer(ptr) => Body::from_raw(ptr as _, false),
                            _ => unimplemented!("index indexing"),
                        })
                    }
                }

                fn collision(&self, handle: Handle) -> Option<Collision> {
                    let collision = self.collisions.borrow().get(&handle).cloned();
                    unsafe {
                        collision.map(|h| match h.inner() {
                            HandleInner::Pointer(ptr) => Collision::from_raw(ptr as _, false),
                            _ => unimplemented!("index indexing"),
                        })
                    }
                }

                fn take_body(&mut self, handle: Handle) -> Option<Body> {
                    let body = self.bodies.borrow_mut().take(&handle);
                    unsafe {
                        body.map(|h| match h.inner() {
                            HandleInner::Pointer(ptr) => Body::from_raw(ptr as _, true),
                            _ => unimplemented!("index indexing"),
                        })
                    }
                }

                fn take_collision(&mut self, handle: Handle) -> Option<Collision> {
                    let collision = self.collisions.borrow_mut().take(&handle);
                    unsafe {
                        collision.map(|h| match h.inner() {
                            HandleInner::Pointer(ptr) => Collision::from_raw(ptr as _, true),
                            _ => unimplemented!("index indexing"),
                        })
                    }
                }
            }

        )*
    }
}

// We don't need to impl `Drop` because collisions and bodies are dropped when
// ffi::NewtonDestroyAllBodies is called in `Newton`'s Drop implementation.
set! {
    /// Storage of bodies & collisions in a HashSet.
    struct HashStorage<HashSet>

    /// Storage of bodies & collisions in a BTree.
    struct BTreeStorage<BTreeSet>
}
