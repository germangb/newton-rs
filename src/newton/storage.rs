use std::collections::{BTreeSet, HashSet};
use std::sync::RwLock;

use crate::body::{Body, NewtonBody};
use crate::collision::{Collision, NewtonCollision};
use crate::handle::{Handle, HandleInner};
use crate::joint::Joint;

/// Data structure for Newton Bodies & Collisions.
pub trait NewtonStorage {
    /// Stores the given Newton Body.
    fn move_body(&self, body: Body) -> Handle;

    /// Stores the given collision.
    fn move_collision(&self, col: Collision) -> Handle;

    /// Stores the given constraint
    fn move_constraint(&self, con: Joint) -> Handle;

    /// Borrows a Newton Body.
    fn body(&self, handle: Handle) -> Option<Body>;

    /// Borrows a Newton Collision.
    fn collision(&self, handle: Handle) -> Option<Collision>;

    /// Borrows constraint
    fn constraint(&self, handle: Handle) -> Option<Joint>;

    /// Retakes ownership of a Newton Body.
    fn take_body(&mut self, handle: Handle) -> Option<Body>;

    /// Retakes ownership of a Newton Collision.
    fn take_collision(&mut self, handle: Handle) -> Option<Collision>;

    fn take_constraint(&mut self, handle: Handle) -> Option<Collision>;
}

macro_rules! set {
    ($( $(#[$($meta:meta)+])* struct $name:ident < $data_struct:ident > )*) => {
        $(
            #[derive(Debug, Default)]
            $(#[$($meta)+])*
            pub struct $name {
                bodies: RwLock<$data_struct<Handle>>,
                collisions: RwLock<$data_struct<Handle>>,
            }

            impl Drop for $name {
                fn drop(&mut self) {
                    let collisions = self.collisions.read().unwrap();
                    for col in collisions.iter() {
                        if let HandleInner::Pointer(col) = col.inner() {
                            unsafe {
                                let _ = Collision::from_raw(col as _, true);
                            }
                        }
                    }
                }
            }

            impl NewtonStorage for $name {
                fn move_body(&self, body: Body) -> Handle {
                    let handle = Handle::from_ptr(body.as_raw() as _);
                    self.bodies.write().unwrap().insert(handle.clone());
                    handle
                }

                fn move_collision(&self, col: Collision) -> Handle {
                    let handle = Handle::from_ptr(col.as_raw() as _);
                    self.collisions.write().unwrap().insert(handle.clone());
                    handle
                }

                fn move_constraint(&self, con: Joint) -> Handle {
                    unimplemented!()
                }

                fn body(&self, handle: Handle) -> Option<Body> {
                    let body = self.bodies.read().unwrap().get(&handle).cloned();
                    unsafe {
                        body.map(|h| match h.inner() {
                            HandleInner::Pointer(ptr) => Body::from_raw(ptr as _, false),
                            _ => unimplemented!("index indexing"),
                        })
                    }
                }

                fn collision(&self, handle: Handle) -> Option<Collision> {
                    let collision = self.collisions.read().unwrap().get(&handle).cloned();
                    unsafe {
                        collision.map(|h| match h.inner() {
                            HandleInner::Pointer(ptr) => Collision::from_raw(ptr as _, false),
                            _ => unimplemented!("index indexing"),
                        })
                    }
                }

                fn constraint(&self, handle: Handle) -> Option<Joint> {
                    unimplemented!()
                }

                fn take_body(&mut self, handle: Handle) -> Option<Body> {
                    let body = self.bodies.write().unwrap().take(&handle);
                    unsafe {
                        body.map(|h| match h.inner() {
                            HandleInner::Pointer(ptr) => Body::from_raw(ptr as _, true),
                            _ => unimplemented!("index indexing"),
                        })
                    }
                }

                fn take_collision(&mut self, handle: Handle) -> Option<Collision> {
                    let collision = self.collisions.write().unwrap().take(&handle);
                    unsafe {
                        collision.map(|h| match h.inner() {
                            HandleInner::Pointer(ptr) => Collision::from_raw(ptr as _, true),
                            _ => unimplemented!("index indexing"),
                        })
                    }
                }

                fn take_constraint(&mut self, handle: Handle) -> Option<Collision> {
                    unimplemented!()
                }
            }

        )*
    }
}

set! {
    /// Storage of bodies & collisions in a HashSet.
    struct HashStorage<HashSet>

    /// Storage of bodies & collisions in a BTree.
    struct BTreeStorage<BTreeSet>
}
