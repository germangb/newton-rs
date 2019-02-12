use std::collections::HashSet;
use std::marker::PhantomData;
use std::ptr::NonNull;
use std::sync::RwLock;
use std::time::Duration;

use super::body::{Body, BodyHandle};
use super::collision::{Collision, CollisionHandle};
use super::ffi;

/// A guard for the asynchronous update
///
/// Dropping this type blocks the thread until the world update has finished.
#[derive(Debug)]
pub struct AsyncUpdate<'world>(NonNull<ffi::NewtonWorld>, PhantomData<&'world ()>);

impl<'world> AsyncUpdate<'world> {
    /// Blocks the current thread until the world update has finished.
    pub fn finish(self) {}
}

impl<'world> Drop for AsyncUpdate<'world> {
    fn drop(&mut self) {
        unsafe { ffi::NewtonWaitForUpdateToFinish(self.0.as_ptr()) }
    }
}

#[derive(Debug)]
pub struct Newton {
    world: NonNull<ffi::NewtonWorld>,

    /// Collisions owned by the Newton type.
    ///
    /// Collisions in this collection are long lived and created when
    /// the application calls [`into_handle`][into_handle] on a borrowed Collision type.
    ///
    /// [into_handle]: #
    pub(crate) collisions: RwLock<HashSet<CollisionHandle>>,

    /// Bodies owned by the Newton type.
    ///
    /// Also created when the application calls the corresponding [`into_handle`] on a
    /// BodyHandle
    pub(crate) bodies: RwLock<HashSet<BodyHandle>>,
}

impl Newton {
    pub fn new() -> Self {
        unsafe {
            let world = ffi::NewtonCreate();
            Self {
                world: NonNull::new_unchecked(world),
                collisions: Default::default(),
                bodies: Default::default(),
            }
        }
    }

    /// Borrows a body owned by the newton context.
    ///
    /// Unlike with `body_owned`, the returned body doesn't get destroyed
    /// when the returned `Body` is dropped.
    pub fn body(&self, handle: &BodyHandle) -> Option<Body> {
        unimplemented!()
    }

    /// Takes ownership of a body stored in this Newton context.
    ///
    /// Use this method whenever you want to destroy a long-lived body
    /// referenced by a handle obtained obtained using the [`into_handle`][method]
    /// method.
    ///
    /// [method]: #
    pub fn body_owned(&mut self, handle: &BodyHandle) -> Option<Body> {
        unimplemented!()
    }

    /// Steps the simulation by a fixed amount (synchronous).
    ///
    /// This method is a wrapper for `NewtonUpdate`
    pub fn update(&mut self, step: Duration) {
        let seconds = step.as_secs() * 1_000_000_000 + step.subsec_nanos() as u64;
        unsafe { ffi::NewtonUpdate(self.as_ptr(), seconds as f32 / 1_000_000_000.0) }
    }

    /// Steps the simulation by a fixed amount (asynchronous)
    ///
    /// Unline `update`, this method doesn't block the current thread.
    pub fn update_async(&mut self, step: Duration) -> AsyncUpdate {
        let seconds = step.as_secs() * 1_000_000_000 + step.subsec_nanos() as u64;
        unsafe { ffi::NewtonUpdateAsync(self.as_ptr(), seconds as f32 / 1_000_000_000.0) }

        AsyncUpdate(self.world, PhantomData)
    }

    pub const fn as_ptr(&self) -> *const ffi::NewtonWorld {
        self.world.as_ptr()
    }

    #[rustfmt::skip]
    fn drop_owned_collisions(&mut self) {
        let set = self.collisions.read().unwrap();
        set.iter()
            .map(|col| Collision { newton: self, collision: col.0, owned: true, })
            .for_each(drop);
    }

    #[rustfmt::skip]
    fn drop_owned_bodies(&mut self) {
        let set = self.bodies.read().unwrap();
        set.iter()
            .map(|body| Body { newton: self, body: body.0, owned: true, })
            .for_each(drop);
    }
}

impl Drop for Newton {
    fn drop(&mut self) {
        unsafe {
            self.drop_owned_collisions();
            self.drop_owned_bodies();
            ffi::NewtonDestroyAllBodies(self.as_ptr());
            ffi::NewtonDestroy(self.as_ptr());
        }
    }
}
