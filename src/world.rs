use std::collections::HashSet;
use std::mem;
use std::sync::RwLock;
use std::time::Duration;

use super::body::{Body, Handle as BodyHandle};
use super::collision::{Collision, Handle as CollisionHandle};
use super::ffi;

/// A guard for the asynchronous update
///
/// Dropping this type blocks the thread until the world update has finished.
#[derive(Debug)]
pub struct AsyncUpdate<'world>(&'world mut Newton);

impl<'world> Drop for AsyncUpdate<'world> {
    fn drop(&mut self) {
        unsafe { ffi::NewtonWaitForUpdateToFinish(self.0.as_ptr()) }
    }
}

// TODO generalize handle storage
//      NewtonStorage trait that defaults to HashSet?
#[derive(Debug, Default)]
pub struct NewtonData {
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

#[derive(Debug)]
pub struct Newton(pub(crate) *const ffi::NewtonWorld);

impl Newton {
    pub fn create() -> Self {
        unsafe {
            let world = ffi::NewtonCreate();
            let udata = Box::new(NewtonData::default());
            ffi::NewtonWorldSetUserData(world, mem::transmute(udata));
            Self(world)
        }
    }

    /// Sets the solver model.
    ///
    /// - `steps = 0` Exact solver (default one)
    /// - `step > 0` Linear solver with the given number of linear steps.
    pub fn set_solver(&mut self, steps: usize) {
        unsafe { ffi::NewtonSetSolverModel(self.as_ptr(), steps as _) };
    }

    /// Sets the number of threads Newton will run the simulation on.
    /// By default, the simulation is single-threaded.
    pub fn set_threads(&mut self, threads: usize) {
        unsafe { ffi::NewtonSetThreadsCount(self.as_ptr(), threads as _) };
    }

    /// Borrows a body owned by the newton context.
    ///
    /// Unlike with `body_owned`, the returned body doesn't get destroyed
    /// when the returned `Body` is dropped.
    pub fn body(&self, handle: &BodyHandle) -> Option<Body> {
        let data = unsafe { userdata(self.as_ptr()) };
        let body = data.bodies.read().unwrap().get(handle).map(|h| Body {
            newton: self,
            body: h.0,
            owned: false,
        });

        unsafe { mem::forget(data) };
        body
    }

    /// Consumes body without dropping it
    pub(crate) fn leak(self) {
        unsafe { mem::forget(self) };
    }

    pub(crate) fn move_body(&self, mut body: Body) -> BodyHandle {
        body.owned = false;
        let data = unsafe { userdata(self.as_ptr()) };
        let handle = BodyHandle(body.body);
        data.bodies.write().unwrap().insert(handle.clone());
        unsafe { mem::forget(data) };
        handle
    }

    pub(crate) fn move_collision(&self, mut collision: Collision) -> CollisionHandle {
        collision.owned = false;
        let data = unsafe { userdata(self.as_ptr()) };
        let handle = CollisionHandle(collision.collision);
        data.collisions.write().unwrap().insert(handle.clone());
        unsafe { mem::forget(data) };
        handle
    }

    /// Takes ownership of a body stored in this Newton context.
    ///
    /// Use this method whenever you want to destroy a long-lived body
    /// referenced by a handle obtained obtained using the [`into_handle`][method]
    /// method.
    ///
    /// [method]: #
    pub fn body_owned(&mut self, handle: &BodyHandle) -> Option<Body> {
        let data = unsafe { userdata(self.as_ptr()) };
        let handle = data.bodies.write().unwrap().take(handle);
        unsafe { mem::forget(data) };

        if let Some(handle) = handle {
            Some(Body {
                newton: self,
                body: handle.0,
                owned: true,
            })
        } else {
            None
        }
    }

    pub const fn as_ptr(&self) -> *const ffi::NewtonWorld {
        self.0
    }

    /// Drops any bodies owned by this NewtonWorld
    #[rustfmt::skip]
    pub fn drop_collisions(&mut self) {
        let data = unsafe { userdata(self.as_ptr()) };
        let set = data.collisions.read().unwrap();
        set.iter()
            .map(|col| Collision { newton: self, collision: col.0, owned: true, })
            .for_each(drop);
        drop(set);
        unsafe { mem::forget(data) };
    }

    /// Drops any bodies owned by this NewtonWorld
    pub fn drop_bodies(&mut self) {
        let data = unsafe { userdata(self.as_ptr()) };
        let set = data.bodies.read().unwrap();
        set.iter()
            .map(|bod| Body {
                newton: self,
                body: bod.0,
                owned: true,
            })
            .for_each(drop);
        drop(set);
        unsafe { mem::forget(data) };
    }
}

/// ffi wrappers
impl Newton {
    /// Invalidated any cached contacts information.
    ///
    /// Useful for when you synchronize a simulation over a network and need
    /// to run the simulation deterministically.
    pub fn invalidate(&mut self) {
        unsafe { ffi::NewtonInvalidateCache(self.as_ptr()) }
    }

    /// Steps the simulation by a fixed amount (synchronous).
    pub fn update(&mut self, step: Duration) {
        let seconds = step.as_secs() * 1_000_000_000 + step.subsec_nanos() as u64;
        unsafe { ffi::NewtonUpdate(self.as_ptr(), seconds as f32 / 1_000_000_000.0) }
    }

    /// Steps the simulation by a fixed amount (asynchronous) without blocking the
    /// current thread of execution.
    ///
    /// ## Notes
    ///
    /// If you intend to perform network synchronization, synchronous `update` is
    /// the preferred approach.
    pub fn update_async(&mut self, step: Duration) -> AsyncUpdate {
        let seconds = step.as_secs() * 1_000_000_000 + step.subsec_nanos() as u64;
        unsafe { ffi::NewtonUpdateAsync(self.as_ptr(), seconds as f32 / 1_000_000_000.0) }
        AsyncUpdate(self)
    }
}

impl Drop for Newton {
    fn drop(&mut self) {
        unsafe {
            self.drop_collisions();
            self.drop_bodies();

            let _ = userdata(self.as_ptr());
            ffi::NewtonDestroyAllBodies(self.as_ptr());
            ffi::NewtonDestroy(self.as_ptr());
        }
    }
}

unsafe fn userdata(ptr: *const ffi::NewtonWorld) -> Box<NewtonData> {
    Box::from_raw(ffi::NewtonWorldGetUserData(ptr) as _)
}
