use std::collections::HashSet;
use std::marker::PhantomData;
use std::ptr::NonNull;
use std::sync::RwLock;
use std::time::Duration;

use super::body::{Body, BodyHandle};
use super::collision::{Collision, CollisionHandle};

use super::ffi;

// TODO:
//   - NewtonSetPosUpdateCallback
//   - NewtonSetMultiThreadSolverOnSingleIsland
#[derive(Debug, Default)]
pub struct Builder {
    /// Possible values:
    ///   * `None` Exact solver (used by default)
    ///   * `Some(n)` Linear solver (n iterations)
    solver: Option<usize>,

    /// To run the physics step simulation is multiple threads
    /// (Threads are not managed by Rust)
    threads: Option<usize>,

    /// Sets the contact merge tolerance (ffi::NewtonSetContactMergeTolerance)
    ///
    /// If the variant is `None`, the function is not called.
    merge_tolerance: Option<f32>,

    /// Sets the number of sumulation substeps by calling
    /// `ffi::NewtonSetNumberOfSubsteps` if not a `None`
    substeps: Option<usize>,
}

impl Builder {
    /// Sets the solver model to linear with a fixed iteration number.
    pub fn linear_solver(&mut self, iters: usize) -> &mut Self {
        assert_ne!(0, iters);
        self.solver = Some(iters);
        self
    }

    /// Sets the solver model to the default one.
    ///
    /// If this value is set, the wrapper calls `NewtonSetSolverModel`.
    pub fn exact_solver(&mut self) -> &mut Self {
        self.solver = None;
        self
    }

    /// Sets the simulation number of substeps.
    ///
    /// The ffi function that is called as a result is `NewtonSetNumberOfSubsteps`.
    /// If `None`, the function is not called and a default value is used.
    pub fn substeps(&mut self, substeps: usize) -> &mut Self {
        // TODO can it be 0?
        //assert_ne!(0, substeps);
        self.substeps = Some(substeps);
        self
    }

    /// Sets the number of threads to run the simulation one.
    ///
    /// By no calling this method, the simulation will be single-threaded.
    /// You may call `threads_max` to set it to the # of CPU cores.
    pub fn threads(&mut self, threads: usize) -> &mut Self {
        self.threads = Some(threads);
        self
    }

    /// Sets number of threads to the maximum number of CPU cores.
    pub fn cpu_threads(&mut self) -> &mut Self {
        self.threads = Some(num_cpus::get());
        self
    }

    pub fn build(&self) -> Newton {
        unsafe {
            let world = ffi::NewtonCreate();
            if let Some(threads) = self.threads {
                ffi::NewtonSetThreadsCount(world, threads as _);
            }
            if let Some(steps) = self.solver {
                ffi::NewtonSetSolverModel(world, steps as _);
            }
            if let Some(tol) = self.merge_tolerance {
                ffi::NewtonSetContactMergeTolerance(world, tol);
            }
            if let Some(subs) = self.substeps {
                ffi::NewtonSetNumberOfSubsteps(world, subs as _);
            }
            Newton {
                world: NonNull::new_unchecked(world),
                collisions: Default::default(),
                bodies: Default::default(),
            }
        }
    }
}

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

impl Default for Newton {
    #[inline]
    fn default() -> Self {
        Self::builder().build()
    }
}

impl Newton {
    pub fn builder() -> Builder {
        Builder::default()
    }

    /// Steps the simulation by a fixed amount (synchronous).
    ///
    /// This method is a wrapper for `NewtonUpdate`
    pub fn update(&mut self, step: Duration) {
        let seconds = step.as_secs() * 1_000_000_000 + step.subsec_nanos() as u64;
        unsafe { ffi::NewtonUpdate(self.as_ptr(), seconds as f32 / 1_000_000_000.0) }
    }

    pub fn collision(&self, handle: &CollisionHandle) -> Option<Collision> {
        self.collisions
            .read()
            .expect("Collision set")
            .get(handle)
            .map(|handle| Collision {
                newton: self,
                collision: handle.0,
                owned: false,
            })
    }

    pub fn body(&self, handle: &BodyHandle) -> Option<Body> {
        self.bodies
            .read()
            .expect("Body set")
            .get(handle)
            .map(|handle| Body {
                newton: self,
                body: handle.0,
                owned: false,
            })
    }

    /// Takes ownership of the given collision
    ///
    /// If you don't intend to drop the collision, you must call `into_handle` on the
    /// returned type, or use the `collision` method instead.
    // TODO borrow self mutably?
    pub fn collision_owned(&self, handle: &CollisionHandle) -> Option<Collision> {
        self.collisions
            .write()
            .expect("Collision set")
            .take(handle)
            .map(|handle| Collision {
                newton: self,
                collision: handle.0,
                owned: true,
            })
    }

    /// Takes ownership of the given body.
    pub fn body_owned(&self, handle: &BodyHandle) -> Option<Body> {
        self.bodies
            .write()
            .expect("Collision set")
            .take(handle)
            .map(|handle| Body {
                newton: self,
                body: handle.0,
                owned: true,
            })
    }

    /// Steps the simulation by a fixed amount (asynchronous)
    ///
    /// Unline `update`, this method doesn't block the current thread.
    pub fn update_async(&mut self, step: Duration) -> AsyncUpdate {
        let seconds = step.as_secs() * 1_000_000_000 + step.subsec_nanos() as u64;
        unsafe { ffi::NewtonUpdateAsync(self.as_ptr(), seconds as f32 / 1_000_000_000.0) }

        AsyncUpdate(self.world, PhantomData)
    }

    /// Returns the wrapped pointer.
    pub const fn as_ptr(&self) -> *const ffi::NewtonWorld {
        self.world.as_ptr()
    }

    pub(crate) fn move_collision(&self, mut collision: Collision) -> CollisionHandle {
        collision.owned = false;
        let handle = CollisionHandle(collision.collision);

        // transfer ownership to Newton
        self.collisions
            .write()
            .expect("Collision set")
            .insert(handle.clone());

        handle
    }

    pub(crate) fn move_body(&self, mut body: Body) -> BodyHandle {
        body.owned = false;
        let handle = BodyHandle(body.body);

        // transfer ownership to Newton
        self.bodies
            .write()
            .expect("Collision set")
            .insert(handle.clone());

        handle
    }

    fn drop_owned_collisions(&self) {
        self.collisions
            .read()
            .expect("Owned collisions set")
            .iter()
            .map(|h| Collision {
                newton: self,
                collision: h.0,
                owned: true,
            })
            .for_each(drop);
    }

    fn drop_owned_bodies(&self) {
        self.bodies
            .read()
            .expect("Owned body set")
            .iter()
            .map(|h| Body {
                newton: self,
                body: h.0,
                owned: true,
            })
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
