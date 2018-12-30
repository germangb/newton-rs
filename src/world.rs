use ffi;

use super::body::{Bodies, Body, BodyLocked, BodyLockedMut, Builder as BodyBuilder, NewtonBody};
use super::command::Command;
use super::{channel, Lock, LockError, Locked, LockedMut, Result, Rx, Shared, Tx, Types, Weak};
use crate::collision::{Builder as CollisionBuilder, NewtonCollision};

use crate::body::BodyData;
use std::{
    marker::PhantomData,
    mem,
    num::NonZeroU32,
    ops::{Deref, DerefMut},
    os::raw,
    ptr,
    sync::mpsc,
    time::Duration,
};

pub struct Builder<T> {
    solver: Solver,
    threads: usize,
    broad: Broadphase,
    debug: Option<&'static str>,
    _phantom: PhantomData<T>,
}

impl<T> Builder<T> {
    pub fn threads(mut self, th: usize) -> Self {
        self.threads = th;
        self
    }

    /// Sets the number of threads to the number of CPUs available in the system.
    pub fn max_threads(mut self) -> Self {
        self.threads = num_cpus::get();
        self
    }

    pub fn debug(mut self, name: &'static str) -> Self {
        self.debug = Some(name);
        self
    }

    pub fn exact_solver(mut self) -> Self {
        self.solver = Solver::Exact;
        self
    }

    pub fn linear_solver(mut self, steps: usize) -> Self {
        self.solver = Solver::Linear(steps);
        self
    }

    pub fn default_broadphase(mut self) -> Self {
        self.broad = Broadphase::Default;
        self
    }

    pub fn persistent_broadphase(mut self) -> Self {
        self.broad = Broadphase::Persistent;
        self
    }

    pub fn build(self) -> World<T> {
        World::new(self.broad, self.solver, self.threads, self.debug)
    }
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum Broadphase {
    Default = ffi::NEWTON_BROADPHASE_DEFAULT as _,
    Persistent = ffi::NEWTON_BROADPHASE_PERSINTENT as _,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum Solver {
    /// Newton's exact solver
    Exact,
    /// Limit the number of linear steps (`n > 0`). Lower steps is faster but less accurate
    Linear(usize),
}

/// A world is the heart of the physics simulation.
#[derive(Debug, Clone)]
pub struct World<T>(Shared<Lock<NewtonWorld<T>>>, *const ffi::NewtonWorld);

unsafe impl<T> Send for World<T> {}
unsafe impl<T> Sync for World<T> {}

#[derive(Debug)]
pub struct NewtonWorld<T> {
    world: *mut ffi::NewtonWorld,

    /// Tx end of the command channel.
    pub(crate) tx: Tx<Command>,
    /// Rx end of the command channel, used by NewtonBody to signal the destruction of a body.
    rx: Rx<Command>,

    _phantom: PhantomData<T>,
}

fn create_world<T>() -> NewtonWorld<T> {
    let world = unsafe { ffi::NewtonCreate() };
    let (tx, rx) = channel();
    NewtonWorld {
        world,
        tx,
        rx,
        _phantom: PhantomData,
    }
}

#[doc(hidden)]
#[derive(Debug)]
pub struct WorldData<T> {
    /// We keep a reference to the world so we can obtain a copy of the lock from a `NewtonWorld`
    /// when needed.
    pub(crate) world: Weak<Lock<NewtonWorld<T>>>,

    /// Debug name
    debug: Option<&'static str>,

    /// A Tx end of the command channel, to recover it quickly from the userdata.
    tx: Tx<Command>,
}

/// A read-only lock of a newton world.
#[derive(Debug)]
pub struct WorldLocked<'a, T>(Locked<'a, NewtonWorld<T>>);

#[derive(Debug)]
pub struct WorldLockedMut<'a, T>(LockedMut<'a, NewtonWorld<T>>);

/// A type returned by the call to the asynchronous world update. When this type is dropped, the current
/// thread is blocked until the simulation step has finished.
#[derive(Debug)]
pub struct WorldUpdateAsync<'a, T>(*mut ffi::NewtonWorld, PhantomData<&'a T>);

unsafe impl<'a, T> Send for WorldUpdateAsync<'a, T> {}
unsafe impl<'a, T> Sync for WorldUpdateAsync<'a, T> {}

impl<'a, T> WorldUpdateAsync<'a, T> {
    /// Consumes the object and blocks the current thread until the world update is finished.
    pub fn finish(self) {}
}

impl<T> World<T> {
    pub fn builder() -> Builder<T> {
        Builder {
            solver: Solver::Exact,
            threads: 1,
            broad: Broadphase::Default,
            debug: None,
            _phantom: PhantomData,
        }
    }

    /// Creates a new World
    pub fn new(
        broadphase: Broadphase,
        solver: Solver,
        threads: usize,
        debug: Option<&'static str>,
    ) -> Self {
        let newton_world = create_world::<T>();

        let world_raw = newton_world.world;
        let tx = newton_world.tx.clone();
        let lock = Shared::new(Lock::new(newton_world));

        let userdata = Shared::new(WorldData {
            world: Shared::downgrade(&lock),
            tx,
            debug,
        });

        if threads > 1 {
            unsafe {
                ffi::NewtonSetThreadsCount(world_raw, threads as _);
            }
        }

        unsafe {
            match solver {
                Solver::Exact => ffi::NewtonSetSolverModel(world_raw, 0),
                Solver::Linear(n) => ffi::NewtonSetSolverModel(world_raw, n as _),
            }
        }

        unsafe {
            ffi::NewtonWorldSetUserData(world_raw, mem::transmute(userdata));
        }
        World(lock, world_raw)
    }

    #[inline]
    pub fn try_read(&self) -> Result<WorldLocked<T>> {
        Ok(WorldLocked(self.0.try_read()?))
    }

    #[inline]
    pub fn try_write(&self) -> Result<WorldLockedMut<T>> {
        #[cfg(feature = "debug")]
        {
            let udata = unsafe { userdata::<T>(self.1) };
            Ok(WorldLockedMut(self.0.try_write(udata.debug)?))
        }
        #[cfg(not(feature = "debug"))]
        {
            Ok(WorldLockedMut(self.0.try_write(None)?))
        }
    }

    #[inline]
    pub fn read(&self) -> WorldLocked<T> {
        self.try_read().unwrap()
    }

    #[inline]
    pub fn write(&self) -> WorldLockedMut<T> {
        self.try_write().unwrap()
    }
}

#[derive(Debug)]
pub struct ConvexCast<'a, T> {
    count: usize,
    contacts: Vec<ffi::NewtonWorldConvexCastReturnInfo>,
    body: *mut NewtonBody<T>,
    _phantom: PhantomData<&'a T>,
}

impl<'a, T> Drop for ConvexCast<'a, T> {
    fn drop(&mut self) {
        unsafe {
            let _ = Box::from_raw(self.body);
        }
    }
}

#[derive(Debug)]
pub struct CastInfo<V> {
    pub point: V,
    pub normal: V,
    pub penetration: f32,
}

// TODO test, make sure collision matches the body
// TODO FIXME refactor...
impl<'a, T: Types> Iterator for ConvexCast<'a, T> {
    type Item = (&'a NewtonBody<T>, CastInfo<T::Vector>);

    fn next(&mut self) -> Option<Self::Item> {
        let contact = self.contacts.get(self.count).map(|info| {
            let mut boxed = unsafe { Box::from_raw(self.body) };

            boxed.body = info.m_hitBody as _;
            unsafe {
                boxed.collision.collision = ffi::NewtonBodyGetCollision(info.m_hitBody);
            }

            let (point, normal) = unsafe {
                let mut point: T::Vector = mem::zeroed();
                let mut normal: T::Vector = mem::zeroed();

                ptr::copy(info.m_point.as_ptr(), &mut point as *mut _ as *mut f32, 3);
                ptr::copy(info.m_normal.as_ptr(), &mut normal as *mut _ as *mut f32, 3);

                (point, normal)
            };

            let info = CastInfo {
                penetration: info.m_penetration,
                point,
                normal,
            };

            unsafe { (mem::transmute(Box::into_raw(boxed)), info) }
        });

        self.count += 1;
        contact
    }
}

impl<T: Types> NewtonWorld<T> {
    pub fn convex_cast<C>(
        &self,
        matrix: &T::Matrix,
        target: &T::Vector,
        shape: &NewtonCollision<T>,
        max_contacts: usize,
        prefilter: C,
    ) -> ConvexCast<T>
    where
        C: Fn(&NewtonBody<T>, &NewtonCollision<T>) -> bool + 'static,
    {
        let mut return_info = Vec::with_capacity(max_contacts);

        let userdata = unsafe { userdata::<T>(self.world) };

        let prefilter_world = (prefilter, userdata.clone());
        let contacts = unsafe {
            ffi::NewtonWorldConvexCast(
                self.world,
                mem::transmute(matrix),
                mem::transmute(target),
                shape.as_raw(),
                // hitParam
                &mut 0.0,
                // userdata
                mem::transmute(&prefilter_world),
                Some(prefilter_callback::<T, C>),
                // cast info
                return_info.as_mut_ptr(),
                max_contacts as raw::c_int,
                // thread index
                0,
            )
        };

        let world = Weak::upgrade(&userdata.world).unwrap();
        let body = unsafe { Box::new(NewtonBody::null(world)) };

        unsafe {
            return_info.set_len(contacts as usize);

            return ConvexCast {
                count: 0,
                contacts: return_info,
                body: Box::into_raw(body),
                _phantom: PhantomData,
            };
        }

        unsafe extern "C" fn prefilter_callback<T, C>(
            body: *const ffi::NewtonBody,
            collision: *const ffi::NewtonCollision,
            udata: *const raw::c_void,
        ) -> raw::c_uint
        where
            T: Types,
            C: Fn(&NewtonBody<T>, &NewtonCollision<T>) -> bool + 'static,
        {
            let (ref callback, ref world): &(C, Shared<WorldData<T>>) = mem::transmute(udata);

            let body = NewtonBody::new_not_owned(body as _);
            if callback(&body, &body.collision) {
                1
            } else {
                0
            }
        }
    }
}

impl<T> NewtonWorld<T> {
    pub fn bodies(&self) -> Bodies<T> {
        unsafe {
            let world = self.world;
            let first = ffi::NewtonWorldGetFirstBody(world);
            let body = Box::new(NewtonBody::new_not_owned(first));

            Bodies {
                world,
                next: first,
                body: Box::into_raw(body),
                _phantom: PhantomData,
            }
        }
    }

    pub fn body_count(&self) -> raw::c_int {
        unsafe { ffi::NewtonWorldGetBodyCount(self.world) }
    }

    pub fn constraint_count(&self) -> raw::c_int {
        unsafe { ffi::NewtonWorldGetConstraintCount(self.world) }
    }

    /// Non-blocking variant of the `update` method.
    pub fn update_async(&mut self, step: Duration) -> WorldUpdateAsync<T> {
        self.flush_commands();
        unsafe {
            ffi::NewtonUpdateAsync(self.world, as_seconds(step));
        }
        WorldUpdateAsync(self.world, PhantomData)
    }

    /// Destroys unreferenced bodies and steps the simulation synchronously
    pub fn update(&mut self, step: Duration) {
        self.flush_commands();
        unsafe { ffi::NewtonUpdate(self.world, as_seconds(step)) }
    }

    pub fn invalidate_cache(&mut self) {
        unsafe { ffi::NewtonInvalidateCache(self.world) }
    }

    pub fn as_raw(&self) -> *const ffi::NewtonWorld {
        self.world
    }

    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonWorld {
        self.world
    }

    fn flush_commands(&mut self) {
        while let Ok(cmd) = self.rx.try_recv() {
            match cmd {
                Command::DestroyBody(b) => {
                    unsafe { super::body::drop_body::<T>(b) };
                }
            }
        }
    }
}

impl<T: Types> NewtonWorld<T> {
    pub fn collision(&mut self) -> CollisionBuilder<T> {
        CollisionBuilder::new(self)
    }

    pub fn body<'b>(&mut self, collision: &'b NewtonCollision<T>) -> BodyBuilder<'_, 'b, T> {
        BodyBuilder::new(self, collision)
    }

    pub fn for_each_body_in_aabb<I>(&mut self, (min, max): (&T::Vector, &T::Vector), it: I)
    where
        I: Fn(&mut NewtonBody<T>) -> i32 + 'static,
    {
        unimplemented!()
    }
}

impl<'w, T> Deref for WorldLocked<'w, T> {
    type Target = NewtonWorld<T>;

    #[inline]
    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'w, T> Deref for WorldLockedMut<'w, T> {
    type Target = NewtonWorld<T>;

    #[inline]
    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'w, T> DerefMut for WorldLockedMut<'w, T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

impl<T> Drop for NewtonWorld<T> {
    fn drop(&mut self) {
        let world = self.world;
        self.flush_commands();
        unsafe {
            let _: Shared<WorldData<T>> = mem::transmute(ffi::NewtonWorldGetUserData(world));
            ffi::NewtonWaitForUpdateToFinish(world);
            ffi::NewtonMaterialDestroyAllGroupID(world);
            ffi::NewtonDestroy(world);
        }
    }
}

impl<'a, T> Drop for WorldUpdateAsync<'a, T> {
    fn drop(&mut self) {
        unsafe {
            ffi::NewtonWaitForUpdateToFinish(self.0);
        }
    }
}

fn as_seconds(step: Duration) -> f32 {
    let nanos = step.as_secs() as f32 * 1_000_000_000.0 + step.subsec_nanos() as f32;
    nanos / 1_000_000_000.0
}

pub(crate) unsafe fn userdata<T>(world: *const ffi::NewtonWorld) -> Shared<WorldData<T>> {
    let udata: Shared<WorldData<T>> = mem::transmute(ffi::NewtonWorldGetUserData(world));
    let udata_cloned = udata.clone();
    mem::forget(udata);
    udata_cloned
}
