use std::collections::HashSet;
use std::marker::PhantomData;
use std::mem;
use std::os::raw::{c_longlong, c_void};
use std::ptr;
use std::time::Duration;

use storage::{BTreeStorage, NewtonStorage};

use crate::body::{iter::Bodies, Body, NewtonBody};
use crate::collision::{Collision, ConvexShape, NewtonCollision};
use crate::ffi;
use crate::handle::{Handle, HandleInner};
use crate::Vec3;

/// Data structured for bodies & collisions.
pub mod storage;

/// Type returned by an asynchronous update.
#[derive(Debug)]
pub struct AsyncUpdate<'a>(&'a Newton);

impl<'a> AsyncUpdate<'a> {
    /// Waits for the newton world update to finish, blocking the current thread.
    pub fn finish(self) {}
}

impl<'a> Drop for AsyncUpdate<'a> {
    fn drop(&mut self) {
        let world = self.0.as_raw();
        unsafe { ffi::NewtonWaitForUpdateToFinish(world) }
    }
}

/// Wrapper around NewtonWorld.
#[derive(Debug)]
pub struct Newton {
    raw: *const ffi::NewtonWorld,
    owned: bool,
}

// Trait objects are "fat pointers" (their size is not the same as usize).
// I need to wrap the trait object in a struct and introduce an extra layer of indirection (pointer to a pointer)...
// The only way this could backfire is if the application calls `storage` and `storage_mut` very often in the program.
//
// An alternative to remove this indirection is to make Newton generic (Newton<S>) over the NewtonStorage.
struct UserData {
    storage: Box<dyn NewtonStorage>,
}

/// Newton World builder.
#[derive(Default)]
pub struct Config {
    threads: Option<usize>,
    linear_steps: Option<usize>,
    storage: Option<Box<dyn NewtonStorage>>,
}

impl Config {
    pub fn threads(mut self, threads: usize) -> Self {
        self.threads = Some(threads);
        self
    }

    pub fn linear(mut self, steps: usize) -> Self {
        self.linear_steps = Some(steps);
        self
    }

    pub fn storage<S: NewtonStorage + 'static>(mut self, storage: S) -> Self {
        self.storage = Some(Box::new(storage));
        self
    }

    pub fn build(self) -> Newton {
        Newton::from_config(self)
    }
}

impl Newton {
    pub unsafe fn from_raw(raw: *const ffi::NewtonWorld, owned: bool) -> Self {
        Self { raw, owned }
    }

    pub const fn as_raw(&self) -> *const ffi::NewtonWorld {
        self.raw
    }

    pub fn into_raw(self) -> *const ffi::NewtonWorld {
        self.raw
    }

    fn from_config(conf: Config) -> Self {
        unsafe {
            let raw = ffi::NewtonCreate();

            let storage = conf.storage.unwrap_or(Box::new(BTreeStorage::default()));
            let storage = Box::new(UserData { storage });

            ffi::NewtonWorldSetUserData(raw, mem::transmute(storage));

            if let Some(threads) = conf.threads {
                ffi::NewtonSetThreadsCount(raw, threads as _);
            }
            if let Some(steps) = conf.linear_steps {
                ffi::NewtonSetSolverModel(raw, steps as _);
            }

            Self { raw, owned: true }
        }
    }

    pub fn create() -> Self {
        Self::from_config(Config::default())
    }

    pub fn config() -> Config {
        Config::default()
    }

    pub fn storage(&self) -> &Box<dyn NewtonStorage> {
        unsafe {
            let udata: &Box<UserData> = mem::transmute(&ffi::NewtonWorldGetUserData(self.raw));
            &udata.storage
        }
    }

    pub fn storage_mut(&mut self) -> &mut Box<dyn NewtonStorage> {
        unsafe {
            let mut udata: &mut Box<UserData> =
                mem::transmute(&mut ffi::NewtonWorldGetUserData(self.raw));
            &mut udata.storage
        }
    }

    pub fn threads(&self) -> usize {
        unsafe { ffi::NewtonGetThreadsCount(self.as_raw()) as _ }
    }

    pub fn constraints(&self) -> usize {
        unsafe { ffi::NewtonWorldGetConstraintCount(self.as_raw()) as _ }
    }

    pub fn bodies(&self) -> usize {
        unsafe { ffi::NewtonWorldGetBodyCount(self.as_raw()) as usize }
    }

    pub fn bodies_iter(&self) -> Bodies {
        let next = unsafe { ffi::NewtonWorldGetFirstBody(self.as_raw()) };
        Bodies { newton: self.as_raw(), next, _phantom: PhantomData }
    }

    /// Invalidated any cached contacts.
    pub fn invalidate(&mut self) {
        unsafe { ffi::NewtonInvalidateCache(self.as_raw()) }
    }

    /// Steps the simulation by a fixed amount (synchronous).
    pub fn update(&mut self, step: Duration) {
        let seconds = step.as_secs() * 1_000_000_000 + step.subsec_nanos() as u64;
        unsafe { ffi::NewtonUpdate(self.as_raw(), seconds as f32 / 1_000_000_000.0) }
    }

    /// Steps the simulation by a fixed amount (asynchronous) without blocking the
    /// current thread of execution.
    ///
    /// ## Notes
    ///
    /// Not recommended if you intend to run simulation deterministically.
    pub fn update_async(&mut self, step: Duration) -> AsyncUpdate {
        let seconds = step.as_secs() * 1_000_000_000 + step.subsec_nanos() as u64;
        unsafe { ffi::NewtonUpdateAsync(self.as_raw(), seconds as f32 / 1_000_000_000.0) }
        AsyncUpdate(self)
    }

    /// Samples world with a ray, and runs the given filter closure for every
    /// body that intersects.
    ///
    /// The filter closure can be used to implement various flavors of ray casting.
    /// Refer to the [official wiki][wiki] for further info.
    ///
    /// [wiki]: http://newtondynamics.com/wiki/index.php5?title=NewtonWorldRayCast
    pub fn ray_cast<F, P>(&self, p0: Vec3, p1: Vec3, mut filter: F, mut prefilter: P, thread: usize)
        where F: FnMut(Body, Collision, Vec3, Vec3, f32) -> f32,
              P: FnMut(Body, Collision) -> bool
    {
        type Userdata<'a, F, P> = (&'a mut F, &'a mut P);
        let mut user_data = (&mut filter, &mut prefilter);
        unsafe {
            ffi::NewtonWorldRayCast(self.as_raw(),
                                    p0.as_ptr(),
                                    p1.as_ptr(),
                                    Some(cfilter::<F, P>),
                                    mem::transmute(&mut user_data),
                                    Some(cprefilter::<F, P>),
                                    thread as _);
        }

        unsafe extern "C" fn cprefilter<F, P>(body: *const ffi::NewtonBody,
                                              collision: *const ffi::NewtonCollision,
                                              user_data: *const c_void)
                                              -> u32
            where F: FnMut(Body, Collision, Vec3, Vec3, f32) -> f32,
                  P: FnMut(Body, Collision) -> bool
        {
            let mut filter: &mut Userdata<F, P> = mem::transmute(user_data);
            let body = Body::from_raw(body, false);
            let collision = Collision::from_raw(collision, false);

            if (filter.1)(body, collision) {
                1
            } else {
                0
            }
        }

        unsafe extern "C" fn cfilter<F, P>(body: *const ffi::NewtonBody,
                                           collision: *const ffi::NewtonCollision,
                                           contact: *const f32,
                                           normal: *const f32,
                                           collision_id: c_longlong,
                                           user_data: *const c_void,
                                           intersect_param: f32)
                                           -> f32
            where F: FnMut(Body, Collision, Vec3, Vec3, f32) -> f32,
                  P: FnMut(Body, Collision) -> bool
        {
            let mut filter: &mut Userdata<F, P> = mem::transmute(user_data);

            let body = Body::from_raw(body, false);
            let collision = Collision::from_raw(collision, false);

            // We can safely cast from *const f32 to &Vector because the later has the
            // same representation.
            let contact = mem::transmute::<_, &Vec3>(contact).clone();
            let normal = mem::transmute::<_, &Vec3>(normal).clone();
            filter.0(body, collision, contact, normal, intersect_param)
        }
    }
}

impl Drop for Newton {
    fn drop(&mut self) {
        if self.owned {
            unsafe {
                let udata = ffi::NewtonWorldGetUserData(self.raw);
                let _: Box<UserData> = Box::from_raw(udata as _);
                ffi::NewtonDestroyAllBodies(self.raw);
                ffi::NewtonMaterialDestroyAllGroupID(self.raw);
                ffi::NewtonDestroy(self.raw);
            }
        }
    }
}
