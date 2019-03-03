use std::marker::PhantomData;
use std::mem;
use std::os::raw::{c_longlong, c_void};
use std::time::Duration;

use ray_cast::RayCastAlgorithm;
use storage::{BTreeStorage, NewtonStorage};

use crate::body::{iter::Bodies, Body, NewtonBody};
use crate::collision::{Collision, ConvexShape, NewtonCollision};
use crate::ffi;
use crate::math::{Mat4, Vec3, Vec4};

/// Implementation of useful ray_cast algorithms.
pub mod ray_cast;
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

pub struct ConvexCastInfo<'a> {
    pub body: Body<'a>,
    pub point: Vec4,
    pub normal: Vec4,
    pub contact_id: c_longlong,
    pub penetration: f32,
}

#[derive(Debug)]
pub struct ConvexCastResult<'a> {
    newton: &'a Newton,
    info: Vec<ffi::NewtonWorldConvexCastReturnInfo>,
    hit_param: f32,
}

impl<'a> ConvexCastResult<'a> {
    pub fn hit_param(&self) -> f32 {
        self.hit_param
    }

    pub fn len(&self, index: usize) -> usize {
        self.info.len()
    }

    pub fn get(&self, index: usize) -> Option<ConvexCastInfo> {
        self.info.get(index).map(|info| ConvexCastInfo { body: unsafe {
                                                             Body::from_raw(info.m_hitBody, false)
                                                         },
                                                         point: info.m_point,
                                                         normal: info.m_normal,
                                                         contact_id: info.m_contactID,
                                                         penetration: info.m_penetration })
    }
}

/// Wrapper around NewtonWorld.
#[derive(Debug)]
pub struct Newton {
    raw: *const ffi::NewtonWorld,
    owned: bool,
}

unsafe impl Send for Newton {}
unsafe impl Sync for Newton {}

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
pub struct NewtonConfig {
    threads: Option<usize>,
    linear_steps: Option<usize>,
    storage: Option<Box<dyn NewtonStorage>>,
}

impl NewtonConfig {
    /// Use all CPU cores.
    pub fn max_threads(mut self) -> Self {
        self.threads = Some(num_cpus::get());
        self
    }

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

    fn from_config(conf: NewtonConfig) -> Self {
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
        Self::from_config(NewtonConfig::default())
    }

    pub fn config() -> NewtonConfig {
        NewtonConfig::default()
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

    /// Projects a convex collision shape in the world, and returns all the contacts that it generates.
    ///
    /// It is the equivalent of `ray_cast`, but for solid rays with convex geometry.
    ///
    /// This function can be used to implement a character controller, for example.
    pub fn convex_cast<C, P>(&self,
                             matrix: Mat4,
                             target: Vec3,
                             shape: &C,
                             mut prefilter: P,
                             max_contacts: usize,
                             thread_idx: usize)
                             -> ConvexCastResult
        where C: ConvexShape,
              P: FnMut(Body, Collision) -> bool + Send + Sync
    {
        let mut info = Vec::with_capacity(max_contacts);
        let mut hit_param = 0.0;
        let contacts = unsafe {
            ffi::NewtonWorldConvexCast(self.as_raw(),
                                       matrix[0].as_ptr(),
                                       target.as_ptr(),
                                       shape.as_raw(),
                                       &mut hit_param,
                                       mem::transmute(&mut prefilter),
                                       Some(prefilter_callback::<P>),
                                       info.as_mut_ptr(),
                                       max_contacts as _,
                                       thread_idx as _)
        };

        unsafe {
            info.set_len(contacts as usize);
        }

        return ConvexCastResult { newton: self, info, hit_param };

        unsafe extern "C" fn prefilter_callback<P>(body: *const ffi::NewtonBody,
                                                   col: *const ffi::NewtonCollision,
                                                   udata: *const c_void)
                                                   -> u32
            where P: FnMut(Body, Collision) -> bool + Send
        {
            let b = Body::from_raw(body, false);
            let c = Collision::from_raw(col, false);

            if mem::transmute::<_, &mut P>(udata)(b, c) {
                1
            } else {
                0
            }
        }
    }

    /// Samples world with a ray.
    ///
    /// A ray is defined by a starting position `p0`, and an ending `p1`. Both are specified
    /// in world-space coordinates.
    ///
    /// # Example
    ///
    /// The `ray_cast` module provides implementations for common ray-casting implementations.
    ///
    /// ```
    /// use newton::Newton;
    /// use newton::newton::ray_cast::{ClosestHit, NClosestHits, RayHit};
    ///
    /// let newton = Newton::create();
    ///
    /// let (p0, p1) = get_ray();
    ///
    /// // get the closest hit
    /// let first: Option<RayHit> = newton.ray_cast::<ClosestHit>(p0, p1, ());
    ///
    /// // three closest hits
    /// let first_three: Vec<RayHit> = newton.ray_cast::<NClosestHits>(p0, p1, 3);
    ///
    /// # use newton::math::Vec3;
    /// # fn get_ray() -> (Vec3, Vec3) { ([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]) }
    /// ```
    pub fn ray_cast<'a, A: RayCastAlgorithm<'a>>(&'a self,
                                                 p0: Vec3,
                                                 p1: Vec3,
                                                 params: A::Params)
                                                 -> A::Result {
        A::ray_cast(self, p0, p1, params)
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
