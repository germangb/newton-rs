use std::collections::HashSet;
use std::mem;
use std::os::raw::{c_longlong, c_void};
use std::sync::RwLock;
use std::time::Duration;

use super::body::{Body, BodyTrait, Bodies, IntoBody, BodyOld, HandleOld as BodyHandle};
use super::collision::{IntoCollision, Collision, CollisionTrait, CollisionOld, HandleOld as CollisionHandle};
use super::ffi;
use super::math::Vector;
use super::Handle;

/// A guard for the asynchronous update
///
/// Dropping this type blocks the thread until the world update has finished.
#[derive(Debug)]
pub struct AsyncUpdate<'world>(&'world mut Newton);

impl<'world> Drop for AsyncUpdate<'world> {
    fn drop(&mut self) {
        let world = self.0.as_raw();
        unsafe { ffi::NewtonWaitForUpdateToFinish(world) }
    }
}

// TODO generalize handle storage
//      NewtonStorage trait that defaults to HashSet?
#[derive(Debug, Default)]
pub struct NewtonData {
    /// Bodies & collisions that are owned by the newton world.
    /// Whenever an object is consumed by its `into_handle` method, it ends up here.
    pub(crate) owned: RwLock<HashSet<Handle>>,
}

#[derive(Debug)]
pub struct Newton(pub(crate) *const ffi::NewtonWorld);

unsafe impl Send for Newton {}
unsafe impl Sync for Newton {}

impl Newton {
    pub fn create() -> Self {
        unsafe {
            let world = ffi::NewtonCreate();
            let udata = Box::new(NewtonData::default());
            ffi::NewtonWorldSetUserData(world, mem::transmute(udata));
            Self(world)
        }
    }

    pub(crate) fn user_data(&self) -> &Box<NewtonData> {
        unsafe { mem::transmute(&ffi::NewtonWorldGetUserData(self.0)) }
    }

    pub fn move_body<'a, B: IntoBody<'a>>(&'a self, mut body: B) -> Handle {
        let mut body = body.into_body();
        let handle = Handle(body.as_raw() as _);
        match &mut body {
            Body::Dynamic(ref mut body) => body.set_owner(self.0 as _),
            Body::Kinematic(ref mut body) => body.set_owner(self.0 as _),
        }
        self.user_data().owned.write().unwrap().insert(handle.clone());
        handle
    }

    pub fn move_collision<'a, C: IntoCollision<'a>>(&'a self, mut collision: C) -> Handle {
        unimplemented!()
    }

    pub fn bodies(&self) -> Bodies {
        Bodies(self)
    }

    /// Sets the solver model.
    ///
    /// - `steps = 0` Exact solver (default one)
    /// - `step > 0` Linear solver with the given number of linear steps.
    pub fn set_solver(&mut self, steps: usize) {
        unsafe { ffi::NewtonSetSolverModel(self.as_raw(), steps as _) };
    }

    /// Sets the number of threads Newton will run the simulation on.
    /// By default, the simulation is single-threaded.
    pub fn set_threads(&mut self, threads: usize) {
        unsafe { ffi::NewtonSetThreadsCount(self.as_raw(), threads as _) };
    }

    pub fn threads(&self) -> usize {
        unsafe { ffi::NewtonGetThreadsCount(self.as_raw()) as _ }
    }

    pub fn constraints(&self) -> usize {
        unsafe { ffi::NewtonWorldGetConstraintCount(self.as_raw()) as _ }
    }

    pub const fn as_raw(&self) -> *const ffi::NewtonWorld {
        self.0
    }
}

/// ffi wrappers
impl Newton {
    /// Invalidated any cached contacts information.
    ///
    /// Useful for when you synchronize a simulation over a network and need
    /// to run the simulation deterministically.
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
    /// If you intend to perform network synchronization, synchronous `update` is
    /// the preferred approach.
    pub fn update_async(&mut self, step: Duration) -> AsyncUpdate {
        let seconds = step.as_secs() * 1_000_000_000 + step.subsec_nanos() as u64;
        unsafe { ffi::NewtonUpdateAsync(self.as_raw(), seconds as f32 / 1_000_000_000.0) }
        AsyncUpdate(self)
    }
}

pub type RayCastPrefilter<'a> = Box<dyn FnMut(BodyOld, CollisionOld) -> bool + Send + 'a>;

/// Ray & convex collision casting
impl Newton {
    /// Samples world with a ray, and runs the given filter closure for every
    /// body that intersects.
    ///
    /// The filter closure can be used to implement various flavors of ray casting.
    /// Refer to the [official wiki][wiki] for further info.
    ///
    /// [wiki]: http://newtondynamics.com/wiki/index.php5?title=NewtonWorldRayCast
    #[rustfmt::skip]
    pub fn ray_cast<F>(&self,
                       p0: &Vector,
                       p1: &Vector,
                       mut filter: F,

                       // lifetime is elided
                       mut prefilter: Option<RayCastPrefilter>,
                       thread: usize)
    where
        F: FnMut(BodyOld, CollisionOld, &Vector, &Vector, f32) -> f32,
    {
        type Userdata<'a, T> = (&'a mut T, &'a Newton, Option<&'a mut RayCastPrefilter<'a>>);
        let mut user_data = (&mut filter, &Newton, prefilter.as_mut());
        unsafe {
            ffi::NewtonWorldRayCast(self.as_raw(),
                                    p0.as_ptr(),
                                    p1.as_ptr(),
                                    Some(cfilter::<F>),
                                    mem::transmute(&mut user_data),
                                    Some(cprefilter::<F>),
                                    thread as _);
        }

        unsafe extern "C" fn cprefilter<F>(body: *const ffi::NewtonBody,
                                        collision: *const ffi::NewtonCollision,
                                        user_data: *const c_void) -> u32
            where
                F: FnMut(BodyOld, CollisionOld, &Vector, &Vector, f32) -> f32,
        {
            let mut filter: &mut Userdata<F> = mem::transmute(user_data);
            let body = BodyOld { newton: filter.1, body, owned: false };
            let collision = CollisionOld { newton: filter.1, collision, owned: false };

            let res = filter.2.as_mut().map(|pre| pre(body, collision));
            match res {
                Some(true) | None => 1,
                Some(false) => 0,
            }
        }

        unsafe extern "C" fn cfilter<F>(body: *const ffi::NewtonBody,
                                        collision: *const ffi::NewtonCollision,
                                        hit_contact: *const f32,
                                        hit_normal: *const f32,
                                        collision_id: c_longlong,
                                        user_data: *const c_void,
                                        intersect_param: f32) -> f32
            where
                F: FnMut(BodyOld, CollisionOld, &Vector, &Vector, f32) -> f32,
        {
            let mut filter: &mut Userdata<F> = mem::transmute(user_data);

            let body = BodyOld { newton: filter.1, body, owned: false };
            let collision = CollisionOld { newton: filter.1, collision, owned: false };

            // We can safely cast from *const f32 to &Vector because the later has the
            // same representation.
            let contact: &Vector = mem::transmute(hit_contact);
            let normal: &Vector = mem::transmute(hit_normal);
            filter.0(body, collision, contact, normal, intersect_param)
        }
    }
}

impl Drop for Newton {
    fn drop(&mut self) {
        unsafe {
            let _: Box<NewtonData> = Box::from_raw(ffi::NewtonWorldGetUserData(self.as_raw()) as _);
            //ffi::NewtonDestroyAllBodies(self.as_raw());
            ffi::NewtonDestroy(self.as_raw());
        }
    }
}

unsafe fn userdata<'world>(ptr: *const ffi::NewtonWorld) -> &'world Box<NewtonData> {
    mem::transmute(&ffi::NewtonWorldGetUserData(ptr))
}
