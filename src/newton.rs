use std::collections::HashSet;
use std::mem;
use std::os::raw::{c_longlong, c_void};
use std::ptr;
use std::sync::RwLock;
use std::time::Duration;

use super::body::{iters::Bodies, Body, NewtonBody};
use super::collision::{Collision, NewtonCollision};
use super::ffi;
use super::Handle;

/// Type returned by an asynchronous update.
#[derive(Debug)]
pub struct AsyncUpdate<'a>(&'a mut Newton);

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

// Heap memory to support the wrapper
#[derive(Default)]
struct UserData {
    bodies: RwLock<HashSet<Handle>>,
    collisions: RwLock<HashSet<Handle>>,
}

/// Newton dynamics context
#[derive(Debug)]
pub struct Newton {
    raw: *const ffi::NewtonWorld,
}

unsafe impl Send for Newton {}
unsafe impl Sync for Newton {}

impl Newton {
    pub unsafe fn from_raw(raw: *const ffi::NewtonWorld) -> Self {
        Self { raw }
    }

    pub fn create() -> Self {
        let data = Box::new(UserData::default());
        let raw = unsafe {
            let raw = ffi::NewtonCreate();
            ffi::NewtonWorldSetUserData(raw, mem::transmute(data));
            raw
        };
        Self { raw }
    }

    fn user_data(&self) -> &Box<UserData> {
        unsafe {
            let udata = &ffi::NewtonWorldGetUserData(self.raw);
            mem::transmute(udata)
        }
    }

    pub(crate) fn move_body2(&self, body: Body) -> Handle {
        let handle = Handle::Pointer(body.as_raw() as _);
        self.user_data()
            .bodies
            .write()
            .unwrap()
            .insert(handle.clone());
        handle
    }

    pub(crate) fn move_collision2(&self, collision: Collision) -> Handle {
        let handle = Handle::Pointer(collision.as_raw() as _);
        self.user_data()
            .collisions
            .write()
            .unwrap()
            .insert(handle.clone());
        handle
    }

    /// Sets the solver model.
    ///
    /// - `steps = 0` Exact solver
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

    pub fn bodies(&self) -> usize {
        unsafe { ffi::NewtonWorldGetBodyCount(self.as_raw()) as usize }
    }

    pub fn bodies_iter(&self) -> Bodies {
        let next = unsafe { ffi::NewtonWorldGetFirstBody(self.as_raw()) };
        Bodies { newton: self, next }
    }

    pub fn body(&self, handle: Handle) -> Option<Body> {
        let body = self
            .user_data()
            .bodies
            .read()
            .unwrap()
            .get(&handle)
            .cloned();
        unsafe {
            body.map(|h| match h {
                Handle::Pointer(ptr) => Body::from_raw(ptr as _, false),
                _ => unimplemented!("index indexing"),
            })
        }
    }

    pub fn body_take(&self, handle: Handle) -> Option<Body> {
        let body = self.user_data().bodies.write().unwrap().take(&handle);
        unsafe {
            body.map(|h| match h {
                Handle::Pointer(ptr) => Body::from_raw(ptr as _, true),
                _ => unimplemented!("index indexing"),
            })
        }
    }

    pub fn collision(&self, handle: Handle) -> Option<Collision> {
        let collision = self
            .user_data()
            .collisions
            .read()
            .unwrap()
            .get(&handle)
            .cloned();
        unsafe {
            collision.map(|h| match h {
                Handle::Pointer(ptr) => Collision::from_raw(ptr as _, false),
                _ => unimplemented!("index indexing"),
            })
        }
    }

    pub fn collision_take(&self, handle: Handle) -> Option<Collision> {
        let collision = self.user_data().collisions.write().unwrap().take(&handle);
        unsafe {
            collision.map(|h| match h {
                Handle::Pointer(ptr) => Collision::from_raw(ptr as _, true),
                _ => unimplemented!("index indexing"),
            })
        }
    }

    pub const fn as_raw(&self) -> *const ffi::NewtonWorld {
        self.raw
    }

    pub fn into_raw(self) -> *const ffi::NewtonWorld {
        self.raw
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
                       p0: [f32; 3],
                       p1: [f32; 3],
                       mut filter: F,

                       // lifetime is elided
                       //mut prefilter: Option<RayCastPrefilter>,
                       thread: usize)
    where
        F: FnMut(Body, Collision, [f32; 3], [f32; 3],  f32) -> f32,
    {
        type Userdata<'a, T> = (&'a mut T, );
        let mut user_data = (&mut filter, );
        unsafe {
            ffi::NewtonWorldRayCast(self.as_raw(),
                                    p0.as_ptr(),
                                    p1.as_ptr(),
                                    Some(cfilter::<F>),
                                    mem::transmute(&mut user_data),
                                    None,
                                    thread as _);
        }

        /*
        unsafe extern "C" fn cprefilter<F>(body: *const ffi::NewtonBody,
                                        collision: *const ffi::NewtonCollision,
                                        user_data: *const c_void) -> u32
            where
                F: FnMut(Body, Collision, [f32; 3], [f32; 3],  f32) -> f32,
        {
            let mut filter: &mut Userdata<F> = mem::transmute(user_data);
            let body = Body::from_raw(body, false);
            let collision = Collision::from_raw(collision, false);

            let res = filter.2.as_mut().map(|pre| pre(body, collision));
            match res {
                Some(true) | None => 1,
                Some(false) => 0,
            }
        }
        */

        unsafe extern "C" fn cfilter<F>(body: *const ffi::NewtonBody,
                                        collision: *const ffi::NewtonCollision,
                                        contact: *const f32,
                                        normal: *const f32,
                                        collision_id: c_longlong,
                                        user_data: *const c_void,
                                        intersect_param: f32) -> f32
            where
                F: FnMut(Body, Collision, [f32; 3], [f32; 3], f32) -> f32,
        {
            let mut filter: &mut Userdata<F> = mem::transmute(user_data);

            let body = Body::from_raw(body, false);
            let collision = Collision::from_raw(collision, false);

            // We can safely cast from *const f32 to &Vector because the later has the
            // same representation.
            let mut contact = [*contact, *contact.offset(1), *contact.offset(2)];
            let mut normal = [*normal, *normal.offset(1), *normal.offset(2)];
            filter.0(body, collision, contact, normal, intersect_param)
        }
    }
}

impl Drop for Newton {
    fn drop(&mut self) {
        unsafe {
            let udata = ffi::NewtonWorldGetUserData(self.as_raw());
            let udata: Box<UserData> = Box::from_raw(udata as _);

            // free owned bodies & collisions
            udata
                .bodies
                .write()
                .unwrap()
                .iter()
                .map(|h| match h {
                    Handle::Pointer(ptr) => Body::from_raw(*ptr as _, true),
                    _ => unimplemented!("index indexing"),
                })
                .for_each(drop);
            udata
                .collisions
                .write()
                .unwrap()
                .iter()
                .map(|h| match h {
                    Handle::Pointer(ptr) => Collision::from_raw(*ptr as _, true),
                    _ => unimplemented!("index indexing"),
                })
                .for_each(drop);

            ffi::NewtonDestroyAllBodies(self.as_raw());
            ffi::NewtonDestroy(self.as_raw());
        }
    }
}
