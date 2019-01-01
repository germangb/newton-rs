use ffi;

use super::body::{BodyData, BodyLocked, NewtonBodies, NewtonBodiesMut, NewtonBody};
use super::collision::NewtonCollision;
//use super::contact::Contacts;
use super::lock::{Lock, LockError, Locked, LockedMut, Shared, Weak};
use super::material::{GroupId, NewtonMaterial};
use super::{channel, Matrix, Quaternion, Result, Rx, Tx, Vector};

use std::{
    marker::PhantomData,
    mem,
    ops::{Deref, DerefMut},
    os::raw,
    ptr,
    time::Duration,
};

/// A world is the heart of the physics simulation.
#[derive(Debug, Clone)]
pub struct World<B, C>(Shared<Lock<NewtonWorld<B, C>>>, *const ffi::NewtonWorld);

unsafe impl<B, C> Send for World<B, C> {}
unsafe impl<B, C> Sync for World<B, C> {}

#[derive(Debug, Clone)]
pub struct WeakWorld<B, C>(Weak<Lock<NewtonWorld<B, C>>>, *const ffi::NewtonWorld);

unsafe impl<B, C> Send for WeakWorld<B, C> {}
unsafe impl<B, C> Sync for WeakWorld<B, C> {}

#[derive(Debug)]
pub struct NewtonWorld<B, C> {
    world: *mut ffi::NewtonWorld,

    /// Tx end of the command channel.
    pub(crate) tx: Tx<Command>,
    /// Rx end of the command channel, used by NewtonBody to signal the destruction of a body.
    rx: Rx<Command>,

    _phantom: PhantomData<(B, C)>,
}

#[doc(hidden)]
pub struct WorldData<B, C> {
    /// We keep a reference to the world so we can obtain a copy of the lock from a `NewtonWorld`
    /// when needed.
    pub(crate) world: Weak<Lock<NewtonWorld<B, C>>>,
    /// Debug name
    debug: Option<&'static str>,
    /// A Tx end of the command channel, to recover it quickly from the userdata.
    tx: Tx<Command>,
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum Command {
    DestroyBody(*mut ffi::NewtonBody),
}

pub struct Builder<B, C> {
    solver: Solver,
    threads: usize,
    broad: Broadphase,
    debug: Option<&'static str>,
    _phantom: PhantomData<(B, C)>,
}

impl<B, C> Builder<B, C> {
    pub fn new() -> Self {
        Builder {
            solver: Solver::Exact,
            threads: 1,
            broad: Broadphase::Default,
            debug: None,
            _phantom: PhantomData,
        }
    }

    pub fn threads(mut self, th: usize) -> Self {
        self.threads = th;
        self
    }

    /// Sets the number of threads to the number of CPUs available in the system.
    pub fn max_threads(mut self) -> Self {
        self.threads = num_cpus::get();
        self
    }

    /// Set debug name, for use in Error reporting
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

    pub fn build(self) -> World<B, C> {
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

fn create_world<B, C>() -> NewtonWorld<B, C> {
    let world = unsafe { ffi::NewtonCreate() };
    let (tx, rx) = channel();
    NewtonWorld {
        world,
        tx,
        rx,
        _phantom: PhantomData,
    }
}

/// A read-only lock of a newton world.
#[derive(Debug)]
pub struct WorldLocked<'a, B, C>(Locked<'a, NewtonWorld<B, C>>);

#[derive(Debug)]
pub struct WorldLockedMut<'a, B, C>(LockedMut<'a, NewtonWorld<B, C>>);

/// A type returned by the call to the asynchronous world update. When this type is dropped, the current
/// thread is blocked until the simulation step has finished.
#[derive(Debug)]
pub struct WorldUpdateAsync<'a>(*mut ffi::NewtonWorld, PhantomData<&'a ()>);

unsafe impl<'a> Send for WorldUpdateAsync<'a> {}
unsafe impl<'a> Sync for WorldUpdateAsync<'a> {}

impl<'a> WorldUpdateAsync<'a> {
    /// Consumes the object and blocks the current thread until the world update is finished.
    pub fn finish(self) {}
}

impl<B, C> Default for World<B, C> {
    #[inline]
    fn default() -> Self {
        World::new(Broadphase::Default, Solver::Exact, 1, None)
    }
}

impl<B, C> World<B, C> {
    /// Creates a new World
    pub fn new(
        broadphase: Broadphase,
        solver: Solver,
        threads: usize,
        debug: Option<&'static str>,
    ) -> Self {
        let newton_world = create_world();

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
    pub fn try_read(&self) -> Result<WorldLocked<B, C>> {
        Ok(WorldLocked(self.0.try_read()?))
    }

    #[inline]
    pub fn try_write(&self) -> Result<WorldLockedMut<B, C>> {
        #[cfg(feature = "debug")]
        let udata = unsafe { userdata::<B, C>(self.1).debug };
        #[cfg(not(feature = "debug"))]
        let udata = None;
        Ok(WorldLockedMut(self.0.try_write(udata)?))
    }

    #[inline]
    pub fn read(&self) -> WorldLocked<B, C> {
        self.try_read().unwrap()
    }

    #[inline]
    pub fn write(&self) -> WorldLockedMut<B, C> {
        self.try_write().unwrap()
    }
}

#[derive(Debug)]
pub struct ConvexCast<'a, B, C> {
    count: usize,
    contacts: Vec<ffi::NewtonWorldConvexCastReturnInfo>,
    body: *mut NewtonBody<B, C>,
    _phantom: PhantomData<&'a ()>,
}

impl<'a, B, C> Drop for ConvexCast<'a, B, C> {
    fn drop(&mut self) {
        unsafe {
            let _ = Box::from_raw(self.body);
        }
    }
}

#[derive(Debug)]
pub struct CastInfo {
    pub point: Vector,
    pub normal: Vector,
    pub penetration: f32,
}

// TODO test, make sure collision matches the body
// TODO FIXME refactor...
impl<'a, B: 'a, C: 'a> Iterator for ConvexCast<'a, B, C> {
    type Item = (&'a NewtonBody<B, C>, CastInfo);

    fn next(&mut self) -> Option<Self::Item> {
        let contact = self.contacts.get(self.count).map(|info| {
            let mut boxed = unsafe { Box::from_raw(self.body) };

            boxed.body = info.m_hitBody as _;
            unsafe {
                boxed.collision.collision = ffi::NewtonBodyGetCollision(info.m_hitBody);
            }

            let (point, normal) = unsafe {
                let mut point: Vector = mem::zeroed();
                let mut normal: Vector = mem::zeroed();

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

impl<B, C> NewtonWorld<B, C> {
    /// Shoots a ray from `p0` to `p1` and calls the application callback whenever the ray hits a body.
    pub fn ray_cast<F, P>(&self, p0: &Vector, p1: &Vector, filt: F, prefilt: P)
    where
        F: Fn(
            &NewtonBody<B, C>,
            &NewtonCollision<B, C>,
            &Vector,
            &Vector,
            raw::c_longlong,
            f32,
        ) -> f32,
        P: Fn(&NewtonBody<B, C>, &NewtonCollision<B, C>) -> bool,
    {
        unsafe {
            ffi::NewtonWorldRayCast(
                self.world,
                mem::transmute(p0),
                mem::transmute(p1),
                Some(filter::<B, C, F, P>),
                mem::transmute(&(&filt, &prefilt)),
                Some(prefilter::<B, C, F, P>),
                0,
            );
        }

        unsafe extern "C" fn prefilter<B, C, F, P>(
            body: *const ffi::NewtonBody,
            collision: *const ffi::NewtonCollision,
            user_data: *const raw::c_void,
        ) -> raw::c_uint
        where
            P: Fn(&NewtonBody<B, C>, &NewtonCollision<B, C>) -> bool,
            F: Fn(
                &NewtonBody<B, C>,
                &NewtonCollision<B, C>,
                &Vector,
                &Vector,
                raw::c_longlong,
                f32,
            ) -> f32,
        {
            let body = NewtonBody::new_not_owned(body as _);
            let collision = NewtonCollision::new_not_owned(collision as _);

            if mem::transmute::<_, &(&F, &P)>(user_data).1(&body, &collision) {
                1
            } else {
                0
            }
        }

        unsafe extern "C" fn filter<B, C, F, P>(
            body: *const ffi::NewtonBody,
            shape_hit: *const ffi::NewtonCollision,
            hit_contact: *const f32,
            hit_normal: *const f32,
            collision_id: raw::c_longlong,
            user_data: *const raw::c_void,
            intersect_param: f32,
        ) -> f32
        where
            P: Fn(&NewtonBody<B, C>, &NewtonCollision<B, C>) -> bool,
            F: Fn(
                &NewtonBody<B, C>,
                &NewtonCollision<B, C>,
                &Vector,
                &Vector,
                raw::c_longlong,
                f32,
            ) -> f32,
        {
            let body = NewtonBody::new_not_owned(body as _);
            let collision = NewtonCollision::new_not_owned(shape_hit as _);

            mem::transmute::<_, &(&F, &P)>(user_data).0(
                &body,
                &collision,
                mem::transmute(hit_contact),
                mem::transmute(hit_normal),
                collision_id,
                intersect_param,
            )
        }
    }

    /// Projects a collision shape from matrix origin towards a target returning all hits that
    /// the geometry would produce.
    pub fn convex_cast<Callback>(
        &self,
        matrix: &Matrix,
        target: &Vector,
        shape: &NewtonCollision<B, C>,
        hit_param: &mut f32,
        max_contacts: usize,
        prefilter: Callback,
    ) -> ConvexCast<B, C>
    where
        Callback: Fn(&NewtonBody<B, C>, &NewtonCollision<B, C>) -> bool + 'static,
    {
        let userdata = unsafe { userdata(self.world) };
        let prefilter_world = (prefilter, userdata.clone());

        let mut return_info = Vec::with_capacity(max_contacts);
        let contacts = unsafe {
            ffi::NewtonWorldConvexCast(
                self.world,
                mem::transmute(matrix),
                mem::transmute(target),
                shape.as_raw(),
                // hitParam
                hit_param,
                // userdata
                mem::transmute(&prefilter_world),
                Some(prefilter_callback::<B, C, Callback>),
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

        unsafe extern "C" fn prefilter_callback<B, C, Callback>(
            body: *const ffi::NewtonBody,
            collision: *const ffi::NewtonCollision,
            udata: *const raw::c_void,
        ) -> raw::c_uint
        where
            Callback: Fn(&NewtonBody<B, C>, &NewtonCollision<B, C>) -> bool + 'static,
        {
            let (ref callback, ref world): &(Callback, Shared<WorldData<B, C>>) =
                mem::transmute(udata);

            let body = NewtonBody::new_not_owned(body as _);
            if callback(&body, &body.collision) {
                1
            } else {
                0
            }
        }
    }

    /// Calls the given closure for every body inside the given AABB
    pub fn for_each_body_in_aabb<I>(&self, (min, max): (&Vector, &Vector), mut it: I)
    where
        I: FnMut(&NewtonBody<B, C>) -> bool,
    {
        unsafe {
            ffi::NewtonWorldForEachBodyInAABBDo(
                self.world,
                mem::transmute(min),
                mem::transmute(max),
                Some(body_iterator::<B, C, I>),
                mem::transmute(&mut it),
            );
        }

        unsafe extern "C" fn body_iterator<B, C, I: FnMut(&NewtonBody<B, C>) -> bool>(
            body: *const ffi::NewtonBody,
            user_data: *const raw::c_void,
        ) -> raw::c_int {
            let body = NewtonBody::new_not_owned(body as _);
            if mem::transmute::<_, &mut I>(user_data)(&body) {
                1
            } else {
                0
            }
        }
    }

    /// Get an iterator for Materials
    pub fn materials(&self) -> ! {
        unimplemented!()
    }

    /// Get a mutable iterator for Materials
    pub fn materials_mut(&mut self) -> ! {
        unimplemented!()
    }

    /// Generates a single or a touple of material group IDs
    pub fn create_materials<M: super::material::Materials>(&mut self) -> M {
        unsafe { M::from(self.world) }
    }

    #[inline]
    pub fn material_set_thickness(&mut self, (u, v): (GroupId, GroupId), thick: f32) {
        assert!(thick >= 0.0);
        unsafe { ffi::NewtonMaterialSetSurfaceThickness(self.world, u.0, v.0, thick) };
    }

    #[inline]
    pub fn material_set_friction(&mut self, (u, v): (GroupId, GroupId), fs: f32, fk: f32) {
        assert!(fs >= 0.0 && fk >= 0.0);
        unsafe { ffi::NewtonMaterialSetDefaultFriction(self.world, u.0, v.0, fs, fk) };
    }

    #[inline]
    pub fn material_set_elasticity(&mut self, (u, v): (GroupId, GroupId), rest: f32) {
        assert!(rest >= 0.0);
        unsafe { ffi::NewtonMaterialSetDefaultElasticity(self.world, u.0, v.0, rest) };
    }

    #[inline]
    pub fn material_set_softness(&mut self, (u, v): (GroupId, GroupId), soft: f32) {
        assert!(soft >= 0.0);
        unsafe { ffi::NewtonMaterialSetDefaultSoftness(self.world, u.0, v.0, soft) };
    }

    #[inline]
    pub fn material_set_collidable(&mut self, (u, v): (GroupId, GroupId), collid: bool) {
        unsafe {
            ffi::NewtonMaterialSetDefaultCollidable(
                self.world,
                u.0,
                v.0,
                if collid { 1 } else { 0 },
            )
        };
    }

    pub fn material_set_collision_callback<Callback>(
        &mut self,
        (u, v): (GroupId, GroupId),
        callback: Callback,
    ) where
        Callback:
            Fn(&NewtonMaterial, &NewtonBody<B, C>, &NewtonBody<B, C>, raw::c_int) -> bool + 'static,
    {
        unsafe {
            // free any previously set callback
            let udata = ffi::NewtonMaterialGetUserData(self.world, u.0, v.0);
            if !udata.is_null() {
                let boxed_callback: Box<Callback> = Box::from_raw(udata as _);
            }

            ffi::NewtonMaterialSetCallbackUserData(
                self.world,
                u.0,
                v.0,
                mem::transmute(Box::new(callback)),
            );
            ffi::NewtonMaterialSetCollisionCallback(
                self.world,
                u.0,
                v.0,
                Some(super::callbacks::aabb_overlap_callback::<B, C, Callback>),
                Some(super::callbacks::contacts_process_callback),
            )
        }
    }

    /// Returns a body iterator
    pub fn bodies(&self) -> NewtonBodies<B, C> {
        unsafe {
            let world = self.world;
            let first = ffi::NewtonWorldGetFirstBody(world);
            let body = Box::new(NewtonBody::new_not_owned(first));

            NewtonBodies {
                world,
                next: first,
                body: Box::into_raw(body),
                _phantom: PhantomData,
            }
        }
    }

    /// Returns a mutable body iterator
    pub fn bodies_mut(&mut self) -> NewtonBodiesMut<B, C> {
        unsafe {
            let world = self.world;
            let first = ffi::NewtonWorldGetFirstBody(world);
            let body = Box::new(NewtonBody::new_not_owned(first));

            NewtonBodiesMut {
                world,
                next: first,
                body: Box::into_raw(body),
                _phantom: PhantomData,
            }
        }
    }

    #[inline]
    pub fn body_count(&self) -> raw::c_int {
        unsafe { ffi::NewtonWorldGetBodyCount(self.world) }
    }

    pub fn constraint_count(&self) -> raw::c_int {
        unsafe { ffi::NewtonWorldGetConstraintCount(self.world) }
    }

    /// Non-blocking variant of the `update` method.
    pub fn update_async(&mut self, step: Duration) -> WorldUpdateAsync {
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

    #[inline]
    pub fn invalidate_cache(&mut self) {
        unsafe { ffi::NewtonInvalidateCache(self.world) }
    }

    #[inline]
    pub fn as_raw(&self) -> *const ffi::NewtonWorld {
        self.world
    }

    #[inline]
    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonWorld {
        self.world
    }

    fn flush_commands(&mut self) {
        while let Ok(cmd) = self.rx.try_recv() {
            match cmd {
                Command::DestroyBody(b) => {
                    unsafe { super::body::drop_body::<B, C>(b) };
                }
            }
        }
    }

    // TODO Is this a Hack? It looks like a Hack
    fn destroy_materials<Callback>(&mut self, _: Callback)
    where
        Callback:
            Fn(&NewtonMaterial, &NewtonBody<B, C>, &NewtonBody<B, C>, raw::c_int) -> bool + 'static,
    {
        unsafe {
            ffi::NewtonMaterialDestroyAllGroupID(self.world);
            let mut material = ffi::NewtonWorldGetFirstMaterial(self.world);
            while !material.is_null() {
                let udata = ffi::NewtonMaterialGetMaterialPairUserData(material);
                if !udata.is_null() {
                    let boxed_callback: Box<Callback> = Box::from_raw(udata as _);
                }
                material = ffi::NewtonWorldGetNextMaterial(self.world, material);
            }
            ffi::NewtonMaterialDestroyAllGroupID(self.world);
        }
    }
}

impl<'w, B, C> Deref for WorldLocked<'w, B, C> {
    type Target = NewtonWorld<B, C>;
    #[inline]
    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'w, B, C> Deref for WorldLockedMut<'w, B, C> {
    type Target = NewtonWorld<B, C>;
    #[inline]
    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'w, B, C> DerefMut for WorldLockedMut<'w, B, C> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

impl<B, C> Drop for NewtonWorld<B, C> {
    fn drop(&mut self) {
        let world = self.world;
        unsafe {
            ffi::NewtonWaitForUpdateToFinish(world);
        }
        self.flush_commands();
        unsafe {
            let _: Shared<WorldData<B, C>> = mem::transmute(ffi::NewtonWorldGetUserData(world));
            self.destroy_materials(|_, _, _, _| false);
            ffi::NewtonDestroyAllBodies(world);
            ffi::NewtonDestroy(world);
        }
    }
}

impl<'a> Drop for WorldUpdateAsync<'a> {
    fn drop(&mut self) {
        unsafe { ffi::NewtonWaitForUpdateToFinish(self.0) };
    }
}

fn as_seconds(step: Duration) -> f32 {
    let nanos = step.as_secs() as f32 * 1_000_000_000.0 + step.subsec_nanos() as f32;
    nanos / 1_000_000_000.0
}

pub(crate) unsafe fn userdata<B, C>(world: *const ffi::NewtonWorld) -> Shared<WorldData<B, C>> {
    let udata: Shared<WorldData<B, C>> = mem::transmute(ffi::NewtonWorldGetUserData(world));
    let udata_cloned = udata.clone();
    mem::forget(udata);
    udata_cloned
}
