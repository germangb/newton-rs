use ffi;

use super::body::{BodyLocked, NewtonBody, NewtonBodyData};
use super::collision::NewtonCollision;
use super::iter::{NewtonBodies, NewtonBodiesMut};
use super::lock::{Lock, Locked, LockedMut};
use super::material::{GroupId, NewtonMaterial};
use super::{channel, Matrix, Quaternion, Result, Rx, Shared, Tx, Vector, Weak};

use std::{
    marker::PhantomData,
    mem,
    ops::{Deref, DerefMut},
    os::raw,
    ptr,
    ptr::NonNull,
    time::Duration,
};

#[derive(Debug)]
pub struct World<B, C> {
    world: Shared<Lock<NewtonWorld<B, C>>>,
    debug: Option<&'static str>,
}

#[derive(Debug)]
pub struct NewtonWorld<B, C> {
    world: NonNull<ffi::NewtonWorld>,
    /// Tx end of the command channel.
    pub(crate) tx: Tx<Command>,
    /// Rx end of the command channel, used by NewtonBody to signal the destruction of a body.
    rx: Rx<Command>,
    /// Buffer to hold world casting (ray & convex) information
    contacts: Lock<Vec<ffi::NewtonWorldConvexCastReturnInfo>>,
    _phantom: PhantomData<(B, C)>,
}

// Compiler complains about the raw pointer
unsafe impl<B, C> Send for NewtonWorld<B, C> {}
unsafe impl<B, C> Sync for NewtonWorld<B, C> {}

#[doc(hidden)]
pub struct NewtonWorldData<B, C> {
    /// We keep a reference to the world so we can obtain a copy of the lock from a `NewtonWorld`
    /// when needed.
    pub(crate) world: Weak<Lock<NewtonWorld<B, C>>>,
    /// A Tx end of the command channel, to recover it quickly from the userdata.
    tx: Tx<Command>,
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum Command {
    DestroyBody(*mut ffi::NewtonBody),
}

pub struct WorldBuilder<B, C> {
    solver: Solver,
    threads: usize,
    broad: Broadphase,
    debug: Option<&'static str>,
    _phantom: PhantomData<(B, C)>,
}

impl<B, C> WorldBuilder<B, C> {
    pub fn new() -> Self {
        WorldBuilder {
            solver: Solver::Exact,
            threads: 1,
            broad: Broadphase::Default,
            debug: None,
            _phantom: PhantomData,
        }
    }

    pub fn threads(&mut self, th: usize) -> &mut Self {
        self.threads = th;
        self
    }

    /// Sets the number of threads to the number of CPUs available in the system.
    pub fn max_threads(&mut self) -> &mut Self {
        self.threads = num_cpus::get();
        self
    }

    /// Set debug name, for use in Error reporting
    pub fn debug(&mut self, name: &'static str) -> &mut Self {
        self.debug = Some(name);
        self
    }

    pub fn exact_solver(mut self) -> Self {
        self.solver = Solver::Exact;
        self
    }

    pub fn linear_solver(&mut self, steps: usize) -> &mut Self {
        self.solver = Solver::Linear(steps);
        self
    }

    pub fn default_broadphase(&mut self) -> &mut Self {
        self.broad = Broadphase::Default;
        self
    }

    pub fn persistent_broadphase(&mut self) -> &mut Self {
        self.broad = Broadphase::Persistent;
        self
    }

    pub fn build(&mut self) -> World<B, C> {
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
        world: NonNull::new(world).unwrap(),
        tx,
        rx,
        contacts: Lock::new(Vec::with_capacity(16)),
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
pub struct WorldUpdateAsync<'a>(NonNull<ffi::NewtonWorld>, PhantomData<&'a ()>);

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
    pub fn builder() -> WorldBuilder<B, C> {
        WorldBuilder::new()
    }

    /// Creates a new World
    fn new(
        broadphase: Broadphase,
        solver: Solver,
        threads: usize,
        debug: Option<&'static str>,
    ) -> Self {
        let newton_world = create_world();

        let world_raw = newton_world.world.as_ptr();
        let tx = newton_world.tx.clone();
        let lock = Shared::new(Lock::new(newton_world));

        let userdata = Shared::new(NewtonWorldData {
            world: Shared::downgrade(&lock),
            tx,
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
        World { world: lock, debug }
    }

    #[inline]
    pub fn try_read(&self) -> Result<WorldLocked<B, C>> {
        Ok(WorldLocked(self.world.try_read()?))
    }

    #[inline]
    pub fn try_write(&self) -> Result<WorldLockedMut<B, C>> {
        Ok(WorldLockedMut(self.world.try_write(self.debug)?))
    }

    #[inline]
    pub fn read(&self) -> WorldLocked<B, C> {
        WorldLocked(self.world.read())
    }

    #[inline]
    pub fn write(&self) -> WorldLockedMut<B, C> {
        WorldLockedMut(self.world.write(self.debug))
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
                self.world.as_ptr(),
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
        mut max_contacts: usize,
        prefilter: Callback,
    ) -> &[ffi::NewtonWorldConvexCastReturnInfo]
    where
        Callback: Fn(&NewtonBody<B, C>, &NewtonCollision<B, C>) -> bool + 'static,
    {
        let mut info = self.contacts.write(None);

        if max_contacts > info.capacity() {
            max_contacts = info.capacity();
        }

        let contacts = unsafe {
            ffi::NewtonWorldConvexCast(
                self.world.as_ptr(),
                mem::transmute(matrix),
                mem::transmute(target),
                shape.as_raw(),
                // hitParam
                hit_param,
                // userdata
                mem::transmute(&prefilter),
                Some(prefilter_callback::<B, C, Callback>),
                // cast info
                info.as_mut_ptr(),
                max_contacts as raw::c_int,
                // thread index
                0,
            )
        };

        unsafe {
            return std::slice::from_raw_parts(info.as_ptr(), contacts as usize);
        }

        /*
        let body = unsafe { Box::new(NewtonBody::null_not_owned()) };

        unsafe {
            return_info.set_len(contacts as usize);

            return ConvexCast {
                count: 0,
                contacts: return_info,
                body: Box::into_raw(body),
                _phantom: PhantomData,
            };
        }
        */

        unsafe extern "C" fn prefilter_callback<B, C, Callback>(
            body: *const ffi::NewtonBody,
            collision: *const ffi::NewtonCollision,
            udata: *const raw::c_void,
        ) -> raw::c_uint
        where
            Callback: Fn(&NewtonBody<B, C>, &NewtonCollision<B, C>) -> bool + 'static,
        {
            let callback: &Callback = mem::transmute(udata);

            let body = NewtonBody::new_not_owned(body as _);
            let collision = NewtonCollision::new_not_owned(collision as _);
            if callback(&body, &collision) {
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
                self.world.as_ptr(),
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
        unsafe { ffi::NewtonMaterialSetSurfaceThickness(self.world.as_ptr(), u.0, v.0, thick) };
    }

    #[inline]
    pub fn material_set_friction(&mut self, (u, v): (GroupId, GroupId), fs: f32, fk: f32) {
        assert!(fs >= 0.0 && fk >= 0.0);
        unsafe { ffi::NewtonMaterialSetDefaultFriction(self.world.as_ptr(), u.0, v.0, fs, fk) };
    }

    #[inline]
    pub fn material_set_elasticity(&mut self, (u, v): (GroupId, GroupId), rest: f32) {
        assert!(rest >= 0.0);
        unsafe { ffi::NewtonMaterialSetDefaultElasticity(self.world.as_ptr(), u.0, v.0, rest) };
    }

    #[inline]
    pub fn material_set_softness(&mut self, (u, v): (GroupId, GroupId), soft: f32) {
        assert!(soft >= 0.0);
        unsafe { ffi::NewtonMaterialSetDefaultSoftness(self.world.as_ptr(), u.0, v.0, soft) };
    }

    #[inline]
    pub fn material_set_collidable(&mut self, (u, v): (GroupId, GroupId), collid: bool) {
        unsafe {
            ffi::NewtonMaterialSetDefaultCollidable(
                self.world.as_ptr(),
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
            let udata = ffi::NewtonMaterialGetUserData(self.world.as_ptr(), u.0, v.0);
            if !udata.is_null() {
                let _boxed_callback: Box<Callback> = Box::from_raw(udata as _);
            }

            ffi::NewtonMaterialSetCallbackUserData(
                self.world.as_ptr(),
                u.0,
                v.0,
                mem::transmute(Box::new(callback)),
            );
            ffi::NewtonMaterialSetCollisionCallback(
                self.world.as_ptr(),
                u.0,
                v.0,
                Some(super::callbacks::aabb_overlap_callback::<B, C, Callback>),
                Some(super::callbacks::contacts_process_callback),
            )
        }
    }

    /// Returns a body iterator
    pub fn bodies(&self) -> NewtonBodies<B, C> {
        NewtonBodies::new(self)
    }

    /// Returns a mutable body iterator
    pub fn bodies_mut(&mut self) -> NewtonBodiesMut<B, C> {
        NewtonBodiesMut::new(self)
    }

    #[inline]
    pub fn body_count(&self) -> raw::c_int {
        unsafe { ffi::NewtonWorldGetBodyCount(self.world.as_ptr()) }
    }

    pub fn constraint_count(&self) -> raw::c_int {
        unsafe { ffi::NewtonWorldGetConstraintCount(self.world.as_ptr()) }
    }

    /// Non-blocking variant of the `update` method.
    pub fn update_async(&mut self, step: Duration) -> WorldUpdateAsync {
        self.flush_commands(false);
        unsafe {
            ffi::NewtonUpdateAsync(self.world.as_ptr(), as_seconds(step));
        }
        WorldUpdateAsync(self.world, PhantomData)
    }

    /// Destroys unreferenced bodies and steps the simulation synchronously
    pub fn update(&mut self, step: Duration) {
        self.flush_commands(false);
        unsafe { ffi::NewtonUpdate(self.world.as_ptr(), as_seconds(step)) }
    }

    #[inline]
    pub fn invalidate_cache(&mut self) {
        unsafe { ffi::NewtonInvalidateCache(self.world.as_ptr()) }
    }

    #[inline]
    pub fn as_raw(&self) -> *const ffi::NewtonWorld {
        self.world.as_ptr()
    }

    #[inline]
    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonWorld {
        self.world.as_ptr()
    }

    fn dispatch_command(&self, cmd: Command) {
        match cmd {
            Command::DestroyBody(b) => {
                unsafe { super::body::drop_body::<B, C>(b) };
            }
        }
    }

    fn flush_commands(&self, blocking: bool) {
        if blocking {
            /// TODO FIXME this blocks the thread without the timeout...
            while let Ok(cmd) = self.rx.recv_timeout(Duration::new(0, 0)) {
                self.dispatch_command(cmd)
            }
        } else {
            for cmd in self.rx.try_iter() {
                self.dispatch_command(cmd)
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
            ffi::NewtonMaterialDestroyAllGroupID(self.world.as_ptr());
            let mut material = ffi::NewtonWorldGetFirstMaterial(self.world.as_ptr());
            while !material.is_null() {
                let udata = ffi::NewtonMaterialGetMaterialPairUserData(material);
                if !udata.is_null() {
                    let boxed_callback: Box<Callback> = Box::from_raw(udata as _);
                }
                material = ffi::NewtonWorldGetNextMaterial(self.world.as_ptr(), material);
            }
            ffi::NewtonMaterialDestroyAllGroupID(self.world.as_ptr());
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
        let world = self.world.as_ptr();
        unsafe {
            ffi::NewtonWaitForUpdateToFinish(world);
            let _: Shared<NewtonWorldData<B, C>> =
                mem::transmute(ffi::NewtonWorldGetUserData(world));
        }
        self.flush_commands(true);
        //println!("DROP world");
        unsafe {
            self.destroy_materials(|_, _, _, _| false);
            ffi::NewtonDestroyAllBodies(world);
            ffi::NewtonDestroy(world);
        }
    }
}

impl<'a> Drop for WorldUpdateAsync<'a> {
    fn drop(&mut self) {
        unsafe { ffi::NewtonWaitForUpdateToFinish(self.0.as_ptr()) };
    }
}

fn as_seconds(step: Duration) -> f32 {
    let nanos = step.as_secs() as f32 * 1_000_000_000.0 + step.subsec_nanos() as f32;
    nanos / 1_000_000_000.0
}

pub(crate) unsafe fn userdata<B, C>(
    world: *const ffi::NewtonWorld,
) -> Shared<NewtonWorldData<B, C>> {
    let udata: Shared<NewtonWorldData<B, C>> = mem::transmute(ffi::NewtonWorldGetUserData(world));
    let udata_cloned = udata.clone();
    mem::forget(udata);
    udata_cloned
}
