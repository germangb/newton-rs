use ffi;

use super::collision::NewtonCollision;
use super::command::Command;
use super::joint::{Contacts, Joints};
use super::world::{NewtonWorld, WorldLockedMut};
use super::{Lock, Locked, LockedMut, Result, Shared, Tx, Types, Weak};

use std::{
    cell::Cell,
    marker::PhantomData,
    mem,
    ops::{Deref, DerefMut},
    os::raw,
    sync::mpsc,
    time::Duration,
};

#[derive(Debug, Clone)]
pub struct Body<T>(
    Shared<Lock<NewtonWorld<T>>>,
    Shared<Lock<NewtonBody<T>>>,
    *const ffi::NewtonBody,
);

#[derive(Debug)]
pub struct NewtonBody<T> {
    body: *mut ffi::NewtonBody,

    /// Bodies must be dropped before the world is.
    world: Shared<Lock<NewtonWorld<T>>>,

    /// A non-owned `NewtonCollision`
    pub(crate) collision: NewtonCollision<T>,

    /// Owned `NewtonBody`s, when dropped, are destroyed and therefore removed from the simulation
    owned: bool,
    /// The Tx end of the command channel. Only for owned variants,
    tx: Option<Tx<Command>>,
}

pub struct Builder<'a, 'b, T: Types> {
    world: &'a mut NewtonWorld<T>,
    collision: &'b NewtonCollision<T>,

    /// Type of the body (Dynamic or Kinematic)
    type_: Type,
    /// A name given to the collision.
    debug: Option<&'static str>,

    /// Initial transformation
    transform: Option<T::Matrix>,
}

impl<'a, 'b, T: Types> Builder<'a, 'b, T> {
    pub fn new(world: &'a mut NewtonWorld<T>, collision: &'b NewtonCollision<T>) -> Self {
        Builder {
            world,
            collision,
            type_: Type::Dynamic,
            debug: None,
            transform: None,
        }
    }

    /// Consumes the builder and returns a body
    pub fn build(self) -> Body<T> {
        Body::new(
            self.world,
            self.collision,
            self.type_,
            &self.transform.unwrap(),
            self.debug,
        )
    }

    pub fn transform(mut self, transform: T::Matrix) -> Self {
        self.transform = Some(transform);
        self
    }

    pub fn debug(mut self, name: &'static str) -> Self {
        self.debug = Some(name);
        self
    }

    pub fn dynamic(mut self) -> Self {
        self.type_ = Type::Dynamic;
        self
    }

    pub fn kinematic(mut self) -> Self {
        self.type_ = Type::Kinematic;
        self
    }
}

impl<T> NewtonBody<T> {
    /// Wraps a raw `ffi::NewtonBody` pointer
    pub(crate) unsafe fn new_not_owned(body: *mut ffi::NewtonBody) -> Self {
        let udata = userdata::<T>(body);
        let collision = ffi::NewtonBodyGetCollision(body);
        NewtonBody {
            owned: false,
            body,
            world: Weak::upgrade(&udata.world).unwrap(),
            collision: NewtonCollision::new_not_owned(collision),
            tx: None,
        }
    }
}

pub(crate) unsafe fn drop_body<T>(body: *const ffi::NewtonBody) {
    let _: Shared<BodyData<T>> = mem::transmute(ffi::NewtonBodyGetUserData(body));
    ffi::NewtonDestroyBody(body);
}

pub(crate) unsafe fn userdata<T>(body: *const ffi::NewtonBody) -> Shared<BodyData<T>> {
    let udata: Shared<BodyData<T>> = mem::transmute(ffi::NewtonBodyGetUserData(body));
    let udata_cloned = udata.clone();
    mem::forget(udata);
    udata_cloned
}

#[doc(hidden)]
#[derive(Debug)]
pub struct BodyData<T> {
    /// A reference to the `World` context so it can be referenced in callbacks and so on
    world: Weak<Lock<NewtonWorld<T>>>,
    /// A reference to itself
    body: Weak<Lock<NewtonBody<T>>>,

    /// Debug name
    debug: Option<&'static str>,

    /// A callback that is called by the `NewtonBody` to apply gravity and other forces to update
    /// the state of a dynamic body.
    force_and_torque: Cell<Option<*mut ()>>,
}

impl<T> Drop for BodyData<T> {
    fn drop(&mut self) {
        if let Some(closure) = self.force_and_torque.get() {
            // TODO FIXME Is this OK? Probably not
            unsafe {
                ::std::ptr::drop_in_place(closure);
            }
        }
    }
}

// TODO remove pub visibility
/// Iterator over the bodies in a `NewtonWorld`
#[derive(Debug)]
pub struct Bodies<'a, T> {
    pub(crate) world: *mut ffi::NewtonWorld,

    /// A raw pointer to the next body to be returned by the iterator
    pub(crate) next: *mut ffi::NewtonBody,
    /// The NewtonBody reference that gets returned by the iterator. This data is stored on the
    /// heap so it may have a slight effect in performance.
    pub(crate) body: *mut NewtonBody<T>,

    pub(crate) _phantom: PhantomData<&'a T>,
}

/// Reference to a `NewtonWorld`
#[derive(Debug)]
pub struct BodyLocked<'a, T>(Locked<'a, NewtonWorld<T>>, Locked<'a, NewtonBody<T>>);

/// Mutable reference to a `NewtonWorld`
#[derive(Debug)]
pub struct BodyLockedMut<'a, T>(LockedMut<'a, NewtonWorld<T>>, LockedMut<'a, NewtonBody<T>>);

#[repr(i32)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub enum Type {
    Dynamic = ffi::NEWTON_DYNAMIC_BODY as _,
    /// Bodies that only have an effect at collision level.
    Kinematic = ffi::NEWTON_KINEMATIC_BODY as _,
}

impl<T> Body<T> {
    pub unsafe fn from_raw_parts(body: *mut ffi::NewtonBody) -> Self {
        let udata = userdata::<T>(body);
        let body_rc = Weak::upgrade(&udata.body).unwrap();
        let world_rc = Weak::upgrade(&udata.world).unwrap();
        Body(world_rc, body_rc, body as *const _)
    }
}

impl<T: Types> Body<T> {
    pub fn new(
        world: &mut NewtonWorld<T>,
        collision: &NewtonCollision<T>,
        ty: Type,
        matrix: &T::Matrix,
        debug: Option<&'static str>,
    ) -> Self {
        let collision = collision.as_raw();
        let world_ptr = world.as_raw_mut();

        unsafe {
            let transform = mem::transmute(matrix);
            let body_ptr = match ty {
                Type::Dynamic => ffi::NewtonCreateDynamicBody(world_ptr, collision, transform),
                Type::Kinematic => ffi::NewtonCreateKinematicBody(world_ptr, collision, transform),
            };

            let world_udata = super::world::userdata(world_ptr);

            let world_lock = Weak::upgrade(&world_udata.world).unwrap();
            let body_lock = Shared::new(Lock::new(NewtonBody {
                body: body_ptr,
                world: world_lock.clone(),
                tx: Some(world.tx.clone()),
                collision: NewtonCollision::new_not_owned(collision as _),
                owned: true,
            }));

            let userdata = Shared::new(BodyData {
                body: Shared::downgrade(&body_lock),
                world: Shared::downgrade(&world_lock),
                force_and_torque: Cell::new(None),
                debug,
            });

            ffi::NewtonBodySetUserData(body_ptr, mem::transmute(userdata));

            Body(world_lock, body_lock, body_ptr)
        }
    }

    pub fn try_read(&self) -> Result<BodyLocked<T>> {
        let world = self.0.try_read()?;
        let body = self.1.try_read()?;
        Ok(BodyLocked(world, body))
    }

    pub fn try_write(&self) -> Result<BodyLockedMut<T>> {
        #[cfg(feature = "debug")]
        {
            let udata = unsafe { userdata::<T>(self.2) };
            let world = self.0.try_write(udata.debug)?;
            let body = self.1.try_write(udata.debug)?;
            Ok(BodyLockedMut(world, body))
        }
        #[cfg(not(feature = "debug"))]
        {
            let world = self.0.try_write(None)?;
            let body = self.1.try_write(None)?;
            Ok(BodyLockedMut(world, body))
        }
    }

    pub fn read(&self) -> BodyLocked<T> {
        self.try_read().unwrap()
    }

    pub fn write(&self) -> BodyLockedMut<T> {
        self.try_write().unwrap()
    }
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum SleepState {
    Awake = 0,
    /// Dynamic body equilibrium state
    Sleeping = 1,
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum FreezeState {
    Freeze = 1,
    Unfreeze = 0,
}

impl<T: Types> NewtonBody<T> {
    pub fn collision(&self) -> &NewtonCollision<T> {
        &self.collision
    }

    pub fn contact_joints(&self) -> Contacts<T> {
        unimplemented!()
    }

    pub fn joints(&self) -> Joints<T> {
        unimplemented!()
    }

    pub fn body_type(&self) -> Type {
        unsafe { mem::transmute(ffi::NewtonBodyGetType(self.body)) }
    }

    pub fn is_dynamic(&self) -> bool {
        self.body_type() == Type::Dynamic
    }

    pub fn is_kinematic(&self) -> bool {
        self.body_type() == Type::Kinematic
    }

    pub fn as_raw(&self) -> *const ffi::NewtonBody {
        self.body
    }

    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonBody {
        self.body
    }

    pub fn sleep_state(&self) -> SleepState {
        unsafe { mem::transmute(ffi::NewtonBodyGetSleepState(self.body)) }
    }

    pub fn set_sleep_state(&mut self, state: SleepState) {
        unsafe { ffi::NewtonBodySetSleepState(self.body, mem::transmute(state)) }
    }

    pub fn set_freeze_state(&mut self, state: FreezeState) {
        unsafe { ffi::NewtonBodySetFreezeState(self.body, mem::transmute(state)) }
    }

    pub fn awake(&mut self) {
        self.set_sleep_state(SleepState::Awake)
    }

    pub fn aabb(&self) -> (T::Vector, T::Vector) {
        unsafe {
            let (mut min, mut max) = mem::zeroed();
            ffi::NewtonBodyGetAABB(
                self.body,
                mem::transmute(&mut min),
                mem::transmute(&mut max),
            );
            (min, max)
        }
    }

    pub fn set_mass(&mut self, mass: f32) {
        unsafe {
            let collision = ffi::NewtonBodyGetCollision(self.body);
            ffi::NewtonBodySetMassProperties(self.body, mass, collision);
        }
    }

    pub fn set_linear_damping(&mut self, damping: f32) {
        unsafe { ffi::NewtonBodySetLinearDamping(self.body, damping) };
    }

    pub fn set_matrix(&mut self, matrix: &T::Matrix) {
        unsafe { ffi::NewtonBodySetMatrix(self.body, mem::transmute(matrix)) };
    }

    pub fn set_velocity(&mut self, matrix: &T::Vector) {
        unsafe { ffi::NewtonBodySetVelocity(self.body, mem::transmute(matrix)) };
    }

    pub fn set_force(&mut self, force: &T::Vector) {
        unsafe { ffi::NewtonBodySetForce(self.body, mem::transmute(force)) };
    }

    pub fn set_torque(&mut self, torque: &T::Vector) {
        unsafe { ffi::NewtonBodySetTorque(self.body, mem::transmute(torque)) };
    }

    pub fn id(&self) -> raw::c_int {
        unsafe { ffi::NewtonBodyGetID(self.body) }
    }

    pub fn set_matrix_no_sleep(&mut self, matrix: &T::Matrix) {
        unsafe { ffi::NewtonBodySetMatrixNoSleep(self.body, mem::transmute(matrix)) };
    }

    pub fn set_collidable(&mut self, collidable: bool) {
        unsafe {
            ffi::NewtonBodySetCollidable(self.body, collidable as _);
        }
    }

    pub fn matrix(&self) -> T::Matrix {
        unsafe {
            let mut mat: T::Matrix = mem::zeroed();
            ffi::NewtonBodyGetMatrix(self.body, mem::transmute(&mut mat));
            mat
        }
    }

    pub fn position(&self) -> T::Vector {
        unsafe {
            let mut pos: T::Vector = mem::zeroed();
            ffi::NewtonBodyGetPosition(self.body, mem::transmute(&mut pos));
            pos
        }
    }

    // TODO FIXME unsafe unsafe unsafe...
    pub fn set_force_and_torque<C>(&mut self, callback: C)
    where
        C: Fn(&mut NewtonBody<T>, Duration, raw::c_int) + 'static,
    {
        let datum = unsafe { userdata::<T>(self.body) };

        if let Some(ptr) = datum.force_and_torque.get() {
            unsafe {
                let _: Box<C> = Box::from_raw(ptr as _);
            }
        }

        unsafe {
            datum
                .force_and_torque
                .set(Some(mem::transmute(Box::new(callback))));

            ffi::NewtonBodySetForceAndTorqueCallback(
                self.body,
                Some(force_and_torque_callback::<T, C>),
            );
        }
    }
}

impl<'a, T> Iterator for Bodies<'a, T> {
    type Item = &'a NewtonBody<T>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.next.is_null() {
            None
        } else {
            unsafe {
                let mut boxed = unsafe { Box::from_raw(self.body) };
                boxed.body = self.next;
                boxed.collision.collision = ffi::NewtonBodyGetCollision(self.next);

                self.next = ffi::NewtonWorldGetNextBody(self.world, boxed.body);
                Some(mem::transmute(Box::into_raw(boxed)))
            }
        }
    }
}

impl<'a, T> Deref for BodyLocked<'a, T> {
    type Target = NewtonBody<T>;

    fn deref(&self) -> &Self::Target {
        self.1.deref()
    }
}

impl<'a, T> Deref for BodyLockedMut<'a, T> {
    type Target = NewtonBody<T>;

    fn deref(&self) -> &Self::Target {
        self.1.deref()
    }
}

impl<'a, T> DerefMut for BodyLockedMut<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.1.deref_mut()
    }
}

impl<T> Drop for NewtonBody<T> {
    fn drop(&mut self) {
        if self.owned {
            unsafe {
                let body = self.body;
                if let &Some(ref tx) = &self.tx {
                    tx.send(Command::DestroyBody(body)).unwrap();
                }
            }
        }
    }
}

impl<'a, T> Drop for Bodies<'a, T> {
    fn drop(&mut self) {
        unsafe {
            let _: Box<NewtonBody<T>> = Box::from_raw(self.body);
        }
    }
}

extern "C" fn force_and_torque_callback<T, C>(
    body: *const ffi::NewtonBody,
    timestep: raw::c_float,
    thread: raw::c_int,
) where
    T: Types,
    C: Fn(&mut NewtonBody<T>, Duration, raw::c_int) + 'static,
{
    let udata = unsafe { userdata::<T>(body as _) };

    if let Some(ptr) = udata.force_and_torque.get() {
        let callback = unsafe { mem::transmute::<_, Box<C>>(ptr) };

        let mut body = unsafe { NewtonBody::<T>::new_not_owned(body as _) };
        callback(&mut body, to_duration(timestep), thread);
        mem::forget(callback);
    }
}

fn to_duration(timestep: f32) -> Duration {
    const NANO: u64 = 1_000_000_000;
    let nanos = (NANO as f32 * timestep) as u64;
    Duration::new(nanos / NANO, (nanos % NANO) as u32)
}
