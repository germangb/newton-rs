use ffi;

use super::collision::{
    CollisionLockedMut, CollisionParams, CollisionUserDataInner, NewtonCollision,
};
use super::joint::{Contacts, Joints};
use super::world::{NewtonWorld, WorldLockedMut};
use super::{Lock, Locked, LockedMut, Result, Shared, Types, Weak};

use std::cell::Cell;
use std::marker::PhantomData;
use std::mem;
use std::ops::{Deref, DerefMut};
use std::os::raw;

use std::time::Duration;

pub type BodyId = i32;

#[derive(Debug, Clone)]
pub struct Body<T>(Shared<Lock<NewtonWorld<T>>>, Shared<Lock<NewtonBody<T>>>);

#[derive(Debug, Clone)]
pub struct NewtonBody<T> {
    pub(crate) world: Shared<Lock<NewtonWorld<T>>>,
    pub(crate) body: *mut ffi::NewtonBody,
    pub(crate) owned: bool,
}

#[derive(Debug)]
pub(crate) struct BodyUserDataInner<T> {
    world: Weak<Lock<NewtonWorld<T>>>,
    body: Weak<Lock<NewtonBody<T>>>,
    force_and_torque: Cell<Option<*mut ()>>,
}

impl<T> Drop for BodyUserDataInner<T> {
    fn drop(&mut self) {
        unsafe {
            if let Some(closure) = self.force_and_torque.get() {
                // TODO FIXME Is this OK?
                ::std::ptr::drop_in_place(closure);
            }
        }
    }
}

// TODO FIXME check what happensa when collect() is called!!!
#[derive(Debug)]
pub struct Bodies<'a, T> {
    pub(crate) world: *mut ffi::NewtonWorld,
    pub(crate) next: *mut ffi::NewtonBody,
    pub(crate) body: *mut NewtonBody<T>,
    pub(crate) _phantom: PhantomData<&'a T>,
}

#[derive(Debug)]
pub struct BodiesMut<'a, T> {
    pub(crate) world: *mut ffi::NewtonWorld,
    pub(crate) next: *mut ffi::NewtonBody,
    pub(crate) body: *mut NewtonBody<T>,
    pub(crate) _phantom: PhantomData<&'a T>,
}

#[derive(Debug)]
pub struct BodyLocked<'a, T>(Locked<'a, NewtonWorld<T>>, Locked<'a, NewtonBody<T>>);

#[derive(Debug)]
pub struct BodyLockedMut<'a, T>(LockedMut<'a, NewtonWorld<T>>, LockedMut<'a, NewtonBody<T>>);

#[repr(i32)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub enum BodyType {
    Dynamic = ffi::NEWTON_DYNAMIC_BODY as _,
    Kinematic = ffi::NEWTON_KINEMATIC_BODY as _,
}

impl<T> Body<T> {
    pub unsafe fn from_raw_parts(body: *mut ffi::NewtonBody) -> Self {
        let udata: Shared<BodyUserDataInner<T>> = mem::transmute(ffi::NewtonBodyGetUserData(body));

        let body_rc = Weak::upgrade(&udata.body).unwrap();
        let world_rc = Weak::upgrade(&udata.world).unwrap();
        mem::forget(udata);

        Body(world_rc, body_rc)
    }
}

impl<T: Types> Body<T> {
    pub fn new(
        world: &mut NewtonWorld<T>,
        collision: &NewtonCollision<T>,
        ty: BodyType,
        matrix: &T::Matrix,
        user_data: Option<T::UserData>,
    ) -> Self {
        unsafe {
            let collision = collision.as_raw();
            // get refs from userdata
            let udata: Shared<CollisionUserDataInner<T>> =
                mem::transmute(ffi::NewtonCollisionGetUserData(collision));

            let world = world.as_raw();

            // world userdata
            let world_userdata: Weak<Lock<NewtonWorld<T>>> =
                mem::transmute(ffi::NewtonWorldGetUserData(world));
            let world_rc_cell = Weak::upgrade(&world_userdata).unwrap();

            mem::forget(world_userdata);

            let transform = mem::transmute(matrix);
            let body_raw = match ty {
                BodyType::Dynamic => ffi::NewtonCreateDynamicBody(world, collision, transform),
                BodyType::Kinematic => ffi::NewtonCreateKinematicBody(world, collision, transform),
            };

            let newton_body = Shared::new(Lock::new(NewtonBody {
                world: world_rc_cell.clone(),
                body: body_raw,
                owned: true,
            }));

            let userdata = BodyUserDataInner {
                body: Shared::downgrade(&newton_body),
                world: Shared::downgrade(&world_rc_cell),
                force_and_torque: Cell::new(None),
            };
            ffi::NewtonBodySetUserData(body_raw, mem::transmute(Shared::new(userdata)));

            mem::forget(udata);
            Body(world_rc_cell, newton_body)
        }
    }

    pub fn try_read(&self) -> Result<BodyLocked<T>> {
        unimplemented!()
    }

    pub fn try_write(&self) -> Result<BodyLockedMut<T>> {
        unimplemented!()
    }

    pub fn read(&self) -> BodyLocked<T> {
        let world = self.0.read();
        let body = self.1.read();
        BodyLocked(world, body)
    }

    pub fn write(&self) -> BodyLockedMut<T> {
        let world = self.0.write();
        let body = self.1.write();
        BodyLockedMut(world, body)
    }
}

pub fn dynamic<T: Types>(
    world: &mut NewtonWorld<T>,
    collision: &NewtonCollision<T>,
    transform: &T::Matrix,
    user_data: Option<T::UserData>,
) -> Body<T> {
    Body::new(world, collision, BodyType::Dynamic, transform, user_data)
}

pub fn kinematic<T: Types>(
    world: &mut NewtonWorld<T>,
    collision: &NewtonCollision<T>,
    transform: &T::Matrix,
    user_data: Option<T::UserData>,
) -> Body<T> {
    Body::new(world, collision, BodyType::Kinematic, transform, user_data)
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum SleepState {
    Awake = 0,
    Sleeping = 1,
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum FreezeState {
    Freeze = 1,
    Unfreeze = 0,
}

impl<T: Types> NewtonBody<T> {
    pub fn collision_params(&self) -> Shared<CollisionParams> {
        unsafe {
            let collision = ffi::NewtonBodyGetCollision(self.body);
            let collision_udata: Shared<CollisionUserDataInner<T>> =
                mem::transmute(ffi::NewtonCollisionGetUserData(collision));

            let params = collision_udata.params.clone();
            mem::forget(collision_udata);
            params
        }
    }

    pub fn contact_joints(&self) -> Contacts<T> {
        unimplemented!()
    }

    pub fn joints(&self) -> Joints<T> {
        unimplemented!()
    }

    pub fn body_type(&self) -> BodyType {
        unsafe { mem::transmute(ffi::NewtonBodyGetType(self.body)) }
    }

    pub fn is_dynamic(&self) -> bool {
        self.body_type() == BodyType::Dynamic
    }

    pub fn is_kinematic(&self) -> bool {
        self.body_type() == BodyType::Kinematic
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

    pub fn id(&self) -> BodyId {
        unsafe { ffi::NewtonBodyGetID(self.body) }
    }

    pub fn set_matrix_no_sleep(&mut self, matrix: &T::Matrix) {
        unsafe { ffi::NewtonBodySetMatrixNoSleep(self.body, mem::transmute(matrix)) };
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

    /*
    pub fn set_force_and_torque_callback<C: super::ForceAndTorque<T>>(&mut self) {
        unsafe {
            ffi::NewtonBodySetForceAndTorqueCallback(
                self.body,
                Some(force_and_torque_callback::<T, C>),
            );
        }
    }
    */

    // TODO FIXME unsafe unsafe unsafe...
    // It is less verbose than the ^^^ option, but is it actually better?
    pub fn set_force_and_torque<C>(&mut self, callback: C)
    where
        C: Fn(&mut NewtonBody<T>, Duration, raw::c_int) + 'static,
    {
        let datum = self.body_datum();

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

    fn body_datum(&self) -> Shared<BodyUserDataInner<T>> {
        unsafe {
            let body_userdata: Shared<BodyUserDataInner<T>> =
                mem::transmute(ffi::NewtonBodyGetUserData(self.body));

            let cloned = body_userdata.clone();
            mem::forget(body_userdata);
            cloned
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

                self.next = ffi::NewtonWorldGetNextBody(self.world, boxed.body);
                Some(mem::transmute(Box::into_raw(boxed)))
            }
        }
    }
}

impl<'a, T> Iterator for BodiesMut<'a, T> {
    type Item = &'a mut NewtonBody<T>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.next.is_null() {
            None
        } else {
            unsafe {
                let mut boxed = unsafe { Box::from_raw(self.body) };
                boxed.body = self.next;

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
            // By freeing this body we are mutating the world, so...
            let _ = self.world.write();

            unsafe {
                let body = self.body;
                let _: Shared<BodyUserDataInner<T>> =
                    mem::transmute(ffi::NewtonBodyGetUserData(body));
                ffi::NewtonDestroyBody(body)
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

impl<'a, T> Drop for BodiesMut<'a, T> {
    fn drop(&mut self) {
        unsafe {
            let _: Box<NewtonBody<T>> = Box::from_raw(self.body);
        }
    }
}

unsafe extern "C" fn force_and_torque_callback<T, C>(
    body: *const ffi::NewtonBody,
    timestep: raw::c_float,
    thread: raw::c_int,
) where
    T: Types,
    C: Fn(&mut NewtonBody<T>, Duration, raw::c_int) + 'static,
{
    let udata: Shared<BodyUserDataInner<T>> = mem::transmute(ffi::NewtonBodyGetUserData(body));
    let world_rc = Weak::upgrade(&udata.world).unwrap();

    if let Some(ptr) = udata.force_and_torque.get() {
        let callback = mem::transmute::<_, Box<C>>(ptr);

        let nanos = (1_000_000_000.0 * timestep) as u64;
        let timestep = Duration::new(nanos / 1_000_000_000, (nanos % 1_000_000_000) as u32);

        let mut body = NewtonBody {
            owned: false,
            world: world_rc,
            body: body as *mut _,
        };
        callback(&mut body, timestep, thread);

        mem::forget(callback);
    }

    mem::forget(udata);
}
