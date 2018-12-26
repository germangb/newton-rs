use ffi;

use super::collision::{CollisionParams, CollisionRefMut, CollisionUserDataInner, NewtonCollision};
use super::world::{NewtonWorld, WorldRefMut};
use super::Types;

use super::{Shared, Weak};

use std::cell::Cell;
use std::cell::{Ref, RefCell, RefMut};
use std::marker::PhantomData;
use std::mem;
use std::ops::{Deref, DerefMut};
use std::os::raw;

//use std::rc::{Rc, Weak};

use std::time::Duration;

pub type BodyId = i32;

#[derive(Debug, Clone)]
pub struct Body<T>(
    pub(crate) Shared<RefCell<NewtonWorld<T>>>,
    pub(crate) Shared<RefCell<NewtonBody<T>>>,
    pub(crate) *mut ffi::NewtonBody,
);

#[derive(Debug, Clone)]
pub struct NewtonBody<T> {
    pub(crate) world: Shared<RefCell<NewtonWorld<T>>>,
    pub(crate) body: *mut ffi::NewtonBody,
    pub(crate) owned: bool,
}

//#[derive(Debug)]
pub(crate) struct BodyUserDataInner<T> {
    pub(crate) world: Weak<RefCell<NewtonWorld<T>>>,
    pub(crate) body: Weak<RefCell<NewtonBody<T>>>,
    pub(crate) force_and_torque: Cell<Option<*mut ()>>,
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

#[derive(Debug)]
pub struct BodyRef<'a, T>(
    pub(crate) Ref<'a, NewtonWorld<T>>,
    pub(crate) Ref<'a, NewtonBody<T>>,
    pub(crate) *mut ffi::NewtonBody,
);

#[derive(Debug)]
pub struct BodyRefMut<'a, T>(
    pub(crate) RefMut<'a, NewtonWorld<T>>,
    pub(crate) RefMut<'a, NewtonBody<T>>,
    pub(crate) *mut ffi::NewtonBody,
);

#[derive(Debug)]
pub struct ContactJoints<'a, T>(PhantomData<&'a T>);

#[derive(Debug)]
pub struct Joints<'a, T>(PhantomData<&'a T>);

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

        Body(world_rc, body_rc, body)
    }
}

impl<T: Types> Body<T> {
    pub fn new(
        world: &mut NewtonWorld<T>,
        collision: &NewtonCollision<T>,
        ty: BodyType,
        matrix: &T::Matrix,
    ) -> Self {
        unsafe {
            // get refs from userdata
            let udata: Shared<CollisionUserDataInner<T>> =
                mem::transmute(ffi::NewtonCollisionGetUserData(collision.collision));

            // world userdata
            let world_userdata: Weak<RefCell<NewtonWorld<T>>> =
                mem::transmute(ffi::NewtonWorldGetUserData(world.0));
            let world_rc_cell = Weak::upgrade(&world_userdata).unwrap();

            mem::forget(world_userdata);

            let transform = mem::transmute(matrix);
            let body_raw = match ty {
                BodyType::Dynamic => {
                    ffi::NewtonCreateDynamicBody(world.0, collision.collision, transform)
                }
                BodyType::Kinematic => {
                    ffi::NewtonCreateKinematicBody(world.0, collision.collision, transform)
                }
            };

            let newton_body = Shared::new(RefCell::new(NewtonBody {
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
            Body(world_rc_cell, newton_body, body_raw)
        }
    }

    pub fn borrow(&self) -> BodyRef<T> {
        let world = self.0.borrow();
        let body = self.1.borrow();
        BodyRef(world, body, self.2)
    }

    pub fn borrow_mut(&self) -> BodyRefMut<T> {
        let world = self.0.borrow_mut();
        let body = self.1.borrow_mut();
        BodyRefMut(world, body, self.2)
    }
}

pub fn dynamic<T: Types>(
    world: &mut NewtonWorld<T>,
    collision: &NewtonCollision<T>,
    transform: &T::Matrix,
) -> Body<T> {
    Body::new(world, collision, BodyType::Dynamic, transform)
}

pub fn kinematic<T: Types>(
    world: &mut NewtonWorld<T>,
    collision: &NewtonCollision<T>,
    transform: &T::Matrix,
) -> Body<T> {
    Body::new(world, collision, BodyType::Kinematic, transform)
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

    pub fn contact_joints(&self) -> ContactJoints<T> {
        unimplemented!()
    }

    pub fn joints(&self) -> Joints<T> {
        unimplemented!()
    }

    pub fn body_type(&self) -> raw::c_int {
        unsafe { mem::transmute(ffi::NewtonBodyGetType(self.body)) }
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

impl<'a, T> Deref for BodyRef<'a, T> {
    type Target = NewtonBody<T>;

    fn deref(&self) -> &Self::Target {
        self.1.deref()
    }
}

impl<'a, T> Deref for BodyRefMut<'a, T> {
    type Target = NewtonBody<T>;

    fn deref(&self) -> &Self::Target {
        self.1.deref()
    }
}

impl<'a, T> DerefMut for BodyRefMut<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.1.deref_mut()
    }
}

impl<T> Drop for NewtonBody<T> {
    fn drop(&mut self) {
        if self.owned {
            // By freeing this body we are mutating the world, so...
            let _ = self.world.borrow_mut();

            unsafe {
                let body = self.body;
                let _: Shared<BodyUserDataInner<T>> =
                    mem::transmute(ffi::NewtonBodyGetUserData(body));
                ffi::NewtonDestroyBody(body)
            }
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
