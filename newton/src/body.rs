use ffi;

use super::collision::{CollisionParams, CollisionRefMut, CollisionUserDataInner, NewtonCollision};
use super::world::{NewtonWorld, WorldRefMut};
use super::Types;

use std::cell::{Ref, RefCell, RefMut};
use std::mem;
use std::ops::{Deref, DerefMut};
use std::os::raw;
use std::rc::{Rc, Weak};
use std::marker::PhantomData;

pub type BodyId = i32;

#[derive(Debug, Clone)]
pub struct Body<T>(pub(crate) Rc<RefCell<NewtonWorld<T>>>, pub(crate) Rc<RefCell<NewtonBody<T>>>, pub(crate) *mut ffi::NewtonBody);

#[derive(Debug, Clone)]
pub struct NewtonBody<T> {
    world: Rc<RefCell<NewtonWorld<T>>>,
    body: *mut ffi::NewtonBody,
    owned: bool,
}

#[derive(Debug)]
pub(crate) struct BodyUserDataInner<T> {
    pub(crate) world: Weak<RefCell<NewtonWorld<T>>>,
    pub(crate) body: Weak<RefCell<NewtonBody<T>>>,
}

#[derive(Debug)]
pub struct BodyRef<'a, T>(pub(crate) Ref<'a, NewtonWorld<T>>, pub(crate) Ref<'a, NewtonBody<T>>, pub(crate) *mut ffi::NewtonBody);

#[derive(Debug)]
pub struct BodyRefMut<'a, T>(pub(crate) RefMut<'a, NewtonWorld<T>>, pub(crate) RefMut<'a, NewtonBody<T>>, pub(crate) *mut ffi::NewtonBody);

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
        let udata: Rc<BodyUserDataInner<T>> = mem::transmute(ffi::NewtonBodyGetUserData(body));

        let body_rc = Weak::upgrade(&udata.body).unwrap();
        let world_rc = Weak::upgrade(&udata.world).unwrap();
        mem::forget(udata);

        Body(world_rc, body_rc, body)
    }
}

impl<T: Types> Body<T> {
    pub fn new(collision: &mut NewtonCollision<T>, ty: BodyType, matrix: &T::Matrix) -> Self {
        unsafe {
            // get refs from userdata
            let udata: Rc<CollisionUserDataInner<T>> =
                mem::transmute(ffi::NewtonCollisionGetUserData(collision.as_raw()));

            let world_rc_cell = Weak::upgrade(&udata.world).unwrap();
            let transform = mem::transmute(matrix);
            let body_raw = match ty {
                BodyType::Dynamic => ffi::NewtonCreateDynamicBody(
                    collision.world_raw,
                    collision.collision,
                    transform,
                ),
                BodyType::Kinematic => ffi::NewtonCreateKinematicBody(
                    collision.world_raw,
                    collision.collision,
                    transform,
                ),
            };

            /*
            ffi::NewtonBodySetForceAndTorqueCallback(
                body_raw,
                Some(force_and_torque_callback::<App>),
            );
            */

            let newton_body = Rc::new(RefCell::new(NewtonBody {
                world: world_rc_cell.clone(),
                body: body_raw,
                owned: true,
            }));

            let userdata = BodyUserDataInner {
                body: Rc::downgrade(&newton_body),
                world: Rc::downgrade(&world_rc_cell),
            };
            ffi::NewtonBodySetUserData(body_raw, mem::transmute(Rc::new(userdata)));

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

pub fn dynamic<T: Types>(collision: &mut NewtonCollision<T>, transform: &T::Matrix) -> Body<T> {
    Body::new(collision, BodyType::Dynamic, transform)
}

pub fn kinematic<T: Types>(collision: &mut NewtonCollision<T>, transform: &T::Matrix) -> Body<T> {
    Body::new(collision, BodyType::Kinematic, transform)
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
    pub fn collision_params(&self) -> Rc<CollisionParams> {
        unsafe {
            let collision = ffi::NewtonBodyGetCollision(self.body);
            let collision_udata: Rc<CollisionUserDataInner<T>> =
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

    pub fn set_force_and_torque_callback<C: super::ForceAndTorque<T>>(&mut self) {
        unsafe {
            ffi::NewtonBodySetForceAndTorqueCallback(
                self.body,
                Some(force_and_torque_callback::<T, C>),
            );
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
                let _: Rc<BodyUserDataInner<T>> =
                    mem::transmute(ffi::NewtonBodyGetUserData(body));
                ffi::NewtonDestroyBody(body)
            }
        }
    }
}

unsafe extern "C" fn force_and_torque_callback<T: Types, C: super::ForceAndTorque<T>>(
    body: *const ffi::NewtonBody,
    timestep: raw::c_float,
    _thread: raw::c_int,
) {
    let udata: Rc<BodyUserDataInner<T>> = mem::transmute(ffi::NewtonBodyGetUserData(body));
    let world_rc = Weak::upgrade(&udata.world).unwrap();
    mem::forget(udata);

    let mut body = NewtonBody {
        owned: false,
        world: world_rc,
        body: body as *mut _,
    };

    C::force_and_torque(&mut body)
}
