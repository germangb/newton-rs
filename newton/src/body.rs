use ffi;

use super::collision::{CollisionRefMut, CollisionUserDataInner, CollisionParams, NewtonCollision};
use super::world::{NewtonWorld, WorldRefMut};
use super::{Application, Types};

use std::cell::{Ref, RefCell, RefMut};
use std::mem;
use std::ops::{Deref, DerefMut};
use std::os::raw;
use std::rc::{Rc, Weak};

pub type BodyId = i32;

#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub enum BodyType {
    Dynamic,
    Kinematic,
}

#[derive(Debug)]
pub(crate) struct BodyUserDataInner<App> {
    pub(crate) world: Weak<RefCell<NewtonWorld<App>>>,
    pub(crate) body: Weak<RefCell<NewtonBody<App>>>,
}

#[derive(Debug)]
pub struct Body<App>(
    pub(crate) Rc<RefCell<NewtonWorld<App>>>,
    pub(crate) Rc<RefCell<NewtonBody<App>>>,
    pub(crate) *mut ffi::NewtonBody,
);

impl<App> Body<App> {
    pub unsafe fn from_raw_parts(body: *mut ffi::NewtonBody) -> Self {
        let udata: Rc<BodyUserDataInner<App>> = mem::transmute(ffi::NewtonBodyGetUserData(body));

        let body_rc = Weak::upgrade(&udata.body).unwrap();
        let world_rc = Weak::upgrade(&udata.world).unwrap();
        mem::forget(udata);

        Body(world_rc, body_rc, body)
    }
}

impl<F: Types, App: Application<Types = F>> Body<App> {
    pub fn new(collision: CollisionRefMut<App>, ty: BodyType, matrix: &F::Matrix) -> Self {
        unsafe {
            // get refs from userdata
            let udata: Rc<CollisionUserDataInner<App>> =
                mem::transmute(ffi::NewtonCollisionGetUserData(collision.as_raw()));

            let world_rc_cell = Weak::upgrade(&udata.world).unwrap();
            let transform = mem::transmute(matrix);
            let body_raw = match ty {
                BodyType::Dynamic => {
                    ffi::NewtonCreateDynamicBody(collision.2, collision.as_raw(), transform)
                }
                BodyType::Kinematic => {
                    ffi::NewtonCreateKinematicBody(collision.2, collision.as_raw(), transform)
                }
            };

            ffi::NewtonBodySetForceAndTorqueCallback(
                body_raw,
                Some(force_and_torque_callback::<App>),
            );

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

    pub fn borrow(&self) -> BodyRef<App> {
        let world = self.0.borrow();
        let body = self.1.borrow();
        BodyRef(world, body, self.2)
    }

    pub fn borrow_mut(&self) -> BodyRefMut<App> {
        let world = self.0.borrow_mut();
        let body = self.1.borrow_mut();
        BodyRefMut(world, body, self.2)
    }
}

#[derive(Debug, Clone)]
pub struct NewtonBody<App> {
    world: Rc<RefCell<NewtonWorld<App>>>,
    body: *mut ffi::NewtonBody,
    owned: bool,
}

#[derive(Debug)]
pub struct BodyRef<'w, 'b, App>(
    pub(crate) Ref<'w, NewtonWorld<App>>,
    pub(crate) Ref<'b, NewtonBody<App>>,
    pub(crate) *mut ffi::NewtonBody,
);

#[derive(Debug)]
pub struct BodyRefMut<'w, 'b, App>(
    pub(crate) RefMut<'w, NewtonWorld<App>>,
    pub(crate) RefMut<'b, NewtonBody<App>>,
    pub(crate) *mut ffi::NewtonBody,
);

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

impl<F: Types, App: Application<Types = F>> NewtonBody<App> {
    pub fn collision_params(&self) -> Rc<CollisionParams> {
        unsafe {
            let collision = ffi::NewtonBodyGetCollision(self.body);
            let collision_udata: Rc<CollisionUserDataInner<App>> = mem::transmute(ffi::NewtonCollisionGetUserData(collision));

            let params = collision_udata.params.clone();
            mem::forget(collision_udata);
            params
        }
    }

    pub fn as_raw(&self) -> *const ffi::NewtonBody {
        self.body
    }

    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonBody {
        self.body
    }

    pub fn sleep_state(&self) -> SleepState {
        unsafe {
            mem::transmute(ffi::NewtonBodyGetSleepState(self.body))
        }
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

    pub fn aabb(&self) -> (F::Vector, F::Vector) {
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

    pub fn set_matrix(&mut self, matrix: &F::Matrix) {
        unsafe { ffi::NewtonBodySetMatrix(self.body, mem::transmute(matrix)) };
    }

    pub fn set_velocity(&mut self, matrix: &F::Vector) {
        unsafe { ffi::NewtonBodySetVelocity(self.body, mem::transmute(matrix)) };
    }

    pub fn set_force(&mut self, force: &F::Vector) {
        unsafe { ffi::NewtonBodySetForce(self.body, mem::transmute(force)) };
    }

    pub fn set_torque(&mut self, torque: &F::Vector) {
        unsafe { ffi::NewtonBodySetTorque(self.body, mem::transmute(torque)) };
    }

    pub fn id(&self) -> BodyId {
        unsafe { ffi::NewtonBodyGetID(self.body) }
    }

    pub fn set_matrix_no_sleep(&mut self, matrix: &F::Matrix) {
        unsafe { ffi::NewtonBodySetMatrixNoSleep(self.body, mem::transmute(matrix)) };
    }

    pub fn matrix(&self) -> F::Matrix {
        unsafe {
            let mut mat: F::Matrix = mem::zeroed();
            ffi::NewtonBodyGetMatrix(self.body, mem::transmute(&mut mat));
            mat
        }
    }

    pub fn position(&self) -> F::Vector {
        unsafe {
            let mut pos: F::Vector = mem::zeroed();
            ffi::NewtonBodyGetPosition(self.body, mem::transmute(&mut pos));
            pos
        }
    }
}

impl<'w, 'b, App> Deref for BodyRef<'w, 'b, App> {
    type Target = NewtonBody<App>;

    fn deref(&self) -> &Self::Target {
        self.1.deref()
    }
}

impl<'w, 'b, App> Deref for BodyRefMut<'w, 'b, App> {
    type Target = NewtonBody<App>;

    fn deref(&self) -> &Self::Target {
        self.1.deref()
    }
}

impl<'w, 'b, App> DerefMut for BodyRefMut<'w, 'b, App> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.1.deref_mut()
    }
}

impl<App> Drop for NewtonBody<App> {
    fn drop(&mut self) {
        if self.owned {
            let _ = self.world.borrow_mut();
            unsafe {
                let body = self.body;
                let _: Rc<BodyUserDataInner<App>> =
                    mem::transmute(ffi::NewtonBodyGetUserData(body));
                ffi::NewtonDestroyBody(body)
            }
        }
    }
}

unsafe extern "C" fn force_and_torque_callback<App: Application>(body: *const ffi::NewtonBody,
                                                                 timestep: raw::c_float,
                                                                 _thread: raw::c_int) {

    let udata: Rc<BodyUserDataInner<App>> = mem::transmute(ffi::NewtonBodyGetUserData(body));
    let world_rc = Weak::upgrade(&udata.world).unwrap();
    mem::forget(udata);

    let mut body = NewtonBody {
        owned: false,
        world: world_rc,
        body: body as *mut _,
    };

    App::force_and_torque(&mut body)
}
