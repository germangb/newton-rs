use ffi;

use super::collision::CollisionRefMut;
use super::world::{NewtonWorld, WorldRefMut};
use super::{Application, Types};

use std::cell::{Ref, RefCell, RefMut};
use std::mem;
use std::ops::{Deref, DerefMut};
use std::os::raw;
use std::rc::{Rc, Weak};

macro_rules! implement_bodies {
    ($(
        $(#[$($meta:meta)+])+
        struct $struct_name:ident<$gen_type:ident> { ffi::$ffi:ident }
    )+) => {$(
        $(#[$($meta)+])+
        pub struct $struct_name<App>(
            pub(crate) Rc<RefCell<NewtonWorld<App>>>,
            pub(crate) Rc<RefCell<NewtonBody<App>>>,
            pub(crate) *mut ffi::NewtonBody,
        );
        impl<$gen_type> $struct_name<$gen_type> {
            pub unsafe fn from_raw_parts(world: *mut ffi::NewtonWorld, body: *mut ffi::NewtonBody) -> Self {
                // TODO use unsafe Weak & Rc functions instead of the safe ones
                // Ok, there are no unsafe funcions for Weak. Oh well

                // world userdatum
                let world_datum = ffi::NewtonWorldGetUserData(world);
                let world_datum: Weak<RefCell<NewtonWorld<$gen_type>>> = mem::transmute(world_datum);
                let world_datum_rc = Weak::upgrade(&world_datum).unwrap();

                // body userdatum
                let body_datum = ffi::NewtonBodyGetUserData(body);
                let body_datum: Weak<RefCell<NewtonBody<$gen_type>>> = mem::transmute(body_datum);
                let body_datum_rc = Weak::upgrade(&body_datum).unwrap();

                mem::forget(world_datum);
                mem::forget(body_datum);

                $struct_name(world_datum_rc, body_datum_rc, body)
            }
        }
        impl<F: Types, $gen_type: Application<Types = F>> $struct_name<$gen_type> {
            pub fn new(collision: CollisionRefMut<App>, matrix: &F::Matrix) -> Self {
                unsafe {
                    // get refs from userdata
                    let udata: Box<(Weak<RefCell<NewtonWorld<$gen_type>>>, Weak<RefCell<Self>>)> =
                        mem::transmute(ffi::NewtonCollisionGetUserData(collision.as_raw()));

                    let world_rc_cell = Weak::upgrade(&udata.0).unwrap();
                    let body_raw = ffi::$ffi(
                        collision.2,
                        collision.as_raw(),
                        mem::transmute(matrix),
                    );

                    let newton_body = Rc::new(RefCell::new(NewtonBody(world_rc_cell.clone(), body_raw)));

                    let newton_body_weak = Rc::downgrade(&newton_body);
                    ffi::NewtonBodySetUserData(body_raw, mem::transmute(newton_body_weak));

                    mem::forget(udata);
                    $struct_name(world_rc_cell, newton_body, body_raw)
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
    )+};
}

#[derive(Debug, Clone)]
pub enum Body<App> {
    Dynamic(DynamicBody<App>),
    Kinematic(KinematicBody<App>),
}

impl<App> Body<App> {
    pub fn dynamic(self) -> Option<DynamicBody<App>> {
        match self {
            Body::Dynamic(b) => Some(b),
            _ => None,
        }
    }

    pub fn kinematic(self) -> Option<KinematicBody<App>> {
        match self {
            Body::Kinematic(b) => Some(b),
            _ => None,
        }
    }

    pub fn is_dynamic(&self) -> bool {
        match &self {
            &Body::Dynamic(_) => true,
            _ => false,
        }
    }

    pub fn is_kinematic(&self) -> bool {
        match &self {
            &Body::Kinematic(_) => true,
            _ => false,
        }
    }
}

implement_bodies! {
    #[derive(Debug, Clone)]
    struct DynamicBody<App> { ffi::NewtonCreateDynamicBody }

    #[derive(Debug, Clone)]
    struct KinematicBody<App> { ffi::NewtonCreateKinematicBody }
}

#[derive(Debug, Clone)]
pub struct NewtonBody<App>(
    pub(crate) Rc<RefCell<NewtonWorld<App>>>,
    pub(crate) *mut ffi::NewtonBody,
);

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

impl<F: Types, App: Application<Types = F>> NewtonBody<App> {
    pub fn as_raw(&self) -> *const ffi::NewtonBody {
        self.1
    }

    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonBody {
        self.1
    }

    pub fn set_sleep_state(&mut self, state: SleepState) {
        unsafe { ffi::NewtonBodySetSleepState(self.as_raw_mut(), mem::transmute(state)) }
    }

    pub fn awake(&mut self) {
        self.set_sleep_state(SleepState::Awake)
    }

    pub fn aabb(&self) -> (F::Vector, F::Vector) {
        unsafe {
            let (mut min, mut max) = mem::zeroed();
            ffi::NewtonBodyGetAABB(self.1, mem::transmute(&mut min), mem::transmute(&mut max));
            (min, max)
        }
    }

    pub fn set_mass(&mut self, mass: f32) {
        unsafe {
            let collision = ffi::NewtonBodyGetCollision(self.1);
            ffi::NewtonBodySetMassProperties(self.1, mass, collision);
        }
    }

    pub fn set_linear_damping(&mut self, damping: f32) {
        unsafe { ffi::NewtonBodySetLinearDamping(self.1, damping) };
    }

    pub fn set_matrix(&mut self, matrix: &F::Matrix) {
        unsafe { ffi::NewtonBodySetMatrix(self.1, mem::transmute(matrix)) };
    }

    pub fn set_velocity(&mut self, matrix: &F::Vector) {
        unsafe { ffi::NewtonBodySetVelocity(self.1, mem::transmute(matrix)) };
    }

    pub fn set_matrix_no_sleep(&mut self, matrix: &F::Matrix) {
        unsafe { ffi::NewtonBodySetMatrixNoSleep(self.1, mem::transmute(matrix)) };
    }

    pub fn matrix(&self) -> F::Matrix {
        unsafe {
            let mut mat: F::Matrix = mem::zeroed();
            ffi::NewtonBodyGetMatrix(self.1, mem::transmute(&mut mat));
            mat
        }
    }

    pub fn position(&self) -> F::Vector {
        unsafe {
            let mut pos: F::Vector = mem::zeroed();
            ffi::NewtonBodyGetPosition(self.1, mem::transmute(&mut pos));
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
        let _ = self.0.borrow_mut();
        let body = self.1;
        unsafe {
            //eprintln!("drop body");
            let _: Weak<Rc<RefCell<NewtonBody<App>>>> =
                mem::transmute(ffi::NewtonBodyGetUserData(body));
            ffi::NewtonDestroyBody(body)
        }
    }
}
