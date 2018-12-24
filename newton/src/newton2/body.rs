use ffi;

use super::{Application, Types};
use super::world::{NewtonWorld, WorldRefMut};
use super::collision::{CollisionRefMut};

use std::mem;
use std::rc::{Rc, Weak};
use std::cell::{RefCell, Ref, RefMut};
use std::os::raw;
use std::ops::{Deref, DerefMut};

#[derive(Debug)]
pub enum Body<App> {
    Dynamic(DynamicBody<App>),
    Kinematic(KinematicBody<App>),
}

/// Ref-counted reference to a `NewtonBody`
#[derive(Debug)]
pub struct DynamicBody<App>(pub(crate) Rc<RefCell<NewtonWorld<App>>>, pub(crate) Rc<RefCell<NewtonBody<App>>>, pub(crate) *mut ffi::NewtonBody);

/// Ref-counted reference to a (*Kinematic*) `NewtonBody`
#[derive(Debug)]
pub struct KinematicBody<App>(pub(crate) Rc<RefCell<NewtonWorld<App>>>, pub(crate) Rc<RefCell<NewtonBody<App>>>, pub(crate) *mut ffi::NewtonBody);

#[doc(hidden)]
#[derive(Debug)]
pub struct NewtonBody<App>(pub(crate) Rc<RefCell<NewtonWorld<App>>>, pub(crate) *mut ffi::NewtonBody);

/// Immutable reference to a `NewtonBody`
#[derive(Debug)]
pub struct BodyRef<'w, 'b, App>(pub(crate) Ref<'w, NewtonWorld<App>>, pub(crate) Ref<'b, NewtonBody<App>>, pub(crate) *mut ffi::NewtonBody);

/// Mutable reference to a `NewtonBody`
#[derive(Debug)]
pub struct BodyRefMut<'w, 'b, App>(pub(crate) RefMut<'w, NewtonWorld<App>>, pub(crate) RefMut<'b, NewtonBody<App>>, pub(crate) *mut ffi::NewtonBody);

impl<F: Types, App: Application<Types=F>> DynamicBody<App> {
    pub fn new(collision: CollisionRefMut<App>, matrix: &F::Matrix) -> DynamicBody<App> {
        unsafe {
            // get refs from userdata
            let udata: Box<(Weak<RefCell<NewtonWorld<App>>>, Weak<RefCell<Self>>)> = mem::transmute(ffi::NewtonCollisionGetUserData(collision.as_raw()));

            let world_rc_cell = Weak::upgrade(&udata.0).unwrap();
            let body_raw = ffi::NewtonCreateDynamicBody(collision.2, collision.as_raw(), mem::transmute(matrix));

            let newton_body = Rc::new(RefCell::new(NewtonBody(world_rc_cell.clone(), body_raw)));

            mem::forget(udata);
            DynamicBody(world_rc_cell, newton_body, body_raw)
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
        unsafe {
            eprintln!("drop body");
            ffi::NewtonDestroyBody(self.1)
        }
    }
}
