use crate::body;
use crate::userdata::*;
use ffi;

use std::marker::PhantomData;
use std::mem;
use std::rc::{Rc, Weak};

#[derive(Debug)]
pub struct NewtonWorldPtr<C>(pub(crate) *mut ffi::NewtonWorld, pub(crate) PhantomData<C>);

#[derive(Debug)]
pub struct NewtonBodyPtr<C>(
    pub(crate) *mut ffi::NewtonBody,
    pub(crate) Rc<NewtonWorldPtr<C>>,
);

#[derive(Debug)]
pub struct NewtonCollisionPtr<C>(
    pub(crate) *mut ffi::NewtonCollision,
    pub(crate) Rc<NewtonWorldPtr<C>>,
);

impl<C> Drop for NewtonWorldPtr<C> {
    fn drop(&mut self) {
        unsafe {
            let _: Rc<C> = mem::transmute(ffi::NewtonWorldGetUserData(self.0));
            ffi::NewtonDestroy(self.0);
        }
    }
}

impl<C> Drop for NewtonBodyPtr<C> {
    fn drop(&mut self) {
        unsafe {
            let _: Box<BodyUserData<C>> = Box::from_raw(ffi::NewtonBodyGetUserData(self.0) as _);
            ffi::NewtonDestroyBody(self.0)
        }
    }
}

impl<V> Drop for NewtonCollisionPtr<V> {
    fn drop(&mut self) {
        unsafe { ffi::NewtonDestroyCollision(self.0) }
    }
}

#[derive(Debug)]
pub struct NewtonJointPtr<C>(
    pub(crate) *mut ffi::NewtonJoint,
    pub(crate) Weak<NewtonBodyPtr<C>>,
    pub(crate) Weak<NewtonBodyPtr<C>>,
);

impl<V> Drop for NewtonJointPtr<V> {
    fn drop(&mut self) {
        let parent = Weak::upgrade(&self.1);
        let child = Weak::upgrade(&self.2);

        if let Some(b) = parent.and(child) {
            let world: *mut ffi::NewtonWorld = (b.1).0;
            unsafe { ffi::NewtonDestroyJoint(world, self.0) }
        }
    }
}
