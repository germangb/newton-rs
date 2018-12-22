use crate::body;
use crate::ffi;
use crate::userdata::*;

use std::marker::PhantomData;
use std::rc::Rc;

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
        unsafe { ffi::NewtonDestroy(self.0) }
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
