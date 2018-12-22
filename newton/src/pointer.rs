use crate::body;
use crate::ffi;

use std::marker::PhantomData;
use std::rc::Rc;

#[doc(hidden)]
#[derive(Debug)]
pub struct NewtonWorldPtr<C>(pub(crate) *mut ffi::NewtonWorld, pub(crate) PhantomData<C>);

#[doc(hidden)]
#[derive(Debug)]
pub struct NewtonBodyPtr<C>(pub(crate) *mut ffi::NewtonBody, pub(crate) PhantomData<C>);

#[doc(hidden)]
#[derive(Debug)]
pub struct NewtonCollisionPtr<C>(
    pub(crate) *mut ffi::NewtonCollision,
    // Keep a reference to the world so that ALL collisions are dropped before the world is
    pub(crate) Rc<NewtonWorldPtr<C>>,
    pub(crate) PhantomData<C>,
);

impl<C> Drop for NewtonWorldPtr<C> {
    fn drop(&mut self) {
        unsafe { ffi::NewtonDestroy(self.0) }
    }
}

impl<C> Drop for NewtonBodyPtr<C> {
    fn drop(&mut self) {
        unsafe {
            let _: Box<body::UserData<C>> = Box::from_raw(ffi::NewtonBodyGetUserData(self.0) as _);
            ffi::NewtonDestroyBody(self.0)
        }
    }
}

impl<V> Drop for NewtonCollisionPtr<V> {
    fn drop(&mut self) {
        unsafe {
            //let _: Box<CollisionInfo> = Box::from_raw(ffi::NewtonCollisionGetUserData(self.0) as _);
            ffi::NewtonDestroyCollision(self.0);
        }
    }
}
