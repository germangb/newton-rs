use crate::body;
use ffi;

use std::cell::RefCell;
use std::marker::PhantomData;
use std::mem;
use std::rc::{Rc, Weak};

#[derive(Debug)]
pub struct NewtonWorldPtr<C>(pub(crate) *mut ffi::NewtonWorld, pub(crate) PhantomData<C>);

#[derive(Debug)]
pub struct NewtonBodyPtr<C>(
    pub(crate) *mut ffi::NewtonBody,
    pub(crate) Rc<RefCell<NewtonWorldPtr<C>>>,
);

#[derive(Debug)]
pub struct NewtonCollisionPtr<C>(
    pub(crate) *mut ffi::NewtonCollision,
    pub(crate) Rc<RefCell<NewtonWorldPtr<C>>>,
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
            let world = self.1.borrow_mut();
            let _: Box<crate::body::BodyUserData<C>> =
                Box::from_raw(ffi::NewtonBodyGetUserData(self.0) as _);
            ffi::NewtonDestroyBody(self.0)
        }
    }
}

impl<V> Drop for NewtonCollisionPtr<V> {
    fn drop(&mut self) {
        let _world = self.1.borrow_mut();
        unsafe {
            let ptr = ffi::NewtonCollisionGetUserData(self.0);
            if !ptr.is_null() {
                /// TODO FIXME standarize what is saved as userdata!!
                let _: Box<crate::collision::HeightFieldParams<f32>> = Box::from_raw(ptr as _);
            }
            ffi::NewtonDestroyCollision(self.0);
        }
    }
}

#[derive(Debug)]
pub struct NewtonJointPtr<C>(
    pub(crate) *mut ffi::NewtonJoint,
    pub(crate) Rc<RefCell<NewtonWorldPtr<C>>>,
    pub(crate) Weak<NewtonBodyPtr<C>>,
    pub(crate) Weak<NewtonBodyPtr<C>>,
);

impl<V> Drop for NewtonJointPtr<V> {
    fn drop(&mut self) {
        let parent = Weak::upgrade(&self.2);
        let child = Weak::upgrade(&self.3);

        if let Some(b) = parent.and(child) {
            let world_mut_ref = b.1.try_borrow_mut().expect("Cannot perform Mutable borrow");
            let world: *mut ffi::NewtonWorld = world_mut_ref.0;
            unsafe { ffi::NewtonDestroyJoint(world, self.0) }
        }
    }
}
