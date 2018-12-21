use crate::body::{NewtonBody, UserData as BodyUserData};
use crate::ffi;
use crate::NewtonConfig;

use std::cell::RefCell;
use std::marker::PhantomData;
use std::mem;
use std::ptr;
use std::rc::{Rc, Weak};
use std::time::Duration;

#[doc(hidden)]
#[derive(Debug)]
pub struct NewtonWorldPtr<C>(pub(crate) *mut ffi::NewtonWorld, PhantomData<C>);

impl<C> Drop for NewtonWorldPtr<C> {
    fn drop(&mut self) {
        unsafe { ffi::NewtonDestroy(self.0) }
    }
}

#[derive(Debug, Clone)]
pub struct NewtonWorld<C> {
    pub(crate) world: Rc<NewtonWorldPtr<C>>,
    pub(crate) raw: *mut ffi::NewtonWorld,
}

impl<C> NewtonWorld<C>
where
    C: NewtonConfig,
{
    pub fn new() -> NewtonWorld<C> {
        assert_config!(C);
        unsafe {
            let raw = ffi::NewtonCreate();

            NewtonWorld {
                world: Rc::new(NewtonWorldPtr(raw, PhantomData)),
                raw,
            }
        }
    }

    pub fn update(&self, step: Duration) {
        unsafe {
            let secs = step.as_secs() as f64 + (step.subsec_nanos() as f64) / 1_000_000_000_f64;
            ffi::NewtonUpdate(self.raw, secs as f32);
        }
    }

    pub fn set_substeps(&self, steps: usize) {
        unsafe {
            ffi::NewtonSetNumberOfSubsteps(self.raw, steps as _);
        }
    }

    pub fn body_count(&self) -> usize {
        unsafe { ffi::NewtonWorldGetBodyCount(self.raw) as usize }
    }
}
