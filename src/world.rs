use crate::body::NewtonBody;
use crate::ffi;
use crate::traits::{NewtonData, Vector};

use std::marker::PhantomData;
use std::rc::Rc;
use std::time::Duration;

#[derive(Debug, Clone)]
pub struct NewtonWorld<V> {
    pub(crate) world: *mut ffi::NewtonWorld,
    _ph: PhantomData<V>,
}

impl<V> Drop for NewtonWorld<V> {
    fn drop(&mut self) {
        unsafe {
            ffi::NewtonDestroy(self.world);
        }
    }
}

impl<V: NewtonData> NewtonWorld<V> {
    pub fn new() -> Rc<Self> {
        unsafe {
            Rc::new(NewtonWorld {
                world: ffi::NewtonCreate(),
                _ph: PhantomData,
            })
        }
    }

    pub fn update(&self, step: Duration) {
        unsafe {
            // TODO wait for as_float_secs() stabilization
            let nanos = step.subsec_nanos();
            let secs = (step.as_secs() as f64) + (nanos as f64) / (1_000_000_000 as f64);
            ffi::NewtonUpdate(self.world, secs as f32);
        }
    }
}
