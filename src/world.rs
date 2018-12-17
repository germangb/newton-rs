use crate::body::NewtonBody;
use crate::ffi;
use crate::traits::{NewtonMath, Vector};
use crate::{RefCount, WorldRef};

use std::marker::PhantomData;
use std::time::Duration;

#[derive(Debug, Clone)]
pub struct NewtonWorld<V> {
    pub(crate) world: RefCount<WorldRef>,
    _ph: PhantomData<V>,
}

impl<V: NewtonMath> NewtonWorld<V> {
    pub fn new() -> Self {
        unsafe {
            NewtonWorld {
                world: RefCount::new(WorldRef(ffi::NewtonCreate())),
                _ph: PhantomData,
            }
        }
    }

    pub fn update(&self, step: Duration) {
        unsafe {
            // TODO wait for as_float_secs() stabilization
            let nanos = step.subsec_nanos();
            let secs = (step.as_secs() as f64) + (nanos as f64) / (1_000_000_000 as f64);
            ffi::NewtonUpdate(self.world.0, secs as f32);
        }
    }
}
