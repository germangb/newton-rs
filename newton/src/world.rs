use ffi;

use crate::body::{self, Bodies, DynamicBody};
use crate::pointer::*;
use crate::NewtonApp;

use std::cell::RefCell;
use std::marker::PhantomData;
use std::rc::{Rc, Weak};
use std::time::Duration;
use std::{mem, ptr};

#[derive(Debug, Clone)]
pub struct World<C> {
    pub(crate) world: Rc<RefCell<NewtonWorldPtr<C>>>,
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum BroadPhaseAlgorithm {
    Default = ffi::NEWTON_BROADPHASE_DEFAULT as i32,
    Persistent = ffi::NEWTON_BROADPHASE_PERSINTENT as i32,
}

impl<C: NewtonApp> World<C> {
    pub fn new(algorithm: BroadPhaseAlgorithm, app: C) -> Self {
        let world = unsafe {
            let raw = ffi::NewtonCreate();
            ffi::NewtonSelectBroadphaseAlgorithm(raw, mem::transmute(algorithm));

            let world = Rc::new(RefCell::new(NewtonWorldPtr(raw, PhantomData)));
            ffi::NewtonWorldSetUserData(raw, mem::transmute(Rc::downgrade(&world)));

            world
        };

        World { world }
    }

    pub fn bodies(&self) -> Bodies<C> {
        let world = self.world.borrow();
        let ptr = world.0;
        unsafe {
            Bodies {
                world,
                world_ptr: ptr,
                current: ffi::NewtonWorldGetFirstBody(ptr),
                _ph: PhantomData,
            }
        }
    }

    pub fn update(&self, step: Duration) {
        let world = self.world.borrow_mut();
        let secs = step.as_secs() as f64 + (step.subsec_nanos() as f64) / 1_000_000_000_f64;
        unsafe {
            ffi::NewtonUpdate(world.0, secs as f32);
        }
    }

    pub fn set_substeps(&self, steps: usize) {
        let world = self.world.borrow();
        unsafe {
            ffi::NewtonSetNumberOfSubsteps(world.0, steps as _);
        }
    }

    pub fn body_count(&self) -> usize {
        let world = self.world.borrow();
        unsafe { ffi::NewtonWorldGetBodyCount(world.0) as usize }
    }

    pub fn constraint_count(&self) -> usize {
        let world = self.world.borrow();
        unsafe { ffi::NewtonWorldGetConstraintCount(world.0) as usize }
    }
}
