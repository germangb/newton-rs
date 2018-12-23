use ffi;

use crate::body::{self, DynamicBody};
use crate::pointer::*;
use crate::userdata::*;
use crate::NewtonApp;

use std::marker::PhantomData;
use std::rc::{Rc, Weak};
use std::time::Duration;
use std::{mem, ptr};

#[derive(Debug, Clone)]
pub struct World<C> {
    pub(crate) world: Rc<NewtonWorldPtr<C>>,
    pub(crate) raw: *mut ffi::NewtonWorld,
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum BroadPhaseAlgorithm {
    Default = ffi::NEWTON_BROADPHASE_DEFAULT as i32,
    Persistent = ffi::NEWTON_BROADPHASE_PERSINTENT as i32,
}

impl<C> World<C>
where
    C: NewtonApp,
{
    pub fn new(algorithm: BroadPhaseAlgorithm, app: C) -> Self {
        unsafe {
            let raw = ffi::NewtonCreate();
            ffi::NewtonSelectBroadphaseAlgorithm(raw, mem::transmute(algorithm));
            let world = Rc::new(NewtonWorldPtr(raw, PhantomData));
            ffi::NewtonWorldSetUserData(raw, mem::transmute(Rc::downgrade(&world)));
            World { world, raw }
        }
    }

    pub fn create() -> World<C> {
        unsafe {
            let raw = ffi::NewtonCreate();

            World {
                world: Rc::new(NewtonWorldPtr(raw, PhantomData)),
                raw,
            }
        }
    }

    // FIXME messy and not very flexible...
    pub fn bodies_in_aabb(&self, min: C::Vector, max: C::Vector) -> Vec<DynamicBody<C>> {
        unsafe {
            let mut bodies = Vec::<DynamicBody<C>>::new();
            ffi::NewtonWorldForEachBodyInAABBDo(
                self.raw,
                mem::transmute(&min),
                mem::transmute(&max),
                Some(bodies_in_aabb::<C>),
                mem::transmute(&mut bodies),
            );

            return bodies;
        }

        unsafe extern "C" fn bodies_in_aabb<C>(
            body: *const ffi::NewtonBody,
            user_data: *const ::std::os::raw::c_void,
        ) -> i32 {
            let bodies: &mut Vec<DynamicBody<C>> = mem::transmute(user_data);
            let udata: Box<BodyUserData<C>> = mem::transmute(ffi::NewtonBodyGetUserData(body));

            match (Weak::upgrade(&udata.body), Weak::upgrade(&udata.collision)) {
                (Some(body), Some(collision)) => {
                    let raw = body.0;
                    bodies.push(DynamicBody {
                        body,
                        collision,
                        raw,
                    });
                }
                _ => {}
            }

            //mem::forget(bodies);
            mem::forget(udata);
            1
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

    pub fn constraint_count(&self) -> usize {
        unsafe { ffi::NewtonWorldGetConstraintCount(self.raw) as usize }
    }
}
