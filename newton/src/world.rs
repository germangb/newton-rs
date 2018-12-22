use crate::body::{self, NewtonBody, UserData as BodyUserData};
use crate::ffi;
use crate::pointer::*;
use crate::NewtonConfig;

use std::marker::PhantomData;
use std::rc::{Rc, Weak};
use std::time::Duration;
use std::{mem, ptr};

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
        unsafe {
            let raw = ffi::NewtonCreate();

            NewtonWorld {
                world: Rc::new(NewtonWorldPtr(raw, PhantomData)),
                raw,
            }
        }
    }

    // FIXME messy and not very flexible...
    pub fn bodies_in_aabb(&self, min: C::Vector3, max: C::Vector3) -> Vec<NewtonBody<C>> {
        unsafe {
            let mut bodies = Vec::<NewtonBody<C>>::new();
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
            let bodies: &mut Vec<NewtonBody<C>> = mem::transmute(user_data);
            let udata: Box<body::UserData<C>> = mem::transmute(ffi::NewtonBodyGetUserData(body));

            match (Weak::upgrade(&udata.body), Weak::upgrade(&udata.collision)) {
                (Some(body), Some(collision)) => {
                    let raw = body.0;
                    bodies.push(NewtonBody {
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
}
