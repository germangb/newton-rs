use crate::body::{NewtonBody, NewtonBodyInner, UserData as BodyUserData};
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
pub struct WorldRef(pub(crate) *mut ffi::NewtonWorld);

#[derive(Debug)]
pub struct NewtonWorld<V> {
    pub(crate) world: Rc<WorldRef>,
    _ph: PhantomData<V>,
}

#[derive(Debug)]
pub struct Bodies<V>(Vec<NewtonBody<V>>);

impl<V> ::std::ops::Deref for Bodies<V> {
    type Target = [NewtonBody<V>];

    fn deref(&self) -> &Self::Target {
        &self.0[..]
    }
}

impl<V> NewtonWorld<V>
where
    V: NewtonConfig,
{
    pub fn new() -> NewtonWorld<V> {
        unsafe {
            let world = ffi::NewtonCreate();

            NewtonWorld {
                world: Rc::new(WorldRef(world)),
                _ph: PhantomData,
            }
        }
    }

    pub fn update(&self, step: Duration) {
        unsafe {
            let secs = step.as_secs() as f64 + (step.subsec_nanos() as f64) / 1_000_000_000_f64;
            ffi::NewtonUpdate(self.world.0, secs as f32);
        }
    }

    pub fn set_substeps(&self, steps: usize) {
        unsafe {
            ffi::NewtonSetNumberOfSubsteps(self.world.0, steps as _);
        }
    }

    pub fn body_count(&self) -> usize {
        unsafe { ffi::NewtonWorldGetBodyCount(self.world.0) as usize }
    }

    pub fn bodies(&self, p0: V::Vector3, p1: V::Vector3) -> Bodies<V> {
        type BodyVec<V> = RefCell<Vec<NewtonBody<V>>>;

        unsafe {
            // XXX fix pointers
            let p0_ptr = &p0 as *const _ as *const f32;
            let p1_ptr = &p1 as *const _ as *const f32;

            let bodies: BodyVec<V> = RefCell::new(Vec::new());
            ffi::NewtonWorldForEachBodyInAABBDo(
                self.world.0,
                p0_ptr,
                p1_ptr,
                Some(newton_body_iterator::<V>),
                &bodies as *const _ as *const _,
            );

            return Bodies(bodies.into_inner());
        }

        // FIXME
        unsafe extern "C" fn newton_body_iterator<V>(
            body: *const ffi::NewtonBody,
            user_data: *const std::ffi::c_void,
        ) -> i32 {
            let bodies: &BodyVec<V> = mem::transmute(user_data);
            let mut vec = bodies.borrow_mut();

            // body weak shared reference
            let body = ffi::NewtonBodyGetUserData(body);
            let body_udata: BodyUserData<V> = mem::transmute(body);

            if let Some(b) = body_udata.body.upgrade() {
                vec.push(b);
            }

            1
        }
    }
}

impl Drop for WorldRef {
    fn drop(&mut self) {
        unsafe {
            ffi::NewtonDestroy(self.0);
        }
    }
}
