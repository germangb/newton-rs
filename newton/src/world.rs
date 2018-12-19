use crate::body::{NewtonBody, NewtonBodyInner, UserData as BodyUserData};
use crate::ffi;
use crate::NewtonConfig;

use std::cell::RefCell;
use std::marker::PhantomData;
use std::mem;
use std::ptr;
use std::rc::{Rc, Weak};
use std::sync::mpsc;
use std::time::Duration;

type Rx<T> = mpsc::Receiver<T>;
type Tx<T> = mpsc::Sender<T>;

#[doc(hidden)]
#[derive(Debug)]
pub struct WorldRef(pub(crate) *mut ffi::NewtonWorld);

#[derive(Debug)]
pub struct NewtonWorld<V> {
    pub(crate) world: Rc<WorldRef>,

    bodies: Rx<()>,
    joints: Rx<()>,

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

#[doc(hidden)]
#[derive(Debug)]
struct UserData {
    bodies: Tx<()>,
    joints: Tx<()>,
}

impl<V> NewtonWorld<V>
where
    V: NewtonConfig,
{
    pub fn new() -> NewtonWorld<V> {
        unsafe {
            let world = ffi::NewtonCreate();

            let (b_tx, b_rx) = mpsc::channel();
            let (j_tx, j_rx) = mpsc::channel();

            ffi::NewtonWorldSetUserData(
                world,
                mem::transmute(Box::new(UserData {
                    bodies: b_tx,
                    joints: j_tx,
                })),
            );

            NewtonWorld {
                world: Rc::new(WorldRef(world)),
                bodies: b_rx,
                joints: j_rx,
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
            let _: Box<UserData> = Box::from_raw(ffi::NewtonWorldGetUserData(self.0) as _);
            ffi::NewtonDestroy(self.0);
        }
    }
}
