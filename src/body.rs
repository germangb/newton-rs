use ffi;
use NewtonData;
use world::WorldRef;

use std::mem;
use std::rc::Rc;
use std::marker::PhantomData;

pub type NewtonBody<V> = Rc<NewtonBodyInner<V>>;

#[derive(Debug)]
pub struct NewtonBodyInner<V> {
    pub(crate) world: Rc<WorldRef>,
    pub(crate) body: *mut ffi::NewtonBody,
    pub(crate) _ph: PhantomData<V>,
}

pub struct NewtonBodyBuilder<V: NewtonData> {
    pub(crate) world: Rc<WorldRef>,
    pub(crate) matrix: V::Matrix4,
    pub(crate) collision: *mut ffi::NewtonCollision,
    pub(crate) compute_mass: bool,
    pub(crate) mass: f32,
}

impl<V: NewtonData> NewtonBodyBuilder<V> {
    pub fn compute_mass(mut self, mass: f32) -> NewtonBodyBuilder<V> {
        self.mass = mass;
        self.compute_mass = true;
        self
    }

    pub fn build(self) -> NewtonBody<V> {
        unsafe {
            // XXX fix pointer
            let ptr = &self.matrix as *const _ as *const f32;
            let body = ffi::NewtonCreateDynamicBody(self.world.0, self.collision, ptr);

            // XXX remove this callback
            ffi::NewtonBodySetForceAndTorqueCallback(body, Some(cb_apply_force));

            if self.compute_mass {
                ffi::NewtonBodySetMassProperties(body, 1.0, self.collision);
            }

            let body = Rc::new(NewtonBodyInner {
                world: self.world,
                body,
                _ph: PhantomData,
            });

            let datum = mem::transmute(Rc::downgrade(&body));
            ffi::NewtonBodySetUserData(body.body, datum);

            body
        }
    }
}

impl<V> NewtonBodyInner<V>
where
    V: NewtonData,
{
    pub fn matrix(&self) -> V::Matrix4 {
        unsafe {
            // XXX pointers
            let mut mat = mem::zeroed();
            ffi::NewtonBodyGetMatrix(self.body, &mut mat as *mut _ as *mut f32);

            mat
        }
    }
}

impl<V> Drop for NewtonBodyInner<V> {
    fn drop(&mut self) {
        unsafe {
            ffi::NewtonDestroyBody(self.body);
        }
    }
}

// XXX
extern "C" fn cb_apply_force(body: *const ffi::NewtonBody, _timestep: f32, _thread_idx: i32) {
    unsafe { ffi::NewtonBodySetForce(body, [0.0, -4.0, 0.0].as_ptr()); }
}
