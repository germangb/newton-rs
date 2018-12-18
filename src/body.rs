use crate::collision::NewtonCollision;
use crate::ffi;
use crate::traits::{NewtonData, Vector};
use crate::world::NewtonWorld;

use std::marker::PhantomData;
use std::rc::Rc;

#[derive(Debug)]
pub struct NewtonBody<V> {
    pub(crate) world: Rc<NewtonWorld<V>>,
    pub(crate) body: *mut ffi::NewtonBody,

    pub(crate) owned: bool,
    _ph: PhantomData<V>,
}

impl<V> Drop for NewtonBody<V> {
    fn drop(&mut self) {
        if self.owned {
            unsafe { ffi::NewtonDestroyBody(self.body) }
        }
    }
}

impl<V: NewtonData> NewtonBody<V> {
    pub fn new(
        world: Rc<NewtonWorld<V>>,
        collision: &NewtonCollision<V>,
        offset: V::Matrix4,
    ) -> Rc<Self> {
        unsafe {
            let body =
                ffi::NewtonCreateDynamicBody(world.world, collision.collision, offset.as_ptr());
            ffi::NewtonBodySetForceAndTorqueCallback(body, Some(cb_apply_force));
            return Rc::new(NewtonBody {
                world,
                owned: true,
                body,
                _ph: PhantomData,
            });
        }

        extern "C" fn cb_apply_force(
            body: *const ffi::NewtonBody,
            _timestep: f32,
            _thread_idx: i32,
        ) {
            unsafe {
                ffi::NewtonBodySetForce(body, [0.0, -1.0, 0.0].as_ptr());
            }
        }
    }

    pub fn aabb(&self) -> (V::Vector3, V::Vector3) {
        unsafe {
            let mut p0 = V::Vector3::zero();
            let mut p1 = V::Vector3::zero();
            ffi::NewtonBodyGetAABB(self.body, p0.as_mut_ptr(), p1.as_mut_ptr());
            (p0, p1)
        }
    }

    pub fn matrix(&self) -> V::Matrix4 {
        unsafe {
            let mut mat = V::Matrix4::zero();
            ffi::NewtonBodyGetMatrix(self.body, mat.as_mut_ptr());
            mat
        }
    }

    pub fn collision(&self) -> NewtonCollision<V> {
        unimplemented!()
    }

    pub fn set_linear_damping(&self, damping: f32) {
        unsafe {
            ffi::NewtonBodySetLinearDamping(self.body, damping);
        }
    }

    pub fn set_full_mass_matrix(&self, mass: f32, inertia: V::Matrix4) {
        unsafe {
            ffi::NewtonBodySetFullMassMatrix(self.body, mass, inertia.as_ptr());
        }
    }

    pub fn set_mass_properties(&self, mass: f32, collision: &NewtonCollision<V>) {
        unsafe {
            ffi::NewtonBodySetMassProperties(self.body, mass, collision.collision);
        }
    }

    pub fn set_mass_matrix(&self, mass: f32, inertia: (f32, f32, f32)) {
        unsafe {
            ffi::NewtonBodySetMassMatrix(self.body, mass, inertia.0, inertia.1, inertia.2);
        }
    }

    pub fn set_force(&self, force: V::Vector3) {
        unsafe {
            ffi::NewtonBodySetForce(self.body, force.as_ptr());
        }
    }
}
