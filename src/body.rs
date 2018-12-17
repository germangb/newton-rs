use crate::collision::NewtonCollision;
use crate::ffi;
use crate::traits::{NewtonMath, Vector};
use crate::world::NewtonWorld;
use crate::{BodyRef, CollisionRef, RefCount, WorldRef};

use std::marker::PhantomData;

#[derive(Debug, Clone)]
pub struct NewtonBody<V> {
    pub(crate) world: RefCount<WorldRef>,
    pub(crate) body: RefCount<BodyRef>,
    pub(crate) collision: RefCount<CollisionRef>,
    _ph: PhantomData<V>,
}

impl<V: NewtonMath> NewtonBody<V> {
    pub fn new(world: &NewtonWorld<V>, collision: NewtonCollision<V>, offset: V::Matrix4) -> Self {
        unsafe {
            let body =
                ffi::NewtonCreateDynamicBody(world.world.0, collision.collision.0, offset.as_ptr());
            ffi::NewtonBodySetForceAndTorqueCallback(body, Some(cb_apply_force));
            return NewtonBody {
                world: RefCount::clone(&world.world),
                collision: RefCount::clone(&collision.collision),
                body: RefCount::new(BodyRef(body)),
                _ph: PhantomData,
            };
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

    pub fn get_aabb(&self) -> (V::Vector3, V::Vector3) {
        unsafe {
            let mut p0 = V::Vector3::zero();
            let mut p1 = V::Vector3::zero();
            ffi::NewtonBodyGetAABB(self.body.0, p0.as_mut_ptr(), p1.as_mut_ptr());
            (p0, p1)
        }
    }

    pub fn get_matrix(&self) -> V::Matrix4 {
        unsafe {
            let mut mat = V::Matrix4::zero();
            ffi::NewtonBodyGetMatrix(self.body.0, mat.as_mut_ptr());
            mat
        }
    }

    pub fn set_mass_matrix(&self, mass: f32, inertia: (f32, f32, f32)) {
        unsafe {
            ffi::NewtonBodySetMassMatrix(self.body.0, mass, inertia.0, inertia.1, inertia.2);
        }
    }

    pub fn set_force(&self, force: V::Vector3) {
        unsafe {
            ffi::NewtonBodySetForce(self.body.0, force.as_ptr());
        }
    }
}
