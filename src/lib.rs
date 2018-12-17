extern crate cgmath;

use std::rc::{Rc, Weak};
use std::marker::PhantomData;
use std::time::Duration;

pub mod ffi;
pub mod traits;

use traits::*;

pub type ShapeId = i32;

#[derive(Debug, Clone)]
pub struct WorldRef(*mut ffi::NewtonWorld);

#[derive(Debug, Clone)]
pub struct CollisionRef(*mut ffi::NewtonCollision);

#[derive(Debug, Clone)]
pub struct BodyRef(*mut ffi::NewtonBody);

unsafe impl Send for WorldRef {}
unsafe impl Send for CollisionRef {}

impl Drop for WorldRef {
    fn drop(&mut self) {
        unsafe {
            ffi::NewtonDestroyAllBodies(self.0);
            ffi::NewtonDestroy(self.0);
        }
    }
}

impl Drop for CollisionRef {
    fn drop(&mut self) {
        unsafe { ffi::NewtonDestroyCollision(self.0) }
    }
}

impl Drop for BodyRef {
    fn drop(&mut self) {
        unsafe { ffi::NewtonDestroyBody(self.0) }
    }
}

#[derive(Debug, Clone)]
pub struct NewtonWorld<V> {
    world: Rc<WorldRef>,
    _ph: PhantomData<V>,
}

#[derive(Debug, Clone)]
pub struct NewtonBody<V> {
    world: Rc<WorldRef>,
    body: Rc<BodyRef>,
    _ph: PhantomData<V>,
}

#[derive(Clone)]
pub enum CollisionShape {
    Box {
        dx: f32,
        dy: f32,
        dz: f32,
    },
}

#[derive(Clone)]
pub struct NewtonCollision<V> {
    world: Rc<WorldRef>,
    collision: Rc<CollisionRef>,
    shape: CollisionShape,
    _ph: PhantomData<V>,
}

impl<V> NewtonWorld<V>
where
    V: NewtonMath,
{
    pub fn new() -> NewtonWorld<V> {
        unsafe {
            NewtonWorld {
                world: Rc::new(WorldRef(ffi::NewtonCreate())),
                _ph: PhantomData,
            }
        }
    }

    pub fn update(&mut self, step: Duration) {
        unsafe {
            // as_float_secs() is unstable
            let nanos = step.subsec_nanos();
            let secs = (step.as_secs() as f64) + (nanos as f64) / (1_000_000_000 as f64);
            ffi::NewtonUpdate(self.world.0, secs as f32);
        }
    }

    pub fn create_body(&self, collision: NewtonCollision<V>, offset: V::Matrix4) -> NewtonBody<V> {
        unsafe {
            let body = ffi::NewtonCreateDynamicBody(self.world.0, collision.collision.0, offset.as_ptr());
            //panic!();

            ffi::NewtonBodySetForceAndTorqueCallback(body, Some(cb_apply_force));

            return NewtonBody {
                world: Rc::clone(&self.world),
                body: Rc::new(BodyRef(body)),
                _ph: PhantomData,
            }
        }

        extern "C" fn cb_apply_force(body: *const ffi::NewtonBody, _timestep: f32, _thread_idx: i32) {
            unsafe {
                ffi::NewtonBodySetForce(body, [0.0, -1.0, 0.0].as_ptr());
            }
        }
    }

    pub fn create_box(&self, size: (f32, f32, f32), id: ShapeId, offset: Option<V::Matrix4>) -> NewtonCollision<V> {
        unsafe {
            let collision = if let &Some(ref m) = &offset {
                ffi::NewtonCreateBox(self.world.0, size.0, size.1, size.2, id, m.as_ptr())
            } else {
                ffi::NewtonCreateBox(self.world.0, size.0, size.1, size.2, id, ::std::ptr::null())
            };
            NewtonCollision {
                world: Rc::clone(&self.world),
                collision: Rc::new(CollisionRef(collision)),
                shape: CollisionShape::Box {
                    dx: size.0,
                    dy: size.1,
                    dz: size.2,
                },
                _ph: PhantomData,
            }
        }
    }
}

impl<V> NewtonBody<V>
where
    V: NewtonMath
{
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