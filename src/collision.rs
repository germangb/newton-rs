use ffi;
use NewtonData;
use world::{NewtonWorld, WorldRef};
use body::{NewtonBodyInner, NewtonBodyBuilder};

use std::rc::Rc;
use std::mem;
use std::ptr;
use std::marker::PhantomData;

#[derive(Debug)]
pub struct NewtonCuboid<V> {
    pub(self) world: Rc<WorldRef>,
    collision: *mut ffi::NewtonCollision,
    _ph: PhantomData<V>,
}

impl<V> NewtonCuboid<V>
where
    V: NewtonData,
{
    pub fn new(world: &NewtonWorld<V>, dx: f32, dy: f32, dz: f32) -> NewtonCuboid<V> {
        unsafe {
            NewtonCuboid {
                world: Rc::clone(&world.world),
                collision: ffi::NewtonCreateBox(world.world.0, dx, dy, dz, 0, ptr::null()),
                _ph: PhantomData,
            }
        }
    }

    pub fn body(&self, matrix: V::Matrix4) -> NewtonBodyBuilder<V> {
        NewtonBodyBuilder {
            world: self.world.clone(),
            matrix,
            collision: self.collision,
            compute_mass: false,
            mass: 0.0,
        }
    }
}

impl<V> Drop for NewtonCuboid<V> {
    fn drop(&mut self) {
        unsafe {
            ffi::NewtonDestroyCollision(self.collision);
        }
    }
}

