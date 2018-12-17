use crate::ffi;
use crate::traits::{NewtonMath, Vector};
use crate::world::NewtonWorld;
use crate::{CollisionRef, RefCount, WorldRef};

use std::marker::PhantomData;

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub struct ShapeId(pub i32);

#[derive(Clone)]
pub enum Shape {
    Box { dx: f32, dy: f32, dz: f32 },
}

#[derive(Clone)]
pub struct NewtonCollision<V> {
    pub(crate) world: RefCount<WorldRef>,
    pub(crate) collision: RefCount<CollisionRef>,
    shape: Shape,
    _ph: PhantomData<V>,
}

impl<V: NewtonMath> NewtonCollision<V> {
    pub fn new_box(
        world: &NewtonWorld<V>,
        size: (f32, f32, f32),
        id: ShapeId,
        offset: Option<V::Matrix4>,
    ) -> Self {
        unsafe {
            let offset_ptr = if let &Some(ref m) = &offset {
                m.as_ptr()
            } else {
                ::std::ptr::null()
            };
            let collision =
                ffi::NewtonCreateBox(world.world.raw, size.0, size.1, size.2, id.0, offset_ptr);

            NewtonCollision {
                world: RefCount::clone(&world.world),
                collision: RefCount::new(CollisionRef::from_raw_parts(collision, true)),
                shape: Shape::Box {
                    dx: size.0,
                    dy: size.1,
                    dz: size.2,
                },
                _ph: PhantomData,
            }
        }
    }

    pub fn shape(&self) -> &Shape {
        &self.shape
    }
}
