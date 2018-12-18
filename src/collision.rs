use crate::ffi;
use crate::traits::{NewtonData, Vector};
use crate::world::NewtonWorld;

use std::marker::PhantomData;
use std::rc::Rc;

#[derive(Hash, Debug, Clone, Copy, Eq, PartialEq)]
pub struct ShapeId(pub i32);

impl From<i32> for ShapeId {
    fn from(n: i32) -> ShapeId {
        ShapeId(n)
    }
}

#[derive(Debug)]
pub struct NewtonCollision<V> {
    pub(crate) world: Rc<NewtonWorld<V>>,
    pub(crate) collision: *mut ffi::NewtonCollision,
    pub(crate) owned: bool,
    _ph: PhantomData<V>,
}

impl<V> Drop for NewtonCollision<V> {
    fn drop(&mut self) {
        if self.owned {
            unsafe { ffi::NewtonDestroyCollision(self.collision) }
        }
    }
}

impl<V: NewtonData> NewtonCollision<V> {
    pub fn from_raw(world: Rc<NewtonWorld<V>>, collision: *mut ffi::NewtonCollision) -> Self {
        NewtonCollision {
            world,
            collision,
            owned: true,
            _ph: PhantomData,
        }
    }

    pub fn new_box(
        world: Rc<NewtonWorld<V>>,
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
                ffi::NewtonCreateBox(world.world, size.0, size.1, size.2, id.0, offset_ptr);

            Self::from_raw(world, collision)
        }
    }
}
