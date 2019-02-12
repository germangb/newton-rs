use std::marker::PhantomData;
use std::mem;
use std::ptr::NonNull;

use super::ffi;
use super::world::Newton;
use super::{Matrix, Vector};

/// NewtonCollision wrapper.
///
/// Collisions in Rust are borrow types of the Newton context.
#[derive(Debug)]
pub struct Collision<'world> {
    pub(crate) newton: &'world Newton,
    /// Underlying NewtonCollision pointer
    ///
    /// This is the pointer passed to all calls to `NewtonCollision*` API functions.
    pub(crate) collision: NonNull<ffi::NewtonCollision>,
    /// If owned is set to true, the underlying NewtonCollision
    /// will be dropped along with this type.
    pub(crate) owned: bool,
}

/// A handle to a collision that is owned by the underlying NewtonWorld.
///
/// This type is meant for long-lived collisions. The underlying NewtonCollision
/// is accessible through [`collision`][collision] and [`collision_mut`][collision_mut]
///
/// [collision]: #
/// [collision_mut]: #
#[derive(Debug, Hash, Eq, PartialEq, Clone)]
pub struct CollisionHandle(pub(crate) NonNull<ffi::NewtonCollision>);

impl CollisionHandle {
    /// Returns the underlying NewtonCollision pointer.
    pub const fn as_ptr(&self) -> *const ffi::NewtonCollision {
        self.0.as_ptr()
    }
}

impl<'world> Collision<'world> {
    /// Sets collision scale
    pub fn set_scale(&self, scale: &Vector) {
        unsafe {
            ffi::NewtonCollisionSetScale(self.as_ptr(), scale[0], scale[1], scale[2]);
        }
    }

    /// Creates a sphere collision
    pub fn sphere(
        newton: &'world Newton,
        radius: f32,
        offset: Option<&Matrix>,
    ) -> Collision<'world> {
        unsafe {
            let offset = mem::transmute(offset);
            let sphere = ffi::NewtonCreateSphere(newton.as_ptr(), radius, 0, offset);

            Collision {
                newton,
                collision: NonNull::new_unchecked(sphere),
                owned: true,
            }
        }
    }

    /// Transfer ownership to the Newton context and return a handle to access
    /// the collision later.
    ///
    /// Method intended to be used for long-lived bodies.
    pub fn into_handle(mut self) -> CollisionHandle {
        unimplemented!()
    }

    /// Returns underlying NewtonCollision pointer
    pub const fn as_ptr(&self) -> *const ffi::NewtonCollision {
        self.collision.as_ptr()
    }
}

impl<'world> Drop for Collision<'world> {
    fn drop(&mut self) {
        if self.owned {
            unsafe { ffi::NewtonDestroyCollision(self.as_ptr()) }
        }
    }
}
