use std::marker::PhantomData;
use std::mem;
use std::ptr::NonNull;

use super::ffi;
use super::world::Newton;
use super::{Matrix, Vector};

#[derive(Debug)]
pub struct Builder {
    /// Collision offset.
    ///
    /// Applies a constant offset to the collision geometry.
    offset: Option<Matrix>,

    /// Collision scale.
    ///
    /// By setting this optional, `NewtonBodySetCollisionScale` is called.
    scale: Option<Vector>,
}

impl Builder {
    pub fn offset(&mut self, offset: Matrix) -> &mut Self {
        self.offset = Some(offset);
        self
    }

    pub fn scale(&mut self, scale: Vector) -> &mut Self {
        self.scale = Some(scale);
        self
    }

    /// Builds a box collision
    pub fn build_box<'world>(
        &self,
        newton: &'world Newton,
        dx: f32,
        dy: f32,
        dz: f32,
    ) -> Collision<'world> {
        unsafe {
            let offset = mem::transmute(self.offset.as_ref());
            let collision = ffi::NewtonCreateBox(newton.as_ptr(), dx, dy, dz, 0, offset);

            if let Some(scale) = self.scale {
                ffi::NewtonCollisionSetScale(collision, scale[0], scale[1], scale[2]);
            }

            Collision {
                newton,
                collision: NonNull::new_unchecked(collision),
                owned: true,
            }
        }
    }
}

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
    pub fn builder() -> Builder {
        Builder {
            offset: None,
            scale: None,
        }
    }

    /// Pass ownership of the NewtonCollision to the borrowed Newton.
    ///
    /// The returned handle can be used to re-borrow the collision later.
    pub fn into_handle(mut self) -> CollisionHandle {
        self.newton.move_collision(self)
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
