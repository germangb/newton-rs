use std::marker::PhantomData;

use crate::collision::NewtonCollision;
use crate::ffi;
use crate::math::Mat4;
use crate::newton::Newton;

/// Newton mesh wrapper.
///
/// Meshes can be used to collect geometry information (vertex & indices) from
/// Collisions in order to be displayed by the application rendering engine.
pub struct Mesh<'a> {
    raw: *const ffi::NewtonMesh,
    owned: bool,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> Drop for Mesh<'a> {
    fn drop(&mut self) {
        unsafe { ffi::NewtonMeshDestroy(self.raw) }
    }
}

impl<'a> Mesh<'a> {
    pub fn create(newton: &'a Newton) -> Self {
        unsafe {
            let raw = ffi::NewtonMeshCreate(newton.as_raw());
            Self::from_raw(raw, true)
        }
    }

    pub fn from_collision<C>(_: &'a Newton, col: &C) -> Self
        where C: NewtonCollision
    {
        unsafe {
            let raw = ffi::NewtonMeshCreateFromCollision(col.as_raw());
            Self::from_raw(raw, true)
        }
    }

    pub unsafe fn from_raw(raw: *const ffi::NewtonMesh, owned: bool) -> Self {
        Self { raw, owned, _phantom: PhantomData }
    }

    pub fn as_raw(&self) -> *const ffi::NewtonMesh {
        self.raw
    }

    pub fn into_raw(self) -> *const ffi::NewtonMesh {
        self.raw
    }
}
