use ffi;

use crate::collision::Collision;
use crate::NewtonApp;

use std::marker::PhantomData;

#[derive(Debug)]
pub struct Mesh<C> {
    raw: *mut ffi::NewtonMesh,
    _phantom: PhantomData<C>,
}

impl<C: NewtonApp> Mesh<C> {
    pub fn from(collision: &Collision<C>) -> Self {
        unimplemented!()
    }
}

impl<C> Drop for Mesh<C> {
    fn drop(&mut self) {
        unsafe { ffi::NewtonMeshDestroy(self.raw) }
    }
}
