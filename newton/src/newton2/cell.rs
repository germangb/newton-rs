use ffi;

use super::world::NewtonWorld;

use std::rc::Rc;
use std::cell::{RefCell, Ref, RefMut};
use std::ops::{Deref, DerefMut};

#[derive(Debug)]
pub struct WorldRef<'world>(pub(crate) Ref<'world, NewtonWorld>, pub(crate) *mut ffi::NewtonWorld);

#[derive(Debug)]
pub struct WorldRefMut<'world>(pub(crate) RefMut<'world, NewtonWorld>, pub(crate) *mut ffi::NewtonWorld);

impl<'world> Deref for WorldRef<'world> {
    type Target = NewtonWorld;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'world> Deref for WorldRefMut<'world> {
    type Target = NewtonWorld;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'world> DerefMut for WorldRefMut<'world> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

