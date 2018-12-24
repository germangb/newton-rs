use ffi;

use super::Application;

use std::mem;
use std::time::Duration;
use std::cell::RefCell;
use std::rc::Rc;
use std::ops::{Deref, DerefMut};
use std::cell::{Ref, RefMut};
use std::os::raw;
use std::rc::Weak;
use std::marker::PhantomData;

/// Ref counted pointer to a `NewtonWorld`.
#[derive(Debug)]
pub struct World<App>(Rc<RefCell<NewtonWorld<App>>>, *mut ffi::NewtonWorld);

#[doc(hidden)]
#[derive(Debug)]
pub struct NewtonWorld<App>(*mut ffi::NewtonWorld, PhantomData<App>);

/// Immutable reference to a `NewtonBody`
#[derive(Debug)]
pub struct WorldRef<'w, App>(pub(crate) Ref<'w, NewtonWorld<App>>, pub(crate) *mut ffi::NewtonWorld);

/// Mutable reference to a `NewtonBody`
#[derive(Debug)]
pub struct WorldRefMut<'w, App>(pub(crate) RefMut<'w, NewtonWorld<App>>, pub(crate) *mut ffi::NewtonWorld);

impl<App> World<App> {
    pub fn new() -> Self {
        let world = unsafe { ffi::NewtonCreate() };
        let world_rc_cell = Rc::new(RefCell::new(NewtonWorld(world, PhantomData)));
        unsafe {
            ffi::NewtonWorldSetUserData(world, mem::transmute(Rc::downgrade(&world_rc_cell)));
        }
        World(world_rc_cell, world)
    }

    pub fn borrow(&self) -> WorldRef<App> {
        WorldRef(self.0.borrow(), self.1)
    }

    pub fn borrow_mut(&self) -> WorldRefMut<App> {
        WorldRefMut(self.0.borrow_mut(), self.1)
    }
}

impl<App> NewtonWorld<App> {
    pub fn body_count(&self) -> raw::c_int {
        unsafe {
            ffi::NewtonWorldGetBodyCount(self.0)
        }
    }

    pub fn constraint_count(&self) -> raw::c_int {
        unsafe {
            ffi::NewtonWorldGetConstraintCount(self.0)
        }
    }

    pub fn update(&mut self, step: Duration) {
        let nanos = step.as_secs() as f32 * 1_000_000_000.0 + step.subsec_nanos() as f32;
        unsafe {
            ffi::NewtonUpdate(self.0, nanos);
        }
    }

    pub fn as_raw(&self) -> *const ffi::NewtonWorld {
        self.0 as *const _
    }

    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonWorld {
        self.0
    }
}

impl<'w, App> Deref for WorldRef<'w, App> {
    type Target = NewtonWorld<App>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'w, App> Deref for WorldRefMut<'w, App> {
    type Target = NewtonWorld<App>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'w, App> DerefMut for WorldRefMut<'w, App> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

impl<App> Drop for NewtonWorld<App> {
    fn drop(&mut self) {
        let world = self.0;
        unsafe {
            eprintln!("drop world");
            let _: Weak<RefCell<Self>> = mem::transmute(ffi::NewtonWorldGetUserData(world));
            ffi::NewtonDestroy(world);
        }
    }
}
