use ffi;

use super::body::{Body, BodyRef, BodyRefMut, NewtonBody};

use std::cell::RefCell;
use std::cell::{Ref, RefMut};
use std::marker::PhantomData;
use std::mem;
use std::ops::{Deref, DerefMut};
use std::os::raw;
use std::rc::Rc;
use std::rc::Weak;
use std::time::Duration;

#[derive(Debug, Clone)]
pub struct World<T>(Rc<RefCell<NewtonWorld<T>>>, *mut ffi::NewtonWorld);

#[derive(Debug)]
pub struct NewtonWorld<T>(pub(crate) *mut ffi::NewtonWorld, PhantomData<T>);

// TODO normalize userdata
/*
#[derive(Debug)]
pub struct WorldUserData<T> {
    world: Weak<RefCell<NewtonWorld<T>>>,
}
*/

#[derive(Debug)]
pub struct WorldRef<'w, T>(
    pub(crate) Ref<'w, NewtonWorld<T>>,
    pub(crate) *mut ffi::NewtonWorld,
);

#[derive(Debug)]
pub struct WorldRefMut<'w, T>(
    pub(crate) RefMut<'w, NewtonWorld<T>>,
    pub(crate) *mut ffi::NewtonWorld,
);

#[derive(Debug)]
pub struct Bodies<'a, T> {
    world: *mut ffi::NewtonWorld,
    next: *mut ffi::NewtonBody,
    body: *mut NewtonBody<T>,
    _ph: PhantomData<&'a T>,
}

#[derive(Debug)]
pub struct BodiesMut<'a, T> {
    world: *mut ffi::NewtonWorld,
    next: *mut ffi::NewtonBody,
    body: *mut NewtonBody<T>,
    _ph: PhantomData<&'a T>,
}

impl<'a, T> Iterator for Bodies<'a, T> {
    type Item = &'a NewtonBody<T>;

    fn next(&mut self) -> Option<Self::Item> {
        let mut boxed = unsafe { Box::from_raw(self.body) };
        boxed.body = self.next;

        if self.next.is_null() {
            return None;
        } else {
            unsafe {
                self.next = ffi::NewtonWorldGetNextBody(self.world, boxed.body);
                Some(mem::transmute(Box::into_raw(boxed)))
            }
        }
    }
}

impl<'a, T> Iterator for BodiesMut<'a, T> {
    type Item = &'a mut NewtonBody<T>;

    fn next(&mut self) -> Option<Self::Item> {
        let mut boxed = unsafe { Box::from_raw(self.body) };
        boxed.body = self.next;

        if self.next.is_null() {
            return None;
        } else {
            unsafe {
                self.next = ffi::NewtonWorldGetNextBody(self.world, boxed.body);
                Some(mem::transmute(Box::into_raw(boxed)))
            }
        }
    }
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum BroadphaseAlgorithm {
    Default = ffi::NEWTON_BROADPHASE_DEFAULT as _,
    Persistent = ffi::NEWTON_BROADPHASE_PERSINTENT as _,
}

impl<T> World<T> {
    pub fn new(broadphase: BroadphaseAlgorithm) -> Self {
        let world = unsafe {
            let world = ffi::NewtonCreate();
            ffi::NewtonSelectBroadphaseAlgorithm(world, mem::transmute(broadphase));
            world
        };
        let world_rc_cell = Rc::new(RefCell::new(NewtonWorld(world, PhantomData)));
        unsafe {
            ffi::NewtonWorldSetUserData(world, mem::transmute(Rc::downgrade(&world_rc_cell)));
        }
        World(world_rc_cell, world)
    }

    pub fn borrow(&self) -> WorldRef<T> {
        WorldRef(self.0.borrow(), self.1)
    }

    pub fn borrow_mut(&self) -> WorldRefMut<T> {
        WorldRefMut(self.0.borrow_mut(), self.1)
    }
}

impl<T> NewtonWorld<T> {
    // TODO (performance) extra level of indirection may affect performance
    // TODO FIXME code repetition
    pub fn bodies(&self) -> Bodies<T> {
        let first = unsafe { ffi::NewtonWorldGetFirstBody(self.0) };

        let body = Box::new(NewtonBody {
            owned: false,
            body: first,
            world: self.world_datum(),
        });

        Bodies {
            world: self.0,
            next: first,
            body: Box::into_raw(body),
            _ph: PhantomData,
        }
    }

    fn world_datum(&self) -> Rc<RefCell<NewtonWorld<T>>> {
        unsafe {
            let world_userdata: Weak<RefCell<NewtonWorld<T>>> =
                mem::transmute(ffi::NewtonWorldGetUserData(self.0));
            let world = Weak::upgrade(&world_userdata).unwrap();

            mem::forget(world_userdata);
            world
        }
    }

    // TODO (performance) extra level of indirection may affect performance
    // TODO FIXME code repetition
    pub fn bodies_mut(&self) -> BodiesMut<T> {
        let first = unsafe { ffi::NewtonWorldGetFirstBody(self.0) };

        let body = Box::new(NewtonBody {
            owned: false,
            body: first,
            world: self.world_datum(),
        });

        BodiesMut {
            world: self.0,
            next: first,
            body: Box::into_raw(body),
            _ph: PhantomData,
        }
    }

    pub fn body_count(&self) -> raw::c_int {
        unsafe { ffi::NewtonWorldGetBodyCount(self.0) }
    }

    pub fn constraint_count(&self) -> raw::c_int {
        unsafe { ffi::NewtonWorldGetConstraintCount(self.0) }
    }

    pub fn update(&mut self, step: Duration) {
        let nanos = step.as_secs() as f32 * 1_000_000_000.0 + step.subsec_nanos() as f32;
        unsafe {
            ffi::NewtonUpdate(self.0, nanos / 1_000_000_000.0);
        }
    }

    pub fn as_raw(&self) -> *const ffi::NewtonWorld {
        self.0 as *const _
    }

    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonWorld {
        self.0
    }
}

impl<'w, T> Deref for WorldRef<'w, T> {
    type Target = NewtonWorld<T>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'w, T> Deref for WorldRefMut<'w, T> {
    type Target = NewtonWorld<T>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'w, T> DerefMut for WorldRefMut<'w, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

impl<T> Drop for NewtonWorld<T> {
    fn drop(&mut self) {
        let world = self.0;
        unsafe {
            let _: Weak<RefCell<Self>> = mem::transmute(ffi::NewtonWorldGetUserData(world));
            ffi::NewtonDestroy(world);
        }
    }
}

pub fn create<T>() -> World<T> {
    World::new(BroadphaseAlgorithm::Default)
}
