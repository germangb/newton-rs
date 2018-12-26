use ffi;

use super::body::{Bodies, BodiesMut, Body, BodyRef, BodyRefMut, NewtonBody};
use super::{Shared, Types, Weak};

use crate::collision::NewtonCollision;
use std::cell::RefCell;
use std::cell::{Ref, RefMut};
use std::marker::PhantomData;
use std::mem;
use std::num::NonZeroU32;
use std::ops::{Deref, DerefMut};
use std::os::raw;
use std::time::Duration;

#[repr(i32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum Broadphase {
    Default = ffi::NEWTON_BROADPHASE_DEFAULT as _,
    Persistent = ffi::NEWTON_BROADPHASE_PERSINTENT as _,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum Solver {
    Exact,
    Linear(u32),
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum Threads {
    One,
    Multiple(u32),
}

#[derive(Debug, Clone)]
pub struct World<T>(Shared<RefCell<NewtonWorld<T>>>, *mut ffi::NewtonWorld);

#[derive(Debug)]
pub struct WorldUpdateAsync<'a, T>(*mut ffi::NewtonWorld, PhantomData<&'a T>);

#[derive(Debug)]
pub struct NewtonWorld<T>(*mut ffi::NewtonWorld, PhantomData<T>);

// TODO normalize userdata
/*
#[derive(Debug)]
pub struct WorldUserData<T> {
    world: Weak<RefCell<NewtonWorld<T>>>,
}
*/

#[derive(Debug)]
pub struct WorldRef<'a, T>(Ref<'a, NewtonWorld<T>>);

#[derive(Debug)]
pub struct WorldRefMut<'a, T>(RefMut<'a, NewtonWorld<T>>);

impl<T> World<T> {
    pub fn new(broadphase: Broadphase, solver: Solver, threads: Threads) -> Self {
        let world = unsafe {
            let world = ffi::NewtonCreate();
            ffi::NewtonSelectBroadphaseAlgorithm(world, mem::transmute(broadphase));
            match solver {
                Solver::Exact => ffi::NewtonSetSolverModel(world, 0),
                Solver::Linear(n) => {
                    assert!(n > 0);
                    ffi::NewtonSetSolverModel(world, n as _)
                }
            }
            match threads {
                Threads::One | Threads::Multiple(1) => {}
                Threads::Multiple(n) => ffi::NewtonSetThreadsCount(world, n as _),
            }
            world
        };
        let world_rc_cell = Shared::new(RefCell::new(NewtonWorld(world, PhantomData)));
        unsafe {
            ffi::NewtonWorldSetUserData(world, mem::transmute(Shared::downgrade(&world_rc_cell)));
        }
        World(world_rc_cell, world)
    }

    pub fn borrow(&self) -> WorldRef<T> {
        WorldRef(self.0.borrow())
    }

    pub fn borrow_mut(&self) -> WorldRefMut<T> {
        WorldRefMut(self.0.borrow_mut())
    }
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum Filter {
    Ignore = 0,
    Keep = 1,
}

impl<T: Types> NewtonWorld<T> {
    pub fn convex_cast<C>(
        &self,
        matrix: &T::Matrix,
        target: &T::Vector,
        shape: &NewtonCollision<T>,
        max_contacts: raw::c_int,
        prefilter: C,
    ) where
        C: Fn(&NewtonBody<T>, &NewtonCollision<T>) -> Filter,
    {
        unimplemented!()
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

    // TODO (performance) extra level of indirection may affect performance
    // TODO FIXME code repetition
    pub fn bodies_mut(&mut self) -> BodiesMut<T> {
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

    fn world_datum(&self) -> Shared<RefCell<NewtonWorld<T>>> {
        unsafe {
            let world_userdata: Weak<RefCell<NewtonWorld<T>>> =
                mem::transmute(ffi::NewtonWorldGetUserData(self.0));
            let world = Weak::upgrade(&world_userdata).unwrap();

            mem::forget(world_userdata);
            world
        }
    }

    pub fn body_count(&self) -> raw::c_int {
        unsafe { ffi::NewtonWorldGetBodyCount(self.0) }
    }

    pub fn constraint_count(&self) -> raw::c_int {
        unsafe { ffi::NewtonWorldGetConstraintCount(self.0) }
    }

    pub fn update_async(&mut self, step: Duration) -> WorldUpdateAsync<T> {
        let world = self.0;
        let nanos = step.as_secs() as f32 * 1_000_000_000.0 + step.subsec_nanos() as f32;
        unsafe {
            ffi::NewtonUpdateAsync(world, nanos / 1_000_000_000.0);
        }
        WorldUpdateAsync(world, PhantomData)
    }

    pub fn update(&mut self, step: Duration) {
        let nanos = step.as_secs() as f32 * 1_000_000_000.0 + step.subsec_nanos() as f32;
        unsafe {
            ffi::NewtonUpdate(self.0, nanos / 1_000_000_000.0);
        }
    }

    pub fn invalidate_cache(&mut self) {
        unsafe { ffi::NewtonInvalidateCache(self.0) }
    }

    pub fn as_raw(&self) -> *const ffi::NewtonWorld {
        self.0 as *const _
    }

    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonWorld {
        self.0
    }
}

impl<T: Types> NewtonWorld<T> {
    pub fn for_each_body_in_aabb<I>(&mut self, (min, max): (&T::Vector, &T::Vector), it: I)
    where
        I: Fn(&mut NewtonBody<T>) -> Result<(), ()> + 'static,
    {
        unimplemented!()
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
            ffi::NewtonWaitForUpdateToFinish(world);
            ffi::NewtonMaterialDestroyAllGroupID(world);
            ffi::NewtonDestroy(world);
        }
    }
}

impl<'a, T> Drop for WorldUpdateAsync<'a, T> {
    fn drop(&mut self) {
        unsafe {
            ffi::NewtonWaitForUpdateToFinish(self.0);
        }
    }
}

pub fn create<T>() -> World<T> {
    World::new(Broadphase::Default, Solver::Exact, Threads::One)
}
