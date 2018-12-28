use ffi;

use super::body::{Bodies, BodiesMut, Body, BodyLocked, BodyLockedMut, NewtonBody};
use super::{Error, Lock, Locked, LockedMut, Result, Shared, Types, Weak};
use crate::collision::NewtonCollision;

use std::{
    marker::PhantomData,
    mem,
    num::NonZeroU32,
    ops::{Deref, DerefMut},
    os::raw,
    ptr,
    time::Duration,
};

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
pub struct World<T>(Shared<Lock<NewtonWorld<T>>>);

#[cfg(feature = "sync")]
#[derive(Debug)]
pub struct WorldUpdateAsync<'a, T>(*mut ffi::NewtonWorld, PhantomData<&'a T>);

#[derive(Debug)]
pub struct NewtonWorld<T>(*mut ffi::NewtonWorld, PhantomData<T>);

// TODO normalize userdata
/*
#[derive(Debug)]
pub struct WorldUserData<T> {
    world: Weak<Lock<NewtonWorld<T>>>,
}
*/

#[derive(Debug)]
pub struct WorldLocked<'a, T>(Locked<'a, NewtonWorld<T>>);

#[derive(Debug)]
pub struct WorldLockedMut<'a, T>(LockedMut<'a, NewtonWorld<T>>);

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
            #[cfg(feature = "sync")]
            match threads {
                Threads::One | Threads::Multiple(1) => {}
                Threads::Multiple(n) => ffi::NewtonSetThreadsCount(world, n as _),
            }
            world
        };
        let world_rc_cell = Shared::new(Lock::new(NewtonWorld(world, PhantomData)));
        unsafe {
            ffi::NewtonWorldSetUserData(world, mem::transmute(Shared::downgrade(&world_rc_cell)));
        }
        World(world_rc_cell)
    }

    pub fn try_read(&self) -> Result<WorldLocked<T>> {
        unimplemented!()
    }

    pub fn try_write(&self) -> Result<WorldLocked<T>> {
        unimplemented!()
    }

    pub fn read(&self) -> WorldLocked<T> {
        WorldLocked(self.0.read())
    }

    pub fn write(&self) -> WorldLockedMut<T> {
        WorldLockedMut(self.0.write())
    }
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum Filter {
    Ignore = 0,
    Keep = 1,
}

#[derive(Debug)]
pub struct ConvexCast<'a, T> {
    count: usize,
    contacts: Vec<ffi::NewtonWorldConvexCastReturnInfo>,
    body: *mut NewtonBody<T>,
    _phantom: PhantomData<&'a T>,
}

impl<'a, T> Drop for ConvexCast<'a, T> {
    fn drop(&mut self) {
        unsafe {
            let _ = Box::from_raw(self.body);
        }
    }
}

#[derive(Debug)]
pub struct CastInfo<V> {
    pub point: V,
    pub normal: V,
    pub penetration: f32,
}

// TODO FIXME refactor...
impl<'a, T: Types> Iterator for ConvexCast<'a, T> {
    type Item = (&'a NewtonBody<T>, CastInfo<T::Vector>);

    fn next(&mut self) -> Option<Self::Item> {
        let contact = self.contacts.get(self.count).map(|info| {
            let mut boxed = unsafe { Box::from_raw(self.body) };

            boxed.body = info.m_hitBody as _;

            let (point, normal) = unsafe {
                let mut point: T::Vector = mem::zeroed();
                let mut normal: T::Vector = mem::zeroed();

                ptr::copy(info.m_point.as_ptr(), &mut point as *mut _ as *mut f32, 3);
                ptr::copy(info.m_normal.as_ptr(), &mut normal as *mut _ as *mut f32, 3);

                (point, normal)
            };

            let info = CastInfo {
                penetration: info.m_penetration,
                point,
                normal,
            };

            unsafe { (mem::transmute(Box::into_raw(boxed)), info) }
        });

        self.count += 1;
        contact
    }
}

impl<T: Types> NewtonWorld<T> {
    pub fn convex_cast<C>(
        &self,
        matrix: &T::Matrix,
        target: &T::Vector,
        shape: &NewtonCollision<T>,
        max_contacts: usize,
        prefilter: C,
    ) -> ConvexCast<T>
    where
        C: Fn(&NewtonBody<T>, &NewtonCollision<T>) -> bool + 'static,
    {
        let mut return_info = Vec::with_capacity(max_contacts);

        let userdata = unsafe { userdata::<T>(self.0) };
        let prefilter_world = (prefilter, userdata.clone());
        let contacts = unsafe {
            ffi::NewtonWorldConvexCast(
                self.0,
                mem::transmute(matrix),
                mem::transmute(target),
                shape.as_raw(),
                // hitParam
                &mut 0.0,
                // userdata
                mem::transmute(&prefilter_world),
                Some(prefilter_callback::<T, C>),
                // cast info
                return_info.as_mut_ptr(),
                max_contacts as raw::c_int,
                // thread index
                0,
            )
        };

        let body = Box::new(NewtonBody {
            world: userdata.clone(),
            owned: false,
            body: 0 as _,
            collision: NewtonCollision {
                world: userdata,
                collision: 0 as _,
                owned: false,
            },
        });

        unsafe {
            return_info.set_len(contacts as usize);

            return ConvexCast {
                count: 0,
                contacts: return_info,
                body: Box::into_raw(body),
                _phantom: PhantomData,
            };
        }

        unsafe extern "C" fn prefilter_callback<T, C>(
            body: *const ffi::NewtonBody,
            collision: *const ffi::NewtonCollision,
            udata: *const raw::c_void,
        ) -> raw::c_uint
        where
            T: Types,
            C: Fn(&NewtonBody<T>, &NewtonCollision<T>) -> bool + 'static,
        {
            let (ref callback, ref world): &(C, Shared<Lock<NewtonWorld<T>>>) =
                mem::transmute(udata);

            let body = NewtonBody {
                world: world.clone(),
                body: body as _,
                owned: false,
                collision: NewtonCollision {
                    world: world.clone(),
                    collision: ffi::NewtonBodyGetCollision(body),
                    owned: false,
                },
            };

            let collision = NewtonCollision {
                world: world.clone(),
                collision: collision as _,
                owned: false,
            };

            if callback(&body, &collision) {
                1
            } else {
                0
            }
        }
    }
}

impl<T> NewtonWorld<T> {
    // TODO (performance) extra level of indirection may affect performance
    // TODO FIXME code repetition
    pub fn bodies(&self) -> Bodies<T> {
        unsafe {
            let first = ffi::NewtonWorldGetFirstBody(self.0);
            let world = userdata::<T>(self.0);
            let body = Box::new(NewtonBody::new_not_owned(first));

            Bodies {
                world: self.0,
                next: first,
                body: Box::into_raw(body),
                _phantom: PhantomData,
            }
        }
    }

    // TODO (performance) extra level of indirection may affect performance
    // TODO FIXME code repetition
    pub fn bodies_mut(&mut self) -> BodiesMut<T> {
        unsafe {
            let first = ffi::NewtonWorldGetFirstBody(self.0);
            let world = userdata::<T>(self.0);
            let body = Box::new(NewtonBody::new_not_owned(first));

            BodiesMut {
                world: self.0,
                next: first,
                body: Box::into_raw(body),
                _phantom: PhantomData,
            }
        }
    }

    pub fn body_count(&self) -> raw::c_int {
        unsafe { ffi::NewtonWorldGetBodyCount(self.0) }
    }

    pub fn constraint_count(&self) -> raw::c_int {
        unsafe { ffi::NewtonWorldGetConstraintCount(self.0) }
    }

    #[cfg(feature = "sync")]
    pub fn update_async(&mut self, step: Duration) -> WorldUpdateAsync<T> {
        unsafe {
            ffi::NewtonUpdateAsync(self.0, as_seconds(step));
        }
        WorldUpdateAsync(self.0, PhantomData)
    }

    pub fn update(&mut self, step: Duration) {
        unsafe { ffi::NewtonUpdate(self.0, as_seconds(step)) }
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

pub(crate) unsafe fn userdata<T>(world: *const ffi::NewtonWorld) -> Shared<Lock<NewtonWorld<T>>> {
    let world_userdata: Weak<Lock<NewtonWorld<T>>> =
        mem::transmute(ffi::NewtonWorldGetUserData(world));
    let world = Weak::upgrade(&world_userdata).unwrap();
    mem::forget(world_userdata);
    world
}

impl<T: Types> NewtonWorld<T> {
    pub fn for_each_body_in_aabb<I>(&mut self, (min, max): (&T::Vector, &T::Vector), it: I)
    where
        I: Fn(&mut NewtonBody<T>) -> i32 + 'static,
    {
        unimplemented!()
    }
}

impl<'w, T> Deref for WorldLocked<'w, T> {
    type Target = NewtonWorld<T>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'w, T> Deref for WorldLockedMut<'w, T> {
    type Target = NewtonWorld<T>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'w, T> DerefMut for WorldLockedMut<'w, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

impl<T> Drop for NewtonWorld<T> {
    fn drop(&mut self) {
        let world = self.0;
        unsafe {
            let _: Weak<Lock<Self>> = mem::transmute(ffi::NewtonWorldGetUserData(world));
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

fn as_seconds(step: Duration) -> f32 {
    let nanos = step.as_secs() as f32 * 1_000_000_000.0 + step.subsec_nanos() as f32;
    nanos / 1_000_000_000.0
}
