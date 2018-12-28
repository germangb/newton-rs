use ffi;

use super::world::{NewtonWorld, WorldLockedMut};
use super::{Lock, Locked, LockedMut, Result, Shared, Types, Weak};

use std::{
    mem,
    ops::{Deref, DerefMut},
    os::raw,
};

#[derive(Debug, Clone)]
pub struct Collision<T>(Shared<Lock<NewtonCollision<T>>>);

#[derive(Debug)]
pub struct NewtonCollision<T> {
    pub(crate) world: Shared<Lock<NewtonWorld<T>>>,
    pub(crate) collision: *mut ffi::NewtonCollision,
    pub(crate) owned: bool,
}

#[derive(Debug)]
pub struct CollisionLocked<'a, T>(Locked<'a, NewtonCollision<T>>);

#[derive(Debug)]
pub struct CollisionLockedMut<'a, T>(LockedMut<'a, NewtonCollision<T>>);

#[derive(Debug)]
pub(crate) struct CollisionUserDataInner<T> {
    pub(crate) world: Weak<Lock<NewtonWorld<T>>>,
    pub(crate) collision: Weak<Lock<NewtonCollision<T>>>,
    pub(crate) params: Shared<CollisionParams>,
}

pub(crate) unsafe fn userdata<T>(
    collision: *const ffi::NewtonCollision,
) -> Shared<CollisionUserDataInner<T>> {
    let udata: Shared<CollisionUserDataInner<T>> =
        mem::transmute(ffi::NewtonCollisionGetUserData(collision));
    let udata_cloned = udata.clone();
    mem::forget(udata);
    udata_cloned
}

#[derive(Debug)]
pub enum CollisionParams {
    Box(f32, f32, f32),
    Sphere(f32),
    Cone(f32, f32),
    Cylinder(f32, f32, f32),
    Capsule(f32, f32, f32),
    Null,
}

impl<T> Collision<T> {
    pub unsafe fn from_raw(raw: *mut ffi::NewtonCollision) -> Self {
        let collision = Weak::upgrade(&userdata(raw).collision).unwrap();
        Collision(collision)
    }
}

impl<T: Types> Collision<T> {
    // TODO FIXME indirections
    pub fn new(world: &mut NewtonWorld<T>, params: CollisionParams, shape_id: raw::c_int) -> Self {
        let world_rc = unsafe {
            let world_ref: Weak<Lock<NewtonWorld<T>>> =
                mem::transmute(ffi::NewtonWorldGetUserData(world.as_raw()));
            let world_ref_rc = Weak::upgrade(&world_ref).unwrap();
            mem::forget(world_ref);
            world_ref_rc
        };

        let world = world.as_raw();

        let collision_raw = unsafe {
            let offset = std::ptr::null();
            match &params {
                &CollisionParams::Box(dx, dy, dz) => {
                    ffi::NewtonCreateBox(world, dx, dy, dz, shape_id, offset)
                }
                &CollisionParams::Sphere(radius) => {
                    ffi::NewtonCreateSphere(world, radius, shape_id, offset)
                }
                &CollisionParams::Cone(radius, height) => {
                    ffi::NewtonCreateCone(world, radius, height, shape_id, offset)
                }
                &CollisionParams::Cylinder(radius0, radius1, height) => {
                    ffi::NewtonCreateCylinder(world, radius0, radius1, height, shape_id, offset)
                }
                &CollisionParams::Capsule(radius0, radius1, height) => {
                    ffi::NewtonCreateCapsule(world, radius0, radius1, height, shape_id, offset)
                }
                &CollisionParams::Null => ffi::NewtonCreateNull(world),
            }
        };

        let params = Shared::new(params);

        let collision = NewtonCollision {
            world: world_rc.clone(),
            collision: collision_raw,
            owned: true,
        };
        let collision_rc = Shared::new(Lock::new(collision));

        let userdata = Shared::new(CollisionUserDataInner {
            collision: Shared::downgrade(&collision_rc),
            world: Shared::downgrade(&world_rc),
            params,
        });

        unsafe {
            ffi::NewtonCollisionSetUserData(collision_raw, mem::transmute(userdata));
        }

        Collision(collision_rc)
    }

    pub fn try_read(&self) -> Result<CollisionLocked<T>> {
        unimplemented!()
    }

    pub fn try_write(&self) -> Result<CollisionLockedMut<T>> {
        unimplemented!()
    }

    pub fn read(&self) -> CollisionLocked<T> {
        let collision_ref = self.0.read();
        CollisionLocked(collision_ref)
    }

    pub fn write(&self) -> CollisionLockedMut<T> {
        let collision_ref = self.0.write();
        CollisionLockedMut(collision_ref)
    }
}

impl<T> NewtonCollision<T> {
    pub fn params(&self) -> &CollisionParams {
        unimplemented!()
    }
    pub fn as_raw(&self) -> *const ffi::NewtonCollision {
        self.collision as *const _
    }

    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonCollision {
        self.collision
    }
}

impl<'a, T> Deref for CollisionLocked<'a, T> {
    type Target = NewtonCollision<T>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'a, T> Deref for CollisionLockedMut<'a, T> {
    type Target = NewtonCollision<T>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'a, T> DerefMut for CollisionLockedMut<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

// TODO review
impl<T> Drop for NewtonCollision<T> {
    fn drop(&mut self) {
        if self.owned {
            let collision = self.collision;
            //let _ = self.world.write();
            unsafe {
                let _: Shared<CollisionUserDataInner<T>> =
                    mem::transmute(ffi::NewtonCollisionGetUserData(collision));
                ffi::NewtonDestroyCollision(collision)
            }
        }
    }
}

pub fn cuboid<T: Types>(
    world: &mut NewtonWorld<T>,
    dx: f32,
    dy: f32,
    dz: f32,
    shape_id: raw::c_int,
) -> Collision<T> {
    let params = CollisionParams::Box(dx, dy, dz);
    Collision::new(world, params, shape_id)
}

pub fn sphere<T: Types>(
    world: &mut NewtonWorld<T>,
    radius: f32,
    shape_id: raw::c_int,
) -> Collision<T> {
    let params = CollisionParams::Sphere(radius);
    Collision::new(world, params, shape_id)
}

pub fn cone<T: Types>(
    world: &mut NewtonWorld<T>,
    radius: f32,
    height: f32,
    shape_id: raw::c_int,
) -> Collision<T> {
    let params = CollisionParams::Cone(radius, height);
    Collision::new(world, params, shape_id)
}

pub fn cylinder<T: Types>(
    world: &mut NewtonWorld<T>,
    radius0: f32,
    radius1: f32,
    height: f32,
    shape_id: raw::c_int,
) -> Collision<T> {
    let params = CollisionParams::Cylinder(radius0, radius1, height);
    Collision::new(world, params, shape_id)
}

pub fn capsule<T: Types>(
    world: &mut NewtonWorld<T>,
    radius0: f32,
    radius1: f32,
    height: f32,
    shape_id: raw::c_int,
) -> Collision<T> {
    let params = CollisionParams::Capsule(radius0, radius1, height);
    Collision::new(world, params, shape_id)
}

pub fn null<T: Types>(world: &mut NewtonWorld<T>) -> Collision<T> {
    let params = CollisionParams::Null;
    Collision::new(world, params, 0)
}
