use ffi;

use super::world::{NewtonWorld, WorldRefMut};
use super::Types;
use super::{Shared, Weak};

use std::cell::{Ref, RefCell, RefMut};
use std::mem;
use std::ops::{Deref, DerefMut};
use std::os::raw;

pub type ShapeId = raw::c_int;

#[derive(Debug, Clone)]
pub struct Collision<T>(
    pub(crate) Shared<RefCell<NewtonCollision<T>>>,
    *mut ffi::NewtonCollision,
);

#[derive(Debug)]
pub struct NewtonCollision<T> {
    /// NewtonCollision holds a reference to the context because all collisions MUST be freed
    /// before the NewtonWorld is
    pub(crate) world: Shared<RefCell<NewtonWorld<T>>>,
    pub(crate) collision: *mut ffi::NewtonCollision,
}

#[derive(Debug)]
pub(crate) struct CollisionUserDataInner<T> {
    pub(crate) collision: Weak<RefCell<NewtonCollision<T>>>,
    pub(crate) params: Shared<CollisionParams>,
}

#[derive(Debug)]
pub struct CollisionRef<'a, T>(
    pub(crate) Ref<'a, NewtonCollision<T>>,
    pub(crate) *mut ffi::NewtonCollision,
);

#[derive(Debug)]
pub struct CollisionRefMut<'a, T>(
    pub(crate) RefMut<'a, NewtonCollision<T>>,
    pub(crate) *mut ffi::NewtonCollision,
);

#[derive(Debug)]
pub enum CollisionParams {
    Box {
        dx: f32,
        dy: f32,
        dz: f32,
    },
    Sphere {
        radius: f32,
    },
    Cone {
        radius: f32,
        height: f32,
    },
    Cylinder {
        radius0: f32,
        radius1: f32,
        height: f32,
    },
    Capsule {
        radius0: f32,
        radius1: f32,
        height: f32,
    },
    Null,
}

impl<T> Collision<T> {
    pub unsafe fn from_raw(raw: *mut ffi::NewtonCollision) -> Self {
        let datum: Shared<CollisionUserDataInner<T>> =
            mem::transmute(ffi::NewtonCollisionGetUserData(raw));

        let collision = Weak::upgrade(&datum.collision).unwrap();
        mem::forget(datum);

        Collision(collision, raw)
    }
}

impl<T: Types> Collision<T> {
    // TODO FIXME indirections
    pub fn new(
        world: &mut NewtonWorld<T>,
        params: CollisionParams,
        shape_id: ShapeId,
        offset: Option<&T::Matrix>,
    ) -> Self {
        let world_rc = unsafe {
            let world_ref: Weak<RefCell<NewtonWorld<T>>> =
                mem::transmute(ffi::NewtonWorldGetUserData(world.as_raw()));
            let world_ref_rc = Weak::upgrade(&world_ref).unwrap();
            mem::forget(world_ref);
            world_ref_rc
        };

        let collision_raw = unsafe {
            let offset = mem::transmute(offset);
            match &params {
                &CollisionParams::Box { dx, dy, dz } => {
                    ffi::NewtonCreateBox(world.as_raw(), dx, dy, dz, shape_id, offset)
                }
                &CollisionParams::Sphere { radius } => {
                    ffi::NewtonCreateSphere(world.as_raw(), radius, shape_id, offset)
                }
                &CollisionParams::Cone { radius, height } => {
                    ffi::NewtonCreateCone(world.as_raw(), radius, height, shape_id, offset)
                }
                &CollisionParams::Cylinder {
                    radius0,
                    radius1,
                    height,
                } => ffi::NewtonCreateCylinder(world.0, radius0, radius1, height, shape_id, offset),
                &CollisionParams::Capsule {
                    radius0,
                    radius1,
                    height,
                } => ffi::NewtonCreateCapsule(world.0, radius0, radius1, height, shape_id, offset),
                &CollisionParams::Null => ffi::NewtonCreateNull(world.0),
            }
        };

        let params = Shared::new(params);

        let collision = NewtonCollision {
            world: world_rc.clone(),
            collision: collision_raw,
        };
        let collision_rc = Shared::new(RefCell::new(collision));

        let userdata = Shared::new(CollisionUserDataInner {
            collision: Shared::downgrade(&collision_rc),
            params,
        });

        unsafe {
            ffi::NewtonCollisionSetUserData(collision_raw, mem::transmute(userdata));
        }

        Collision(collision_rc, collision_raw)
    }

    pub fn borrow(&self) -> CollisionRef<T> {
        let collision_ref = self.0.borrow();
        CollisionRef(collision_ref, self.1)
    }

    pub fn borrow_mut(&self) -> CollisionRefMut<T> {
        let collision_ref = self.0.borrow_mut();
        CollisionRefMut(collision_ref, self.1)
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

impl<'a, T> Deref for CollisionRef<'a, T> {
    type Target = NewtonCollision<T>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'a, T> Deref for CollisionRefMut<'a, T> {
    type Target = NewtonCollision<T>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'a, T> DerefMut for CollisionRefMut<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

impl<T> Drop for NewtonCollision<T> {
    fn drop(&mut self) {
        let collision = self.collision;
        //let _ = self.world.borrow_mut();
        unsafe {
            let _: Shared<CollisionUserDataInner<T>> =
                mem::transmute(ffi::NewtonCollisionGetUserData(collision));
            ffi::NewtonDestroyCollision(collision)
        }
    }
}

pub fn cuboid<T: Types>(
    world: &mut NewtonWorld<T>,
    dx: f32,
    dy: f32,
    dz: f32,
    shape_id: ShapeId,
    offset: Option<&T::Matrix>,
) -> Collision<T> {
    let params = CollisionParams::Box { dx, dy, dz };
    Collision::new(world, params, shape_id, offset)
}

pub fn sphere<T: Types>(
    world: &mut NewtonWorld<T>,
    radius: f32,
    shape_id: ShapeId,
    offset: Option<&T::Matrix>,
) -> Collision<T> {
    let params = CollisionParams::Sphere { radius };
    Collision::new(world, params, shape_id, offset)
}

pub fn cone<T: Types>(
    world: &mut NewtonWorld<T>,
    radius: f32,
    height: f32,
    shape_id: ShapeId,
    offset: Option<&T::Matrix>,
) -> Collision<T> {
    let params = CollisionParams::Cone { radius, height };
    Collision::new(world, params, shape_id, offset)
}

pub fn cylinder<T: Types>(
    world: &mut NewtonWorld<T>,
    radius0: f32,
    radius1: f32,
    height: f32,
    shape_id: ShapeId,
    offset: Option<&T::Matrix>,
) -> Collision<T> {
    let params = CollisionParams::Cylinder {
        radius0,
        radius1,
        height,
    };
    Collision::new(world, params, shape_id, offset)
}

pub fn capsule<T: Types>(
    world: &mut NewtonWorld<T>,
    radius0: f32,
    radius1: f32,
    height: f32,
    shape_id: ShapeId,
    offset: Option<&T::Matrix>,
) -> Collision<T> {
    let params = CollisionParams::Capsule {
        radius0,
        radius1,
        height,
    };
    Collision::new(world, params, shape_id, offset)
}

pub fn null<T: Types>(world: &mut NewtonWorld<T>) -> Collision<T> {
    let params = CollisionParams::Null;
    Collision::new(world, params, 0, None)
}
