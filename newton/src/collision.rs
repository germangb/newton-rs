use ffi;

use super::world::{NewtonWorld, WorldRefMut};
use super::Types;

use std::cell::{Ref, RefCell, RefMut};
use std::mem;
use std::ops::{Deref, DerefMut};
use std::os::raw;
use std::rc::{Rc, Weak};

pub type ShapeId = raw::c_int;

#[derive(Debug, Clone)]
pub struct Collision<T>(pub(crate) Rc<RefCell<NewtonWorld<T>>>, pub(crate) Rc<RefCell<NewtonCollision<T>>>, *mut ffi::NewtonCollision);

#[derive(Debug)]
pub struct NewtonCollision<T> {
    pub(crate) world: Rc<RefCell<NewtonWorld<T>>>,
    pub(crate) collision: *mut ffi::NewtonCollision,
    pub(crate) world_raw: *mut ffi::NewtonWorld,
}

#[derive(Debug)]
pub(crate) struct CollisionUserDataInner<T> {
    pub(crate) world: Weak<RefCell<NewtonWorld<T>>>,
    pub(crate) collision: Weak<RefCell<NewtonCollision<T>>>,
    pub(crate) params: Rc<CollisionParams>,
}

#[derive(Debug)]
pub struct CollisionRef<'a, T>(pub(crate) Ref<'a, NewtonWorld<T>>, pub(crate) Ref<'a, NewtonCollision<T>>, pub(crate) *const ffi::NewtonWorld, pub(crate) *mut ffi::NewtonCollision);

#[derive(Debug)]
pub struct CollisionRefMut<'a, T>(pub(crate) RefMut<'a, NewtonWorld<T>>, pub(crate) RefMut<'a, NewtonCollision<T>>, pub(crate) *const ffi::NewtonWorld, pub(crate) *mut ffi::NewtonCollision);

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
        let datum: Rc<CollisionUserDataInner<T>> =
            mem::transmute(ffi::NewtonCollisionGetUserData(raw));

        let world = Weak::upgrade(&datum.world).unwrap();
        let collision = Weak::upgrade(&datum.collision).unwrap();
        mem::forget(datum);

        Collision(world, collision, raw)
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

        let params = Rc::new(params);

        let collision = NewtonCollision {
            world: world_rc.clone(),
            collision: collision_raw,
            world_raw: world.0,
        };
        let collision_rc = Rc::new(RefCell::new(collision));

        let userdata = Rc::new(CollisionUserDataInner {
            world: Rc::downgrade(&world_rc),
            collision: Rc::downgrade(&collision_rc),
            params,
        });

        unsafe {
            ffi::NewtonCollisionSetUserData(collision_raw, mem::transmute(userdata));
        }

        Collision(world_rc, collision_rc, collision_raw)
    }

    pub fn borrow(&self) -> CollisionRef<T> {
        let world_ref = self.0.borrow();
        let collision_ref = self.1.borrow();
        let world_ptr = world_ref.as_raw();
        CollisionRef(world_ref, collision_ref, world_ptr, self.2)
    }

    pub fn borrow_mut(&self) -> CollisionRefMut<T> {
        let world_ref = self.0.borrow_mut();
        let collision_ref = self.1.borrow_mut();
        let world_ptr = world_ref.as_raw();
        CollisionRefMut(world_ref, collision_ref, world_ptr, self.2)
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
        self.1.deref()
    }
}

impl<'a, T> Deref for CollisionRefMut<'a, T> {
    type Target = NewtonCollision<T>;

    fn deref(&self) -> &Self::Target {
        self.1.deref()
    }
}

impl<'a, T> DerefMut for CollisionRefMut<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.1.deref_mut()
    }
}

impl<T> Drop for NewtonCollision<T> {
    fn drop(&mut self) {
        let collision = self.collision;
        //let _ = self.world.borrow_mut();
        unsafe {
            let _: Rc<CollisionUserDataInner<T>> =
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