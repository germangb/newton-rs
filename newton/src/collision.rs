use ffi;

use super::world::{NewtonWorld, WorldRefMut};
use super::{Application, Types};

use std::cell::{Ref, RefCell, RefMut};
use std::mem;
use std::ops::{Deref, DerefMut};
use std::os::raw;
use std::rc::{Rc, Weak};

pub type ShapeId = raw::c_int;

#[derive(Debug, Clone)]
pub struct Collision<App>(
    pub(crate) Rc<RefCell<NewtonWorld<App>>>,
    pub(crate) Rc<RefCell<NewtonCollision<App>>>,
    *mut ffi::NewtonCollision,
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

pub fn cuboid<T: Types, App: Application<Types = T>>(
    world: &mut NewtonWorld<App>,
    dx: f32,
    dy: f32,
    dz: f32,
    shape_id: ShapeId,
    offset: Option<&T::Matrix>,
) -> Collision<App> {
    let params = CollisionParams::Box { dx, dy, dz };
    Collision::new(world, params, shape_id, offset)
}

pub fn sphere<T: Types, App: Application<Types = T>>(
    world: &mut NewtonWorld<App>,
    radius: f32,
    shape_id: ShapeId,
    offset: Option<&T::Matrix>,
) -> Collision<App> {
    let params = CollisionParams::Sphere { radius };
    Collision::new(world, params, shape_id, offset)
}

pub fn cone<T: Types, App: Application<Types = T>>(
    world: &mut NewtonWorld<App>,
    radius: f32,
    height: f32,
    shape_id: ShapeId,
    offset: Option<&T::Matrix>,
) -> Collision<App> {
    let params = CollisionParams::Cone { radius, height };
    Collision::new(world, params, shape_id, offset)
}

pub fn cylinder<T: Types, App: Application<Types = T>>(
    world: &mut NewtonWorld<App>,
    radius0: f32,
    radius1: f32,
    height: f32,
    shape_id: ShapeId,
    offset: Option<&T::Matrix>,
) -> Collision<App> {
    let params = CollisionParams::Cylinder {
        radius0,
        radius1,
        height,
    };
    Collision::new(world, params, shape_id, offset)
}

pub fn capsule<T: Types, App: Application<Types = T>>(
    world: &mut NewtonWorld<App>,
    radius0: f32,
    radius1: f32,
    height: f32,
    shape_id: ShapeId,
    offset: Option<&T::Matrix>,
) -> Collision<App> {
    let params = CollisionParams::Capsule {
        radius0,
        radius1,
        height,
    };
    Collision::new(world, params, shape_id, offset)
}

pub fn null<T: Types, App: Application<Types = T>>(world: &mut NewtonWorld<App>) -> Collision<App> {
    let params = CollisionParams::Null;
    Collision::new(world, params, 0, None)
}

impl<App> Collision<App> {
    pub unsafe fn from_raw(raw: *mut ffi::NewtonCollision) -> Self {
        let datum: Rc<CollisionUserDataInner<App>> =
            mem::transmute(ffi::NewtonCollisionGetUserData(raw));

        let world = Weak::upgrade(&datum.world).unwrap();
        let collision = Weak::upgrade(&datum.collision).unwrap();
        mem::forget(datum);

        Collision(world, collision, raw)
    }
}

impl<F: Types, App: Application<Types = F>> Collision<App> {
    // TODO FIXME indirections
    pub fn new(
        world: &mut NewtonWorld<App>,
        params: CollisionParams,
        shape_id: ShapeId,
        offset: Option<&F::Matrix>,
    ) -> Self {
        let world_rc = unsafe {
            let world_ref: Weak<RefCell<NewtonWorld<App>>> =
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

    pub fn borrow(&self) -> CollisionRef<App> {
        let world_ref = self.0.borrow();
        let collision_ref = self.1.borrow();
        let world_ptr = world_ref.as_raw();
        CollisionRef(world_ref, collision_ref, world_ptr, self.2)
    }

    pub fn borrow_mut(&self) -> CollisionRefMut<App> {
        let world_ref = self.0.borrow_mut();
        let collision_ref = self.1.borrow_mut();
        let world_ptr = world_ref.as_raw();
        CollisionRefMut(world_ref, collision_ref, world_ptr, self.2)
    }
}

#[derive(Debug)]
pub struct NewtonCollision<App> {
    pub(crate) world: Rc<RefCell<NewtonWorld<App>>>,
    pub(crate) collision: *mut ffi::NewtonCollision,
    pub(crate) world_raw: *mut ffi::NewtonWorld,
}

pub(crate) struct CollisionUserDataInner<App> {
    pub(crate) world: Weak<RefCell<NewtonWorld<App>>>,
    pub(crate) collision: Weak<RefCell<NewtonCollision<App>>>,
    pub(crate) params: Rc<CollisionParams>,
}

#[derive(Debug)]
pub struct CollisionRef<'w, 'c, App>(
    pub(crate) Ref<'w, NewtonWorld<App>>,
    pub(crate) Ref<'c, NewtonCollision<App>>,
    pub(crate) *const ffi::NewtonWorld,
    pub(crate) *mut ffi::NewtonCollision,
);

#[derive(Debug)]
pub struct CollisionRefMut<'w, 'c, App>(
    pub(crate) RefMut<'w, NewtonWorld<App>>,
    pub(crate) RefMut<'c, NewtonCollision<App>>,
    pub(crate) *const ffi::NewtonWorld,
    pub(crate) *mut ffi::NewtonCollision,
);

impl<App> NewtonCollision<App> {
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

impl<'w, 'c, App> Deref for CollisionRef<'w, 'c, App> {
    type Target = NewtonCollision<App>;

    fn deref(&self) -> &Self::Target {
        self.1.deref()
    }
}

impl<'w, 'c, App> Deref for CollisionRefMut<'w, 'c, App> {
    type Target = NewtonCollision<App>;

    fn deref(&self) -> &Self::Target {
        self.1.deref()
    }
}

impl<'w, 'c, App> DerefMut for CollisionRefMut<'w, 'c, App> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.1.deref_mut()
    }
}

impl<App> Drop for NewtonCollision<App> {
    fn drop(&mut self) {
        let collision = self.collision;
        //let _ = self.world.borrow_mut();
        unsafe {
            let _: Rc<CollisionUserDataInner<App>> =
                mem::transmute(ffi::NewtonCollisionGetUserData(collision));
            ffi::NewtonDestroyCollision(collision)
        }
    }
}
