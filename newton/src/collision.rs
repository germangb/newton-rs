use crate::body;
use crate::body::NewtonBody;
use crate::ffi;
use crate::world::{NewtonWorld, NewtonWorldPtr};
use crate::NewtonConfig;

use std::cell::Cell;
use std::marker::PhantomData;
use std::mem;
use std::ptr;
use std::rc::Rc;

#[derive(Debug, Clone)]
pub enum CollisionParams {
    Cuboid { dx: f32, dy: f32, dz: f32 },
    Sphere { radius: f32 },
    Cone { radius: f32, height: f32 },
}

#[derive(Debug)]
pub struct CollisionInfo {
    params: CollisionParams,
}

#[doc(hidden)]
#[derive(Debug)]
pub struct NewtonCollisionPtr<C>(
    pub(crate) *mut ffi::NewtonCollision,
    // Keep a reference to the world so that ALL collisions are dropped before the world is
    pub(crate) Rc<NewtonWorldPtr<C>>,
    PhantomData<C>,
);

impl<V> Drop for NewtonCollisionPtr<V> {
    fn drop(&mut self) {
        unsafe {
            let _: Box<CollisionInfo> = Box::from_raw(ffi::NewtonCollisionGetUserData(self.0) as _);
            ffi::NewtonDestroyCollision(self.0);
        }
    }
}

#[derive(Debug, Clone)]
pub struct NewtonCollision<V> {
    pub(crate) collision: Rc<NewtonCollisionPtr<V>>,

    pub(crate) raw: *mut ffi::NewtonCollision,
}

fn create_collision<V>(
    world: Rc<NewtonWorldPtr<V>>,
    raw: *mut ffi::NewtonCollision,
    params: CollisionParams,
) -> NewtonCollision<V>
where
    V: NewtonConfig,
{
    assert_config!(V);
    unsafe {
        let info = CollisionInfo { params };

        ffi::NewtonCollisionSetUserData(raw, mem::transmute(Box::new(info)));

        NewtonCollision {
            //world,
            collision: Rc::new(NewtonCollisionPtr(raw, world, PhantomData)),
            raw,
        }
    }
}

impl<V: NewtonConfig> NewtonCollision<V> {
    pub fn params(&self) -> CollisionParams {
        unsafe {
            let info: Box<CollisionInfo> =
                Box::from_raw(ffi::NewtonCollisionGetUserData(self.raw) as _);
            let params = info.params.clone();
            mem::forget(info);
            params
        }
    }

    pub fn cuboid(
        world: &NewtonWorld<V>,
        dx: f32,
        dy: f32,
        dz: f32,
        offset: Option<V::Matrix4>,
    ) -> NewtonCollision<V> {
        unsafe {
            create_collision(
                world.world.clone(),
                ffi::NewtonCreateBox(
                    world.world.0,
                    dx,
                    dy,
                    dz,
                    0,
                    mem::transmute(offset.as_ref()),
                ),
                CollisionParams::Cuboid { dx, dy, dz },
            )
        }
    }

    pub fn sphere(
        world: &NewtonWorld<V>,
        radius: f32,
        offset: Option<V::Matrix4>,
    ) -> NewtonCollision<V> {
        unsafe {
            create_collision(
                world.world.clone(),
                ffi::NewtonCreateSphere(world.world.0, radius, 0, mem::transmute(offset.as_ref())),
                CollisionParams::Sphere { radius },
            )
        }
    }

    pub fn cone(
        world: &NewtonWorld<V>,
        radius: f32,
        height: f32,
        offset: Option<V::Matrix4>,
    ) -> NewtonCollision<V> {
        unsafe {
            create_collision(
                world.world.clone(),
                ffi::NewtonCreateCone(
                    world.world.0,
                    radius,
                    height,
                    0,
                    mem::transmute(offset.as_ref()),
                ),
                CollisionParams::Cone { radius, height },
            )
        }
    }

    pub fn body(&self, matrix: V::Matrix4) -> NewtonBody<V> {
        body::create_dynamic_body(self.collision.1.clone(), self.collision.clone(), matrix)
    }
}
