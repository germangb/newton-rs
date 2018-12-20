use crate::body;
use crate::body::{NewtonBody, NewtonBodyInner};
use crate::ffi;
use crate::world::{NewtonWorld, WorldRef};
use crate::NewtonConfig;

use std::cell::Cell;
use std::marker::PhantomData;
use std::mem;
use std::ptr;
use std::rc::Rc;

#[derive(Debug)]
pub enum CollisionParams {
    Cuboid {
        dx: f32,
        dy: f32,
        dz: f32,
    },
    Sphere {
        radius: f32,
    },
    Capsule {
        radius0: f32,
        radius1: f32,
        height: f32,
    },
}

#[derive(Debug)]
pub struct CollisionInfo {
    params: CollisionParams,
    scale: (f32, f32, f32),
}

#[derive(Debug)]
pub struct NewtonCollision<V> {
    world: Rc<WorldRef>,
    pub(crate) collision: *mut ffi::NewtonCollision,

    info: Rc<CollisionInfo>,

    _ph: PhantomData<V>,
}

impl<V> Drop for NewtonCollision<V> {
    fn drop(&mut self) {
        unsafe { ffi::NewtonDestroyCollision(self.collision) }
    }
}

fn create_collision<V>(
    world: &NewtonWorld<V>,
    collision: *mut ffi::NewtonCollision,
    params: CollisionParams,
) -> NewtonCollision<V>
where
    V: NewtonConfig,
{
    assert_config!(V);
    unsafe {
        let info = Rc::new(CollisionInfo {
            scale: (1.0, 1.0, 1.0),
            params,
        });

        //ffi::NewtonCollisionSetUserData(mem::transmute(Rc::downgrade(&info)));

        NewtonCollision {
            world: Rc::clone(&world.world),
            collision,
            info,
            _ph: PhantomData,
        }
    }
}

impl<V: NewtonConfig> NewtonCollision<V> {
    pub fn cuboid(
        world: &NewtonWorld<V>,
        dx: f32,
        dy: f32,
        dz: f32,
        offset: Option<V::Matrix4>,
    ) -> NewtonCollision<V> {
        unsafe {
            create_collision(
                &world,
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
                &world,
                ffi::NewtonCreateSphere(world.world.0, radius, 0, mem::transmute(offset.as_ref())),
                CollisionParams::Sphere { radius },
            )
        }
    }

    pub fn capsule(
        world: &NewtonWorld<V>,
        radius0: f32,
        radius1: f32,
        height: f32,
        offset: Option<V::Matrix4>,
    ) -> NewtonCollision<V> {
        assert_config!(V);
        unsafe {
            create_collision(
                &world,
                ffi::NewtonCreateCapsule(
                    world.world.0,
                    radius0,
                    radius1,
                    height,
                    0,
                    mem::transmute(offset.as_ref()),
                ),
                CollisionParams::Capsule {
                    radius0,
                    radius1,
                    height,
                },
            )
        }
    }

    pub fn body(&self, matrix: V::Matrix4) -> NewtonBody<V> {
        body::create_dynamic_body(self.world.clone(), self.collision, matrix)
    }
}
