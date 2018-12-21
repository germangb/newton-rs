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
use std::slice;

use std::os::raw;

#[derive(Debug, Clone)]
pub enum CollisionParams {
    /// Parameters of a Box collision. The total volume ends up being: `dx * dy * dz`
    Cuboid {
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
        radius: f32,
        height: f32,
    },
    Capsule {
        radius0: f32,
        radius1: f32,
        height: f32,
    },
}

#[derive(Debug)]
pub(crate) struct CollisionInfo {
    params: CollisionParams,

    // used in the polygon iterator
    udata: *mut (),
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

/// A reference counter NewtonCollision
#[derive(Debug, Clone)]
pub struct NewtonCollision<V> {
    pub(crate) collision: Rc<NewtonCollisionPtr<V>>,
    pub(crate) raw: *mut ffi::NewtonCollision,
}

pub type FaceId = i32;
pub type ShapeId = i32;

pub type Face<'a> = (FaceId, &'a [f32]);
pub type Polygons<'a> = Vec<Face<'a>>;

fn create_collision<V>(
    world: Rc<NewtonWorldPtr<V>>,
    raw: *mut ffi::NewtonCollision,
    params: CollisionParams,
) -> NewtonCollision<V>
where
    V: NewtonConfig,
{
    unsafe {
        let info = CollisionInfo {
            params,
            udata: ptr::null_mut(),
        };

        ffi::NewtonCollisionSetUserData(raw, mem::transmute(Box::new(info)));

        NewtonCollision {
            //world,
            collision: Rc::new(NewtonCollisionPtr(raw, world, PhantomData)),
            raw,
        }
    }
}

impl<V: NewtonConfig> NewtonCollision<V> {
    // FIXME messy. Check for correctness
    pub fn polygons(&self, matrix: V::Matrix4) -> Polygons {
        unsafe {
            let mut polygons = Vec::new();

            ffi::NewtonCollisionForEachPolygonDo(
                self.raw,
                mem::transmute(&matrix),
                Some(collision_iterator_callback),
                mem::transmute(&mut polygons),
            );

            return polygons;

            unsafe extern "C" fn collision_iterator_callback(
                user_data: *const raw::c_void,
                vertex_count: raw::c_int,
                face_array: *const f32,
                face_id: raw::c_int,
            ) {
                let polygons: &mut Vec<(i32, &[f32])> = mem::transmute(user_data);
                polygons.push((
                    face_id,
                    slice::from_raw_parts(face_array, vertex_count as usize * 3),
                ));
            }
        }
    }

    pub fn params(&self) -> CollisionParams {
        unsafe {
            let info: Box<CollisionInfo> =
                Box::from_raw(ffi::NewtonCollisionGetUserData(self.raw) as _);
            let params = info.params.clone();
            mem::forget(info);
            params
        }
    }

    pub fn scale(&mut self) -> (f32, f32, f32) {
        unsafe {
            let (mut x, mut y, mut z) = (0.0, 0.0, 0.0);
            ffi::NewtonCollisionGetScale(self.raw, &mut x, &mut y, &mut z);
            (x, y, z)
        }
    }

    pub fn offset(&mut self) -> V::Matrix4 {
        unsafe {
            let mut matrix: V::Matrix4 = mem::zeroed();
            ffi::NewtonCollisionGetMatrix(self.raw, mem::transmute(&mut matrix));
            matrix
        }
    }

    pub fn cuboid(
        world: &NewtonWorld<V>,
        dx: f32,
        dy: f32,
        dz: f32,
        shape_id: ShapeId,
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
                    shape_id,
                    mem::transmute(offset.as_ref()),
                ),
                CollisionParams::Cuboid { dx, dy, dz },
            )
        }
    }

    pub fn sphere(
        world: &NewtonWorld<V>,
        radius: f32,
        shape_id: ShapeId,
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
        shape_id: ShapeId,
        offset: Option<V::Matrix4>,
    ) -> NewtonCollision<V> {
        unsafe {
            create_collision(
                world.world.clone(),
                ffi::NewtonCreateCone(
                    world.world.0,
                    radius,
                    height,
                    shape_id,
                    mem::transmute(offset.as_ref()),
                ),
                CollisionParams::Cone { radius, height },
            )
        }
    }

    pub fn cylinder(
        world: &NewtonWorld<V>,
        radius: f32,
        height: f32,
        shape_id: ShapeId,
        offset: Option<V::Matrix4>,
    ) -> NewtonCollision<V> {
        unsafe {
            create_collision(
                world.world.clone(),
                ffi::NewtonCreateCylinder(
                    world.world.0,
                    radius,
                    radius,
                    height,
                    shape_id,
                    mem::transmute(offset.as_ref()),
                ),
                CollisionParams::Cylinder { radius, height },
            )
        }
    }

    pub fn capsule(
        world: &NewtonWorld<V>,
        radius0: f32,
        radius1: f32,
        height: f32,
        shape_id: ShapeId,
        offset: Option<V::Matrix4>,
    ) -> NewtonCollision<V> {
        unsafe {
            create_collision(
                world.world.clone(),
                ffi::NewtonCreateCapsule(
                    world.world.0,
                    radius0,
                    radius1,
                    height,
                    shape_id,
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
        body::create_dynamic_body(self.collision.1.clone(), self.collision.clone(), matrix)
    }
}
