use crate::body;
use crate::body::NewtonBody;
use crate::pointer::*;
use crate::world::NewtonWorld;
use crate::NewtonConfig;

use crate::ffi;
use crate::ffi::NewtonCollisionInfoRecord;

use std::cell::Cell;
use std::marker::PhantomData;
use std::mem;
use std::ptr;
use std::rc::Rc;
use std::slice;

use std::os::raw;

pub type ShapeId = i32;

#[derive(Debug)]
pub enum NewtonCollision<C> {
    Box(CollisionBox<C>),
    Sphere(CollisionSphere<C>),
    // TODO
    Cylinder,
    Cone(CollisionCone<C>),
    // TODO
    Capsule,
}

macro_rules! collisions {
    ($(
        #[$($meta:meta)+]
        pub struct $struct_ident:ident<C>;
    )+) => {$(
        #[$($meta)+]
        pub struct $struct_ident<C> {
            pub(crate) collision: Rc<NewtonCollisionPtr<C>>,
            pub(crate) raw: *mut ffi::NewtonCollision,
        }
    )+}
}

collisions! {
    #[derive(Debug, Clone)]
    pub struct CollisionBox<C>;

    #[derive(Debug, Clone)]
    pub struct CollisionSphere<C>;

    #[derive(Debug, Clone)]
    pub struct CollisionCone<C>;
}

#[derive(Debug)]
pub struct BoxParams {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug)]
pub struct SphereParams {
    pub radius: f32,
}

#[derive(Debug)]
pub struct ConeParams {
    pub radius: f32,
    pub height: f32,
}

macro_rules! from_into {
    ($(
        NewtonCollision::$variant:ident ( $col:ty ) ,
    )+) => {$(
        impl<C> From<$col> for NewtonCollision<C> {
            fn from(collision: $col) -> Self {
                NewtonCollision::$variant(collision)
            }
        }

        impl<'a, C: Clone> From<&'a $col> for NewtonCollision<C> {
            fn from(collision: &'a $col) -> Self {
                NewtonCollision::$variant(collision.clone())
            }
        }
    )+}
}

from_into! {
    NewtonCollision::Box(CollisionBox<C>),
    NewtonCollision::Sphere(CollisionSphere<C>),
    NewtonCollision::Cone(CollisionCone<C>),
}

macro_rules! collision_methods {
    (fn new ( $($param:ident),+ ) -> ffi::$ffi:ident) => {
        pub fn new(
            world: &NewtonWorld<C>,
            $($param: f32,)+
            shape_id: ShapeId,
            offset: Option<C::Matrix4>,
        ) -> Self {
            unsafe {
                let raw = ffi::$ffi(
                    world.raw,
                    $($param ,)+
                    shape_id,
                    mem::transmute(offset.as_ref()),
                );
                let collision = Rc::new(NewtonCollisionPtr(raw, world.world.clone(), PhantomData));
                Self { collision, raw }
            }
        }
    };
    (fn scale) => {
        pub fn scale(&self) -> (f32, f32, f32) {
            unsafe {
                let (mut x, mut y, mut z) = (0.0, 0.0, 0.0);
                ffi::NewtonCollisionGetScale(self.raw, &mut x, &mut y, &mut z);
                (x, y, z)
            }
        }
        pub fn set_scale(&self, (x, y, z): (f32, f32, f32)) {
            unsafe { ffi::NewtonCollisionSetScale(self.raw, x, y, z) };
        }
    };
    (fn offset, $type:ty) => {
        pub fn offset(&self) -> $type {
            unsafe {
                let mut matrix: $type = mem::zeroed();
                ffi::NewtonCollisionGetMatrix(self.raw, mem::transmute(&mut matrix));
                matrix
            }
        }
        pub fn set_offset(&self, matrix: $type) {
            unsafe { ffi::NewtonCollisionSetMatrix(self.raw, mem::transmute(&matrix)) };
        }
    };
    (fn type_id, $value:expr) => {
        /// Internal type ID of the collision.
        pub const TYPE_ID: i32 = $value as i32;
    };
}

impl<C: NewtonConfig> CollisionBox<C> {
    collision_methods!(fn new(dx, dy, dz) -> ffi::NewtonCreateBox);
    collision_methods!(fn scale);
    collision_methods!(fn offset, C::Matrix4);
    collision_methods!(fn type_id, ffi::SERIALIZE_ID_BOX);

    pub fn params(&self) -> BoxParams {
        unsafe {
            let mut info = mem::zeroed();

            ffi::NewtonCollisionGetInfo(self.raw, &mut info);
            assert_eq!(ffi::SERIALIZE_ID_BOX as i32, info.m_collisionType);

            BoxParams {
                x: info.__bindgen_anon_1.m_box.m_x,
                y: info.__bindgen_anon_1.m_box.m_y,
                z: info.__bindgen_anon_1.m_box.m_z,
            }
        }
    }
}

impl<C: NewtonConfig> CollisionSphere<C> {
    collision_methods!(fn new(radius) -> ffi::NewtonCreateSphere);
    collision_methods!(fn scale);
    collision_methods!(fn offset, C::Matrix4);
    collision_methods!(fn type_id, ffi::SERIALIZE_ID_SPHERE);

    pub fn params(&self) -> SphereParams {
        unsafe {
            let mut info = mem::zeroed();

            ffi::NewtonCollisionGetInfo(self.raw, &mut info);
            assert_eq!(ffi::SERIALIZE_ID_SPHERE as i32, info.m_collisionType);

            SphereParams {
                radius: info.__bindgen_anon_1.m_sphere.m_radio,
            }
        }
    }
}

impl<C: NewtonConfig> CollisionCone<C> {
    collision_methods!(fn new(radius, height) -> ffi::NewtonCreateCone);
    collision_methods!(fn scale);
    collision_methods!(fn offset, C::Matrix4);
    collision_methods!(fn type_id, ffi::SERIALIZE_ID_CONE);

    pub fn params(&self) -> ConeParams {
        unsafe {
            let mut info = mem::zeroed();

            ffi::NewtonCollisionGetInfo(self.raw, &mut info);
            assert_eq!(ffi::SERIALIZE_ID_CONE as i32, info.m_collisionType);

            ConeParams {
                radius: info.__bindgen_anon_1.m_cone.m_radio,
                height: info.__bindgen_anon_1.m_cone.m_height,
            }
        }
    }
}
