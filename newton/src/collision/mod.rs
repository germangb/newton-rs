#[macro_use]
mod macros;
mod heightfield;

use crate::body;
use crate::body::DynamicBody;
use crate::pointer::*;
use crate::world;
use crate::NewtonApp;

use ffi;
use ffi::NewtonCollisionInfoRecord;

use std::cell::Cell;
use std::marker::PhantomData;
use std::mem;
use std::ptr;
use std::rc::Rc;
use std::slice;

use std::os::raw;

pub type ShapeId = i32;

pub trait IntoCollision<C> {
    fn into_collision(self) -> Collision<C>;
}

collision_enum! {
    #[derive(Debug, Clone)]
    pub enum Collision<C> {
        Box(BoxCollision<C>),
        Sphere(SphereCollision<C>),
        Cylinder(CylinderCollision<C>),
        Cone(ConeCollision<C>),
        Capsule(CapsuleCollision<C>),
        HeightField(HeightFieldCollision<C>),
    }
}

pub use self::heightfield::HeightFieldCollision;

collisions! {
    #[derive(Debug, Clone)]
    pub struct BoxCollision<C>;

    #[derive(Debug, Clone)]
    pub struct SphereCollision<C>;

    #[derive(Debug, Clone)]
    pub struct ConeCollision<C>;

    #[derive(Debug, Clone)]
    pub struct CylinderCollision<C>;

    #[derive(Debug, Clone)]
    pub struct CapsuleCollision<C>;
}

pub use self::heightfield::HeightFieldParams;

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

#[derive(Debug)]
pub struct CylinderParams {
    pub radius0: f32,
    pub radius1: f32,
    pub height: f32,
}

#[derive(Debug)]
pub struct CapsuleParams {
    pub radius0: f32,
    pub radius1: f32,
    pub height: f32,
}

impl<C: NewtonApp> BoxCollision<C> {
    pub const TYPE_ID: i32 = ffi::SERIALIZE_ID_BOX as i32;

    collision_methods!(fn new(dx, dy, dz) -> ffi::NewtonCreateBox);
    collision_methods!(fn scale);
    collision_methods!(fn offset, C::Matrix);

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

impl<C: NewtonApp> SphereCollision<C> {
    pub const TYPE_ID: i32 = ffi::SERIALIZE_ID_SPHERE as i32;

    collision_methods!(fn new(radius) -> ffi::NewtonCreateSphere);
    collision_methods!(fn scale);
    collision_methods!(fn offset, C::Matrix);

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

impl<C: NewtonApp> ConeCollision<C> {
    pub const TYPE_ID: i32 = ffi::SERIALIZE_ID_CONE as i32;

    collision_methods!(fn new(radius, height) -> ffi::NewtonCreateCone);
    collision_methods!(fn scale);
    collision_methods!(fn offset, C::Matrix);

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

impl<C: NewtonApp> CylinderCollision<C> {
    pub const TYPE_ID: i32 = ffi::SERIALIZE_ID_CYLINDER as i32;

    collision_methods!(fn new(radius0, radius1, height) -> ffi::NewtonCreateCylinder);
    collision_methods!(fn scale);
    collision_methods!(fn offset, C::Matrix);

    pub fn params(&self) -> CylinderParams {
        unsafe {
            let mut info = mem::zeroed();

            ffi::NewtonCollisionGetInfo(self.raw, &mut info);
            assert_eq!(ffi::SERIALIZE_ID_CYLINDER as i32, info.m_collisionType);

            CylinderParams {
                radius0: info.__bindgen_anon_1.m_cylinder.m_radio0,
                radius1: info.__bindgen_anon_1.m_cylinder.m_radio1,
                height: info.__bindgen_anon_1.m_cylinder.m_height,
            }
        }
    }
}

impl<C: NewtonApp> CapsuleCollision<C> {
    pub const TYPE_ID: i32 = ffi::SERIALIZE_ID_CAPSULE as i32;

    collision_methods!(fn new(radius0, radius1, height) -> ffi::NewtonCreateCapsule);
    collision_methods!(fn scale);
    collision_methods!(fn offset, C::Matrix);

    pub fn params(&self) -> CapsuleParams {
        unsafe {
            let mut info = mem::zeroed();

            ffi::NewtonCollisionGetInfo(self.raw, &mut info);
            assert_eq!(ffi::SERIALIZE_ID_CAPSULE as i32, info.m_collisionType);

            CapsuleParams {
                radius0: info.__bindgen_anon_1.m_capsule.m_radio0,
                radius1: info.__bindgen_anon_1.m_capsule.m_radio1,
                height: info.__bindgen_anon_1.m_capsule.m_height,
            }
        }
    }
}
