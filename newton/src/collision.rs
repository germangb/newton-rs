use crate::body;
use crate::body::Body;
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

macro_rules! collision_enum {
    (
        $(#[$($meta:meta)+])*
        pub enum $typee:ident<$gen:ident> {
            $(
                $(#[$($metai:meta)+])*
                $enumm:ident ( $structt:ty  ) ,
            )+
        }
    ) => {
        $(#[$($meta)+])*
        pub enum $typee <$gen> {
            $(
                $(#[$($metai)+])*
                $enumm ( $structt ) ,
            )+
        }

        impl<$gen> $typee<$gen> {
            pub(crate) fn pointer(&self) -> &Rc<NewtonCollisionPtr<$gen>> {
                match &self {
                    $(
                        &Collision::$enumm (ref c) => &c.collision,
                    )+
                }
            }

            pub fn as_ref(&self) -> *mut ffi::NewtonCollision {
                match &self {
                    $(
                        &Collision::$enumm (ref c) => c.raw,
                    )+
                }
            }
        }

        $(impl<C> IntoCollision<C> for $structt {
            fn into(self) -> Collision<C> {
                Collision::$enumm(self)
            }
        })+
    }
}

pub trait IntoCollision<C> {
    fn into(self) -> Collision<C>;
}

collision_enum! {
    #[derive(Debug)]
    pub enum Collision<C> {
        Box(BoxCollision<C>),
        Sphere(SphereCollision<C>),
        Cylinder(CylinderCollision<C>),
        Cone(ConeCollision<C>),
        Capsule(CapsuleCollision<C>),
    }
}

macro_rules! collisions {
    ($(
        $(#[$($meta:meta)+])*
        pub struct $struct_ident:ident<C>;
    )+) => {$(
        $(#[$($meta)+])*
        pub struct $struct_ident<C> {
            pub(crate) collision: Rc<NewtonCollisionPtr<C>>,
            pub(crate) raw: *mut ffi::NewtonCollision,
        }
    )+}
}

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

macro_rules! collision_methods {
    (fn $method:ident ( $($param:ident),+ ) -> ffi::$ffi:ident) => {
        pub fn $method(
            world: &world::World<C>,
            $($param: f32,)+
            shape_id: ShapeId,
            offset: Option<C::Matrix>,
        ) -> Self {
            unsafe {
                let raw = ffi::$ffi(
                    world.raw,
                    $($param ,)+
                    shape_id,
                    mem::transmute(offset.as_ref()),
                );
                let collision = Rc::new(NewtonCollisionPtr(raw, world.world.clone()));
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
    (fn as_raw) => {
        pub fn as_raw(&self) -> *mut ffi::NewtonCollision {
            self.raw
        }
    };
}

impl<C: NewtonApp> BoxCollision<C> {
    collision_methods!(fn new(dx, dy, dz) -> ffi::NewtonCreateBox);
    collision_methods!(fn scale);
    collision_methods!(fn offset, C::Matrix);
    collision_methods!(fn type_id, ffi::SERIALIZE_ID_BOX);
    collision_methods!(fn as_raw);

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
    collision_methods!(fn new(radius) -> ffi::NewtonCreateSphere);
    collision_methods!(fn scale);
    collision_methods!(fn offset, C::Matrix);
    collision_methods!(fn type_id, ffi::SERIALIZE_ID_SPHERE);
    collision_methods!(fn as_raw);

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
    collision_methods!(fn new(radius, height) -> ffi::NewtonCreateCone);
    collision_methods!(fn scale);
    collision_methods!(fn offset, C::Matrix);
    collision_methods!(fn type_id, ffi::SERIALIZE_ID_CONE);
    collision_methods!(fn as_raw);

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
    collision_methods!(fn new(radius0, radius1, height) -> ffi::NewtonCreateCylinder);
    collision_methods!(fn scale);
    collision_methods!(fn offset, C::Matrix);
    collision_methods!(fn type_id, ffi::SERIALIZE_ID_CYLINDER);
    collision_methods!(fn as_raw);

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
    collision_methods!(fn new(radius0, radius1, height) -> ffi::NewtonCreateCapsule);
    collision_methods!(fn scale);
    collision_methods!(fn offset, C::Matrix);
    collision_methods!(fn type_id, ffi::SERIALIZE_ID_CAPSULE);
    collision_methods!(fn as_raw);

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
