use ffi;

#[macro_use]
use super::macros;
use super::ShapeId;
use crate::pointer::NewtonCollisionPtr;
use crate::world::World;
use crate::NewtonApp;

use std::mem;
use std::ops::{Deref, DerefMut};
use std::os::raw;
use std::rc::Rc;

collisions! {
    #[derive(Debug, Clone)]
    pub struct HeightFieldCollision<C>;
}

impl<C> HeightFieldCollision<C> {
    pub const TYPE_ID: i32 = ffi::SERIALIZE_ID_HEIGHTFIELD as i32;
}

impl<C: NewtonApp> HeightFieldCollision<C> {
    pub fn new(
        world: &World<C>,
        params: HeightFieldParams<f32>,
        shape: ShapeId,
        offset: Option<C::Matrix>,
    ) -> Self {
        unsafe {
            let world_mut = world.world.borrow_mut();

            let params = Box::new(params);
            let collision_raw = ffi::NewtonCreateHeightFieldCollision(
                world_mut.0,
                params.width() as raw::c_int,
                params.height() as raw::c_int,
                // TODO parametrize
                // gridsDiagonals
                0,
                // TODO allow both f32 and i64
                // elevation data type
                0,
                // elevation & attrs
                params.elevation().as_ptr() as *const raw::c_void,
                params.attributes().as_ptr(),
                // vertical scale
                params.vertical_scale(),
                // x & z scale
                params.scale_x(),
                params.scale_z(),
                0,
            );

            let collision = Self {
                collision: Rc::new(NewtonCollisionPtr(collision_raw, world.world.clone())),
                raw: collision_raw,
            };

            if let Some(m) = offset {
                collision.set_offset(m);
            }

            // store the params as user data
            // TODO FIXME dont' leak memory!!!
            ffi::NewtonCollisionSetUserData(collision_raw, mem::transmute(params));

            collision
        }
    }

    collision_methods!(fn offset, C::Matrix);
}

#[derive(Debug, Clone)]
pub struct HeightFieldParams<T> {
    elevation: Vec<T>,
    attributes: Vec<i8>,
    width: usize,
    height: usize,
    scale_x: f32,
    scale_z: f32,
    vertical_scale: f32,
}

impl HeightFieldParams<f32> {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            elevation: vec![0.0; width * height],
            attributes: vec![0; width * height],
            width,
            height,
            scale_x: 1.0,
            scale_z: 1.0,
            vertical_scale: 0.0,
        }
    }

    pub fn elevation(&self) -> &[f32] {
        &self.elevation[..]
    }

    pub fn elevation_mut(&mut self) -> &mut [f32] {
        &mut self.elevation[..]
    }

    pub fn attributes(&self) -> &[i8] {
        &self.attributes[..]
    }

    pub fn attributes_mut(&mut self) -> &mut [i8] {
        &mut self.attributes[..]
    }

    pub fn set_max_height(&mut self, h: f32) {
        self.vertical_scale = h / (0xFFFF_u16 as f32);
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn scale_x(&self) -> f32 {
        self.scale_x
    }

    pub fn scale_z(&self) -> f32 {
        self.scale_z
    }

    pub fn vertical_scale(&self) -> f32 {
        self.vertical_scale
    }

    pub fn set_vertical_scale(&mut self, y: f32) {
        self.vertical_scale = y;
    }

    pub fn set_horizontal_scale(&mut self, x: f32, z: f32) {
        self.scale_x = x;
        self.scale_z = z;
    }
}
