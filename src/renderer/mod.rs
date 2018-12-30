use super::body::NewtonBody;
use super::collision::NewtonCollision;
use super::Types;

use std::marker::PhantomData;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Vertex {
    x: f32,
    y: f32,
    z: f32,
    rgba: u32,
}

#[derive(Debug)]
pub struct Renderer<T> {
    _phantom: PhantomData<T>,
}

impl<T: Types> Renderer<T> {
    pub fn new() -> Self {
        Renderer {
            _phantom: PhantomData,
        }
    }
}
