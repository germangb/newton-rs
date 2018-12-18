macro_rules! check {
    ($gl:expr) => {{
        gl::GetError();
        let result = $gl;
        let error = gl::GetError();
        assert_eq!(gl::NO_ERROR, error);
        result
    }}
}

mod cube;
mod capsule;
mod cylinder;
mod sphere;
mod program;

use self::program::Program;
use self::cube::Cube;

use cgmath::prelude::*;
use cgmath::{Matrix4};

use std::cell::Cell;

#[repr(u8)]
#[derive(Hash, Copy, Clone, Debug, Eq, PartialEq)]
pub enum Primitive {
    Box = 0,
    Cylinder,
    Capsule,
}

#[derive(Hash, Copy, Clone, Debug, Eq, PartialEq)]
pub enum Mode {
    Fill,
    Wireframe,
}

pub struct Renderer {
    program: Program,
    cube: Cube,
    // cache
    primitive: Cell<Option<Primitive>>,
    mode: Cell<Option<Mode>>,
}

#[derive(Copy, Clone, Debug)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Renderer {
    pub fn new() -> Self {
        let render = Renderer {
            program: Program::new(),
            cube: Cube::new(),
            primitive: Cell::new(None),
            mode: Cell::new(None),
        };

        render.reset();
        render
    }

    pub fn set_view(&self, view: Matrix4<f32>) {
        unsafe {
            check!(gl::UniformMatrix4fv(self.program.view, 1, gl::FALSE, view.as_ptr()));
        }
    }

    pub fn set_projection(&self, proj: Matrix4<f32>) {
        unsafe {
            check!(gl::UniformMatrix4fv(self.program.projection, 1, gl::FALSE, proj.as_ptr()));
        }
    }

    fn set_world(&self, world: Matrix4<f32>) {
        unsafe {
            check!(gl::UniformMatrix4fv(self.program.world, 1, gl::FALSE, world.as_ptr()));
        }
    }

    fn set_color(&self, color: Color) {
        unsafe {
            check!(gl::Uniform4f(self.program.color, color.r, color.g, color.b, color.a));
        }
    }

    pub fn set_up_primitive(&self, primitive: Primitive) {
    }

    pub fn render(&self, primitive: Primitive, mode: Mode, color: Color, world: Matrix4<f32>) {
        if primitive == Primitive::Box {
            self.set_world(world * Matrix4::from_scale(0.5));
        } else {
            self.set_world(world);
        }
        self.set_color(color);

        unsafe {
            if Some(mode) != self.mode.get() {
                match mode {
                    Mode::Wireframe => gl::PolygonMode(gl::FRONT_AND_BACK, gl::LINE),
                    Mode::Fill => gl::PolygonMode(gl::FRONT_AND_BACK, gl::FILL),
                }
                self.mode.set(Some(mode));
            }
            match primitive {
                Primitive::Box => {
                    if self.primitive.get() != Some(primitive) { check!(gl::BindVertexArray(self.cube.vao)); }
                    check!(gl::DrawElements(gl::TRIANGLES, self.cube.indices() as _, gl::UNSIGNED_INT, std::ptr::null()));
                },
                _ => unreachable!(),
            }

            self.primitive.set(Some(primitive));
        }
    }

    pub fn reset(&self) {
        self.primitive.set(None);
        self.mode.set(None);
        unsafe {
            check!(gl::Enable(gl::DEPTH_TEST));
            check!(gl::BindVertexArray(0));
            check!(gl::ClearColor(1.0, 1.0, 1.0, 0.0));
            check!(gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT));
            check!(gl::UseProgram(self.program.program));
        }
    }
}