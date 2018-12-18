mod renderer;
mod types;

use std::time::Duration;
use std::rc::Rc;

use cgmath::{Matrix4, perspective, Deg, Point3, Vector3};
use cgmath::prelude::*;

use self::renderer::{Renderer, Primitive, Mode};
pub use self::renderer::Color;

use newton_dynamics::NewtonData;

use newton_dynamics::world::NewtonWorld;
use newton_dynamics::body::NewtonBody;
//use newton_dynamics::{NewtonWorld, NewtonBody};
//use newton_dynamics::traits::*;

#[macro_export]
macro_rules! rgba {
    ($r:expr, $g:expr, $b:expr, $a:expr) => { $crate::Color { r: $r, g: $g, b: $b, a: $a} };
    ($r:expr, $g:expr, $b:expr) => { rgba!($r, $g, $b, 1.0) };
    ($r:expr) => { rgba!($r, $r, $r) };
}

pub use sdl2::event::Event;
pub use sdl2::keyboard::{Keycode, Scancode};

pub struct Sandbox {
    background: Color,
    wireframe: bool,
    keyboard: bool,
}

impl Sandbox {
    pub fn new() -> Sandbox {
        Sandbox {
            background: rgba!(1.0, 1.0, 1.0),
            wireframe: false,
            keyboard: false,
        }
    }

    pub fn background(&mut self, background: Color) {
        self.background = background;
    }

    pub fn set_wireframe(&mut self, wireframe: bool) {
        self.wireframe = wireframe;
    }

    pub fn run<'a, V>(self, world: NewtonWorld<V>, bodies: Vec<NewtonBody<V>>)
    where
        V: NewtonData,
    {
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();

        let window = video_subsystem
            .window("Window", 640, 480)
            .opengl()
            .position_centered()
            .build()
            .unwrap();

        let gl_context = window.gl_create_context().unwrap();
        window.gl_make_current(&gl_context).unwrap();

        gl::load_with(|s| video_subsystem.gl_get_proc_address(s) as _);

        let renderer = Renderer::new();

        let proj = perspective(Deg(55.0_f32), 4.0/3.0, 0.01, 1000.0);
        let view = Matrix4::look_at(Point3::new(12.0, 6.0, 16.0f32) / 2.0, Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));

        renderer.set_projection(proj);
        renderer.set_view(view);

        let mut event_pump = sdl_context.event_pump().unwrap();
        'main: loop {
            for event in event_pump.poll_iter() {
                if let Event::Quit { .. } = event {
                    break 'main;
                }
            }

            world.update(Duration::new(0, 1_000_000_000 / 60));

            renderer.reset();
            for body in bodies.iter() {
                let mut transform = Matrix4::identity();
                unsafe {
                    // XXX pointers
                    std::ptr::copy(&body.matrix() as *const _ as *const f32, &mut transform[0][0] as *mut f32, 16);
                }
                renderer.render(Primitive::Box, Mode::Fill, rgba!(1.0), transform);
            }

            window.gl_swap_window();
        }
    }
}
