mod renderer;
mod types;

use std::marker::PhantomData;
use std::time::Duration;
use std::rc::Rc;

use cgmath::{Matrix4, perspective, Deg, Point3, Vector3};
use cgmath::prelude::*;

use self::renderer::{Renderer, Primitive, Mode};
pub use self::renderer::Color;

use newton_dynamics::NewtonData;

use newton_dynamics::world::NewtonWorld;
use newton_dynamics::body::{NewtonBody, SleepState};
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

pub struct Sandbox<V> {
    background: Color,
    wireframe: bool,
    keyboard: bool,
    aabb: bool,
    _ph: PhantomData<V>,
}

impl<V: NewtonData> Sandbox<V> {
    pub fn new() -> Sandbox<V> {
        Sandbox {
            background: rgba!(1.0, 1.0, 1.0),
            wireframe: false,
            keyboard: false,
            aabb: true,
            _ph: PhantomData,
        }
    }

    pub fn background(&mut self, background: Color) {
        self.background = background;
    }

    pub fn set_wireframe(&mut self, wireframe: bool) {
        self.wireframe = wireframe;
    }

    fn render(&self, renderer: &Renderer, bodies: &[NewtonBody<V>]) {
        renderer.reset();

        for body in bodies.iter() {
            let mut transform = Matrix4::identity();
            unsafe {
                // XXX pointers
                std::ptr::copy(&body.matrix() as *const _ as *const f32, &mut transform[0][0] as *mut f32, 16);
            }
            let color = match body.sleep_state() {
                SleepState::Sleeping => rgba!(1.0),
                SleepState::Awake => rgba!(1.0, 0.0, 1.0),
            };
            renderer.render(Primitive::Box, Mode::Fill, color, transform);
        }

        if self.aabb {
            // aabb
            // XXX pointers
            for body in bodies.iter() {
                unsafe {
                    let mut aabb: (Vector3<f32>, Vector3<f32>) = std::mem::zeroed();
                    let mut position: Vector3<f32> = std::mem::zeroed();

                    std::ptr::copy(&body.position() as *const _ as *const f32, &mut position as *mut _ as *mut f32, 3);
                    std::ptr::copy(&body.aabb() as *const _ as *const f32, &mut aabb as *mut _ as *mut f32, 6);

                    let scale = aabb.1 - aabb.0;

                    let mut transform = Matrix4::from_translation(position) *
                        Matrix4::from_nonuniform_scale(scale.x, scale.y, scale.z);

                    renderer.render(Primitive::Box, Mode::Wireframe, rgba!(0.0), transform);
                }
            }
        }
    }

    pub fn run<B>(self, world: NewtonWorld<V>, bodies: B)
    where
        B: Fn() -> Vec<NewtonBody<V>>,
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
            self.render(&renderer, &bodies());

            window.gl_swap_window();
        }
    }
}
