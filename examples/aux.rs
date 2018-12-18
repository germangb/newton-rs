mod renderer;

use self::renderer::Renderer;

use std::iter::IntoIterator;
use std::time::Duration;

use newton_dynamics::body::NewtonBody;
use newton_dynamics::traits::NewtonData;
use newton_dynamics::world::NewtonWorld;

pub use sdl2::event::Event;

use cgmath::prelude::*;
use cgmath::{perspective, Deg, Matrix4, Point3, Vector3};

pub trait Example<V> {
    fn bodies(&self) -> &[NewtonBody<V>];
    fn world(&self) -> (&NewtonWorld<V>, Duration);
    fn event(&mut self, event: Event) {}
}

pub fn run<V, E>(mut example: E)
where
    V: NewtonData,
    E: Example<V>,
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
    let mut event_pump = sdl_context.event_pump().unwrap();

    let renderer = Renderer::new();
    let projection = perspective(Deg(55.0), 4.0 / 3.0, 0.01, 1000.0).into();
    let view = Matrix4::look_at(
        Point3::new(8.0, 8.0, 8.0),
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    );

    'main: loop {
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. } => break 'main,
                _ => example.event(event),
            }
        }

        let (world, step) = example.world();
        world.update(step);

        renderer.clear();
        renderer.set_light(Vector3::new(1.0, 1.0, 1.0));
        renderer.set_view_projection(projection, view);

        for body in example.bodies() {
            renderer.render_box(Matrix4::identity(), Vector3::new(1.0, 1.0, 1.0));
        }

        window.gl_swap_window();
    }
}
