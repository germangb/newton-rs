extern crate cgmath;
extern crate cgmath_culling;

extern crate rand;
extern crate stb;
extern crate gl;

#[macro_use]
extern crate lazy_static;
extern crate newton_dynamics;
extern crate sdl2;

mod renderer;

use newton_dynamics::{
    NewtonWorld,
    NewtonCollision,
    NewtonBody,
};

use newton_dynamics::traits::{Vector, NewtonMath};

use cgmath::{Matrix4, PerspectiveFov, Point3, Rad, Vector3, Vector4, prelude::*};
use cgmath_culling::{BoundingBox, FrustumCuller, Intersection};

use sdl2::event::Event;

use renderer::{Camera, Renderer};

mod color {
    use renderer::Color3;
    pub static WHITE: Color3 = Color3 { x: 1.0, y: 1.0, z: 1.0 };
    pub static RED: Color3 = Color3 { x: 1.0, y: 0.25, z: 0.25 };
}

fn main() {
    let mut example = example::Example::new();

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

    let mut renderer = Renderer::new().unwrap();
    println!("renderer={:?}", renderer);

    renderer.set_light(Vector3::new(2.0, -3.0, -1.0)).unwrap();

    let mut event_pump = sdl_context.event_pump().unwrap();
    let mut angle = 0.0f32;

    // frustum culling
    let mut camera = Camera::new(PerspectiveFov {
        fovy: Rad(0.8726),
        aspect: 4.0 / 3.0,
        near: 0.01,
        far: 1000.0,
    });

    'main: loop {
        for event in event_pump.poll_iter() {
            if let Event::Quit { .. } = event {
                break 'main;
            }
        }

        // update simulation
        example.update(2.0 / 60.0);

        angle += 0.0025;
        camera.update_view(
            Point3::new(angle.sin() * 12.0, 8.0, angle.cos() * 12.0),
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        );

        let frustum_culler = FrustumCuller::from_matrix(camera.projection() * camera.view());

        renderer.clear().unwrap();
        renderer.set_camera(&camera).unwrap();

        let mut transform = example.ground().get_matrix();
        renderer.render_box(Matrix4::from_nonuniform_scale(6.0, 0.2, 6.0) * transform, color::RED).unwrap();

        for mat in example.bodies().map(|b| b.get_matrix()) {
            renderer.render_box(mat, color::WHITE).unwrap();
        }

        window.gl_swap_window();
    }
}

mod example {
    use rand;
    use rand::Rng;

    use cgmath::{Matrix4, Vector3, Vector4, prelude::*};
    use std::ptr;
    use std::slice::Iter;

    use newton_dynamics::{NewtonWorld, NewtonCollision, NewtonBody, traits::NewtonCgmath};

    pub struct Example {
        world: NewtonWorld<NewtonCgmath>,
        ground_body: NewtonBody<NewtonCgmath>,
        box_bodies: Vec<NewtonBody<NewtonCgmath>>,
    }

    impl Example {
        pub fn new() -> Self {
            let world = NewtonWorld::new();
            let mut box_bodies = Vec::new();

            let cs_box = world.create_box((1.0, 1.0, 1.0), 0, None);
            let cs_ground = world.create_box((6.0, 0.2, 6.0), 0, None);

            let ground_body = world.create_body(cs_ground, Matrix4::identity());

            let mut rng = rand::thread_rng();
            let mut tm = Matrix4::identity();
            for x in -1..2 {
                for z in -1..2 {
                    for y in 0..4 {
                        let (off_x, off_y, off_z) = rng.gen::<(f32, f32, f32)>();

                        tm.replace_col(
                            3,
                            Vector4::new(
                                1.5 * x as f32 + off_x * 0.2,
                                2.00 * y as f32 + 2.0 + off_y * 0.2 + x as f32 * 0.5,
                                1.5 * z as f32 + off_z * 0.2,
                                1.0,
                            ),
                        );

                        let cube = world.create_body(cs_box.clone(), tm);
                        cube.set_mass_matrix(1.0, (1.0, 1.0, 1.0));

                        box_bodies.push(cube);
                    }
                }
            }

            Example {
                world,
                ground_body,
                box_bodies,
            }
        }

        pub fn update(&mut self, timestep: f32) {
            let nano = timestep.fract() * 1_000_000_000_f32;
            self.world.update(::std::time::Duration::new(timestep as u64, nano as u32))
        }

        pub fn ground(&self) -> &NewtonBody<NewtonCgmath> {
            &self.ground_body
        }

        pub fn bodies(&self) -> Iter<NewtonBody<NewtonCgmath>> {
            self.box_bodies.iter()
        }

    }
}

