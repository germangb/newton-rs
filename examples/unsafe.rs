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

use self::newton_dynamics::ffi;

use cgmath::{Matrix4, PerspectiveFov, Point3, Rad, Vector3, prelude::*};
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

        let mut transform = Matrix4::identity();
        let mut aabb = BoundingBox::new();

        // render ground
        unsafe {
            ffi::NewtonBodyGetMatrix(example.ground(), transform.as_mut_ptr());
        }
        renderer
            .render_box(
                Matrix4::from_nonuniform_scale(6.0, 0.2, 6.0) * transform,
                color::RED,
            )
            .unwrap();

        // render box
        for b in example.bodies() {
            unsafe {
                ffi::NewtonBodyGetAABB(*b, aabb.min.as_mut_ptr(), aabb.max.as_mut_ptr());
            }

            // render box if it lies inside the view frustum
            match frustum_culler.test_bounding_box(aabb) {
                Intersection::Inside | Intersection::Partial => {
                    unsafe {
                        ffi::NewtonBodyGetMatrix(*b, transform.as_mut_ptr());
                    }
                    renderer.render_box(transform, color::WHITE).unwrap();
                }
                _ => {}
            }
        }

        window.gl_swap_window();
    }
}

mod example {
    use ffi;
    use rand;
    use rand::Rng;

    use cgmath::{Matrix4, Vector3, Vector4, prelude::*};
    use std::ptr;

    pub struct Example {
        world: *mut ffi::NewtonWorld,
        ground_body: *mut ffi::NewtonBody,
        box_bodies: Vec<*mut ffi::NewtonBody>,
    }

    impl Example {
        pub fn new() -> Self {
            unsafe {
                let mut example = Self {
                    world: ffi::NewtonCreate(),
                    ground_body: ptr::null_mut(),
                    box_bodies: Vec::new(),
                };

                example.init_example();
                example
            }
        }

        pub fn update(&mut self, timestep: f32) {
            unsafe {
                ffi::NewtonUpdate(self.world, timestep);
            }
        }

        pub fn ground(&self) -> *mut ffi::NewtonBody {
            self.ground_body
        }

        pub fn bodies(&self) -> ::std::slice::Iter<*mut ffi::NewtonBody> {
            self.box_bodies.iter()
        }

        unsafe fn init_example(&mut self) {
            let mut tm = Matrix4::identity();

            let cs_box = ffi::NewtonCreateBox(self.world, 1.0, 1.0, 1.0, 0, tm.as_ptr());
            let cs_ground = ffi::NewtonCreateBox(self.world, 6.0, 0.2, 6.0, 0, tm.as_ptr());

            self.ground_body = ffi::NewtonCreateDynamicBody(self.world, cs_ground, tm.as_ptr());
            ffi::NewtonBodySetForceAndTorqueCallback(self.ground_body, Some(cb_apply_force));

            let mut rng = rand::thread_rng();

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
                        let cube = ffi::NewtonCreateDynamicBody(self.world, cs_box, tm.as_ptr());

                        // Assign non-zero mass to sphere to make it dynamic.
                        ffi::NewtonBodySetMassMatrix(cube, 1.0, 1.0, 1.0, 1.0);

                        // Install the callbacks to track the body positions.
                        ffi::NewtonBodySetForceAndTorqueCallback(cube, Some(cb_apply_force));

                        self.box_bodies.push(cube);
                    }
                }
            }

            ffi::NewtonDestroyCollision(cs_ground);
            ffi::NewtonDestroyCollision(cs_box);
        }
    }

    extern "C" fn cb_apply_force(body: *const ffi::NewtonBody, timestep: f32, _thread_idx: i32) {
        unsafe {
            // Apply gravity force
            ffi::NewtonBodySetForce(body, Vector3::new(0.0, -1.0, 0.0).as_ptr());
        }
    }

    impl Drop for Example {
        fn drop(&mut self) {
            unsafe {
                ffi::NewtonDestroyAllBodies(self.world);
                ffi::NewtonDestroy(self.world);
            }
        }
    }
}

