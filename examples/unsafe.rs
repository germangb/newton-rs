extern crate newton_dynamics;

extern crate cgmath;
extern crate gl;
#[macro_use]
extern crate lazy_static;
extern crate sdl2;

mod renderer;

use self::newton_dynamics::bindgen as ffi;

use cgmath::{
    Matrix4,
    PerspectiveFov,
    Point3,
    Rad,
    Vector3,

    prelude::*,
};

use sdl2::event::Event;

mod color {
    use renderer::Color3;

    pub static RED: Color3 = Color3 { x: 1.0, y: 0.0, z: 0.0 };
    pub static GREEN: Color3 = Color3 { x: 0.0, y: 1.0, z: 0.0 };
    pub static BLUE: Color3 = Color3 { x: 0.0, y: 1.0, z: 0.0 };
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

    let mut renderer = renderer::Renderer::new().unwrap();
    println!("renderer={:?}", renderer);

    renderer.set_light(Vector3::new(2.0, -3.0, -1.0)).unwrap();

    renderer
        .set_camera_transforms(
            PerspectiveFov {
                fovy: Rad(0.8726),
                aspect: 4.0 / 3.0,
                near: 0.01,
                far: 1000.0,
            },
            Point3::new(2.0, 6.0, 8.0),
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        )
        .unwrap();

    let mut event_pump = sdl_context.event_pump().unwrap();
    'main: loop {
        for event in event_pump.poll_iter() {
            if let Event::Quit { .. } = event {
                break 'main;
            }
        }

        example.update(2.0 / 60.0);

        renderer.clear().unwrap();

        let mut transform = Matrix4::identity();

        // render ground
        unsafe { ffi::NewtonBodyGetMatrix(example.ground(), transform.as_mut_ptr()) };
        renderer
            .render_box(
                Matrix4::from_nonuniform_scale(8.0, 0.2, 8.0) * transform,
                color::RED,
            )
            .unwrap();

        // render box
        for b in example.bodies() {
            unsafe { ffi::NewtonBodyGetMatrix(*b, transform.as_mut_ptr()) };
            renderer
                .render_box(
                    Matrix4::from_nonuniform_scale(1.0, 1.0, 1.0) * transform,
                    color::GREEN,
                )
                .unwrap();
        }

        window.gl_swap_window();
    }
}

mod example {
    use ffi;

    use cgmath::{
        Matrix4,
        Vector3,
        Vector4,
        
        prelude::*,
    };

    pub struct Example {
        world: *mut ffi::NewtonWorld,
        ground_collision: *mut ffi::NewtonCollision,
        box_collision: *mut ffi::NewtonCollision,

        ground_body: *mut ffi::NewtonBody,
        box_bodies: Vec<*mut ffi::NewtonBody>,
    }

    impl Example {
        pub fn new() -> Self {
            unsafe {
                let world = ffi::NewtonCreate();
                let (ground_collision, box_collision, ground_body, box_bodies) =
                    Self::add_bodies(world);

                Self {
                    world,

                    ground_collision,
                    box_collision,

                    ground_body,
                    box_bodies,
                }
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

        unsafe fn add_bodies(
            world: *mut ffi::NewtonWorld,
        ) -> (
            *mut ffi::NewtonCollision,
            *mut ffi::NewtonCollision,
            *mut ffi::NewtonBody,
            Vec<*mut ffi::NewtonBody>,
        ) {
            let mut tm = Matrix4::identity();

            let cs_box = ffi::NewtonCreateBox(world, 1.0, 1.0, 1.0, 0, tm.as_ptr());
            let cs_ground = ffi::NewtonCreateBox(world, 8.0, 0.2, 8.0, 0, tm.as_ptr());

            let ground = ffi::NewtonCreateDynamicBody(world, cs_ground, tm.as_ptr());
            ffi::NewtonBodySetForceAndTorqueCallback(ground, Some(cb_apply_force));

            let mut bodies = Vec::new();

            for i in 0..4 {
                tm.replace_col(3, Vector4::new(0.1 * i as f32, 4.0 + i as f32, 0.0, 1.0));

                let bbox = ffi::NewtonCreateDynamicBody(world, cs_box, tm.as_ptr());

                // Assign non-zero mass to sphere to make it dynamic.
                ffi::NewtonBodySetMassMatrix(bbox, 1.0, 1.0, 1.0, 1.0);

                // Install the callbacks to track the body positions.
                ffi::NewtonBodySetForceAndTorqueCallback(bbox, Some(cb_apply_force));

                bodies.push(bbox);
            }

            (cs_ground, cs_box, ground, bodies)
        }
    }

    impl Drop for Example {
        fn drop(&mut self) {
            unsafe {
                ffi::NewtonDestroyCollision(self.ground_collision);
                ffi::NewtonDestroyCollision(self.box_collision);
                ffi::NewtonDestroyAllBodies(self.world);
                ffi::NewtonDestroy(self.world);
            }
        }
    }

    extern "C" fn cb_apply_force(body: *const ffi::NewtonBody, timestep: f32, _thread_idx: i32) {
        // Fetch body position
        let mut pos = Vector3::zero();

        unsafe {
            ffi::NewtonBodyGetPosition(body, pos.as_mut_ptr());

            // Apply gravity force
            ffi::NewtonBodySetForce(body, Vector3::new(0.0, -1.0, 0.0).as_ptr());

            //println!("> body pos={pos:?}", pos = pos);
        }
    }
}
