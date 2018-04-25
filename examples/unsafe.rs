fn main() {
    let mut context = example::Example::new();

    let timestep = 1.0 / 60.0;
    for step in 0..512 {
        println!("{}-th step", step);
        context.update(timestep);
    }
}

mod example {
    extern crate cgmath;
    extern crate newton_dynamics;

    use self::newton_dynamics::bindgen as ffi;
    use self::cgmath::{Matrix4, Vector3, Vector4, prelude::*,};

    pub struct Example {
        world: *mut ffi::NewtonWorld,
        box_collision: *mut ffi::NewtonCollision,
        sphere_collision: *mut ffi::NewtonCollision,
        box_body: *mut ffi::NewtonBody,
        sphere_body: *mut ffi::NewtonBody,
    }

    impl Example {
        pub fn new() -> Self {
            unsafe {
                let world = ffi::NewtonCreate();
                let (box_collision, sphere_collision, box_body, sphere_body) =
                    Self::add_bodies(world);

                Self {
                    world,
                    box_collision,
                    sphere_collision,
                    box_body,
                    sphere_body,
                }
            }
        }

        pub fn update(&mut self, timestep: f32) {
            unsafe {
                ffi::NewtonUpdate(self.world, timestep);
            }
        }

        pub fn destroy(&mut self) {
            unsafe {
                ffi::NewtonDestroyCollision(self.box_collision);
                ffi::NewtonDestroyCollision(self.sphere_collision);
                ffi::NewtonDestroyAllBodies(self.world);
                ffi::NewtonDestroy(self.world);
            }
        }

        unsafe fn add_bodies(
            world: *mut ffi::NewtonWorld,
        ) -> (
            *mut ffi::NewtonCollision,
            *mut ffi::NewtonCollision,
            *mut ffi::NewtonBody,
            *mut ffi::NewtonBody,
        ) {
            let mut tm = Matrix4::identity();

            let cs_sphere = ffi::NewtonCreateSphere(world, 1.0, 0, tm.as_ptr());
            let cs_ground = ffi::NewtonCreateBox(world, 100.0, 0.1, 100.0, 0, tm.as_ptr());

            let ground = ffi::NewtonCreateDynamicBody(world, cs_ground, tm.as_ptr());

            tm.replace_col(3, Vector4::new(0.0, 8.0, 0.0, 1.0));
            let sphere = ffi::NewtonCreateDynamicBody(world, cs_sphere, tm.as_ptr());

            // Assign non-zero mass to sphere to make it dynamic.
            ffi::NewtonBodySetMassMatrix(sphere, 1.0, 1.0, 1.0, 1.0);

            // Install the callbacks to track the body positions.
            ffi::NewtonBodySetForceAndTorqueCallback(sphere, Some(cb_apply_force));
            ffi::NewtonBodySetForceAndTorqueCallback(ground, Some(cb_apply_force));

            (cs_ground, cs_sphere, ground, sphere)
        }
    }

    impl Drop for Example {
        fn drop(&mut self) {
            self.destroy();
        }
    }

    extern "C" fn cb_apply_force(body: *const ffi::NewtonBody, timestep: f32, _thread_idx: i32) {
        // Fetch body position
        let mut pos = Vector3::zero();

        unsafe {
            ffi::NewtonBodyGetPosition(body, pos.as_mut_ptr());

            // Apply gravity force
            ffi::NewtonBodySetForce(body, Vector3::new(0.0, -1.0, 0.0).as_ptr());

            println!("> body pos={pos:?}", pos = pos);
        }
    }
}
