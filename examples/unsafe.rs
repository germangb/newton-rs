//FIXME Crashes because collision objects are not being freed

extern crate newton_dynamics;
extern crate cgmath;

use newton_dynamics::bindgen as ffi;

use std::mem;

use cgmath::{
    prelude::*,

    Vector3,
    Vector4,
    Matrix4,
};

fn main() {
    unsafe {
        let world = ffi::NewtonCreate();

        add_bodies(world);

        let timestep = 1.0 / 60.0;
        for step in 0..512 {
            println!("{}-th step", step);
            ffi::NewtonUpdate(world, timestep);
        }

        ffi::NewtonDestroyAllBodies(world);
        ffi::NewtonDestroy(world);
    }
}

extern "C" fn cb_apply_force(body: *const ffi::NewtonBody, timestep: f32, _thread_idx: i32) {
    // Fetch body position
    let mut pos = Vector3::zero();

    unsafe {
        ffi::NewtonBodyGetPosition(body, pos.as_mut_ptr());

        // Apply gravity force
        ffi::NewtonBodySetForce(body, Vector3::new(0.0, -1.0, 0.0).as_ptr());

        println!("> body pos={pos:?}", pos=pos);
    }
}

unsafe fn add_bodies(world: *const ffi::NewtonWorld) {
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
}
