#![allow(unused_variables)]

use newton::types::Cgmath;

use newton::body::{self, NewtonBody};
use newton::collision;
use newton::world::{self, Broadphase, Solver, Threads, World};

use newton::sandbox;
use newton::sandbox::cgmath::{vec3, Matrix4, Quaternion, Vector3};
use std::time::Duration;

fn main() {
    let world: World<Cgmath> = world::create();

    let mut w = world.write();
    let pool = [
        collision::cuboid(&mut w, 16.0, 0.1, 16.0, 0, None),
        collision::cuboid(&mut w, 1.0, 1.0, 1.0, 0, None),
    ];

    //let bodies = w.bodies_mut();

    let floor = body::dynamic(
        &mut w,
        &pool[0].read(),
        &position(0.0, -6.0, 0.0),
        Some("floor"),
    );
    //let ceiling = body::dynamic(&mut w, &pool[0].borrow(), &position(0.0, 6.0, 0.0));
    drop(w);

    for body in world.write().bodies_mut() {
        println!("{:?}", body);
    }
    for body in world.read().bodies() {
        println!("{:?}", body.position());
    }

    // run a visual simulation
    let mut w = world.write();

    let sphere = collision::sphere(&mut w, 1.0, 0, None);
    let coll = pool[1].write();

    let cube_2 = body::dynamic(&mut w, &coll, &position(0.5, 3.5, 0.25), None);
    let cube_3 = body::dynamic(&mut w, &sphere.read(), &position(-0.25, -1.0, -0.5), None);
    let cube_4 = body::dynamic(&mut w, &coll, &position(0.1, 0.5, 0.4), Some("cube"));

    drop(w);

    //  cube_2.borrow_mut().set_mass(1.0);
    //  cube_2
    //      .borrow_mut()
    //      .set_force_and_torque_callback::<Gravity>();
    //
    cube_2.write().set_mass(1.0);
    //cube_2.borrow_mut().set_force_and_torque(None);

    //let gravity = |b: &mut NewtonBody<_>, _: Duration| b.set_force(&vec3(0.0, -9.8, 0.0));

    let gravity = vec3(0.0, -9.8, 0.0);
    cube_2
        .write()
        .set_force_and_torque(move |b, _, _| b.set_force(&gravity));
    //  cube_4
    //      .borrow_mut()
    //      .set_force_and_torque_callback::<Gravity>();

    cube_4.write().set_mass(1.0);
    cube_4.write().set_force_and_torque(gravity_callback);

    for body in world.write().bodies_mut() {
        //println!("{:?}", body);
        //body.set_velocity(&vec3(0.0, 1.0, 0.0));
        //body.set_linear_damping(0.5);
    }

    let (min, max) = (vec3(-10.0, -10.0, -10.0), vec3(10.0, 10.0, 10.0));

    //world
    //    .borrow_mut()
    //    .for_each_body_in_aabb((&min, &max), |b| Ok(()));

    sandbox::run(world.read().as_raw());
}

fn gravity_callback(b: &mut NewtonBody<Cgmath>, _: Duration, _: i32) {
    b.set_force(&vec3(0.0, -1.0, 0.0))
}

fn position(x: f32, y: f32, z: f32) -> Matrix4<f32> {
    Matrix4::from_translation(vec3(x, y, z))
}
