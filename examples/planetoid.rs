use newton::body;
use newton::collision;
use newton::world::{self, World};
use newton::Types;

use newton::sandbox;

use cgmath::{prelude::*, Matrix4, Quaternion, Vector3};
use std::time::Duration;

#[derive(Clone)]
enum M {}
unsafe impl Types for M {
    type Vector = Vector3<f32>;
    type Matrix = Matrix4<f32>;
    type Quaternion = Quaternion<f32>;
}

fn main() {
    let world: World<M> = World::default();

    let sphere = collision::Builder::new(&mut world.write())
        .sphere(16.0)
        .build();
    let planet = body::Builder::new(&mut world.write(), &sphere.read())
        .transform(Matrix4::identity())
        //.mass(1000.0)
        .build();

    let cube = collision::Builder::new(&mut world.write())
        .cuboid(2.0, 2.0, 2.0)
        .build();
    let sat = body::Builder::new(&mut world.write(), &cube.read())
        .transform(Matrix4::from_translation(Vector3::new(24.0, 0.0, 0.0)))
        .dynamic()
        .mass(4.0)
        .force_torque_callback(|b, _, _| {
            let d = b.position().magnitude2() * 0.001;
            let gravity = b.position().normalize() * -9.81 / d;
            b.set_force(&gravity);
        })
        .build();

    let sat_pos = sat.read().position();
    sat.write().add_impulse(
        &Vector3::new(0.0, 8.0, 4.0),
        &sat_pos,
        Duration::new(0, 1000000000 / 60),
    );

    run(world);
}

fn run(world: World<M>) {
    let ptr = world.read().as_raw();
    sandbox::run(ptr);
}
