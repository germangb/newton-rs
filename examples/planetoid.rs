use newton::body;
use newton::collision;
use newton::world::{self, World};

use newton::sandbox;

use cgmath::{prelude::*, vec3, Matrix4, Quaternion, Vector3};
use std::time::Duration;

use std::sync::mpsc::{self, Receiver, Sender};

fn main() {
    let world: World<(), ()> = World::default();

    let (pm, sm) = world.write().create_materials();

    let cube = collision::Builder::new(&mut world.write())
        .cuboid(2.0, 2.0, 2.0)
        .build();
    let sat = body::Builder::new(&mut world.write(), &cube.read())
        .transform(Matrix4::from_translation(Vector3::new(24.0, 0.0, 0.0)))
        .dynamic()
        .material(sm)
        .mass(4.0)
        .force_torque_callback(|b, _, _| {
            let d = b.position().magnitude2() * 0.001;
            let gravity = b.position().normalize() * -9.81 / d;
            b.set_force(&gravity);
        })
        .build();

    let sphere = collision::Builder::new(&mut world.write())
        .sphere(16.0)
        .build();

    let planet = body::Builder::new(&mut world.write(), &sphere.read())
        .transform(Matrix4::identity())
        .material(pm)
        .build();

    let sat_pos = sat.read().position();
    let s = Duration::new(0, 1000000000 / 60);
    sat.write().add_impulse(&vec3(0.0, 8.0, 4.0), &sat_pos, s);

    let (tx, rx) = mpsc::channel::<()>();

    // Destroy sat after 4 seconds
    std::thread::spawn(move || {
        if let Ok(_) = rx.recv() {
            sat.write().destroy();
            //drop(sat);
        }
    });

    world
        .write()
        .material_set_collision_callback((pm, sm), move |_m, _b0, _b1, _| {
            &tx.send(());
            true
        });

    //world.write().update(s);

    sandbox::run(world);
}
