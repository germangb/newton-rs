use newton::body::{Body, NewtonBodyData};
use newton::collision::Collision;
use newton::world::{self, World};

use newton::sandbox;

use cgmath::{prelude::*, vec3, Matrix4, Quaternion, Vector3};
use std::time::Duration;

use std::sync::mpsc::{self, Receiver, Sender};

fn main() {
    let world: World<i32, ()> = World::builder().build();

    let (pm, sm) = world.write().create_materials();

    let cube = Collision::builder(&mut world.write())
        .cuboid(2.0, 2.0, 2.0)
        .build();
    let sat = Body::builder(&mut world.write(), &cube.read())
        .transform(Matrix4::from_translation(Vector3::new(24.0, 0.0, 0.0)))
        .dynamic()
        .data(42)
        .material(sm)
        .mass(4.0)
        .force_torque_callback(|b, _, _| {
            let d = b.position().magnitude2() * 0.001;
            let gravity = b.position().normalize() * -9.81 / d;
            b.set_force(&gravity);

            let data = NewtonBodyData::from(b);
            //println!("data: {:?}", data.get());
        })
        .build();

    let sphere = Collision::builder(&mut world.write()).sphere(16.0).build();
    let planet = Body::builder(&mut world.write(), &sphere.read())
        .material(pm)
        .build();

    let sat_pos = sat.read().position();
    let s = Duration::new(0, 1000000000 / 60);
    sat.write().add_impulse(&vec3(0.0, 8.0, 4.0), &sat_pos, s);

    let (tx, rx) = mpsc::channel::<()>();

    // Destroy sat after 4 seconds
    let t = std::thread::spawn(move || {
        if let Ok(_) = rx.recv() {
            drop(sat);
        }
    });

    world
        .write()
        .material_set_collision_callback((pm, sm), move |_m, _b0, _b1, _| true);

    //world.write().update(s);

    sandbox::run(world);

    std::thread::sleep(Duration::new(4, 0));
    tx.send(());
    t.join();
}
