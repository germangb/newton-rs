use newton::prelude::*;
use newton::{Cuboid, DynamicBody, Newton, Sphere};

fn main() {
    // init newton world
    let mut world = Newton::create();

    // Create collisions
    let sphere = Sphere::create(&world, 1.0, None);
    let plane = Cuboid::create(&world, 16.0, 0.1, 16.0, None);

    // create plane
    DynamicBody::create(&world, &plane, pos(0.0, 0.0, 0.0), None).into_handle(&world);

    /// Create free-fall body
    let body = DynamicBody::create(&world, &sphere, pos(0.0, 8.0, 0.0), None);
    body.set_mass(1.0, &sphere);
    body.set_force_and_torque_callback(|b, _, _| {
        let (mass, _) = b.mass();
        b.set_force([0.0, -9.8 * mass, 0.0])
    });

    // drop collisions because we no longer need them and they keep the world borrowed.
    drop(sphere);
    drop(plane);

    println!("Body starts at position = {:?}", body.position());

    // save body for later...
    let h = body.into_handle(&world);

    // simulate 4 seconds...
    for it in 0..(60 * 4) {
        world.update(std::time::Duration::new(0, 1_000_000_000 / 60));

        let body = world.body(h).unwrap();
        println!("position = {:?}", body.position());
    }
}

fn pos(x: f32, y: f32, z: f32) -> [[f32; 4]; 4] {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [x, y, z, 1.0],
    ]
}
