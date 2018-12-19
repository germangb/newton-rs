extern crate newton_sandbox;
extern crate newton;

use newton_sandbox::{Event, EventHandler};
use newton_sandbox::Sandbox;

use newton_sandbox::math::*;
use newton_sandbox::NewtonWorld;
use newton_sandbox::NewtonCuboid;

fn main() {
    let world = NewtonWorld::new();
    let shape = NewtonCuboid::new(&world, 1.0, 1.0, 1.0);

    let bodies: Vec<_> = [
        (0.0, 0.0, 0.0),
        (0.623, 1.5, 0.245),
        (-0.123, 2.64, -0.145),
        (-0.123, 3.84, -0.145),
        (-0.123, 4.94, -0.145),
        (-0.023, 6.0, -0.245),
    ]
        .iter()
        .map(|&(x, y, z)| Vector3::new(x, y, z))
        .map(Matrix4::from_translation)
        .map(|m| {
            shape.body(m)
                .mass_compute(1.0)
                .build()
        })
        .collect();


    let shape = NewtonCuboid::new(&world, 16.0, 1.0, 16.0);
    let floor = shape.body(Matrix4::from_translation(Vector3::new(0.0, -4.0, 0.0)))
        .build();

    // remove floor body
    //drop(floor);

    Sandbox::<()>::new()
        .run(world, || bodies.clone());
}
