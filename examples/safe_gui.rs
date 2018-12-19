extern crate sandbox;
extern crate newton_dynamics as newton;

use sandbox::Sandbox;

use newton::Array;
use newton::world::NewtonWorld;
use newton::collision::NewtonCuboid;

const fn position(x: f32, y: f32, z: f32) -> [[f32; 4]; 4] {
    [[1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [x, y, z, 1.0]]
}

fn main() {
    let world = NewtonWorld::<Array>::new();
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
        .map(|&(x, y, z)| {
            shape.body(position(x, y, z))
                .mass_compute(1.0)
                .build()
        })
        .collect();


    let shape = NewtonCuboid::new(&world, 16.0, 1.0, 16.0);
    let floor = shape.body(position(0.0, -4.0, 0.0))
        .build();

    // remove floor body
    //drop(floor);

    Sandbox::new().run(world, || bodies.clone());
}
