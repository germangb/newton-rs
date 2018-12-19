extern crate newton;

use newton::Array;
use newton::world::NewtonWorld;
use newton::collision::NewtonCuboid;

use std::time::Duration;

fn main() {
    let world = NewtonWorld::<Array>::new();

    let cube = NewtonCuboid::new(&world, 1.0, 1.0, 1.0)
        .body(position(0.0, 0.0, 0.0))
        .mass_compute(1.0)
        .build();

    for i in 0..8 {
        // 5 steps/second
        world.update(Duration::new(0, 1_000_000_000 / 5));

        println!("Iter {}, [x,y,z] = {:?}", i, &cube.matrix()[3][0..3]);
    }
}

const fn position(x: f32, y: f32, z: f32) -> [[f32; 4]; 4] {
    [[1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [x, y, z, 1.0]]
}
