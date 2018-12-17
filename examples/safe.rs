// Integrate the position of a 1x1x1 cube in free fall
//
extern crate cgmath;
extern crate newton_dynamics;

use std::thread;
use std::time::{Duration, Instant};

use cgmath::{prelude::*, Matrix4};

use newton_dynamics::{
    traits::NewtonCgmath as Cgmath, NewtonBody, NewtonCollision, NewtonWorld, ShapeId,
};

fn main() {
    let world: NewtonWorld<Cgmath> = NewtonWorld::new();
    let box_shape = NewtonCollision::new_box(&world, (1.0, 1.0, 1.0), ShapeId(0), None);

    let cube = NewtonBody::new(&world, box_shape, Matrix4::identity());
    cube.set_mass_matrix(1.0, (1.0, 1.0, 1.0)); // unit mass

    let step = Duration::new(0, 250_000_000); // 0.25 second steps
    let start = Instant::now();

    for i in 0..16 {
        thread::sleep(step);
        world.update(step);

        let position = cube.get_matrix()[3];
        println!(
            "Iter #{}, time = {:?}, position = {:?}",
            i,
            start.elapsed(),
            position
        );
    }
}
