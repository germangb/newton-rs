extern crate cgmath;
extern crate newton_dynamics;

use std::thread;
use std::time::Duration;

use cgmath::{prelude::*, Matrix4};

use newton_dynamics::{
    body::NewtonBody,
    collision::{NewtonCollision, ShapeId},
    traits::NewtonCgmath as Cgmath,
    world::NewtonWorld,
};

fn main() {
    let world: NewtonWorld<Cgmath> = NewtonWorld::new();
    let cube_shape = NewtonCollision::new_box(&world, (1.0, 1.0, 1.0), ShapeId(0), None);

    let cube = NewtonBody::new(&world, cube_shape, Matrix4::<f32>::identity());
    cube.set_mass_matrix(1.0, (1.0, 1.0, 1.0));

    for i in 0..8 {
        println!("iter #{}: {:?}", i, cube.get_matrix());

        let step = Duration::new(1, 0);
        world.update(step);
        thread::sleep(step);
    }
}
