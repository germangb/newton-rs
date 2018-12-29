use newton::Cgmath;

use newton::body::{Body, Type};
use newton::collision::{params::Params, Collision};
use newton::world::World;

use newton::sandbox;

use cgmath::{prelude::*, vec3, Matrix4};
use std::time::Duration;

fn main() {
    let world: World<Cgmath> = World::builder()
        .debug("world_0")
        .threads(1)
        .exact_solver()
        .build();

    // collision pool
    let pool = {
        let mut world = world.write();
        [
            world.collision().cuboid(24.0, 1.0, 24.0).build(),
            world.collision().cuboid(1.0, 1.0, 1.0).build(),
        ]
    };

    let floor = world
        .write()
        .body(&pool[0].read())
        .transform(Matrix4::identity())
        .build();

    let transforms = (0..16 * 4)
        .map(|i| (i % 16 / 4, i / 16, i % 16 % 4))
        .map(|(x, y, z)| vec3(x as f32, y as f32, z as f32))
        .map(|p| Matrix4::from_translation(p - vec3(1.5, -4.0, 1.5)));

    let cubes: Vec<_> = transforms
        .map(|p| {
            let cube = world.write().body(&pool[1].read()).transform(p).build();
            let g = vec3(0.0, -9.8, 0.0);

            cube.write().set_mass(1.0);
            cube.write()
                .set_force_and_torque(move |b, _, _| b.set_force(&g));
            cube
        })
        .collect();

    for _ in world.read().bodies() {}

    world.write().update(Duration::new(0, 1000000000 / 60));
    let world = world.read().as_raw();
    sandbox::run(world)
}

fn pos(x: f32, y: f32, z: f32) -> Matrix4<f32> {
    Matrix4::from_translation(vec3(x, y, z))
}
