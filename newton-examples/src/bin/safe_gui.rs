extern crate newton_sandbox;

use newton_sandbox::Sandbox;

use newton_sandbox::math::*;
use newton_sandbox::{NewtonCollision, NewtonWorld};

use newton::callback::Gravity;

fn main() {
    let world = NewtonWorld::new();

    let shapes = [
        NewtonCollision::cuboid(&world, 1.0, 1.0, 1.0, None),
        NewtonCollision::cone(&world, 1.0, 1.0, None),
        NewtonCollision::sphere(&world, 0.6, None),
    ];

    let mut bodies: Vec<_> = [(0.623, 0.245), (-0.123, -0.145), (0.023, -0.245)]
        .iter()
        .cycle()
        .take(32)
        .enumerate()
        .map(|(i, &(x, z))| Vector3::new(x, 4.0 + (i as f32) * 1.2, z))
        .map(Matrix4::from_translation)
        .enumerate()
        .map(|(i, m)| {
            let body = shapes[i % 3].body(m);
            body.set_mass(1.0, &shapes[i % 3]);
            body.set_update::<Gravity>();
            body
        })
        .collect();

    let floor = NewtonCollision::cuboid(&world, 16.0, 0.2, 16.0, None);

    bodies.push(floor.body(Matrix4::from_translation(Vector3::new(0.0, 0.0, 0.0))));

    let wall = NewtonCollision::cuboid(&world, 16.0, 2.0, 0.2, None);
    bodies.push(wall.body(Matrix4::from_translation(Vector3::new(0.0, 1.0, 8.0))));
    bodies.push(wall.body(Matrix4::from_translation(Vector3::new(0.0, 1.0, -8.0))));

    let wall = NewtonCollision::cuboid(&world, 0.2, 2.0, 16.0, None);
    bodies.push(wall.body(Matrix4::from_translation(Vector3::new(8.0, 1.0, 0.0))));
    bodies.push(wall.body(Matrix4::from_translation(Vector3::new(-8.0, 1.0, 0.0))));

    Sandbox::new().size(800, 600).run(world, bodies);
}
