extern crate newton_sandbox;

use newton_sandbox::Sandbox;

use newton_sandbox::math::*;
use newton_sandbox::{NewtonCollision, NewtonWorld};

use newton::callback::Gravity;

fn main() {
    let world = NewtonWorld::new();
    let cube = NewtonCollision::cuboid(&world, 1.0, 1.0, 1.0, None);
    let cone = NewtonCollision::cone(&world, 1.0, 1.0, None);
    let sphere = NewtonCollision::sphere(&world, 0.6, None);

    let mut bodies: Vec<_> = [(0.623, 0.245), (-0.123, -0.145), (0.023, -0.245)]
        .iter()
        .cycle()
        .take(16)
        .enumerate()
        .map(|(i, &(x, z))| Vector3::new(x, (i as f32) * 1.2, z))
        .map(Matrix4::from_translation)
        .enumerate()
        .map(|(i, m)| {
            let body = match i % 3 {
                0 => {
                    let b = cube.body(m);
                    b.set_mass(1.0, &cube);
                    b
                }
                1 => {
                    let b = sphere.body(m);
                    b.set_mass(1.0, &sphere);
                    b
                }
                2 => {
                    let b = cone.body(m);
                    b.set_mass(1.0, &cone);
                    b
                }
                _ => unreachable!(),
            };

            body.set_update::<Gravity>();
            body
        })
        .collect();

    let floor = NewtonCollision::cuboid(&world, 16.0, 0.2, 16.0, None);

    bodies.push(floor.body(Matrix4::from_translation(Vector3::new(0.0, -4.0, 0.0))));

    let wall = NewtonCollision::cuboid(&world, 16.0, 2.0, 0.2, None);
    bodies.push(wall.body(Matrix4::from_translation(Vector3::new(0.0, -3.0, 8.0))));
    bodies.push(wall.body(Matrix4::from_translation(Vector3::new(0.0, -3.0, -8.0))));

    let wall = NewtonCollision::cuboid(&world, 0.2, 2.0, 16.0, None);
    bodies.push(wall.body(Matrix4::from_translation(Vector3::new(8.0, -3.0, 0.0))));
    bodies.push(wall.body(Matrix4::from_translation(Vector3::new(-8.0, -3.0, 0.0))));

    Sandbox::new().size(800, 600).run(world, bodies);
}
