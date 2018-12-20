extern crate newton_sandbox;

use newton_sandbox::Sandbox;

use newton_sandbox::math::*;
use newton_sandbox::{NewtonCollision, NewtonWorld};

use newton::callback::Gravity;

fn main() {
    let world = NewtonWorld::new();
    let shape = NewtonCollision::cuboid(&world, 1.0, 1.0, 1.0, None);

    let bodies: Vec<_> = [(0.623, 0.245), (-0.123, -0.145), (0.023, -0.245)]
        .iter()
        .cycle()
        .take(16) // spawn 16 bodies
        .enumerate()
        .map(|(i, &(x, z))| Vector3::new(x, (i as f32) * 1.2, z))
        .map(Matrix4::from_translation)
        .map(|m| {
            let body = shape.body(m);

            body.set_mass(1.0, &shape);
            body.set_update::<Gravity>();
            body
        })
        .collect();

    let shape: NewtonCollision = NewtonCollision::cuboid(&world, 16.0, 1.0, 16.0, None);
    let floor = shape.body(Matrix4::from_translation(Vector3::new(0.0, -4.0, 0.0)));

    // remove floor body
    //drop(floor);

    Sandbox::new().size(800, 600).run(world, || bodies.clone());
}
