#[macro_use]
extern crate sandbox;
extern crate cgmath;
extern crate newton_dynamics;

use cgmath::prelude::*;
use cgmath::{Matrix4, Vector3};

use sandbox::{Example, Sandbox};

use newton_dynamics::traits::NewtonCgmath;
use newton_dynamics::{NewtonBody, NewtonCollision, NewtonWorld};

use std::rc::Rc;
use std::time::Duration;

struct Demo {
    world: Rc<NewtonWorld<NewtonCgmath>>,
    bodies: Vec<Rc<NewtonBody<NewtonCgmath>>>,
}

impl Example<NewtonCgmath> for Demo {
    fn step(&self) -> (&NewtonWorld<NewtonCgmath>, Duration) {
        (&self.world, Duration::new(0, 1_000_000_000 / 30))
    }

    fn bodies(&self) -> &[Rc<NewtonBody<NewtonCgmath>>] {
        &self.bodies[..]
    }
}

fn main() {
    let world = NewtonWorld::new();
    let shape = NewtonCollision::new_box(world.clone(), (1.0, 1.0, 1.0), 0.into(), None);

    const S: usize = 4;

    let mut bodies: Vec<_> = (0..S * S * S)
        .map(|mut i| {
            let x = (i / (S * S)) as f32 - 2.0;
            i %= S * S;
            let y = (i / S) as f32 - 2.0;
            i %= S;
            let z = i as f32 - 2.0;

            (x * 2.01 + y * 0.25, y * 2.01, z * 2.01 + x * 0.25)
        })
        .map(|(x, y, z)| {
            let transform = Matrix4::from_translation(Vector3::new(x, y, z));
            let body = NewtonBody::new(world.clone(), &shape, transform);
            body.set_mass_properties(1.0, &shape);
            body
        })
        .collect();

    let floor = NewtonCollision::new_box(world.clone(), (32.0, 0.1, 32.0), 0.into(), None);
    bodies.push(NewtonBody::new(
        world.clone(),
        &floor,
        Matrix4::from_translation(Vector3::new(0.0, -5.0, 0.0)),
    ));

    let demo = Demo { world, bodies };

    let mut sandbox = Sandbox::new();
    //sandbox.set_wireframe(true);
    sandbox.run(demo);
}
